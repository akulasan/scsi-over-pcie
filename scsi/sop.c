/*
 *    SCSI over PCI (SoP) driver
 *    Copyright 2012 Hewlett-Packard Development Company, L.P.
 *    Copyright 2012 SanDisk Inc.
 *
 *    This program is licensed under the GNU General Public License
 *    version 2
 *
 *    This program is distributed "as is" and WITHOUT ANY WARRANTY
 *    of any kind whatsoever, including without limitation the implied
 *    warranty of MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
 *    Please see the GNU General Public License v.2 at
 *    http://www.gnu.org/licenses/licenses.en.html for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 *
 *    Questions/Comments/Bugfixes to iss_storagedev@hp.com
 *
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/sched.h>
/* #include <asm/byteorder.h> */
#include <linux/init.h>
#include <linux/version.h>
#include <linux/completion.h>
#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>

#include "sop_kernel_compat.h"
#include "sop.h"

#define DRIVER_VERSION "1.0.0"
#define DRIVER_NAME "sop (v " DRIVER_VERSION ")"
#define SOP "sop"

MODULE_AUTHOR("Hewlett-Packard Company");
MODULE_AUTHOR("SanDisk Inc.");
MODULE_DESCRIPTION("sop driver" DRIVER_VERSION);
MODULE_SUPPORTED_DEVICE("sop devices");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#ifndef PCI_VENDOR_SANDISK
#define PCI_VENDOR_SANDISK 0x15b7
#endif

DEFINE_PCI_DEVICE_TABLE(sop_id_table) = {
	{ PCI_VENDOR_SANDISK, 0x0012, PCI_VENDOR_SANDISK, 0x0000 },
	{ PCI_VENDOR_SANDISK, 0x0021, PCI_VENDOR_SANDISK, 0x0000 },
	{ PCI_VENDOR_SANDISK, 0x2100, PCI_VENDOR_SANDISK, 0x0000 },
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, sop_id_table);

static inline struct sop_device *shost_to_hba(struct Scsi_Host *sh)
{
	unsigned long *priv = shost_priv(sh);
	return (struct sop_device *) *priv;
}

static ssize_t host_show_sopstats(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        struct sop_device *h;
        struct Scsi_Host *shost = class_to_shost(dev);
	int moc;

        h = shost_to_hba(shost);
	moc = atomic_read(&h->max_outstanding_commands);
        return snprintf(buf, 20, "%d\n", moc);
}

static DEVICE_ATTR(sopstats, S_IRUGO, host_show_sopstats, NULL);

static struct device_attribute *sop_host_attrs[] = {
        &dev_attr_sopstats,
        NULL,
};

static int controller_num;

static int sop_queuecommand(struct Scsi_Host *h, struct scsi_cmnd *sc);
static int sop_change_queue_depth(struct scsi_device *sdev,
        int qdepth, int reason);
static int sop_abort_handler(struct scsi_cmnd *sc);
static int sop_device_reset_handler(struct scsi_cmnd *sc);
static int sop_slave_alloc(struct scsi_device *sdev);
static void sop_slave_destroy(struct scsi_device *sdev);
static int sop_compat_ioctl(struct scsi_device *dev, int cmd, void *arg);
static int sop_ioctl(struct scsi_device *dev, int cmd, void *arg);

static struct scsi_host_template sop_template = {
	.module				= THIS_MODULE,
	.name				= DRIVER_NAME,
	.proc_name			= DRIVER_NAME,
	.queuecommand			= sop_queuecommand,
	.change_queue_depth		= sop_change_queue_depth,
	.this_id			= -1,
	.use_clustering			= ENABLE_CLUSTERING,
	.eh_abort_handler		= sop_abort_handler,
	.eh_device_reset_handler	= sop_device_reset_handler,
	.ioctl				= sop_ioctl,
	.slave_alloc			= sop_slave_alloc,
	.slave_destroy			= sop_slave_destroy,
#ifdef CONFIG_COMPAT
	.compat_ioctl			= sop_compat_ioctl,
#endif
#if 0
	.sdev_attrs			= sop_sdev_attrs,
#endif
	.shost_attrs			= sop_host_attrs,
	.max_sectors			= (MAX_SGLS * 8),
};

static pci_ers_result_t sop_pci_error_detected(struct pci_dev *dev,
				enum pci_channel_state error);
static pci_ers_result_t sop_pci_mmio_enabled(struct pci_dev *dev);
static pci_ers_result_t sop_pci_link_reset(struct pci_dev *dev);
static pci_ers_result_t sop_pci_slot_reset(struct pci_dev *dev);
static void sop_pci_resume(struct pci_dev *dev);

static struct pci_error_handlers sop_pci_error_handlers = {
	.error_detected = sop_pci_error_detected,
	.mmio_enabled = sop_pci_mmio_enabled,
	.link_reset = sop_pci_link_reset,
	.slot_reset = sop_pci_slot_reset,
	.resume = sop_pci_resume,
};

/* 
 * 32-bit readq and writeq implementations taken from old
 * version of arch/x86/include/asm/io.h 
 */
#ifndef readq
static inline u64 readq(const volatile void __iomem *addr)
{
	const volatile u32 __iomem *p = addr;
	u32 low, high;

	low = readl(p);
	high = readl(p + 1);

	return low + ((u64)high << 32);
}
#endif

#ifndef writeq
static inline void writeq(u64 val, volatile void __iomem *addr)
{
	writel(val, addr);
	writel(val >> 32, addr+4);
}
#endif

static inline int check_for_read_failure(__iomem void *sig)
{
	/*
	 * do a readl of a known constant value, if it comes back -1,
	 * we know we're not able to read.
	 */
	u64 signature;

	signature = readq(sig);
	return signature == 0xffffffffffffffffULL;
}

static inline int safe_readw(__iomem void *sig, u16 *value,
				const volatile void __iomem *addr)
{
	*value = readw(addr);
	if (unlikely(*value == (u16) 0xffff)) {
		if (check_for_read_failure(sig))
			return -1;
	}
	return 0;
}

static inline int safe_readl(__iomem void *sig, u32 *value,
				const volatile void __iomem *addr)
{
	*value = readl(addr);
	if (unlikely(*value == 0xffffffff)) {
		if (check_for_read_failure(sig))
			return -1;
	}
	return 0;
}

static inline int safe_readq(__iomem void *sig, u64 *value,
				const volatile void __iomem *addr)
{
	*value = readq(addr);
	if (unlikely(*value == 0xffffffffffffffffULL)) {
		if (check_for_read_failure(sig))
			return -1;
	}
	return 0;
}

static void free_q_request_buffers(struct queue_info *q)
{
	if (q->request_bits) {
		kfree(q->request_bits);
		q->request_bits = NULL;
	}
	if (q->request) {
		kfree(q->request);
		q->request = NULL;
	}
}

static int allocate_sgl_area(struct sop_device *h,
		struct queue_info *q)
{
	size_t total_size = q->qdepth * MAX_SGLS *
				sizeof(struct pqi_sgl_descriptor);

	q->sg = pci_alloc_consistent(h->pdev, total_size, &q->sg_bus_addr);
	return (q->sg) ? 0 : -ENOMEM;
}

static void free_sgl_area(struct sop_device *h, struct queue_info *q)
{
	size_t total_size = q->qdepth * MAX_SGLS *
				sizeof(struct pqi_sgl_descriptor);

	if (!q->sg)
		return;
	pci_free_consistent(h->pdev, total_size, q->sg, q->sg_bus_addr);
	q->sg = NULL;
}

static void free_all_q_sgl_areas(struct sop_device *h)
{
	int i;

	for (i = 0; i < h->nr_queues; i++)
		free_sgl_area(h, &h->qinfo[i]);
}

static int allocate_q_request_buffers(struct queue_info *q,
	int nbuffers, int buffersize)
{
	q->qdepth = nbuffers;
	q->request_bits = kzalloc((BITS_TO_LONGS(nbuffers) + 1) *
					sizeof(unsigned long), GFP_KERNEL);
	if (!q->request_bits)
		goto bailout;
	q->request = kzalloc(buffersize * nbuffers, GFP_KERNEL);
	if (!q->request)
		goto bailout;
	return 0;

bailout:
	free_q_request_buffers(q);
	return -ENOMEM;
}

static void free_all_q_request_buffers(struct sop_device *h)
{
	int i;
	for (i = 0; i < h->nr_queues; i++)
		free_q_request_buffers(&h->qinfo[i]);
}

static int pqi_device_queue_array_alloc(struct sop_device *h,
		struct pqi_device_queue **xq, int num_queues,
		u16 n_q_elements, u8 q_element_size_over_16,
		int queue_direction, int starting_queue_id)
{
	void *vaddr = NULL;
	dma_addr_t dhandle;
	int i, nqs_alloced = 0, err = 0;
	int total_size = (n_q_elements * q_element_size_over_16 * 16) +
				sizeof(u64);
	int remainder = total_size % 64;

	total_size += remainder ? 64 - remainder : 0;

	err = -ENOMEM;
	*xq = kzalloc(sizeof(**xq) * num_queues, GFP_KERNEL);
	if (!*xq)
		goto bailout;
	vaddr = pci_alloc_consistent(h->pdev, total_size * num_queues, &dhandle);
	if (!vaddr)
		goto bailout;
	if (queue_direction == PQI_DIR_TO_DEVICE) {
		h->iq_dhandle = dhandle;
		h->iq_vaddr = vaddr;
	} else {
		h->oq_dhandle = dhandle;
		h->oq_vaddr = vaddr;
	}

	if (queue_direction == PQI_DIR_TO_DEVICE) {
		for (i = 0; i < num_queues; i++) {
			int q = i + starting_queue_id;
			if (allocate_q_request_buffers(&h->qinfo[q],
					n_q_elements,
					sizeof(struct sop_request)))
				goto bailout;
		}

		/* Allocate SGL area for each submission queue */
		for (i = 0; i < num_queues; i++) {
			int q = i + starting_queue_id;

			if (allocate_sgl_area(h, &h->qinfo[q]))
				goto bailout;
		}
	}

	err = 0;

	for (i = 0; i < num_queues; i++) {
		(*xq)[i].registers = h->pqireg;
		(*xq)[i].queue_vaddr = vaddr + (i * total_size);
		(*xq)[i].dhandle = dhandle + (i * total_size);
		if (queue_direction == PQI_DIR_TO_DEVICE) {
			(*xq)[i].ci = vaddr + (i * total_size) +
					q_element_size_over_16 * 16 * n_q_elements;
			/* (*xq)[i].pi is unknown now, hardware will tell us later */
		} else {
			(*xq)[i].pi = vaddr + (i * total_size) +
					q_element_size_over_16 * 16 * n_q_elements;
			/* (*xq)[i].ci is unknown now, hardware will tell us later */
		}
		(*xq)[i].unposted_index = 0;
		(*xq)[i].element_size = q_element_size_over_16 * 16;
		(*xq)[i].nelements = n_q_elements;
		(*xq)[i].queue_id = i + starting_queue_id;
		nqs_alloced++;
	}
	return nqs_alloced;

bailout:
	dev_warn(&h->pdev->dev, "Failed to allocate queues\n");
	kfree(*xq);
	free_all_q_sgl_areas(h);
	free_all_q_request_buffers(h);
	if (vaddr)
		pci_free_consistent(h->pdev, total_size, vaddr, dhandle);
	return err;
}

static void pqi_device_queue_init(struct pqi_device_queue *q,
		struct pqi_device_register_set *regs,
		__iomem void *queue_vaddr, __iomem u16 *pi,
		__iomem u16 *ci,
		u16 element_size, u8 nelements,
		dma_addr_t dhandle)
{
	q->registers = regs;
	q->queue_vaddr = queue_vaddr;
	q->pi = pi;
	q->ci = ci;
	q->unposted_index = 0;
	q->element_size = element_size;
	q->nelements = nelements;
	q->dhandle = dhandle;
	spin_lock_init(&q->index_lock);
}

static int pqi_to_device_queue_is_full(struct pqi_device_queue *q,
				int nelements)
{
	u16 qci;
	u32 nfree;

	if (safe_readw(&q->registers->signature, &qci, q->ci))
		return 0;
	qci = le16_to_cpu(*(u16 *) &qci);

	if (q->unposted_index > qci)
		nfree = q->nelements - q->unposted_index + qci - 1;
	else if (q->unposted_index < qci)
		nfree = qci - q->unposted_index - 1;
	else
		nfree = q->nelements;
	return (nfree < nelements);
}

static int pqi_from_device_queue_is_empty(struct pqi_device_queue *q)
{
	u16 qpi;

	if (safe_readw(&q->registers->signature, &qpi, q->pi))
		return 0;
	qpi = le16_to_cpu(qpi);
	return qpi == q->unposted_index;
}

static void *pqi_alloc_elements(struct pqi_device_queue *q,
					int nelements)
{
	void *p;

	if (pqi_to_device_queue_is_full(q, nelements)) {
		printk(KERN_WARNING "pqi device queue %d is full!\n", q->queue_id);
		return ERR_PTR(-ENOMEM);
	}

	/* If the requested number of elements would wrap around the
	 * end of the ring buffer, insert NULL IUs to the end of the
	 * ring buffer.  This simplifies the code which has to fill
	 * in the IUs as it doesn't have to deal with wrapping
	 */
	if (q->nelements - q->unposted_index < nelements) {
		int extra_elements = q->nelements - q->unposted_index;
		if (pqi_to_device_queue_is_full(q, nelements + extra_elements)) {
			printk(KERN_WARNING "pqi_alloc_elements, device queue is full!\n");
			printk(KERN_WARNING "q->nelements = %d, q->unposted_index = %hu, extra_elements = %d\n",
					q->nelements, q->unposted_index, extra_elements);
			return ERR_PTR(-ENOMEM);
		}
		p = q->queue_vaddr + q->unposted_index * q->element_size;
		memset(p, 0, (q->nelements - q->unposted_index) *
						q->element_size);
		q->unposted_index = 0;
	}
	p = q->queue_vaddr + q->unposted_index * q->element_size;
	q->unposted_index = (q->unposted_index + nelements) % q->nelements;
	return p;
}

static int pqi_dequeue_from_device(struct pqi_device_queue *q, void *element)
{
	void *p;

	if (pqi_from_device_queue_is_empty(q))
		return PQI_QUEUE_EMPTY;

	p = q->queue_vaddr + q->unposted_index * q->element_size;
	/* printk(KERN_WARNING "DQ: p = %p, q->unposted_index = %hu, n = %hu\n",
				p, q->unposted_index, q->nelements); */
	memcpy(element, p, q->element_size);
	q->unposted_index = (q->unposted_index + 1) % q->nelements;
	/* printk(KERN_WARNING "After DQ: q->unposted_index = %hu\n",
				q->unposted_index); */
	return 0;
}

static u8 pqi_peek_ui_type_from_device(struct pqi_device_queue *q)
{
	u8 *p;

	p = q->queue_vaddr + q->unposted_index * q->element_size;
	return *p;
}
static u16 pqi_peek_request_id_from_device(struct pqi_device_queue *q)
{
	u8 *p;

	p = q->queue_vaddr + q->unposted_index * q->element_size + 8;
	return *(u16 *) p;
}

static int xmargin=8;
static int amargin=60;

static void print_bytes(unsigned char *c, int len, int hex, int ascii)
{

	int i;
	unsigned char *x;

	if (hex) {
		x = c;
		for (i = 0; i < len; i++) {
			if ((i % xmargin) == 0 && i > 0)
				pr_warn("\n");
			if ((i % xmargin) == 0)
				pr_warn("0x%04x:", i);
			pr_warn(" %02x", *x);
			x++;
		}
		pr_warn("\n");
	}
	if (ascii) {
		x = c;
		for (i = 0; i < len; i++) {
			if ((i % amargin) == 0 && i > 0)
				pr_warn("\n");
			if ((i % amargin) == 0)
				pr_warn("0x%04x:", i);
			if (*x > 26 && *x < 128)
				pr_warn("%c", *x);
			else
				pr_warn(".");
			x++;
		}
		pr_warn("\n");
	}
}

static void print_iu(unsigned char *iu)
{
	u16 iu_length;

	memcpy(&iu_length, &iu[2], 2);
	iu_length = le16_to_cpu(iu_length) + 4;
	printk(KERN_WARNING "***** IU type = 0x%02x, len = %hd, compat_features = %02x *****\n",
			iu[0], iu_length, iu[1]);
	print_bytes(iu, (int) iu_length, 1, 0);
}

static void __attribute__((unused)) print_unsubmitted_commands(struct pqi_device_queue *q)
{
	u16 pi;
	int i;
	unsigned char *iu;
	unsigned long flags;

	spin_lock_irqsave(&q->index_lock, flags);
	pi = q->local_pi;
	if (pi == q->unposted_index) {
		printk(KERN_WARNING "submit queue is empty.\n");
		spin_unlock_irqrestore(&q->index_lock, flags);
		return;
	}
	if (pi < q->unposted_index) {
		for (i = pi; i < q->unposted_index; i++) {
			iu = (unsigned char *) q->queue_vaddr + (i * IQ_IU_SIZE);
			print_iu(iu);
		}
	} else {
		for (i = pi; i < q->nelements; i++) {
			iu = (unsigned char *) q->queue_vaddr + (i * IQ_IU_SIZE);
			print_iu(iu);
		}
		for (i = 0; i < q->unposted_index; i++) {
			iu = (unsigned char *) q->queue_vaddr + (i * IQ_IU_SIZE);
			print_iu(iu);
		}
	}
	spin_unlock_irqrestore(&q->index_lock, flags);
}

static void pqi_notify_device_queue_written(struct sop_device *h, struct pqi_device_queue *q)
{
	unsigned long flags;
	/*
	 * Notify the device that the host has produced data for the device
	 */

	spin_lock_irqsave(&q->index_lock, flags);
	q->local_pi = q->unposted_index;
	writew(q->unposted_index, q->pi);
	spin_unlock_irqrestore(&q->index_lock, flags);
	atomic_inc(&h->max_outstanding_commands);
}

static void pqi_notify_device_queue_read(struct pqi_device_queue *q)
{
	/*
	 * Notify the device that the host has consumed data from the device
	 */
	writew(q->unposted_index, q->ci);
}

static int wait_for_admin_command_ack(struct sop_device *h)
{
	u64 paf;
	u8 function_and_status;
	int count = 0;
	__iomem void *sig = &h->pqireg->signature;

#define ADMIN_SLEEP_INTERVAL_MIN 100 /* microseconds */
#define ADMIN_SLEEP_INTERVAL_MAX 150 /* microseconds */
#define ADMIN_SLEEP_INTERATIONS 1000 /* total of 100 milliseconds */

	do {
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
		if (safe_readq(sig, &paf, &h->pqireg->process_admin_function)) {
			dev_warn(&h->pdev->dev,
				"%s: Failed to read device memory\n", __func__);
			return -1;
		}
		function_and_status = paf & 0xff;
		if (function_and_status == 0x00)
			return 0;
		count++;
	} while (count < ADMIN_SLEEP_INTERATIONS);
	return -1;
}

static int __devinit sop_create_admin_queues(struct sop_device *h)
{
	u64 paf, pqicap, admin_iq_pi_offset, admin_oq_ci_offset;
	u32 status, admin_queue_param;
	u8 function_and_status;
	u8 pqi_device_state;
	int total_admin_queue_size;
	void *admin_iq = NULL, *admin_oq;
	u16 *admin_iq_ci, *admin_oq_pi, *admin_iq_pi, *admin_oq_ci;
	dma_addr_t admin_iq_busaddr, admin_oq_busaddr;
	dma_addr_t admin_iq_ci_busaddr, admin_oq_pi_busaddr;
	u16 msix_vector;
	int rc, count;
	unsigned char *x = (unsigned char *) &paf;
	dma_addr_t admin_q_dhandle;
	char *msg = "";
	__iomem void *sig = &h->pqireg->signature;

	/* Check that device is ready to be set up */
	for (count = 0; count < 10; count++) {
		if (safe_readq(sig, &paf, &h->pqireg->process_admin_function)) {
			msg = "Unable to read process admin function register";
			goto bailout;
		}
		if (safe_readl(sig, &status, &h->pqireg->pqi_device_status)) {
			msg = "Unable to read from device memory";
			goto bailout;
		}
		x = (unsigned char *) &status;
		function_and_status = paf & 0xff;
		pqi_device_state = status & 0xff;
	
		if (function_and_status != PQI_IDLE) {
			dev_warn(&h->pdev->dev,
				"Device not idle during initialization.\n");
			/* return -1; */
		}

		if (pqi_device_state != PQI_READY_FOR_ADMIN_FUNCTION) {
			dev_warn(&h->pdev->dev,
				"Device not ready during initialization.\n");
			/* return -1; */
		}
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
	}

	if (safe_readq(sig, &pqicap, &h->pqireg->capability)) {
		msg = "Unable to read pqi capability register";
		goto bailout;
	}
	memcpy(&h->pqicap, &pqicap, sizeof(h->pqicap));

/* #define ADMIN_QUEUE_ELEMENT_COUNT ((MAX_IO_QUEUES + 1) * 2) */
#define ADMIN_QUEUE_ELEMENT_COUNT 32

	if (h->pqicap.max_admin_iq_elements < ADMIN_QUEUE_ELEMENT_COUNT ||
		h->pqicap.max_admin_oq_elements < ADMIN_QUEUE_ELEMENT_COUNT) {
		dev_warn(&h->pdev->dev, "Can't create %d element PQI admin queues\n",
				ADMIN_QUEUE_ELEMENT_COUNT);
		return -1;
	}

	/*
	 * Calculate total size of inbound and outbound admin queues including
	 * queue elements and producer and consumer indexes, and allocate.
	 * The extra 32 bytes are for the producer and consumer indexes, which
	 * are only u16's, but we count them as 16 bytes each so that
	 * everything comes out cache-line aligned.
	 */
	total_admin_queue_size = ((h->pqicap.admin_iq_element_length * 16) +
				(h->pqicap.admin_oq_element_length * 16)) *
				ADMIN_QUEUE_ELEMENT_COUNT + 32;
	admin_iq = pci_alloc_consistent(h->pdev, total_admin_queue_size,
						&admin_q_dhandle);
	if (!admin_iq) {
		msg = "failed to allocate PQI admin queues";
		goto bailout;
	}
	admin_iq_ci = admin_iq + ADMIN_QUEUE_ELEMENT_COUNT *
				h->pqicap.admin_iq_element_length * 16;
	admin_oq = admin_iq + (h->pqicap.admin_iq_element_length * 16) *
				ADMIN_QUEUE_ELEMENT_COUNT + 16;
	admin_oq_pi = admin_oq + ADMIN_QUEUE_ELEMENT_COUNT *
				h->pqicap.admin_oq_element_length * 16;

	admin_iq_busaddr = admin_q_dhandle;
	admin_iq_ci_busaddr = admin_iq_busaddr +
				(h->pqicap.admin_iq_element_length * 16) *
				ADMIN_QUEUE_ELEMENT_COUNT;
	admin_oq_busaddr = admin_iq_ci_busaddr + 16;
	admin_oq_pi_busaddr = admin_oq_busaddr +
				(h->pqicap.admin_oq_element_length * 16) *
				ADMIN_QUEUE_ELEMENT_COUNT;

#define PQI_REG_ALIGNMENT 16

	if (admin_iq_busaddr % PQI_REG_ALIGNMENT != 0 ||
		admin_oq_busaddr % PQI_REG_ALIGNMENT != 0 ||
		admin_iq_ci_busaddr % PQI_REG_ALIGNMENT != 0 ||
		admin_oq_pi_busaddr % PQI_REG_ALIGNMENT != 0) {
		dev_warn(&h->pdev->dev, "Admin queues are not properly aligned.\n");
		dev_warn(&h->pdev->dev, "admin_iq_busaddr = %p\n",
				(void *) admin_iq_busaddr);
		dev_warn(&h->pdev->dev, "admin_oq_busaddr = %p\n",
				(void *) admin_oq_busaddr);
		dev_warn(&h->pdev->dev, "admin_iq_ci_busaddr = %p\n",
				(void *) admin_iq_ci_busaddr);
		dev_warn(&h->pdev->dev, "admin_oq_pi_busaddr = %p\n",
				(void *) admin_oq_pi_busaddr);
	}

	msix_vector = 0; /* Admin Queue always uses vector [0] */
	admin_queue_param = ADMIN_QUEUE_ELEMENT_COUNT |
			(ADMIN_QUEUE_ELEMENT_COUNT << 8) |
			(msix_vector << 16);

	/* Tell the hardware about the admin queues */
	writeq(admin_iq_busaddr, &h->pqireg->admin_iq_addr);
	writeq(admin_oq_busaddr, &h->pqireg->admin_oq_addr);
	writeq(admin_iq_ci_busaddr, &h->pqireg->admin_iq_ci_addr);
	writeq(admin_oq_pi_busaddr, &h->pqireg->admin_oq_pi_addr);
	writel(admin_queue_param, &h->pqireg->admin_queue_param);
	writeq(PQI_CREATE_ADMIN_QUEUES, &h->pqireg->process_admin_function);

	rc = wait_for_admin_command_ack(h);
	if (rc) {
		if (safe_readq(sig, &paf, &h->pqireg->process_admin_function)) {
			msg = "Failed reading process admin function register";
			goto bailout;
		}
		function_and_status = paf & 0xff;
		dev_warn(&h->pdev->dev,
			"Failed to create admin queues: function_and_status = 0x%02x\n",
			function_and_status);
		if (function_and_status == 0) {
			msg = "Failed waiting for admin command ack";
			goto bailout;
		}
		if (safe_readl(sig, &status, &h->pqireg->pqi_device_status)) {
			msg = "Failed reading pqi device status register";
			goto bailout;
		}
		dev_warn(&h->pdev->dev, "Device status = 0x%08x\n", status);
	}

	/* Get the offsets of the hardware updated producer/consumer indices */
	if (safe_readq(sig, &admin_iq_pi_offset,
				&h->pqireg->admin_iq_pi_offset)) {
		msg = "Unable to read admin iq pi offset register";
		goto bailout;
	}
	if (safe_readq(sig, &admin_oq_ci_offset,
				&h->pqireg->admin_oq_ci_offset)) {
		msg = "Unable to read admin oq ci offset register";
		goto bailout;
	}
	admin_iq_pi = ((void *) h->pqireg) + admin_iq_pi_offset;
	admin_oq_ci = ((void *) h->pqireg) + admin_oq_ci_offset;

	if (safe_readl(sig, &status, &h->pqireg->pqi_device_status)) {
		msg = "Failed to read device status register";
		goto bailout;
	}
	x = (unsigned char *) &status;
	function_and_status = paf & 0xff;
	pqi_device_state = status & 0xff;

	pqi_device_queue_init(&h->admin_q_from_dev, h->pqireg,
		admin_oq, admin_oq_pi, admin_oq_ci,
		(u16) h->pqicap.admin_oq_element_length * 16,
		ADMIN_QUEUE_ELEMENT_COUNT, admin_oq_busaddr);

	pqi_device_queue_init(&h->admin_q_to_dev, h->pqireg,
		admin_iq, admin_iq_pi, admin_iq_ci,
		(u16) h->pqicap.admin_iq_element_length * 16,
		ADMIN_QUEUE_ELEMENT_COUNT, admin_iq_busaddr);

	h->qinfo[0].pqiq = &h->admin_q_from_dev;
	h->qinfo[1].pqiq = &h->admin_q_to_dev;

	/* Allocate request buffers for admin queues */
	if (allocate_q_request_buffers(&h->qinfo[0],
				ADMIN_QUEUE_ELEMENT_COUNT,
				sizeof(struct sop_request))) {
		msg = "Failed to allocate request queue buffer for queue 0";
		goto bailout;
	}
	if (allocate_q_request_buffers(&h->qinfo[1],
				ADMIN_QUEUE_ELEMENT_COUNT,
				sizeof(struct sop_request))) {
		msg = "Failed to allocate request queue buffer for queue 1";
		goto bailout;
	}
	return 0;

bailout:
	if (admin_iq)
		pci_free_consistent(h->pdev, total_admin_queue_size,
					admin_iq, admin_iq_busaddr);
	free_all_q_request_buffers(h);
	dev_warn(&h->pdev->dev, "%s: %s\n", __func__, msg);
	return -1;	
}

static int wait_for_admin_queues_to_become_idle(struct sop_device *h)
{
	int i;
	u64 paf;
	u32 status;
	u8 pqi_device_state, function_and_status;
	__iomem void *sig = &h->pqireg->signature;

	for (i = 0; i < ADMIN_SLEEP_INTERATIONS; i++) {
		if (safe_readq(sig, &paf,
				&h->pqireg->process_admin_function)) {
			dev_warn(&h->pdev->dev,
				"Cannot read process admin function register");
			return -1;
		}
		paf &= 0x0ff;
		if (safe_readl(sig, &status, &h->pqireg->pqi_device_status)) {
			dev_warn(&h->pdev->dev,
				"Cannot read device status register");
			return -1;
		}
		function_and_status = paf & 0xff;
		pqi_device_state = status & 0xff;
		if (function_and_status == PQI_IDLE &&
			pqi_device_state == PQI_READY_FOR_IO)
			return 0;
		if (i == 0)
			dev_warn(&h->pdev->dev,
				"Waiting for admin queues to become idle\n");
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
	}
	dev_warn(&h->pdev->dev,
			"Failed waiting for admin queues to become idle.");
	return -1;
}

static int sop_delete_admin_queues(struct sop_device *h)
{
	u64 paf;
	u32 status;
	u8 function_and_status;
	__iomem void *sig = &h->pqireg->signature;

	if (wait_for_admin_queues_to_become_idle(h))
		return -1;
	writeq(PQI_DELETE_ADMIN_QUEUES, &h->pqireg->process_admin_function);
	if (!wait_for_admin_command_ack(h))
		return 0;

	/* Try to get some clues about why it failed. */
	dev_warn(&h->pdev->dev, "Failed waiting for admin command acknowledgment\n");
	if (safe_readq(sig, &paf,  &h->pqireg->process_admin_function)) {
		dev_warn(&h->pdev->dev,
			"Cannot read process admin function register");
		return -1;
	}
	function_and_status = paf & 0xff;
	dev_warn(&h->pdev->dev,
		"Failed to delete admin queues: function_and_status = 0x%02x\n",
		function_and_status);
	if (function_and_status == 0)
		return -1;
	if (safe_readl(sig, &status, &h->pqireg->pqi_device_status)) {
		dev_warn(&h->pdev->dev,
			"Failed to read device status register");
		return -1;
	}
	dev_warn(&h->pdev->dev, "Device status = 0x%08x\n", status);
	return -1;
}

static int sop_setup_msix(struct sop_device *h)
{
	int i, err;

	struct msix_entry msix_entry[MAX_TOTAL_QUEUES];

	h->nr_queues = num_online_cpus() * 2;
	BUILD_BUG_ON((MAX_TOTAL_QUEUES % 2) != 0);
	if (h->nr_queues > MAX_TOTAL_QUEUES)
		h->nr_queues = MAX_TOTAL_QUEUES;

	h->niqs = h->noqs = h->nr_queues / 2;
	h->niqs = h->noqs;

	dev_warn(&h->pdev->dev, "zzz 1 h->noqs = %d, h->niqs = %d\n",
				h->noqs, h->niqs);

	for (i = 0; i < h->noqs; i++) {
		msix_entry[i].vector = 0;
		msix_entry[i].entry = i;
	}

	if (!pci_find_capability(h->pdev, PCI_CAP_ID_MSIX))
		goto default_int_mode;
	err = pci_enable_msix(h->pdev, msix_entry, h->noqs);
	if (err > 0)
		h->noqs = err;

	dev_warn(&h->pdev->dev,
		"zzz 2 (after pci_enable_msix) h->noqs = %d, h->niqs = %d\n",
		h->noqs, h->niqs);

	if (err >= 0) {
		for (i = 0; i < h->noqs; i++) {
			int idx = i ? i + 1 : i;
			h->qinfo[idx].msix_vector = msix_entry[i].vector;
		}
		h->intr_mode = INTR_MODE_MSIX;
		return 0;
	}
	if (err > 0)
		dev_warn(&h->pdev->dev,
				"only %d MSI-X vectors available\n", err);
	else
		dev_warn(&h->pdev->dev, "MSI-X init failed %d\n", err);
default_int_mode:
	return -1;
}

/* function to determine whether a complete response has been accumulated */
static int sop_response_accumulated(struct sop_request *r)
{
	u16 iu_length;

	if (r->response_accumulated == 0)
		return 0;
	iu_length = le16_to_cpu(*(u16 *) &r->response[2]) + 4;
	return (r->response_accumulated >= iu_length);
}

static void free_request(struct sop_device *h, u8 q, u16 request_id);

static void main_io_path_decode_response_data(struct sop_device *h,
					struct sop_cmd_response *scr,
					struct scsi_cmnd *scmd)
{
	char *msg;
	u8 firmware_bug = 0;

	switch (scr->response[3]) {
	case SOP_TMF_COMPLETE:
	case SOP_TMF_REJECTED:
	case SOP_TMF_FAILED:
	case SOP_TMF_SUCCEEDED:
		/*
		 * There should be no way to submit a Task Management Function
		 * IU via the main i/o path, so don't expect TMF response data.
		 */
		msg = "Received TMF response in main i/o path.\n";
		firmware_bug = 1;
		break;
	case SOP_INCORRECT_LUN:
		msg = "Incorrect LUN response.\n";
		break;
	case SOP_OVERLAPPED_REQUEST_ID_ATTEMPTED:
		msg = "Overlapped request ID attempted.\n";
		break;
	case SOP_INVALID_IU_TYPE:
		msg = "Invaid IU type";
		break;
	case SOP_INVALID_IU_LENGTH:
		msg = "Invalid IU length";
		break;
	case SOP_INVALID_LENGTH_IN_IU:
		msg = "Invalid length in IU";
		break;
	case SOP_MISALIGNED_LENGTH_IN_IU:
		msg = "Misaligned length in IU";
		break;
	case SOP_INVALID_FIELD_IN_IU:
		msg = "Invalid field in IU";
		break;
	case SOP_IU_TOO_LONG:
		msg = "IU too long";
		break;
	default:
		msg = "Unknown response type";
	}
	scmd->result |= (DID_ERROR << 16);
	dev_warn(&h->pdev->dev,
		"Unexpected response in main i/o path: %s. Suspect %s bug.\n",
		msg, firmware_bug ? "firmware" : "driver");
	return;
}

static void handle_management_response(struct sop_device *h, 
					struct management_response_iu * mr,
					struct scsi_cmnd *scmd)
{
	switch (mr->result) {
	case MGMT_RSP_RSLT_GOOD:
		scsi_set_resid(scmd, 0); /* right??? */
		dev_warn(&h->pdev->dev,
			"Management IU response: good result\n");
		return;
	case MGMT_RSP_RSLT_UNKNOWN_ERROR:
		dev_warn(&h->pdev->dev,
			"Management IU response: unknown error\n");
		break;
	case MGMT_RSP_RSLT_INVALID_FIELD_IN_REQUEST_IU:
		dev_warn(&h->pdev->dev,
			"Management IU response: Invalid field in request IU\n");
		break;
	case MGMT_RSP_RSLT_INVALID_FIELD_IN_DATA_OUT_BUFFER:
		dev_warn(&h->pdev->dev,
			"Management IU response: Invalid field in data out buffer\n");
		break;
	case MGMT_RSP_RSLT_VENDOR_SPECIFIC_ERROR:
	case MGMT_RSP_RSLT_VENDOR_SPECIFIC_ERROR2:
		dev_warn(&h->pdev->dev,
			"Management IU response: vendor specific error\n");
		break;
	default:
		dev_warn(&h->pdev->dev,
			"Management IU response: unknown response: %02x\n",
				mr->result);
		break;
	}
	scmd->result |= (DID_ERROR << 16);
}

static void complete_scsi_cmd(struct sop_device *h,
				struct sop_request *r)
{
	struct scsi_cmnd *scmd;
	struct sop_cmd_response *scr;
	struct management_response_iu *mr;
	u16 sense_data_len;
	u16 response_data_len;
	u32 data_xferred;

        scmd = r->scmd;
        scsi_dma_unmap(scmd); /* undo the DMA mappings */

        scmd->result = (DID_OK << 16);           /* host byte */
        scmd->result |= (COMMAND_COMPLETE << 8); /* msg byte */
	free_request(h, r->request_id >> h->qid_shift, r->request_id);

	switch (r->response[0]) {
	case SOP_RESPONSE_CMD_SUCCESS_IU_TYPE:
		scsi_set_resid(scmd, 0);
		break;
	case SOP_RESPONSE_CMD_RESPONSE_IU_TYPE:
		scr = (struct sop_cmd_response *) r->response;
		scmd->result |= scr->status;
		sense_data_len = le16_to_cpu(scr->sense_data_len);
		response_data_len = le16_to_cpu(scr->response_data_len);
		if (unlikely(response_data_len && sense_data_len))
			dev_warn(&h->pdev->dev,
				"Both sense and response data not expected.\n");

		/* copy the sense data */
		if (sense_data_len) {
			if (SCSI_SENSE_BUFFERSIZE < sense_data_len)
				sense_data_len = SCSI_SENSE_BUFFERSIZE;
			memset(scmd->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
			memcpy(scmd->sense_buffer, scr->sense, sense_data_len);
		}

		/* paranoia, check for out of spec firmware */
		if (scr->data_in_xfer_result && scr->data_out_xfer_result)
			dev_warn(&h->pdev->dev,
				"Unexpected bidirectional cmd with status in and out\n");

		/* Calculate residual count */
		if (scr->data_in_xfer_result)
			data_xferred = le32_to_cpu(scr->data_in_xferred);
		else
			if (scr->data_out_xfer_result)
				data_xferred = le32_to_cpu(scr->data_out_xferred);
			else
				data_xferred = r->xfer_size;
		scsi_set_resid(scmd, r->xfer_size - data_xferred);

		if (response_data_len)
			main_io_path_decode_response_data(h, scr, scmd);
		break;
	case SOP_RESPONSE_TASK_MGMT_RESPONSE_IU_TYPE:
		scmd->result |= (DID_ERROR << 16);
		dev_warn(&h->pdev->dev, "got unhandled response type...\n");
		break;
	case SOP_RESPONSE_MANAGEMENT_RESPONSE_IU_TYPE:
		/* FIXME: how the heck are we even getting in here? */
		mr = (struct management_response_iu *) r->response;
		handle_management_response(h, mr, scmd);
		break;
	default:
		scmd->result |= (DID_ERROR << 16);
		dev_warn(&h->pdev->dev, "got UNKNOWN response type...\n");
		break;
	}
	scmd->scsi_done(scmd);
}

irqreturn_t sop_ioq_msix_handler(int irq, void *devid)
{
	u16 request_id, request_idx;
	u8 iu_type;
	u8 sq;
	int rc;
	struct queue_info *q = devid;
	struct sop_device *h = q->h;
#if 0
	printk(KERN_WARNING "=========> Got ioq interrupt, q = %p (%d) vector = %d\n",
			q, q->pqiq->queue_id, q->msix_vector);
#endif
	do {
		struct sop_request *r = q->pqiq->request;

		if (pqi_from_device_queue_is_empty(q->pqiq)) {
			/* dev_warn(&h->pdev->dev, "==== interrupt, ioq %d is empty ====\n",
					q->pqiq->queue_id); */
			break;
		}

		if (r == NULL) {
			/* Receiving completion of a new request */ 
			iu_type = pqi_peek_ui_type_from_device(q->pqiq);
			request_id = pqi_peek_request_id_from_device(q->pqiq);
			request_idx = request_id & h->qid_mask;
			sq = request_id >> h->qid_shift; /* find submit queue */
			r = q->pqiq->request =
				&h->qinfo[sq].request[request_idx];
			r->request_id = request_id;
			r->response_accumulated = 0;
		}
		rc = pqi_dequeue_from_device(q->pqiq,
				&r->response[r->response_accumulated]); 
		if (rc) { /* queue is empty */
			dev_warn(&h->pdev->dev, "=-=-=- io OQ %hhu is empty\n", q->pqiq->queue_id);
			return IRQ_HANDLED;
		}
		r->response_accumulated += q->pqiq->element_size;
		/* dev_warn(&h->pdev->dev, "accumulated %d bytes\n", r->response_accumulated); */
		if (sop_response_accumulated(r)) {
			/* dev_warn(&h->pdev->dev, "accumlated response\n"); */
			q->pqiq->request = NULL;
			wmb();
			WARN_ON((!r->waiting && !r->scmd));
			if (likely(r->scmd)) {
				complete_scsi_cmd(h, r);
				r = NULL;
			} else {
				if (likely(r->waiting)) {
					dev_warn(&h->pdev->dev, "Unexpected, waiting != NULL\n");
					complete(r->waiting);
					r = NULL;
				} else {
					dev_warn(&h->pdev->dev, "r->scmd and r->waiting both null\n");
				}
			}
			pqi_notify_device_queue_read(q->pqiq);
			atomic_dec(&h->max_outstanding_commands);
		}
	} while (1);

	return IRQ_HANDLED;
}

irqreturn_t sop_adminq_msix_handler(int irq, void *devid)
{
	struct queue_info *q = devid;
	u8 iu_type;
	u16 request_id, request_idx;
	int rc;
	struct sop_device *h = q->h;
	u8 sq;

	do {
		struct sop_request *r = h->admin_q_from_dev.request;

		if (pqi_from_device_queue_is_empty(&h->admin_q_from_dev))
			break;

		if (r == NULL) {
			/* Receiving completion of a new request */ 
			iu_type = pqi_peek_ui_type_from_device(&h->admin_q_from_dev);
			request_id = pqi_peek_request_id_from_device(&h->admin_q_from_dev);
			request_idx = request_id & request_id;
			sq = request_id >> h->qid_shift; /* find submit queue */
			r = h->admin_q_from_dev.request =
				&h->qinfo[sq].request[request_idx];
			r->response_accumulated = 0;
		}
		rc = pqi_dequeue_from_device(&h->admin_q_from_dev,
			&r->response[r->response_accumulated]); 
		if (rc) /* queue is empty */
			return IRQ_HANDLED;
		r->response_accumulated += h->admin_q_from_dev.element_size;
		if (sop_response_accumulated(r)) {
			h->admin_q_from_dev.request = NULL;
			wmb();
			complete(r->waiting);
			pqi_notify_device_queue_read(&h->admin_q_from_dev);
			atomic_dec(&h->max_outstanding_commands);
		}
	} while (1);

	return IRQ_HANDLED;
}

static void sop_irq_affinity_hints(struct sop_device *h)
{
	int i, cpu;

	cpu = cpumask_first(cpu_online_mask);
	for (i = 0; i < h->noqs; i++) {
		int idx = i ? i + 1 : i;
		int rc;
		rc = irq_set_affinity_hint(h->qinfo[idx].msix_vector,
					get_cpu_mask(cpu));

		if (rc)
			dev_warn(&h->pdev->dev, "Failed to hint affinity of vector %d to cpu %d\n",
					h->qinfo[idx].msix_vector, cpu);
		cpu = cpumask_next(cpu, cpu_online_mask);
	}
}

static int sop_request_irqs(struct sop_device *h,
					irq_handler_t msix_adminq_handler,
					irq_handler_t msix_ioq_handler)
{
	u8 i, j;
	int rc;

	sop_irq_affinity_hints(h);

	rc = request_irq(h->qinfo[0].msix_vector, msix_adminq_handler, 0,
					h->devname, &h->qinfo[0]);
	if (rc != 0) {
		dev_warn(&h->pdev->dev, "request_irq failed, i = %d\n", 0);
	}
		

	/* for loop starts at 2 to skip over the admin queues */
	for (i = 2; i < h->noqs + 1; i++) {
		rc = request_irq(h->qinfo[i].msix_vector, msix_ioq_handler, 0,
					h->devname, &h->qinfo[i]);
		if (rc != 0) {
			dev_warn(&h->pdev->dev,
					"request_irq failed, i = %d\n", i);
			goto freeirqs;
		}
	}
	return 0;

freeirqs:
	free_irq(h->qinfo[0].msix_vector, &h->qinfo[0]);
	for (j = 2; j < i; j++)
		free_irq(h->qinfo[j].msix_vector, &h->qinfo[j]);
	dev_warn(&h->pdev->dev, "intx mode not implemented.\n");
	return -1;
}

static void sop_free_irqs(struct sop_device *h)
{
	int i;

	for (i = 0; i < h->noqs; i++) {
		int idx, vector;

		idx = i ? i + 1 : 0;
		vector = h->qinfo[idx].msix_vector; 
		irq_set_affinity_hint(vector, NULL);
		free_irq(vector, &h->qinfo[idx]);
	}
}

static void sop_free_irqs_and_disable_msix(
		struct sop_device *h)
{
	sop_free_irqs(h);
#ifdef CONFIG_PCI_MSI
	if (h->intr_mode == INTR_MODE_MSIX && h->pdev->msix_enabled)
		pci_disable_msix(h->pdev);
#endif /* CONFIG_PCI_MSI */
}

/* FIXME: maybe there's a better way to do this */
static int alloc_request(struct sop_device *h, u8 q)
{
	int rc;
	unsigned long flags;

	BUG_ON(h->qinfo[q].qdepth > h->elements_per_io_queue);
	BUG_ON(q > 127); /* high bit reserved for error reporting */

	spin_lock_irqsave(&h->qinfo[q].qlock, flags);
        do {
                rc = (u16) find_first_zero_bit(h->qinfo[q].request_bits,
						h->qinfo[q].qdepth);
                if (rc >= h->qinfo[q].qdepth) {
			spin_unlock_irqrestore(&h->qinfo[q].qlock, flags);
			dev_warn(&h->pdev->dev, "alloc_request failed.\n");
			return -EBUSY;
		}
        } while (test_and_set_bit((int) rc, h->qinfo[q].request_bits));
	spin_unlock_irqrestore(&h->qinfo[q].qlock, flags);
	return rc | (((int) q) << h->qid_shift);
}

static void free_request(struct sop_device *h, u8 q, u16 request_id)
{
	unsigned long flags;

	BUG_ON((request_id >> h->qid_shift) != q);
	spin_lock_irqsave(&h->qinfo[q].qlock, flags);
	clear_bit(request_id & h->qid_mask, h->qinfo[q].request_bits);
	spin_unlock_irqrestore(&h->qinfo[q].qlock, flags);
}

static void fill_create_io_queue_request(struct sop_device *h,
	struct pqi_create_operational_queue_request *r,
	struct pqi_device_queue *q, int to_device, u16 request_id,
	u16 msix_vector)
{
	u8 function_code;

	if (to_device)
		function_code = CREATE_QUEUE_TO_DEVICE;
	else 
		function_code = CREATE_QUEUE_FROM_DEVICE;

	memset(r, 0, sizeof(*r));
	r->iu_type = OPERATIONAL_QUEUE_IU_TYPE;
	r->iu_length = cpu_to_le16(0x003c);
	r->response_oq = 0;
	r->request_id = request_id;
	r->function_code = function_code;
	r->queue_id = cpu_to_le16(q->queue_id);
	r->element_array_addr = cpu_to_le64(q->dhandle);
	r->index_addr = cpu_to_le64(q->dhandle +
			q->nelements * q->element_size);
	r->nelements = cpu_to_le16((u16) q->nelements);
	r->element_length = cpu_to_le16((u16) (q->element_size/16));
	if (to_device) {
		r->iqp.operational_queue_protocol = 0;
	} else {
		r->oqp.interrupt_message_number = cpu_to_le16(msix_vector);
		/* Coalascing is not supported yet */
		r->oqp.operational_queue_protocol = 0;
	}
}

static void fill_delete_io_queue_request(struct sop_device *h,
	struct pqi_delete_operational_queue_request *r, u16 queue_id,
	int to_device, u16 request_id)
{
	u8 function_code;

	if (to_device)
		function_code = DELETE_QUEUE_TO_DEVICE;
	else 
		function_code = DELETE_QUEUE_FROM_DEVICE;

	memset(r, 0, sizeof(*r));
	r->iu_type = OPERATIONAL_QUEUE_IU_TYPE;
	r->iu_length = cpu_to_le16(0x003c);
	r->request_id = request_id;
	r->function_code = function_code;
	r->queue_id = cpu_to_le16(queue_id);
}

static void send_admin_command(struct sop_device *h, u16 request_id)
{
	struct sop_request *request;
	struct pqi_device_queue *aq = &h->admin_q_to_dev;
	DECLARE_COMPLETION_ONSTACK(wait);
	u16 request_idx = request_id & h->qid_mask;

	request = &h->qinfo[aq->queue_id].request[request_idx];
	request->waiting = &wait;
	request->response_accumulated = 0;
	pqi_notify_device_queue_written(h, aq);
	wait_for_completion(&wait);
}

static void send_sop_command(struct sop_device *h, struct queue_info *submitq,
				u16 request_id)
{
	struct sop_request *sopr;
	DECLARE_COMPLETION_ONSTACK(wait);

	sopr = &submitq->request[request_id & h->qid_mask];
	memset(sopr, 0, sizeof(*sopr));
	sopr->request_id = request_id;
	sopr->waiting = &wait;
	sopr->response_accumulated = 0;
	pqi_notify_device_queue_written(h, submitq->pqiq);
	put_cpu();
	wait_for_completion(&wait);
}

static int fill_get_pqi_device_capabilities(struct sop_device *h,
			struct report_pqi_device_capability_iu *r,
			u16 request_id, void *buffer, u32 buffersize)
{
	u64 busaddr;
	struct pqi_device_queue *response_queue = &h->admin_q_from_dev;

	memset(r, 0, sizeof(*r));
	r->iu_type = REPORT_PQI_DEVICE_CAPABILITY;
	r->compatible_features = 0;
	r->iu_length = cpu_to_le16(0x003C);
	r->response_oq = cpu_to_le16(response_queue->queue_id);
	r->work_area = 0;
	r->request_id = request_id;
	r->function_code = 0;
	r->buffer_size = cpu_to_le32(buffersize);

        busaddr = pci_map_single(h->pdev, buffer, buffersize,
                                PCI_DMA_FROMDEVICE);
	r->sg.address = cpu_to_le64(busaddr);
	r->sg.length = cpu_to_le32(buffersize);
	r->sg.descriptor_type = PQI_SGL_DATA_BLOCK;
	if (!busaddr) {
		memset(r, 0, 4); /* NULL IU */
		return -1;
	}
	return 0;
}

static int sop_get_pqi_device_capabilities(struct sop_device *h)
{
	struct report_pqi_device_capability_iu *r;
	volatile struct report_pqi_device_capability_response *resp;
	struct pqi_device_queue *aq = &h->admin_q_to_dev;
	struct pqi_device_capabilities *buffer;
	int request_id;
	u16 request_idx;
	u64 busaddr;

	h->elements_per_io_queue = DRIVER_MAX_IQ_NELEMENTS;
	dev_warn(&h->pdev->dev, "Getting pqi device capabilities\n");
	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	dev_warn(&h->pdev->dev, "Getting pqi device capabilities 2\n");
	r = pqi_alloc_elements(aq, 1);
	request_id = alloc_request(h, aq->queue_id);
	request_idx = request_id & h->qid_mask;
	if (fill_get_pqi_device_capabilities(h, r, request_id, buffer,
						(u32) sizeof(*buffer))) {
		/* we have to submit request (already in queue) but it
		 * is now a NULL IU, and will be ignored by hardware.
		 */
		dev_warn(&h->pdev->dev,
			"pci_map_single failed in fill_get_pqi_device_capabilities\n");
		dev_warn(&h->pdev->dev,
			"Sending NULL IU, this code is untested.\n");
		free_request(h, aq->queue_id, request_id);
		pqi_notify_device_queue_written(h, aq);
		goto error;
	}
	dev_warn(&h->pdev->dev, "Getting pqi device capabilities 3\n");
	send_admin_command(h, request_id);
	dev_warn(&h->pdev->dev, "Getting pqi device capabilities 4\n");
	busaddr = le64_to_cpu(r->sg.address);
	pci_unmap_single(h->pdev, busaddr, sizeof(*buffer),
						PCI_DMA_FROMDEVICE);
	dev_warn(&h->pdev->dev, "Getting pqi device capabilities 5\n");
	resp = (volatile struct report_pqi_device_capability_response *)
			h->qinfo[aq->queue_id].request[request_idx].response;
	if (resp->status != 0) {
		free_request(h, aq->queue_id, request_id);
		goto error;
	}
	free_request(h, aq->queue_id, request_id);
	dev_warn(&h->pdev->dev, "Getting pqi device capabilities 6\n");
	h->max_iqs = le16_to_cpu(buffer->max_iqs);
	h->max_iq_elements = le16_to_cpu(buffer->max_iq_elements);
	h->max_iq_element_length = le16_to_cpu(buffer->max_iq_element_length);
	h->min_iq_element_length = le16_to_cpu(buffer->min_iq_element_length);
	h->max_oqs = le16_to_cpu(buffer->max_oqs);
	h->max_oq_elements = le16_to_cpu(buffer->max_oq_elements);
	h->max_oq_element_length = le16_to_cpu(buffer->max_oq_element_length);
	h->min_oq_element_length = le16_to_cpu(buffer->min_oq_element_length);
	h->intr_coalescing_time_granularity =
		le16_to_cpu( buffer->intr_coalescing_time_granularity);
	h->iq_alignment_exponent = buffer->iq_alignment_exponent;
	h->oq_alignment_exponent = buffer->oq_alignment_exponent;
	h->iq_ci_alignment_exponent = buffer->iq_ci_alignment_exponent;
	h->oq_pi_alignment_exponent = buffer->oq_pi_alignment_exponent;
	h->protocol_support_bitmask =
		le32_to_cpu(buffer->protocol_support_bitmask);
	h->admin_sgl_support_bitmask =
		le16_to_cpu(buffer->admin_sgl_support_bitmask);

	dev_warn(&h->pdev->dev, "Getting pqi device capabilities 7:\n");

	dev_warn(&h->pdev->dev, "max iqs = %hu\n", h->max_iqs);
	dev_warn(&h->pdev->dev, "max iq_elements = %hu\n", h->max_iq_elements);
	dev_warn(&h->pdev->dev, "max iq_element_length = %hu\n", h->max_iq_element_length);
	dev_warn(&h->pdev->dev, "min iq_element_length = %hu\n", h->min_iq_element_length);
	dev_warn(&h->pdev->dev, "max oqs = %hu\n", h->max_oqs);
	dev_warn(&h->pdev->dev, "max oq_elements = %hu\n", h->max_oq_elements);
	dev_warn(&h->pdev->dev, "max oq_element_length = %hu\n", h->max_oq_element_length);
	dev_warn(&h->pdev->dev, "min oq_element_length = %hu\n", h->min_oq_element_length);
	dev_warn(&h->pdev->dev, "intr_coalescing_time_granularity = %hu\n", h->intr_coalescing_time_granularity);
	dev_warn(&h->pdev->dev, "iq_alignment_exponent = %hhu\n", h->iq_alignment_exponent);
	dev_warn(&h->pdev->dev, "oq_alignment_exponent = %hhu\n", h->oq_alignment_exponent);
	dev_warn(&h->pdev->dev, "iq_ci_alignment_exponent = %hhu\n", h->iq_ci_alignment_exponent);
	dev_warn(&h->pdev->dev, "oq_pi_alignment_exponent = %hhu\n", h->oq_pi_alignment_exponent);
	dev_warn(&h->pdev->dev, "protocol support bitmask = 0x%08x\n", h->protocol_support_bitmask);
	dev_warn(&h->pdev->dev, "admin_sgl_support_bitmask = 0x%04x\n", h->admin_sgl_support_bitmask);

	h->elements_per_io_queue = DRIVER_MAX_IQ_NELEMENTS;
	if (h->elements_per_io_queue > DRIVER_MAX_OQ_NELEMENTS)
		h->elements_per_io_queue = DRIVER_MAX_OQ_NELEMENTS;
	if (h->elements_per_io_queue > h->max_oq_elements)
		h->elements_per_io_queue = h->max_oq_elements;
	if (h->elements_per_io_queue > h->max_iq_elements)
		h->elements_per_io_queue = h->max_iq_elements;

	dev_warn(&h->pdev->dev, "elements per i/o queue: %d\n",
			h->elements_per_io_queue);

	kfree(buffer);
	return 0;

error:
	kfree(buffer);
	return -1;
}

static int sop_setup_io_queues(struct sop_device *h)
{

	int i, niqs, noqs;
	struct pqi_create_operational_queue_request *r;
	struct pqi_device_queue *aq = &h->admin_q_to_dev;
	int request_id;
	u16 request_idx;

	/* The queue ids == the msix vector index + 1 for OQs. */
	noqs = pqi_device_queue_array_alloc(h, &h->io_q_from_dev,
			h->noqs - 1, h->elements_per_io_queue,
			OQ_IU_SIZE / 16, PQI_DIR_FROM_DEVICE, 2);
	if (h->noqs - 1 < 0)
		goto bail_out;
	if (h->noqs - 1 != noqs) {
		dev_warn(&h->pdev->dev, "Didn't get all the oqs I wanted\n");
		goto bail_out;
	}

	niqs = pqi_device_queue_array_alloc(h, &h->io_q_to_dev,
			h->niqs - 1 , h->elements_per_io_queue,
			IQ_IU_SIZE / 16, PQI_DIR_TO_DEVICE, 2 + noqs);
	if (niqs < 0)
		goto bail_out;
	if (h->niqs - 1 != niqs) {
		dev_warn(&h->pdev->dev, "Didn't get all the iqs I wanted\n");
		goto bail_out;
	}

	for (i = 0; i < h->noqs - 1; i++) {
		struct pqi_device_queue *q;
		volatile struct pqi_create_operational_queue_response *resp;

		/* Set up i/o queue #i from device */

		q = &h->io_q_from_dev[i];
		spin_lock_init(&q->index_lock);
		h->qinfo[h->io_q_from_dev[i].queue_id].pqiq = q;
		r = pqi_alloc_elements(aq, 1);
		request_id = alloc_request(h, aq->queue_id);
		request_idx = request_id & h->qid_mask;
		if (request_id < 0) { /* FIXME: now what? */
			dev_warn(&h->pdev->dev, "requests unexpectedly exhausted\n");
		}
		fill_create_io_queue_request(h, r, q, 0, request_id, q->queue_id - 1);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_create_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_idx].response;
		if (resp->status != 0)
			dev_warn(&h->pdev->dev, "Failed to set up OQ... now what?\n");
		/* h->io_q_from_dev[i].pi = h->io_q_from_dev[i].queue_vaddr +
						le64_to_cpu(resp->index_offset); */
		h->io_q_from_dev[i].ci = ((void *) h->pqireg) +
						le64_to_cpu(resp->index_offset);
		free_request(h, aq->queue_id, request_id);
	}

	for (i = 0; i < h->niqs - 1; i++) {
		struct pqi_device_queue *q;
		volatile struct pqi_create_operational_queue_response *resp;
		
		/* Set up i/o queue #i to device */
		r = pqi_alloc_elements(aq, 1);
		q = &h->io_q_to_dev[i];
		spin_lock_init(&q->index_lock);
		h->qinfo[h->io_q_to_dev[i].queue_id].pqiq = q;
		request_id = alloc_request(h, aq->queue_id);
		request_idx = request_id & h->qid_mask;
		if (request_id < 0) { /* FIXME: now what? */
			dev_warn(&h->pdev->dev, "requests unexpectedly exhausted 2\n");
		}
		fill_create_io_queue_request(h, r, q, 1, request_id, (u16) -1);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_create_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_idx].response;
		if (resp->status != 0)
			dev_warn(&h->pdev->dev, "Failed to set up IQ... now what?\n");
		/* h->io_q_to_dev[i].ci = h->io_q_to_dev[i].queue_vaddr +
						le64_to_cpu(resp->index_offset); */
		h->io_q_to_dev[i].pi = ((void *) h->pqireg) +
					le64_to_cpu(resp->index_offset);
		free_request(h, aq->queue_id, request_id);
	}

	return 0;

bail_out:
	return -1;
}

static void sop_free_io_queues(struct sop_device *h)
{
	size_t total_size, n_q_elements, element_size;
	int remainder;
	if (h->iq_vaddr) {
		n_q_elements = h->io_q_to_dev[0].nelements;
		element_size = h->io_q_to_dev[0].element_size;
		total_size = n_q_elements * element_size + sizeof(u64);
		remainder = total_size % 64;
		total_size += remainder ? 64 - remainder : 0;
		pci_free_consistent(h->pdev, total_size,
				h->iq_vaddr, h->iq_dhandle);
		h->iq_vaddr = NULL;
		h->iq_dhandle = 0;
	}
	if (h->oq_vaddr) {
		n_q_elements = h->io_q_from_dev[0].nelements;
		element_size = h->io_q_from_dev[0].element_size;
		total_size = n_q_elements * element_size + sizeof(u64);
		remainder = total_size % 64;
		total_size += remainder ? 64 - remainder : 0;
		pci_free_consistent(h->pdev, total_size,
				h->oq_vaddr, h->oq_dhandle);
		h->oq_vaddr = NULL;
		h->oq_dhandle = 0;
	}
}

static int sop_delete_io_queues(struct sop_device *h)
{
	int i;
	struct pqi_delete_operational_queue_request *r;
	struct pqi_device_queue *aq = &h->admin_q_to_dev;
	int request_id;
	u16 request_idx;

	for (i = 0; i < h->noqs - 1; i++) {
		volatile struct pqi_delete_operational_queue_response *resp;
		u16 qid;

		r = pqi_alloc_elements(aq, 1);
		request_id = alloc_request(h, aq->queue_id);
		if (request_id < 0) { /* FIXME: now what? */
			dev_warn(&h->pdev->dev, "requests unexpectedly exhausted\n");
		}
		request_idx = request_id & h->qid_mask;
		qid = h->io_q_from_dev[i].queue_id;
		fill_delete_io_queue_request(h, r, qid, 1, request_id);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_delete_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_idx].response;
		if (resp->status != 0)
			dev_warn(&h->pdev->dev, "Failed to tear down OQ... now what?\n");
		free_request(h, aq->queue_id, request_id);
	}
	for (i = 0; i < h->niqs - 1; i++) {
		volatile struct pqi_delete_operational_queue_response *resp;
		u16 qid;

		r = pqi_alloc_elements(aq, 1);
		request_id = alloc_request(h, aq->queue_id);
		if (request_id < 0) { /* FIXME: now what? */
			dev_warn(&h->pdev->dev, "requests unexpectedly exhausted\n");
		}
		request_idx = request_id & h->qid_mask;
		qid = h->io_q_to_dev[i].queue_id;
		fill_delete_io_queue_request(h, r, qid, 0, request_id);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_delete_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_idx].response;
		if (resp->status != 0)
			dev_warn(&h->pdev->dev, "Failed to tear down IQ... now what?\n");
		free_request(h, aq->queue_id, request_id);
	}
	sop_free_io_queues(h);
	return 0;
}

static int sop_set_dma_mask(struct pci_dev * pdev)
{
	int rc;

	if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(64)) &&
		!pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64)))
		return 0;
	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (!rc)
		rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	return rc;
}

static int sop_register_host(struct sop_device *h)
{
	struct Scsi_Host *sh;
	int rc;

	sh = scsi_host_alloc(&sop_template, sizeof(h));
	if (!sh)
		goto bail;
	sh->io_port = 0;
	sh->n_io_port = 0;
	sh->this_id = -1;
	sh->max_channel = 1;
	sh->max_cmd_len = MAX_COMMAND_SIZE;
	sh->max_lun = 1; /* FIXME are these correct? */
	sh->max_id = 1;
	sh->can_queue = h->elements_per_io_queue;
	sh->cmd_per_lun = h->elements_per_io_queue;
	sh->sg_tablesize = MAX_SGLS; /* FIXME make this bigger */
	sh->hostdata[0] = (unsigned long) h;
	sh->irq = h->qinfo[0].msix_vector;
	sh->unique_id = sh->irq; /* really? */
	h->scsi_host = sh;
	rc = scsi_add_host(sh, &h->pdev->dev);
	if (rc)
		goto add_host_failed;
	scsi_scan_host(sh);
	return 0;

add_host_failed:
	dev_err(&h->pdev->dev, "scsi_add_host failed.\n");
	scsi_host_put(sh);
	return rc;
bail:
	dev_err(&h->pdev->dev, "scsi_host_alloc failed.\n");
	return -ENOMEM;
}

/*
 * sop_figure_request_id_encoding figures how many bits of the
 * request ID to use for queue ID and how many for request ID.
 *
 * The Requst ID field of all the IUs is 2 bytes.  Part of this
 * must include the queue ID, so we know on completion which queue
 * the associated request resides with, and to keep request IDs unique
 * across all queues.   Depending on how many queues there are, the
 * different numbers of bits are used for the queue ID and the request ID.
 * With 256 queues, maximum queue depth is 256, with 128 queues,
 * max queue depth is 512, with 64 queues, max qdepth is 1024, etc.
 *
 * The device may have further limitations on queue depth besides what
 * is imposed by the driver design here.
 */
static void sop_figure_request_id_encoding(struct sop_device *h)
{
	int i;

	BUG_ON(h->noqs > 256 || h->noqs < 2);

	for (i = 7; i >= 0; i--) {
		if (h->noqs <= (1 << i))
			continue;
		h->qid_shift = 16 - (i + 2);
		h->qid_mask = (1 << h->qid_shift) - 1;
		return;
	}
}

#define PQI_RESET_ACTION_SHIFT 5
#define PQI_RESET_ACTION_MASK (0x07 << PQI_RESET_ACTION_SHIFT)
#define PQI_START_RESET (1 << PQI_RESET_ACTION_SHIFT)
#define PQI_SOFT_RESET (1)
#define PQI_START_RESET_COMPLETED (2 << PQI_RESET_ACTION_SHIFT)
static int sop_init_time_host_reset(struct sop_device *h)
{
	u64 paf;
	unsigned char *x;
	u32 status, reset_register, prev;
	u8 function_and_status;
	u8 pqi_device_state;
	__iomem void *sig = &h->pqireg->signature;

	prev = -1;
	dev_warn(&h->pdev->dev, "Resetting host\n");
	writel(PQI_START_RESET | PQI_SOFT_RESET, &h->pqireg->reset);
	do {
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
		/* 
		 * Not using safe_readl here because while in reset we can
		 * get -1 and be unable to read the signature, and this
		 * is normal (I think).
		 */
		reset_register = readl(&h->pqireg->reset);
		if (reset_register != prev)
			dev_warn(&h->pdev->dev, "Reset register is: 0x%08x\n",
				reset_register);
		prev = reset_register;
	} while ((reset_register & PQI_RESET_ACTION_MASK) !=
					PQI_START_RESET_COMPLETED);

	dev_warn(&h->pdev->dev, "Host reset initiated.\n");
	do {
		if (safe_readq(sig, &paf, &h->pqireg->process_admin_function)) {
			dev_warn(&h->pdev->dev,
				"Unable to read process admin function register");
			return -1;
		}
		if (safe_readl(sig, &status, &h->pqireg->pqi_device_status)) {
			dev_warn(&h->pdev->dev, "Unable to read from device memory");
			return -1;
		}
		x = (unsigned char *) &status;
		function_and_status = paf & 0xff;
		pqi_device_state = status & 0xff;
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
	} while (pqi_device_state != PQI_READY_FOR_ADMIN_FUNCTION ||
			function_and_status != PQI_IDLE);
	dev_warn(&h->pdev->dev, "Host reset completed.\n");
	return 0;
}

static int __devinit sop_probe(struct pci_dev *pdev,
			const struct pci_device_id *pci_id)
{
	struct sop_device *h;
	u64 signature;
	int i, rc;
	__iomem void *sig;

	dev_warn(&pdev->dev, SOP "found device: %04x:%04x/%04x:%04x\n",
			pdev->vendor, pdev->device,
			pdev->subsystem_vendor, pdev->subsystem_device);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	atomic_set(&h->max_outstanding_commands, 0);
	h->ctlr = controller_num;
	for (i = 0; i < MAX_TOTAL_QUEUES; i++) {
		spin_lock_init(&h->qinfo[i].qlock);
		h->qinfo[i].h = h;
	}
	controller_num++;
	sprintf(h->devname, "sop-%d\n", h->ctlr);

	h->pdev = pdev;
	pci_set_drvdata(pdev, h);

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_warn(&h->pdev->dev, "unable to enable PCI device\n");
		return rc;
	}

	/* Enable bus mastering (pci_disable_device may disable this) */
	pci_set_master(h->pdev);

	rc = pci_request_regions(h->pdev, SOP);
	if (rc) {
		dev_err(&h->pdev->dev,
			"cannot obtain PCI resources, aborting\n");
		return rc;
	}

	h->pqireg = pci_ioremap_bar(pdev, 0);
	if (!h->pqireg) {
		rc = -ENOMEM;
		goto bail;
	}
	if (reset_devices) {
		rc = sop_init_time_host_reset(h);
		if (rc)
			return -1;
	}
	sig = &h->pqireg->signature;

	if (sop_set_dma_mask(pdev)) {
		dev_err(&pdev->dev, "failed to set DMA mask\n");
		goto bail;
	}

	if (safe_readq(sig, &signature, &h->pqireg->signature)) {
		dev_warn(&pdev->dev, "Unable to read PQI signature\n");
		goto bail;
	}
	if (memcmp("PQI DREG", &signature, sizeof(signature)) != 0) {
		dev_warn(&pdev->dev, "device does not appear to be a PQI device\n");
		goto bail;
	}

	rc = sop_setup_msix(h);
	if (rc != 0)
		goto bail;

	sop_figure_request_id_encoding(h);

	rc = sop_create_admin_queues(h);
	if (rc)
		goto bail;

	rc = sop_request_irqs(h, sop_adminq_msix_handler,
					sop_ioq_msix_handler);
	if (rc != 0)
		goto bail;

	rc = sop_get_pqi_device_capabilities(h);
	if (rc)
		goto bail;

	rc = sop_setup_io_queues(h);
	if (rc)
		goto bail;
	rc = sop_register_host(h);
	if (rc)
		goto bail;

	return 0;
bail:
	if (h && h->pqireg)
		iounmap(h->pqireg);
	kfree(h);
	return -1;
}

static int sop_suspend(__attribute__((unused)) struct pci_dev *pdev,
				__attribute__((unused)) pm_message_t state)
{
	return -ENOSYS;
}

static int sop_resume(__attribute__((unused)) struct pci_dev *pdev)
{
	return -ENOSYS;
}

static void __devexit sop_remove(struct pci_dev *pdev)
{
	struct sop_device *h;

	h = pci_get_drvdata(pdev);
        scsi_remove_host(h->scsi_host);
        scsi_host_put(h->scsi_host);
        h->scsi_host = NULL;
	sop_delete_io_queues(h);
	sop_free_irqs_and_disable_msix(h);
	sop_delete_admin_queues(h);
	if (h && h->pqireg)
		iounmap(h->pqireg);
	pci_disable_device(pdev);
	pci_release_regions(pdev);
	pci_set_drvdata(pdev, NULL);
	kfree(h);
}

static void sop_shutdown(struct pci_dev *pdev)
{
	dev_warn(&pdev->dev, "shutdown called.\n");
}

static struct pci_driver sop_pci_driver = {
	.name = SOP,
	.probe = sop_probe,
	.remove = __devexit_p(sop_remove),
	.id_table = sop_id_table,
	.shutdown = sop_shutdown,
	.suspend = sop_suspend,
	.resume = sop_resume,
	.err_handler = &sop_pci_error_handlers,
};

static int __init sop_init(void)
{
	return pci_register_driver(&sop_pci_driver);
}

static void __exit sop_exit(void)
{
	pci_unregister_driver(&sop_pci_driver);
}

static inline struct sop_device *sdev_to_hba(struct scsi_device *sdev)
{
	unsigned long *priv = shost_priv(sdev->host);
	return (struct sop_device *) *priv;
}

static struct queue_info *find_submission_queue(struct sop_device *h, int cpu)
{
	int q = h->noqs + 1 + (cpu % (h->niqs - 1));
	return &h->qinfo[q];
}

static struct queue_info *find_reply_queue(struct sop_device *h, int cpu)
{
	int q = 2 + (cpu % (h->noqs - 1));
	return &h->qinfo[q];
}


static void fill_sg_data_element(struct pqi_sgl_descriptor *sgld,
				struct scatterlist *sg, u32 *xfer_size)
{
	sgld->address = cpu_to_le64(sg_dma_address(sg));
	sgld->length = cpu_to_le32(sg_dma_len(sg));
	*xfer_size += sg_dma_len(sg);
	sgld->descriptor_type = PQI_SGL_DATA_BLOCK;
}

static void fill_sg_chain_element(struct pqi_sgl_descriptor *sgld,
				struct queue_info *q,
				int sg_block_number, int sg_count)
{
	sgld->address = cpu_to_le64(q->sg_bus_addr + sg_block_number *
				sizeof(*sgld));
	sgld->length = cpu_to_le32(sg_count * sizeof(*sgld));
	sgld->descriptor_type = PQI_SGL_STANDARD_LAST_SEG;
}

static void fill_inline_sg_list(struct sop_limited_cmd_iu *r,
				struct scsi_cmnd *sc, int use_sg,
				u32 *xfer_size)
{
	struct pqi_sgl_descriptor *datasg;
	struct scatterlist *sg;
	int i;
	static const u16 no_sgl_size =
			(u16) (sizeof(*r) - sizeof(r->sg[0]) * 2) - 4;
	BUILD_BUG_ON(sizeof(*r) != 64);
	BUILD_BUG_ON((sizeof(r->sg[0]) != (16)));
	BUILD_BUG_ON((sizeof(*r) - sizeof(r->sg[0]) * 2) != 32);

	BUG_ON(use_sg > 2);
	if (!use_sg) {
		r->iu_length = cpu_to_le16(no_sgl_size);
		return;
	}
	r->iu_length = cpu_to_le16(no_sgl_size + sizeof(*datasg) * use_sg);
	*xfer_size = 0;
	datasg = &r->sg[0];
	scsi_for_each_sg(sc, sg, use_sg, i) {
		fill_sg_data_element(datasg, sg, xfer_size);
		datasg++;
	}
}

static int sop_scatter_gather(struct sop_device *h,
			struct queue_info *q, 
			struct sop_limited_cmd_iu *r,
			struct scsi_cmnd *sc, u32 *xfer_size)
{
	struct scatterlist *sg;
	int sg_block_number;
	int i, j, use_sg;
	struct pqi_sgl_descriptor *datasg;
	static const u16 no_sgl_size =
			(u16) (sizeof(*r) - sizeof(r->sg[0]) * 2) - 4;

	BUG_ON(scsi_sg_count(sc) > MAX_SGLS);

	use_sg = scsi_dma_map(sc);
	if (use_sg < 0)
		return use_sg;

	if (use_sg < 3) {
		fill_inline_sg_list(r, sc, use_sg, xfer_size);
		return 0;
	}

	sg_block_number = (r->request_id & h->qid_mask) * MAX_SGLS;
	*xfer_size = 0;
	r->iu_length = cpu_to_le16(no_sgl_size + sizeof(*datasg) * 2);
	datasg = &r->sg[0];
	j = 0;
	scsi_for_each_sg(sc, sg, use_sg, i) {
		if (j == 1) {
			fill_sg_chain_element(datasg, q,
					sg_block_number, scsi_sg_count(sc) - 1);
			datasg = &q->sg[sg_block_number];
			j++;
		}
		fill_sg_data_element(datasg, sg, xfer_size);
		datasg++;
		j++;
	}
	return 0;
}

static int sop_queuecommand(struct Scsi_Host *shost, struct scsi_cmnd *sc)
{
	struct sop_device *h;
	/* struct scsi_device *sdev = sc->device; */
	struct queue_info *submitq, *replyq;
	struct sop_limited_cmd_iu *r;
	struct sop_request *sopr;
	int request_id;
	int cpu;

	h = sdev_to_hba(sc->device);

	/* reject io to devices other than b0t0l0 */
	if (sc->device->channel != 0 || sc->device->id != 0 || 
		sc->device->lun != 0) {
                sc->result = DID_NO_CONNECT << 16;
                sc->scsi_done(sc);
                return 0;
	}

	cpu = get_cpu();
	submitq = find_submission_queue(h, cpu);
	replyq = find_reply_queue(h, cpu);
	if (!submitq)
		dev_warn(&h->pdev->dev, "queuecommand: q is null!\n");
	if (!submitq->pqiq)
		dev_warn(&h->pdev->dev, "queuecommand: q->pqiq is null!\n");
	r = pqi_alloc_elements(submitq->pqiq, 1);
	if (IS_ERR(r)) {
		dev_warn(&h->pdev->dev, "pqi_alloc_elements returned %ld\n", PTR_ERR(r));
	}
	request_id = alloc_request(h, submitq->pqiq->queue_id);
	if (request_id < 0)
		dev_warn(&h->pdev->dev, "Failed to allocate request! Trouble ahead.\n");

	r->iu_type = SOP_LIMITED_CMD_IU;
	r->compatible_features = 0;
	r->queue_id = cpu_to_le16(replyq->pqiq->queue_id);
	r->work_area = 0;
	r->request_id = request_id;
	sopr = &submitq->request[request_id & h->qid_mask];
	sopr->xfer_size = 0;
	sopr->scmd = sc;
	sc->host_scribble = (unsigned char *) sopr;
	sopr->waiting = NULL;

	switch (sc->sc_data_direction) {
	case DMA_TO_DEVICE:
		r->flags = SOP_DATA_DIR_TO_DEVICE;
		break;
	case DMA_FROM_DEVICE:
		r->flags = SOP_DATA_DIR_FROM_DEVICE;
		break;
	case DMA_NONE:
		r->flags = SOP_DATA_DIR_NONE;
		break;
	case DMA_BIDIRECTIONAL:
		r->flags = SOP_DATA_DIR_RESERVED;
		break;
	}
	memset(r->cdb, 0, 16);
	memcpy(r->cdb, sc->cmnd, sc->cmd_len);
	if (sop_scatter_gather(h, submitq, r, sc, &sopr->xfer_size)) {
		/*
		 * Scatter gather mapping failed.  Tell midlayer to back off.
		 * There's no "unallocating" from the submit ring buffer,
		 * so just make it a null IU and deallocate the corresponding
		 * request.
		 */
		memset(r, 0, 4); /* NULL IU */
		free_request(h, submitq->pqiq->queue_id, request_id);
		pqi_notify_device_queue_written(h, submitq->pqiq);
		return SCSI_MLQUEUE_HOST_BUSY;
	}
	r->xfer_size = cpu_to_le32(sopr->xfer_size);
	pqi_notify_device_queue_written(h, submitq->pqiq);
	put_cpu();
	return 0;
}

static int sop_change_queue_depth(struct scsi_device *sdev,
        int qdepth, int reason)
{
	struct sop_device *h = sdev_to_hba(sdev);

	dev_warn(&h->pdev->dev, "sop_change_queue_depth called but not implemented\n");
	return 0;
}

static void fill_task_mgmt_request(struct sop_task_mgmt_iu *tm,
		struct queue_info *replyq, u16 request_id,
		u16 request_id_to_manage, u8 task_mgmt_function)
{
	memset(tm, 0, sizeof(*tm));
	tm->iu_type = SOP_TASK_MGMT_IU;
	tm->iu_length = cpu_to_le16(0x001C);
	tm->queue_id = cpu_to_le16(replyq->pqiq->queue_id);
	tm->request_id = request_id;
	tm->nexus_id = 0;
	tm->lun = 0;
	tm->request_id_to_manage = request_id_to_manage;
	tm->task_mgmt_function = task_mgmt_function;
}

static int process_task_mgmt_response(struct sop_device *h,
			struct queue_info *submitq, u16 request_id)
{
	struct sop_request *sopr = &submitq->request[request_id & h->qid_mask];
	struct sop_task_mgmt_response *tmr =
		(struct sop_task_mgmt_response *) sopr->response;
	u8 response_code;

	if (tmr->iu_type != SOP_RESPONSE_TASK_MGMT_RESPONSE_IU_TYPE)
		dev_warn(&h->pdev->dev, "Unexpected IU type %hhu in %s\n",
				tmr->iu_type, __func__);
	response_code = tmr->response_code;
	free_request(h, submitq->pqiq->queue_id, request_id);
	switch (response_code) {
	case SOP_TMF_COMPLETE:
	case SOP_TMF_SUCCEEDED:
	case SOP_TMF_REJECTED:
		return SUCCESS;
	}
	return FAILED;
}

static int sop_abort_handler(struct scsi_cmnd *sc)
{
	struct sop_device *h;
	struct sop_request *sopr_to_abort =
			(struct sop_request *) sc->host_scribble;
	struct queue_info *submitq, *replyq;
	struct sop_task_mgmt_iu *abort_cmd;
	int request_id, cpu;

	h = sdev_to_hba(sc->device);

	dev_warn(&h->pdev->dev, "sop_abort_handler: this code is UNTESTED.\n");
	cpu = get_cpu();
	submitq = find_submission_queue(h, cpu);
	replyq = find_reply_queue(h, cpu);
	abort_cmd = pqi_alloc_elements(submitq->pqiq, 1);
	if (IS_ERR(abort_cmd)) {
		dev_warn(&h->pdev->dev, "%s: pqi_alloc_elements returned %ld\n",
				__func__, PTR_ERR(abort_cmd));
		return FAILED;
	}
	request_id = alloc_request(h, submitq->pqiq->queue_id);
	if (request_id < 0) {
		dev_warn(&h->pdev->dev, "%s: Failed to allocate request\n",
					__func__);
		/* don't free it, just let it be a NULL IU */
		return FAILED;
	}
	fill_task_mgmt_request(abort_cmd, replyq, request_id,
				sopr_to_abort->request_id, SOP_ABORT_TASK);
	send_sop_command(h, submitq, request_id);
	return process_task_mgmt_response(h, submitq, request_id);
}

static int sop_device_reset_handler(struct scsi_cmnd *sc)
{
	struct sop_device *h;
	struct sop_request *sopr_to_reset =
			(struct sop_request *) sc->host_scribble;
	struct queue_info *submitq, *replyq;
	struct sop_task_mgmt_iu *reset_cmd;
	int request_id, cpu;

	h = sdev_to_hba(sc->device);

	dev_warn(&h->pdev->dev, "sop_device_reset_handler: this code is UNTESTED.\n");
	cpu = get_cpu();
	submitq = find_submission_queue(h, cpu);
	replyq = find_reply_queue(h, cpu);
	reset_cmd = pqi_alloc_elements(submitq->pqiq, 1);
	if (IS_ERR(reset_cmd)) {
		dev_warn(&h->pdev->dev, "%s: pqi_alloc_elements returned %ld\n",
				__func__, PTR_ERR(reset_cmd));
		return FAILED;
	}
	request_id = alloc_request(h, submitq->pqiq->queue_id);
	if (request_id < 0) {
		dev_warn(&h->pdev->dev, "%s: Failed to allocate request\n",
					__func__);
		/* don't free it, just let it be a NULL IU */
		return FAILED;
	}
	fill_task_mgmt_request(reset_cmd, replyq, request_id,
				sopr_to_reset->request_id, SOP_LUN_RESET);
	send_sop_command(h, submitq, request_id);
	return process_task_mgmt_response(h, submitq, request_id);
}

static int sop_slave_alloc(struct scsi_device *sdev)
{
	/* struct sop_device *h = sdev_to_hba(sdev); */

	/* dev_warn(&h->pdev->dev, "sop_slave_alloc called but not implemented\n"); */
	return 0;
}

static void sop_slave_destroy(struct scsi_device *sdev)
{
	/* struct sop_device *h = sdev_to_hba(sdev); */

	/* dev_warn(&h->pdev->dev, "sop_slave_destroy called but not implemented\n"); */
	return;
}

static int sop_compat_ioctl(struct scsi_device *sdev,
						int cmd, void *arg)
{
	struct sop_device *h = sdev_to_hba(sdev);

	dev_warn(&h->pdev->dev, "sop_compat_ioctl called but not implemented\n");
	return 0;
}

static int sop_ioctl(struct scsi_device *sdev, int cmd, void *arg)
{
	struct sop_device *h = sdev_to_hba(sdev);

	dev_warn(&h->pdev->dev, "sop_ioctl called but not implemented, cmd = 0x%08x\n", cmd);
	return -ENOTTY;
}

static pci_ers_result_t sop_pci_error_detected(struct pci_dev *dev,
				enum pci_channel_state error)
{
	dev_warn(&dev->dev,
		"sop_pci_error_detected called but not implemented\n");
	/* FIXME: implement this. */
	return PCI_ERS_RESULT_NONE;
}

static pci_ers_result_t sop_pci_mmio_enabled(struct pci_dev *dev)
{
	dev_warn(&dev->dev,
		"sop_pci_error_mmio_enabled called but not implemented\n");
	/* FIXME: implement this. */
	return PCI_ERS_RESULT_NONE;
}

static pci_ers_result_t sop_pci_link_reset(struct pci_dev *dev)
{
	dev_warn(&dev->dev,
		"sop_pci_error_link_reset called but not implemented\n");
	/* FIXME: implement this. */
	return PCI_ERS_RESULT_NONE;
}

static pci_ers_result_t sop_pci_slot_reset(struct pci_dev *dev)
{
	dev_warn(&dev->dev,
		"sop_pci_error_slot_reset called but not implemented\n");
	/* FIXME: implement this. */
	return PCI_ERS_RESULT_NONE;
}

static void sop_pci_resume(struct pci_dev *dev)
{
	dev_warn(&dev->dev, "sop_pci_resume called but not implemented\n");
	/* FIXME: implement this. */
	return;
}

/* This gets optimized away, but will fail to compile if we mess up
 * the structure definitions.
 */
static void __attribute__((unused)) verify_structure_defs(void)
{
#define VERIFY_OFFSET(member, offset) \
	BUILD_BUG_ON(offsetof(struct pqi_create_operational_queue_request, \
				member) != offset)

	VERIFY_OFFSET(iu_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(iu_length, 2); 
	VERIFY_OFFSET(response_oq, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(function_code, 10);
	VERIFY_OFFSET(reserved2, 11);
	VERIFY_OFFSET(queue_id, 12);
	VERIFY_OFFSET(reserved3, 14);
	VERIFY_OFFSET(element_array_addr, 16);
	VERIFY_OFFSET(index_addr, 24);
	VERIFY_OFFSET(nelements, 32);
	VERIFY_OFFSET(element_length, 34);
	VERIFY_OFFSET(iqp, 36); 
	VERIFY_OFFSET(oqp, 36); 
	VERIFY_OFFSET(reserved4, 47);
#undef VERIFY_OFFSET
	BUILD_BUG_ON(sizeof(struct pqi_create_operational_queue_request) !=
				64);

#define VERIFY_OFFSET(member, offset) \
	BUILD_BUG_ON(offsetof(struct pqi_create_operational_queue_response, \
		member) != offset)

	VERIFY_OFFSET(ui_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(ui_length, 2);
	VERIFY_OFFSET(response_oq, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(function_code, 10);
	VERIFY_OFFSET(status, 11);
	VERIFY_OFFSET(reserved2, 12);
	VERIFY_OFFSET(index_offset, 16);
	VERIFY_OFFSET(reserved3, 24);
	BUILD_BUG_ON(sizeof(struct pqi_create_operational_queue_response) !=
				64);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct pqi_device_register_set, \
			field) != offset)

	VERIFY_OFFSET(signature, 0);
	VERIFY_OFFSET(process_admin_function, 0x08);
	VERIFY_OFFSET(capability, 0x10);
	VERIFY_OFFSET(legacy_intx_status, 0x18);
	VERIFY_OFFSET(legacy_intx_mask_set, 0x1c);
	VERIFY_OFFSET(legacy_intx_mask_clear, 0x20);
	VERIFY_OFFSET(pqi_device_status, 0x40);
	VERIFY_OFFSET(admin_iq_pi_offset, 0x48);
	VERIFY_OFFSET(admin_oq_ci_offset, 0x50);
	VERIFY_OFFSET(admin_iq_addr, 0x58);
	VERIFY_OFFSET(admin_oq_addr, 0x60);
	VERIFY_OFFSET(admin_iq_ci_addr, 0x68);
	VERIFY_OFFSET(admin_oq_pi_addr, 0x70);
	VERIFY_OFFSET(admin_queue_param, 0x78);
	VERIFY_OFFSET(error_data, 0x80);
	VERIFY_OFFSET(reset, 0x88);
	VERIFY_OFFSET(power_action, 0x90);

#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct pqi_delete_operational_queue_request, \
			field) != offset)
        VERIFY_OFFSET(iu_type, 0);
        VERIFY_OFFSET(compatible_features, 1);
        VERIFY_OFFSET(iu_length, 2);
        VERIFY_OFFSET(response_oq, 4);
        VERIFY_OFFSET(work_area, 6);
        VERIFY_OFFSET(request_id, 8);
        VERIFY_OFFSET(function_code, 10);
        VERIFY_OFFSET(reserved2, 11);
        VERIFY_OFFSET(queue_id, 12);
        VERIFY_OFFSET(reserved3, 14);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct pqi_delete_operational_queue_response, \
			field) != offset)
 	VERIFY_OFFSET(ui_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(ui_length, 2);
	VERIFY_OFFSET(response_oq, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(function_code, 10);
	VERIFY_OFFSET(status, 11);
	VERIFY_OFFSET(reserved2, 12);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct pqi_sgl_descriptor, field) != offset)
	VERIFY_OFFSET(address, 0);
	VERIFY_OFFSET(length, 8);
	VERIFY_OFFSET(reserved, 12);
	VERIFY_OFFSET(descriptor_type, 15);
#undef VERIFY_OFFSET
	BUILD_BUG_ON(sizeof(struct pqi_sgl_descriptor) != 16);

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct sop_limited_cmd_iu, field) != offset)
	VERIFY_OFFSET(iu_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(iu_length, 2);
	VERIFY_OFFSET(queue_id, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(flags, 10);
	VERIFY_OFFSET(reserved, 11);
	VERIFY_OFFSET(xfer_size, 12);
	VERIFY_OFFSET(cdb, 16);
	VERIFY_OFFSET(sg, 32);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct sop_cmd_response, field) != offset)
	VERIFY_OFFSET(iu_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(iu_length, 2);
	VERIFY_OFFSET(queue_id, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(nexus_id, 10);
	VERIFY_OFFSET(data_in_xfer_result, 12);
	VERIFY_OFFSET(data_out_xfer_result, 13);
	VERIFY_OFFSET(reserved, 14);
	VERIFY_OFFSET(status, 17);
	VERIFY_OFFSET(status_qualifier, 18);
	VERIFY_OFFSET(sense_data_len, 20);
	VERIFY_OFFSET(response_data_len, 22);
	VERIFY_OFFSET(data_in_xferred, 24);
	VERIFY_OFFSET(data_out_xferred, 28);
	VERIFY_OFFSET(response, 32);
	VERIFY_OFFSET(sense, 32);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct report_pqi_device_capability_iu, field) != offset)

	VERIFY_OFFSET(iu_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(iu_length, 2);
	VERIFY_OFFSET(response_oq, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(function_code, 10);
	VERIFY_OFFSET(reserved, 11);
	VERIFY_OFFSET(buffer_size, 44);
	VERIFY_OFFSET(sg, 48);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct report_pqi_device_capability_response, field) != offset)
	VERIFY_OFFSET(iu_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(iu_length, 2);
	VERIFY_OFFSET(queue_id, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(function_code, 10);
	VERIFY_OFFSET(status, 11);
	VERIFY_OFFSET(additional_status, 12);
	VERIFY_OFFSET(reserved, 16);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct pqi_device_capabilities, field) != offset)
	VERIFY_OFFSET(length, 0);
	VERIFY_OFFSET(reserved, 2);
	VERIFY_OFFSET(max_iqs, 16);
	VERIFY_OFFSET(max_iq_elements, 18);
	VERIFY_OFFSET(reserved2, 20);
	VERIFY_OFFSET(max_iq_element_length, 24);
	VERIFY_OFFSET(min_iq_element_length, 26);
	VERIFY_OFFSET(max_oqs, 28);
	VERIFY_OFFSET(max_oq_elements, 30);
	VERIFY_OFFSET(reserved3, 32);
	VERIFY_OFFSET(intr_coalescing_time_granularity, 34);
	VERIFY_OFFSET(max_oq_element_length, 36);
	VERIFY_OFFSET(min_oq_element_length, 38);
	VERIFY_OFFSET(iq_alignment_exponent, 40);
	VERIFY_OFFSET(oq_alignment_exponent, 41);
	VERIFY_OFFSET(iq_ci_alignment_exponent, 42);
	VERIFY_OFFSET(oq_pi_alignment_exponent, 43);
	VERIFY_OFFSET(protocol_support_bitmask, 44);
	VERIFY_OFFSET(admin_sgl_support_bitmask, 48);
	VERIFY_OFFSET(reserved4, 50);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct sop_task_mgmt_iu, field) != offset)

	VERIFY_OFFSET(iu_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(iu_length, 2);
	VERIFY_OFFSET(queue_id, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(nexus_id, 10);
	VERIFY_OFFSET(reserved, 12);
	VERIFY_OFFSET(lun, 16);
	VERIFY_OFFSET(protocol_specific, 24);
	VERIFY_OFFSET(reserved2, 26);
	VERIFY_OFFSET(request_id_to_manage, 28);
	VERIFY_OFFSET(task_mgmt_function, 30);
	VERIFY_OFFSET(reserved3, 31);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct sop_task_mgmt_response, field) != offset)
	VERIFY_OFFSET(iu_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(iu_length, 2);
	VERIFY_OFFSET(queue_id, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(nexus_id, 10);
	VERIFY_OFFSET(additional_response_info, 12);
	VERIFY_OFFSET(response_code, 15);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct report_general_iu, field) != offset)
	VERIFY_OFFSET(iu_type, 0);
	VERIFY_OFFSET(compatible_features, 1);
	VERIFY_OFFSET(iu_length, 2);
	VERIFY_OFFSET(queue_id, 4);
	VERIFY_OFFSET(work_area, 6);
	VERIFY_OFFSET(request_id, 8);
	VERIFY_OFFSET(reserved, 10);
	VERIFY_OFFSET(allocation_length, 12);
	VERIFY_OFFSET(reserved2, 16);
	VERIFY_OFFSET(data_in, 32);
#undef VERIFY_OFFSET

#define VERIFY_OFFSET(field, offset) \
	BUILD_BUG_ON(offsetof(struct report_general_response_iu, field) != offset)
		VERIFY_OFFSET(reserved, 0);
		VERIFY_OFFSET(lun_bridge_present_flags, 4);
		VERIFY_OFFSET(reserved2, 5);
		VERIFY_OFFSET(app_clients_present_flags, 8);
		VERIFY_OFFSET(reserved3, 9);
		VERIFY_OFFSET(max_incoming_iu_size, 18);
		VERIFY_OFFSET(max_incoming_embedded_data_buffers, 20);
		VERIFY_OFFSET(max_data_buffers, 22);
		VERIFY_OFFSET(reserved4, 24);
		VERIFY_OFFSET(incoming_iu_type_support_bitmask, 32);
		VERIFY_OFFSET(vendor_specific, 64);
		VERIFY_OFFSET(reserved5, 72);
		VERIFY_OFFSET(queuing_layer_specific_data_len, 74);
		VERIFY_OFFSET(incoming_sgl_support_bitmask, 76);
		VERIFY_OFFSET(reserved6, 78);
#undef VERIFY_OFFSET

}

module_init(sop_init);
module_exit(sop_exit);

