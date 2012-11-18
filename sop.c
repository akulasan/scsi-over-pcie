/*
 *    SCSI over PCI (SoP) Block driver
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

#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/completion.h>

#include "sop_kernel_compat.h"
#include "sop.h"

#define DRIVER_VERSION "1.0.0.2"
#define DRIVER_NAME "sop (v " DRIVER_VERSION ")"
#define SOP "sop"

MODULE_AUTHOR("Hewlett-Packard Company");
MODULE_AUTHOR("SanDisk Inc.");
MODULE_DESCRIPTION("sop driver" DRIVER_VERSION);
MODULE_SUPPORTED_DEVICE("sop devices");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#define PCI_CLASS_STORAGE_SOP	0x010800

DEFINE_PCI_DEVICE_TABLE(sop_id_table) = {
	{ PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_SOP, 0xffffff) },
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, sop_id_table);

static int sop_major;
static int controller_num;

static DEFINE_SPINLOCK(dev_list_lock);
static LIST_HEAD(dev_list);
static struct task_struct *sop_thread;

static int sop_add_disk(struct sop_device *h);
static void sop_remove_disk(struct sop_device *h);
static int sop_thread_proc(void *data);

#ifdef CONFIG_COMPAT
static int sop_compat_ioctl(struct block_device *dev, fmode_t mode, unsigned int cmd, unsigned long arg);
#endif
static int sop_ioctl(struct block_device *dev, fmode_t mode, unsigned int cmd, unsigned long arg);

static const struct block_device_operations sop_fops = {
	.owner			= THIS_MODULE,
#if 0
	.revalidate_disk	= sop_revalidate,
	.getgeo			= sop_getgeo,
#endif
	.ioctl			= sop_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= sop_compat_ioctl,
#endif
};

static u32 sop_dbg_lvl;

static ssize_t
sop_sysfs_show_dbg_lvl(struct device_driver *dd, char *buf)
{
	return sprintf(buf, "%u\n", sop_dbg_lvl);
}

static ssize_t
sop_sysfs_set_dbg_lvl(struct device_driver *dd, const char *buf, size_t count)
{
	int retval = count;
	if(sscanf(buf,"%u",&sop_dbg_lvl)<1){
		printk(KERN_ERR "sop: could not set dbg_lvl\n");
		retval = -EINVAL;
	}
	return retval;
}

static DRIVER_ATTR(dbg_lvl, S_IRUGO|S_IWUSR, sop_sysfs_show_dbg_lvl,
		sop_sysfs_set_dbg_lvl);


/* 
 * 32-bit readq and writeq implementations taken from old
 * version of arch/x86/include/asm/io.h 
 */
#ifndef readq
static inline __u64 readq(const volatile void __iomem *addr)
{
	const volatile u32 __iomem *p = addr;
	u32 low, high;

	low = readl(p);
	high = readl(p + 1);

	return low + ((u64)high << 32);
}
#endif

#ifndef writeq
static inline void writeq(__u64 val, volatile void __iomem *addr)
{
	writel(val, addr);
	writel(val >> 32, addr+4);
}
#endif

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

	dev_warn(&h->pdev->dev, "Allocating %llu bytes for SGL area\n",
				(unsigned long long) total_size);
	q->sg = pci_alloc_consistent(h->pdev, total_size, &q->sg_bus_addr);
	q->sgl = kmalloc(q->qdepth * MAX_SGLS * sizeof(struct scatterlist), GFP_KERNEL);
	dev_warn(&h->pdev->dev, "Allocated ptr is %p, bus addr = %llu, sgl = %p\n",
			q->sg, (unsigned long long) q->sg_bus_addr, q->sgl);
	return (q->sg) ? 0 : -ENOMEM;
}

static void free_sgl_area(struct sop_device *h, struct queue_info *q)
{
	size_t total_size = q->qdepth * MAX_SGLS *
				sizeof(struct pqi_sgl_descriptor);

	if (q->sgl)
		kfree(q->sgl);

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
	BUG_ON(nbuffers > MAX_CMDS);
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

static int allocate_wait_queue(struct queue_info *q)
{
	struct sop_wait_queue *wq;

	/* Allocate wait queue structure for TO DEVICE queue */
	wq = kzalloc(sizeof(struct sop_wait_queue), GFP_KERNEL);
	if (!wq)
		return -ENOMEM;
	q->wq = wq;

	init_waitqueue_head(&wq->iq_full);
	init_waitqueue_entry(&wq->iq_cong_wait, sop_thread);
	bio_list_init(&wq->iq_cong);

	return 0;
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

	dev_warn(&h->pdev->dev, "1 Allocating %d queues %s device...\n", 
		num_queues,
		queue_direction == PQI_DIR_TO_DEVICE ? "TO" : "FROM");

	err = -ENOMEM;
	dev_warn(&h->pdev->dev, "2 kzallocing pqi device queues\n");
	*xq = kzalloc(sizeof(**xq) * num_queues, GFP_KERNEL);
	if (!*xq)
		goto bailout;
	dev_warn(&h->pdev->dev, "3 pci_alloc_consistent ring buffer\n");
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
		dev_warn(&h->pdev->dev, "4 allocating request buffers\n");
		for (i = 0; i < num_queues; i++) {
			int q = i + starting_queue_id;
			struct queue_info *qinfo;

			qinfo = &h->qinfo[q];
			if (allocate_q_request_buffers(qinfo,
					n_q_elements,
					sizeof(struct sop_request)))
				goto bailout;

			dev_warn(&h->pdev->dev, "   5 Allocationg waitq #%d, qid[%d]\n", 
				i, q);
			if (allocate_wait_queue(qinfo))
				goto bailout;
			dev_warn(&h->pdev->dev, "   6 Allocated #%d\n", i);
		}

		/* Allocate SGL area for each submission queue */
		for (i = 0; i < num_queues; i++) {
			int q = i + starting_queue_id;

			dev_warn(&h->pdev->dev, "   7 Allocating SGL "
				"area for submission queue %d q=%d\n", i, q);
			if (allocate_sgl_area(h, &h->qinfo[q]))
				goto bailout;
		}
	}

	dev_warn(&h->pdev->dev, "Memory alloc'ed.\n");
	err = 0;

	for (i = 0; i < num_queues; i++) {
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
		dev_warn(&h->pdev->dev, "zzz bottom of loop, i = %d\n", i);
	}

	dev_warn(&h->pdev->dev, "Alloced %d/%d queues\n", nqs_alloced, num_queues);
	return nqs_alloced;

bailout:
	dev_warn(&h->pdev->dev, "zzz problem allocing queues\n");
	kfree(*xq);
	free_all_q_sgl_areas(h);
	free_all_q_request_buffers(h);
	if (vaddr)
		pci_free_consistent(h->pdev, total_size, vaddr, dhandle);
	return err;
}

static void pqi_device_queue_init(struct pqi_device_queue *q,
		__iomem void *queue_vaddr, __iomem u16 *pi,
		__iomem u16 *ci,
		u16 element_size, u16 nelements,
		dma_addr_t dhandle)
{
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
	u32 qciw;
	u16 qci;
	u32 nfree;

	qciw = readw(q->ci);
	qci = le16_to_cpu(*(u16 *) &qciw);

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

	/* FIXME: shouldn't have to read this every time. */
	qpi = le32_to_cpu(readw(q->pi));
	return qpi == q->unposted_index;
}

static void *pqi_alloc_elements(struct pqi_device_queue *q,
					int nelements)
{
	void *p;

	if (pqi_to_device_queue_is_full(q, nelements)) {
		printk(KERN_WARNING "pqi device queue [%d] is full!\n", q->queue_id);
		printk(KERN_WARNING "  unposted_index = %d, ci = %d, nelements=%d\n",
			q->unposted_index, *(q->ci), q->nelements);
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
			printk(KERN_WARNING "pqi device queue [%d] End is full!\n", 
				q->queue_id);
			printk(KERN_WARNING "q->nelements = %d, q->unposted_index = %hu,"
				" extra_elements = %d\n", q->nelements, 
				q->unposted_index, extra_elements);
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

static void pqi_unalloc_elements(struct pqi_device_queue *q,
					int nelements)
{
	q->unposted_index = (q->unposted_index + q->nelements - nelements) % q->nelements;
}

static int __attribute__((unused)) pqi_enqueue_to_device(struct pqi_device_queue *q, void *element)
{
	void *p;

	if (pqi_to_device_queue_is_full(q, 1))
		return PQI_QUEUE_FULL;

	/* FIXME, this is wrong, shouldn't memcpy. */
	p = q->queue_vaddr + q->unposted_index * q->element_size;
	memcpy(p, element, q->element_size);
	q->unposted_index = (q->unposted_index + 1) % q->nelements;
	return 0;
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

	if (hex)
	{
		x = c;
		for (i=0;i<len;i++)
		{
			if ((i % xmargin) == 0 && i>0) printk("\n");
			if ((i % xmargin) == 0) printk("0x%04x:", i);
			printk(" %02x", *x);
			x++;
		}
		printk("\n");
	}
	if (ascii)
	{
		x = c;
		for (i=0;i<len;i++)
		{
			if ((i % amargin) == 0 && i>0) printk("\n");
			if ((i % amargin) == 0) printk("0x%04x:", i);
			if (*x > 26 && *x < 128) printk("%c", *x);
			else printk(".");
			x++;
		}
		printk("\n");
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

/* Can only be called for Admin queue: the q->local_pi is not updated elsewhere */
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

static void pqi_notify_device_queue_written_admin(struct pqi_device_queue *q)
{
	unsigned long flags;
	/*
	 * Notify the device that the host has produced data for the device
	 */
	spin_lock_irqsave(&q->index_lock, flags);
	q->local_pi = q->unposted_index;
	writew(q->unposted_index, q->pi);
	spin_unlock_irqrestore(&q->index_lock, flags);
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

#define ADMIN_SLEEP_INTERVAL_MIN 100 /* microseconds */
#define ADMIN_SLEEP_INTERVAL_MAX 150 /* microseconds */
#define ADMIN_SLEEP_INTERATIONS 1000 /* total of 100 milliseconds */

	do {
		paf = readq(&h->pqireg->process_admin_function);
		function_and_status = paf & 0xff;
		if (function_and_status == 0x00)
			return 0;
		count++;
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
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

	/* Check that device is ready to be set up */
	for (count = 0; count < 10; count++) {
	paf = readq(&h->pqireg->process_admin_function);
	printk(KERN_WARNING
		"paf = %02x %02x %02x %02x %02x %02x %02x %02x\n",
		x[0], x[1], x[2], x[3],
		x[4], x[5], x[6], x[7]);
	status = readl(&h->pqireg->pqi_device_status);
	x = (unsigned char *) &status;
	printk(KERN_WARNING "pqi device status = %02x %02x %02x %02x\n",
		x[0], x[1], x[2], x[3]);
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
	printk(KERN_WARNING "fas = %d, device_state = %d\n",
		function_and_status, pqi_device_state);
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
	}

	pqicap = readq(&h->pqireg->capability);
	memcpy(&h->pqicap, &pqicap, sizeof(h->pqicap));
	dev_warn(&h->pdev->dev, "PQI max admin IQ elements: %hhu\n",
		h->pqicap.max_admin_iq_elements);
	dev_warn(&h->pdev->dev, "PQI max admin OQ elements: %hhu\n",
		h->pqicap.max_admin_oq_elements);
	dev_warn(&h->pdev->dev, "PQI admin IQ element length: %hhu\n",
		h->pqicap.admin_iq_element_length);
	dev_warn(&h->pdev->dev, "PQI admin OQ element length: %hhu\n",
		h->pqicap.admin_oq_element_length);

#define ADMIN_QUEUE_ELEMENT_COUNT 64	/* ((MAX_IO_QUEUES + 1) * 2) */

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
		dev_warn(&h->pdev->dev, "failed to allocate PQI admin queues\n");
		return -1;
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
		paf = readq(&h->pqireg->process_admin_function);
		function_and_status = paf & 0xff;
		dev_warn(&h->pdev->dev,
			"Failed to create admin queues: function_and_status = 0x%02x\n",
			function_and_status);
		if (function_and_status != 0) {
			status = readl(&h->pqireg->pqi_device_status);
			dev_warn(&h->pdev->dev, "Device status = 0x%08x\n", status);
		}
		goto bailout;
	}

	/* Get the offsets of the hardware updated producer/consumer indices */
	admin_iq_pi_offset = readq(&h->pqireg->admin_iq_pi_offset);
	admin_oq_ci_offset = readq(&h->pqireg->admin_oq_ci_offset);
	dev_warn(&h->pdev->dev, "admin_iq_pi_offset = %p\n",
			(void *) admin_iq_pi_offset);
	dev_warn(&h->pdev->dev, "admin_oq_ci_offset = %p\n",
			(void *) admin_oq_ci_offset);
	admin_iq_pi = ((void *) h->pqireg) + admin_iq_pi_offset;
	dev_warn(&h->pdev->dev,
		"zzz ----> h->pqireg = %p, admin_iq_pi_offset = %llu, admin_iq_pi = %p\n",
		h->pqireg, (unsigned long long) admin_iq_pi_offset, admin_iq_pi);
	admin_oq_ci = ((void *) h->pqireg) + admin_oq_ci_offset;

	status = readl(&h->pqireg->pqi_device_status);
	x = (unsigned char *) &status;
	printk(KERN_WARNING "pqi device status = %02x %02x %02x %02x\n",
		x[0], x[1], x[2], x[3]);
	function_and_status = paf & 0xff;
	pqi_device_state = status & 0xff;

	dev_warn(&h->pdev->dev, "device status = %d\n", pqi_device_state);

	dev_warn(&h->pdev->dev, "Successfully created admin queues\n");

	pqi_device_queue_init(&h->admin_q_from_dev,
		admin_oq, admin_oq_pi, admin_oq_ci,
		(u16) h->pqicap.admin_oq_element_length * 16,
		ADMIN_QUEUE_ELEMENT_COUNT, admin_oq_busaddr);

	pqi_device_queue_init(&h->admin_q_to_dev,
		admin_iq, admin_iq_pi, admin_iq_ci,
		(u16) h->pqicap.admin_iq_element_length * 16,
		ADMIN_QUEUE_ELEMENT_COUNT, admin_iq_busaddr);

	h->qinfo[0].pqiq = &h->admin_q_from_dev;
	h->qinfo[1].pqiq = &h->admin_q_to_dev;

	/* Allocate request buffers for admin queues */
	if (allocate_q_request_buffers(&h->qinfo[0],
				ADMIN_QUEUE_ELEMENT_COUNT,
				sizeof(struct sop_request)))
		goto bailout;
	if (allocate_q_request_buffers(&h->qinfo[1],
				ADMIN_QUEUE_ELEMENT_COUNT,
				sizeof(struct sop_request)))
		goto bailout;
	return 0;

bailout:
	if (admin_iq)
		pci_free_consistent(h->pdev, total_admin_queue_size,
					admin_iq, admin_iq_busaddr);
	free_all_q_request_buffers(h);
	return -1;	
}

static int sop_delete_admin_queues(struct sop_device *h)
{
	u64 paf;
	u32 status;
	int rc;
	u8 pqi_device_state, function_and_status;

	paf = 0x0ff & readq(&h->pqireg->process_admin_function);
	status = readl(&h->pqireg->pqi_device_status);
	function_and_status = paf & 0xff;
	pqi_device_state = status & 0xff;

	if (function_and_status != PQI_IDLE) {
		dev_warn(&h->pdev->dev,
			"Cannot remove admin queues, device not idle\n");
		return -1;
	}

	if (pqi_device_state != PQI_READY_FOR_IO) {
		dev_warn(&h->pdev->dev,
			"Cannot remove admin queues, device not ready for i/o\n");
		return -1;
	}
	writeq(PQI_DELETE_ADMIN_QUEUES, &h->pqireg->process_admin_function);
	rc = wait_for_admin_command_ack(h);
	if (!rc)
		return 0;
	paf = readq(&h->pqireg->process_admin_function);
	function_and_status = paf & 0xff;
	dev_warn(&h->pdev->dev,
		"Failed to delete admin queues: function_and_status = 0x%02x\n",
		function_and_status);
	if (function_and_status != 0) {
		status = readl(&h->pqireg->pqi_device_status);
		dev_warn(&h->pdev->dev, "Device status = 0x%08x\n",
				status);
	}
	return -1;
}

static int sop_setup_msix(struct sop_device *h)
{
	int i, err;

	struct msix_entry msix_entry[MAX_TOTAL_QUEUES];

	h->nr_queues = (num_online_cpus() + 1) * 2;
	BUILD_BUG_ON((MAX_TOTAL_QUEUES % 2) != 0);
	if (h->nr_queues > MAX_TOTAL_QUEUES)
		h->nr_queues = MAX_TOTAL_QUEUES;

	h->niqs = h->noqs = h->nr_queues / 2;

	for (i = 0; i < h->noqs-1; i++) {
		msix_entry[i].vector = 0;
		msix_entry[i].entry = i;
	}

	if (!pci_find_capability(h->pdev, PCI_CAP_ID_MSIX))
		goto default_int_mode;
	err = pci_enable_msix(h->pdev, msix_entry, h->noqs-1);
	if (err > 0)
		h->noqs = err;

	if (err == 0) {
		for (i = 0; i < h->noqs; i++) {
			int idx = i ? i + 1 : 0;
			int vid = i ? i - 1 : 0;
			h->qinfo[idx].msix_vector = msix_entry[vid].vector;
			dev_warn(&h->pdev->dev, "q[%d] msix_entry[%d] = %d\n",
				idx, vid, msix_entry[vid].vector);
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

static void sop_complete_bio(struct sop_device *h,
				struct sop_request *r)
{
	struct sop_cmd_response *scr;
	u16 sense_data_len;
	u16 response_data_len;
	u8 xfer_result;
	u32 data_xferred;
	enum dma_data_direction dma_dir;
	u8 sqid;
	struct scatterlist *sgl;
	int result;

	sqid = (r->request_id >> 8);
	sgl = &h->qinfo[sqid].sgl[(r->request_id & 0x0ff) * MAX_SGLS];

	if (bio_data_dir(r->bio) == WRITE)
		dma_dir = DMA_TO_DEVICE;
	else
		dma_dir = DMA_FROM_DEVICE;
	/* undo the DMA mappings */
	dma_unmap_sg(&h->pdev->dev, sgl, r->num_sg, dma_dir);

	result = 0;

	/* dev_warn(&h->pdev->dev, "Response IU type is 0x%02x\n", r->response[0]); */
	switch (r->response[0]) {
	case SOP_RESPONSE_CMD_SUCCESS_IU_TYPE:
                /* No error to process */
		break;

	case SOP_RESPONSE_CMD_RESPONSE_IU_TYPE:
		scr = (struct sop_cmd_response *) r->response;
		result |= scr->status;
		sense_data_len = le16_to_cpu(scr->sense_data_len);
		response_data_len = le16_to_cpu(scr->response_data_len);
		if (unlikely(response_data_len && sense_data_len))
			dev_warn(&h->pdev->dev,
				"Both sense and response data not expected.\n");

		/* copy the sense data */
		if (sense_data_len) {
			/* Nowherre to transfer sense data in block */
			/*
			if (SCSI_SENSE_BUFFERSIZE < sense_data_len)
				sense_data_len = SCSI_SENSE_BUFFERSIZE;
			memset(scmd->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
			memcpy(scmd->sense_buffer, scr->sense, sense_data_len);
			*/
		}

		/* paranoia, check for out of spec firmware */
		if (scr->data_in_xfer_result && scr->data_out_xfer_result)
			dev_warn(&h->pdev->dev,
				"Unexpected bidirectional cmd with status in and out\n");

		/* Calculate residual count */
		if (scr->data_in_xfer_result) {
			xfer_result = scr->data_in_xfer_result;
			data_xferred = le32_to_cpu(scr->data_in_xferred);
		} else {
			xfer_result = scr->data_out_xfer_result;
			data_xferred = le32_to_cpu(scr->data_out_xferred);
		}
		/* Set the residual transfer size */
		r->bio->bi_size = r->xfer_size - data_xferred;

		if (response_data_len) {
			/* FIXME need to do something correct here... */
			result = -EIO;
			dev_warn(&h->pdev->dev, "Got response data... what to do with it?\n");
		}
		break;

	case SOP_RESPONSE_TASK_MGMT_RESPONSE_IU_TYPE:
		result = -EIO;
		dev_warn(&h->pdev->dev, "got unhandled response type...\n");
		break;

	default:
		result = -EIO;
		dev_warn(&h->pdev->dev, "got UNKNOWN response type...\n");
		break;
	}

        bio_endio(r->bio, result);
	free_request(h, sqid, r->request_id);
}

int sop_msix_handle_ioq(struct queue_info *q)
{
	u16 request_id;
	u8 iu_type;
	u8 sq;
	int rc;
	struct sop_device *h = q->h;
	int ncmd=0;
	struct sop_request *r;

#if 0
	int cpu;

	cpu = smp_processor_id();
	printk(KERN_WARNING "=========> Got ioq interrupt, q = %p (%d) vector = %d, cpu %d\n",
			q, q->pqiq->queue_id, q->msix_vector, cpu);
#endif
	if (pqi_from_device_queue_is_empty(q->pqiq)) {
#if 0
		printk(KERN_WARNING "=========> abort handler[cpu %d] processed %d\n", cpu, ncmd);
#endif
		return IRQ_NONE;
	}

	r = q->pqiq->request;
	do {
		if (r == NULL) {
			/* Receiving completion of a new request */ 
			iu_type = pqi_peek_ui_type_from_device(q->pqiq);
			request_id = pqi_peek_request_id_from_device(q->pqiq);
			sq = request_id >> 8; /* queue that request was submitted on */
			r = q->pqiq->request = &h->qinfo[sq].request[request_id & 0x00ff];
			r->request_id = request_id;
			r->response_accumulated = 0;
		}
		rc = pqi_dequeue_from_device(q->pqiq,
				&r->response[r->response_accumulated]); 
		ncmd++;
		if (rc) { /* queue is empty */
			dev_warn(&h->pdev->dev, "=-=-=- io OQ[%hhu] PI %d CI %d empty(rc=%d)\n", 
				q->pqiq->queue_id, *(q->pqiq->pi), q->pqiq->unposted_index, rc);
			break;
		}
		r->response_accumulated += q->pqiq->element_size;
		/* dev_warn(&h->pdev->dev, "accumulated %d bytes\n", r->response_accumulated); */
		if (sop_response_accumulated(r)) {
			/* dev_warn(&h->pdev->dev, "accumlated response\n"); */
			q->pqiq->request = NULL;
			wmb();
			if (likely(r->bio)) {
				sop_complete_bio(h, r);
				r = NULL;
			} else {
				if (likely(r->waiting)) {
					dev_warn(&h->pdev->dev, "Unexpected, waiting != NULL\n");
					complete(r->waiting);
					r = NULL;
				} else {
					dev_warn(&h->pdev->dev, "r->bio and r->waiting both null\n");
				}
			}
			atomic_dec(&h->cmd_pending);
			atomic_dec(&q->cur_qdepth);
			pqi_notify_device_queue_read(q->pqiq);
		}
		else
			dev_warn(&h->pdev->dev, "Multiple entry completion Q[%d] CI %d\n",
				q->pqiq->queue_id, q->pqiq->unposted_index);
	} while (!pqi_from_device_queue_is_empty(q->pqiq));

#if 0
	printk(KERN_WARNING "=========> exit handler[cpu %d] processed %d\n", cpu, ncmd);
#endif
	return IRQ_HANDLED;
}

int sop_msix_handle_adminq(struct queue_info *q)
{
	u8 iu_type;
	u16 request_id;
	int rc;
	struct sop_device *h = q->h;
	u8 sq;

	if (pqi_from_device_queue_is_empty(&h->admin_q_from_dev)) {
		/* dev_warn(&h->pdev->dev, "admin OQ %p is empty\n", q); */
		return IRQ_NONE;
	}

	printk(KERN_WARNING "Got admin oq interrupt, q = %p (%d)\n", q, q->pqiq->queue_id);
	do {
		struct sop_request *r = h->admin_q_from_dev.request;

		dev_warn(&h->pdev->dev, "admin intr, r = %p\n", r);
		if (r == NULL) {
			/* Receiving completion of a new request */ 
			iu_type = pqi_peek_ui_type_from_device(&h->admin_q_from_dev);
			request_id = pqi_peek_request_id_from_device(&h->admin_q_from_dev);
			sq = request_id >> 8; /* queue that request was submitted on */
			dev_warn(&h->pdev->dev, "new completion, iu type: %hhu, id = %hu\n",
					iu_type, request_id);
			r = h->admin_q_from_dev.request = &h->qinfo[sq].request[request_id & 0x00ff];
			dev_warn(&h->pdev->dev, "intr: r = %p\n", r);
			r->response_accumulated = 0;
		}
		rc = pqi_dequeue_from_device(&h->admin_q_from_dev,
					&r->response[r->response_accumulated]); 
		dev_warn(&h->pdev->dev, "dequeued from q %p\n", q);
		if (rc) { /* queue is empty */
			dev_warn(&h->pdev->dev, "admin OQ %p is empty\n", q);
			break;
		}
		r->response_accumulated += h->admin_q_from_dev.element_size;
		dev_warn(&h->pdev->dev, "accumulated %d bytes\n", r->response_accumulated);
		if (sop_response_accumulated(r)) {
			dev_warn(&h->pdev->dev, "accumlated response\n");
			h->admin_q_from_dev.request = NULL;
			wmb();
			complete(r->waiting);
			pqi_notify_device_queue_read(&h->admin_q_from_dev);
		}

	} while (!pqi_from_device_queue_is_empty(&h->admin_q_from_dev));

	return IRQ_HANDLED;
}

irqreturn_t sop_ioq_msix_handler(int irq, void *devid)
{
	struct queue_info *q = devid;
	int ret;

	spin_lock(&q->qlock);
	ret = sop_msix_handle_ioq(q);
	spin_unlock(&q->qlock);

	return ret;
}

irqreturn_t sop_adminq_msix_handler(int irq, void *devid)
{
	struct queue_info *q = devid;
	int ret;

	spin_lock(&q->qlock);
	ret = sop_msix_handle_adminq(q);
	spin_unlock(&q->qlock);

	return ret;
}

static void sop_irq_affinity_hints(struct sop_device *h)
{
	int i, cpu, ret;

	cpu = cpumask_first(cpu_online_mask);
	ret = irq_set_affinity_hint(h->qinfo[0].msix_vector, get_cpu_mask(cpu));
	dev_warn(&h->pdev->dev, "Affinity for Q[0], vector %d, cpu %x, return %d\n",
				h->qinfo[0].msix_vector, cpu, ret);

	for (i = 2; i < h->noqs+1; i++) {
		ret = irq_set_affinity_hint(h->qinfo[i].msix_vector,
					get_cpu_mask(cpu));
		dev_warn(&h->pdev->dev, "Affinity for Q[%d], vector %d, cpu %x, return %d\n",
					i, h->qinfo[i].msix_vector, cpu, ret);
		cpu = cpumask_next(cpu, cpu_online_mask);
	}
}

static int sop_request_irqs(struct sop_device *h, int intr_type, 
					irq_handler_t msix_handler)
{
	u8 i;
	int rc;

	if (intr_type == PQI_ADMIN_INTR) {
		dev_warn(&h->pdev->dev, "Requesting irq %d for msix vector %d (admin)\n",
				h->qinfo[0].msix_vector, 0);
		rc = request_irq(h->qinfo[0].msix_vector, msix_handler,
						IRQF_SHARED, h->devname, &h->qinfo[0]);
		if (rc != 0) {
			dev_warn(&h->pdev->dev, "request_irq failed, i = %d\n", 0);
			goto default_int_mode;
		}
	}

	else {
		/* for loop starts at 2 to skip over the admin queues */
		for (i = 2; i < h->noqs + 1; i++) {
			dev_warn(&h->pdev->dev, "Requesting irq %d for msix vector %d (oq)\n",
					h->qinfo[i].msix_vector, i - 1);
			rc = request_irq(h->qinfo[i].msix_vector, msix_handler, 
					IRQF_SHARED, h->devname, &h->qinfo[i]);
			if (rc != 0) {
				dev_warn(&h->pdev->dev,
						"request_irq failed, i = %d\n", i);
				/* FIXME release irq's 0 through i - 1 */
				goto default_int_mode;
			}
		}
		sop_irq_affinity_hints(h);
	}
	return 0;

default_int_mode:
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
static u16 alloc_request(struct sop_device *h, u8 q)
{
	u16 rc;

	/* I am encoding q number in high bight of request id */
	BUG_ON(h->qinfo[q].qdepth > MAX_CMDS);
	BUG_ON(q > 127); /* high bit reserved for error reporting */

        do {
                rc = (u16) find_first_zero_bit(h->qinfo[q].request_bits,
						h->qinfo[q].qdepth);
                if (rc >= h->qinfo[q].qdepth-1) {
			/*
			int i, lim;

			dev_warn(&h->pdev->dev, "Q[%d] alloc_request failed. Bit=%d Qdepth=%d.\n",
				q, rc, h->qinfo[q].qdepth);
			lim = BITS_TO_LONGS(h->qinfo[q].qdepth) + 1;
			for (i=0; i < lim; i++)
				dev_warn(&h->pdev->dev, "Bits [%02d]: %08lx\n", i, 
					h->qinfo[q].request_bits[i]);
			*/
                        return (u16) -EBUSY;
		}
        } while (test_and_set_bit((int) rc, h->qinfo[q].request_bits));

	return rc | (q << 8);
}

static void free_request(struct sop_device *h, u8 q, u16 request_id)
{
	BUG_ON((request_id >> 8) != q);
	BUG_ON((request_id & 0x00ff) >= h->qinfo[q].qdepth);
	clear_bit(request_id & 0x00ff, h->qinfo[q].request_bits);
}

static void fill_create_io_queue_request(struct sop_device *h,
	struct pqi_create_operational_queue_request *r,
	struct pqi_device_queue *q, int to_device, u16 request_id,
	u16 msix_vector)
{
	u8 function_code = to_device ? 0x10 : 0x11; /* FIXME magic */

	memset(r, 0, sizeof(*r));
	r->iu_type = 0x60; /* FIXME, magic */
	r->iu_length = cpu_to_le16(0x003c);
	r->response_oq = 0;
	/* r->request_id = cpu_to_le16(alloc_request(h, q->queue_id)); */
	r->request_id = cpu_to_le16(request_id);
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
		dev_warn(&h->pdev->dev,
				"Create OQ, QID[%d], MSIX=%d\n", r->queue_id,
				r->oqp.interrupt_message_number);
	}
}

static void fill_delete_io_queue_request(struct sop_device *h,
	struct pqi_delete_operational_queue_request *r, u16 queue_id,
	int to_device, u16 request_id)
{
	u8 function_code = to_device ? 0x12 : 0x13; /* FIXME magic */

	memset(r, 0, sizeof(*r));
	r->iu_type = 0x60; /* FIXME, magic */
	r->iu_length = cpu_to_le16(0x003c);
	r->request_id = cpu_to_le16(request_id);
	r->function_code = function_code;
	r->queue_id = cpu_to_le16(queue_id);
}

static void send_admin_command(struct sop_device *h, u16 request_id)
{
	struct sop_request *request;
	struct pqi_device_queue *aq = &h->admin_q_to_dev;
	DECLARE_COMPLETION_ONSTACK(wait);

	request = &h->qinfo[aq->queue_id].request[request_id & 0x00ff];
	dev_warn(&h->pdev->dev, "Sending request %p\n", request);
	request->waiting = &wait;
	request->response_accumulated = 0;
	dev_warn(&h->pdev->dev, "sending request %hu\n", request_id);
	pqi_notify_device_queue_written_admin(aq);
	dev_warn(&h->pdev->dev, "waiting for completion\n");
	wait_for_completion(&wait);
	dev_warn(&h->pdev->dev, "wait_for_completion returned\n");
}

static int sop_setup_io_queues(struct sop_device *h)
{

	int i, niqs, noqs;
	struct pqi_create_operational_queue_request *r;
	struct pqi_device_queue *aq = &h->admin_q_to_dev;
	u16 request_id;

	dev_warn(&h->pdev->dev,
		"sizeof struct pqi_create_operational_queue_request is %lu\n",
		(unsigned long) sizeof(struct pqi_create_operational_queue_request));

	/* The queue ids == the msix vector index + 1 for OQs. */
	noqs = pqi_device_queue_array_alloc(h, &h->io_q_from_dev,
			h->noqs - 1, OQ_NELEMENTS, OQ_IU_SIZE / 16,
			PQI_DIR_FROM_DEVICE, 2);
	if (h->noqs - 1 < 0)
		goto bail_out;
	if (h->noqs - 1 != noqs) {
		dev_warn(&h->pdev->dev, "Didn't get all the oqs I wanted\n");
		goto bail_out;
	}

	niqs = pqi_device_queue_array_alloc(h, &h->io_q_to_dev,
			h->niqs - 1 , IQ_NELEMENTS, IQ_IU_SIZE / 16,
			PQI_DIR_TO_DEVICE, 2 + noqs);
	dev_warn(&h->pdev->dev, "niqs = %d, h->niqs = %d\n", niqs, h->niqs);
	if (niqs < 0)
		goto bail_out;
	if (h->niqs - 1 != niqs) {
		dev_warn(&h->pdev->dev, "Didn't get all the iqs I wanted\n");
		goto bail_out;
	}

	dev_warn(&h->pdev->dev,
		"Setting up %d submission queues and %d reply queues\n",
			h->niqs - 1, h->noqs - 1);

	for (i = 0; i < h->noqs - 1; i++) {
		struct pqi_device_queue *q;
		volatile struct pqi_create_operational_queue_response *resp;

		dev_warn(&h->pdev->dev,
			"Setting up io queue %d Qid = %d from device\n", i,
			h->io_q_from_dev[i].queue_id);
		/* Set up i/o queue #i from device */

		q = &h->io_q_from_dev[i];
		spin_lock_init(&q->index_lock);
		h->qinfo[h->io_q_from_dev[i].queue_id].pqiq = q;
		r = pqi_alloc_elements(aq, 1);
		dev_warn(&h->pdev->dev, "xxx2 i = %d, r = %p, q = %d\n",
				i, r, h->io_q_from_dev[i].queue_id);
		request_id = alloc_request(h, aq->queue_id);
		dev_warn(&h->pdev->dev, "Allocated request %hu, %p\n", request_id,
				&h->qinfo[aq->queue_id].request[request_id & 0x00ff]);
		if (request_id < 0) { /* FIXME: now what? */
			dev_warn(&h->pdev->dev, "requests unexpectedly exhausted\n");
		}
		fill_create_io_queue_request(h, r, q, 0, request_id, q->queue_id - 2);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_create_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_id & 0x00ff].response;	
		dev_warn(&h->pdev->dev, "resp.status = %hhu, resp.index_offset = %llu\n",
			resp->status, le64_to_cpu(resp->index_offset));
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
		
		dev_warn(&h->pdev->dev,
			"Setting up io queue %d Qid = %d to device\n", i,
			h->io_q_to_dev[i].queue_id);
		/* Set up i/o queue #i to device */
		r = pqi_alloc_elements(aq, 1);
		q = &h->io_q_to_dev[i];
		spin_lock_init(&q->index_lock);
		h->qinfo[h->io_q_to_dev[i].queue_id].pqiq = q;
		dev_warn(&h->pdev->dev, "xxx1 i = %d, r = %p, q = %p\n",
				i, r, q);
		request_id = alloc_request(h, aq->queue_id);
		if (request_id < 0) { /* FIXME: now what? */
			dev_warn(&h->pdev->dev, "requests unexpectedly exhausted 2\n");
		}
		dev_warn(&h->pdev->dev, "Allocated request %hu, %p\n", request_id,
				&h->qinfo[aq->queue_id].request[request_id & 0x00ff]);
		fill_create_io_queue_request(h, r, q, 1, request_id, (u16) -1);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_create_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_id & 0x00ff].response;	
		dev_warn(&h->pdev->dev, "resp.status = %hhu, resp.index_offset = %llu\n",
			resp->status, le64_to_cpu(resp->index_offset));
		if (resp->status != 0)
			dev_warn(&h->pdev->dev, "Failed to set up IQ... now what?\n");
		/* h->io_q_to_dev[i].ci = h->io_q_to_dev[i].queue_vaddr +
						le64_to_cpu(resp->index_offset); */
		h->io_q_to_dev[i].pi = ((void *) h->pqireg) +
					le64_to_cpu(resp->index_offset);
		dev_warn(&h->pdev->dev, "oq %d pi = %p, offset is %llu\n",
					h->io_q_to_dev[i].queue_id,
					h->io_q_to_dev[i].pi,
					le64_to_cpu(resp->index_offset));
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
	u16 request_id;

	for (i = 0; i < h->noqs - 1; i++) {
		volatile struct pqi_delete_operational_queue_response *resp;
		u16 qid;

		r = pqi_alloc_elements(aq, 1);
		request_id = alloc_request(h, aq->queue_id);
		if (request_id < 0) { /* FIXME: now what? */
			dev_warn(&h->pdev->dev, "requests unexpectedly exhausted\n");
		}
		qid = h->io_q_from_dev[i].queue_id;
		fill_delete_io_queue_request(h, r, qid, 1, request_id);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_delete_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_id & 0x00ff].response;	
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
		qid = h->io_q_to_dev[i].queue_id;
		fill_delete_io_queue_request(h, r, qid, 0, request_id);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_delete_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_id & 0x00ff].response;	
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

static DEFINE_IDA(sop_instance_ida);

static int sop_set_instance(struct sop_device *h)
{
	int instance, error;

	do {
		if (!ida_pre_get(&sop_instance_ida, GFP_KERNEL))
			return -ENODEV;

		spin_lock(&dev_list_lock);
		error = ida_get_new(&sop_instance_ida, &instance);
		spin_unlock(&dev_list_lock);
	} while (error == -EAGAIN);

	if (error)
		return -ENODEV;

	h->instance = instance;
	return 0;
}

static void sop_release_instance(struct sop_device *h)
{
	spin_lock(&dev_list_lock);
	ida_remove(&sop_instance_ida, h->instance);
	spin_unlock(&dev_list_lock);
}

static int __devinit sop_probe(struct pci_dev *pdev,
			const struct pci_device_id *pci_id)
{
	struct sop_device *h;
	u64 signature;
	int i, rc;

	dev_warn(&pdev->dev, SOP "found device: %04x:%04x/%04x:%04x\n",
			pdev->vendor, pdev->device,
			pdev->subsystem_vendor, pdev->subsystem_device);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

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

	dev_warn(&pdev->dev, "pci_resource_len = %llu\n",
			(unsigned long long) pci_resource_len(pdev, 0));
	h->pqireg = pci_ioremap_bar(pdev, 0);
	if (!h->pqireg) {
		rc = -ENOMEM;
		goto bail;
	}

	if (sop_set_dma_mask(pdev)) {
		dev_err(&pdev->dev, "failed to set DMA mask\n");
		goto bail;
	}

	signature = readq(&h->pqireg->signature);
	if (memcmp("PQI DREG", &signature, sizeof(signature)) != 0) {
		dev_warn(&pdev->dev, "device does not appear to be a PQI device\n");
		goto bail;
	}
	dev_warn(&pdev->dev, "device does appear to be a PQI device\n");

	rc = sop_setup_msix(h);
	if (rc != 0)
		goto bail;

	rc = sop_create_admin_queues(h);
	if (rc)
		goto bail;

	rc = sop_request_irqs(h, PQI_ADMIN_INTR, sop_adminq_msix_handler);
	if (rc != 0)
		goto bail;

	dev_warn(&h->pdev->dev, "Setting up i/o queues\n");
	rc = sop_setup_io_queues(h);
	if (rc)
		goto bail;
	dev_warn(&h->pdev->dev, "Finished Setting up i/o queues, rc = %d\n", rc);

	rc = sop_request_irqs(h, PQI_IOQ_INTR, sop_ioq_msix_handler);
	if (rc)
		goto bail;

	rc = sop_set_instance(h);
	if (rc)
		goto bail;

	/* TODO: Need to get capacity and LUN info before continuing */
	h->capacity = 0x8000;		/* TODO: For now hard-code it for FPGA */
	h->max_hw_sectors = 2048;	/* TODO: For now hard code it */
	rc = sop_add_disk(h);
	dev_warn(&h->pdev->dev, "Finished adding disk, rc = %d\n", rc);
	if (rc)
		goto bail;

	spin_lock(&dev_list_lock);
	list_add(&h->node, &dev_list);
	spin_unlock(&dev_list_lock);

	return 0;
bail:
	spin_lock(&dev_list_lock);
	list_del(&h->node);
	spin_unlock(&dev_list_lock);

	/* FIXME: Do other cleanup in cascading order */

	dev_warn(&h->pdev->dev, "Bailing out in probe - not freeing a lot\n");
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
	dev_warn(&pdev->dev, "remove called.\n");
	sop_remove_disk(h);
	sop_release_instance(h);
	sop_delete_io_queues(h);
	sop_free_irqs_and_disable_msix(h);
	dev_warn(&pdev->dev, "irqs freed, msix disabled\n");
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
};

static int __init sop_init(void)
{
	int	result;

	sop_thread = kthread_run(sop_thread_proc, NULL, "sop");
	if (IS_ERR(sop_thread)) {
		printk("SOP Init: Thread creation failed with %p!\n", sop_thread);
		return PTR_ERR(sop_thread);
	}

	result = register_blkdev(sop_major, SOP);
	if (result <= 0)
		goto blkdev_fail;
	sop_major = result;

	result = pci_register_driver(&sop_pci_driver);
	if (result < 0)
		goto register_fail;

	result = driver_create_file(&sop_pci_driver.driver, &driver_attr_dbg_lvl);
	if (result)
		goto create_fail;

	printk("SOP driver initialized!\n");
	return 0;

 create_fail:
	pci_unregister_driver(&sop_pci_driver);

 register_fail:
	printk("SOP Init: PCI register failed with %d!\n", result);
	unregister_blkdev(sop_major, SOP);

 blkdev_fail:
	printk("SOP Init: Blkdev register failed with %d!\n", result);
	kthread_stop(sop_thread);
	return result;

}

static void __exit sop_exit(void)
{
	driver_remove_file(&sop_pci_driver.driver, &driver_attr_dbg_lvl);
	pci_unregister_driver(&sop_pci_driver);
	printk("Unregistering Blockdevice with major=%d\n", sop_major);
	unregister_blkdev(sop_major, SOP);
	kthread_stop(sop_thread);
}

static inline struct sop_device *bdev_to_hba(struct block_device *bdev)
{
	unsigned long *priv = bdev->bd_disk->private_data;
	return (struct sop_device *)priv;
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

static struct queue_info *find_submission_queue_fm_replyq(struct queue_info *sq)
{
	struct sop_device *h = sq->h;
	int rqid = sq->pqiq->queue_id;
	int q = rqid + h->noqs - 1;

	return &h->qinfo[q];
}


#define	SCSI_READ_BASIC			0x08
#define	SCSI_WRITE_BASIC		0x0A

#define	SCSI_CMD_RW_10_PRE		0x20
#define	SCSI_CMD_RW_12_PRE		0xA0
#define	SCSI_CMD_RW_16_PRE		0x80

#define	SOP_GET_BYTE(x, n)		(((x) >> ((n) << 3)) & 0xFF)

#define	SOP_FUA				0x08
#define	SOP_DPO				0x10

/* Prepares the CDB from the bio passed */
static int sop_prepare_cdb(u8 *cdb, struct bio *bio)
{
	u32     num_sec;
	u64     lba;
	u8      cmd_low;
	u8      dpo_fua = 0;

	if (bio_data_dir(bio) == WRITE)
		cmd_low = SCSI_WRITE_BASIC;
	else
		cmd_low = SCSI_READ_BASIC;

	/* Init dpo_fua byte from bio flags */
	if (bio->bi_rw & REQ_FUA)
		dpo_fua |= SOP_FUA;
	if (bio->bi_rw & REQ_NOIDLE)
		dpo_fua |= SOP_DPO;
	cdb[1] = dpo_fua;

	num_sec = bio_sectors(bio);
	lba = cpu_to_le64(bio->bi_sector);

	if (lba < (u64)(0x100000000)) {
		if (num_sec < 0x10000) {
			/* Can use RW_10 */
			cdb[0] = (SCSI_CMD_RW_10_PRE | cmd_low);

			cdb[6] = 0;     /* Reserved */
			cdb[7] = SOP_GET_BYTE(num_sec, 1);
			cdb[8] = SOP_GET_BYTE(num_sec, 0);
			cdb[9] = 0;     /* Control */
		}
		else {
			/* Use RW_12 */
			cdb[0] = (SCSI_CMD_RW_12_PRE | cmd_low);

			cdb[6] = SOP_GET_BYTE(num_sec, 3);
			cdb[7] = SOP_GET_BYTE(num_sec, 2);
			cdb[8] = SOP_GET_BYTE(num_sec, 1);
			cdb[9] = SOP_GET_BYTE(num_sec, 0);
			cdb[10]= 0;     /* Reserved */
			cdb[11]= 0;     /* Control */
		}
		cdb[2] = SOP_GET_BYTE(lba, 3);
		cdb[3] = SOP_GET_BYTE(lba, 2);
		cdb[4] = SOP_GET_BYTE(lba, 1);
		cdb[5] = SOP_GET_BYTE(lba, 0);
	}
	else {
		/* Has to be RW_16 */
		cdb[0] = (SCSI_CMD_RW_16_PRE | cmd_low);

		cdb[2] = SOP_GET_BYTE(lba, 7);
		cdb[3] = SOP_GET_BYTE(lba, 6);
		cdb[4] = SOP_GET_BYTE(lba, 5);
		cdb[5] = SOP_GET_BYTE(lba, 4);
		cdb[6] = SOP_GET_BYTE(lba, 3);
		cdb[7] = SOP_GET_BYTE(lba, 2);
		cdb[8] = SOP_GET_BYTE(lba, 1);
		cdb[9] = SOP_GET_BYTE(lba, 0);
		cdb[10]= SOP_GET_BYTE(num_sec, 3);
		cdb[11]= SOP_GET_BYTE(num_sec, 2);
		cdb[12]= SOP_GET_BYTE(num_sec, 1);
		cdb[13]= SOP_GET_BYTE(num_sec, 0);
		cdb[14]= 0;     /* Reserved+Group Num */
		cdb[15]= 0;     /* Control */
	}

	return 0;
}


/* Prepares the scatterlist for the given bio */
static int sop_prepare_scatterlist(struct bio *bio, struct sop_request *ser, 
                                   struct scatterlist  *sgl, int nsegs)
{
	int	i, len=0, num_sg = 0;
	struct bio_vec  *bv, *prev_bv = NULL;
	struct scatterlist  *cur_sg = NULL;

	/* Init the SG table */
	sg_init_table(sgl, nsegs);

	/* Scan the list of segments in bio */
	bio_for_each_segment(bv, bio, i) {
		/* Process the bv */
		if (prev_bv && __BIOVEC_PHYS_MERGEABLE(prev_bv, bv)) {
			/* Merge the sg */
			cur_sg->length += bv->bv_len;
		}
		else {
			/* Create a new SG */
			if (cur_sg == NULL)
				cur_sg = sgl;
			else
				cur_sg++;
#if 0
			printk("SG[%d] (%d) 0x%p, 0x%x\n", num_sg, i,
				bv->bv_page + bv->bv_offset, bv->bv_len);
#endif
			sg_set_page(cur_sg, bv->bv_page, bv->bv_len,
				bv->bv_offset);
			num_sg++;
		}
		len += bv->bv_len;
		prev_bv = bv;
	}

	bio->bi_idx = i;
	ser->xfer_size = len;

	sg_mark_end(cur_sg);

	return (num_sg);
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
				struct scatterlist *sgl, int num_sg,
				u32 *xfer_size)
{
	struct pqi_sgl_descriptor *datasg;
	int i;
	static const u16 no_sgl_size =
			(u16) (sizeof(*r) - sizeof(r->sg[0]) * 2) - 4;
	BUILD_BUG_ON(sizeof(*r) != 64);
	BUILD_BUG_ON((sizeof(r->sg[0]) != (16)));
	BUILD_BUG_ON((sizeof(*r) - sizeof(r->sg[0]) * 2) != 32);

	BUG_ON(num_sg > 2);
	if (!num_sg) {
		r->iu_length = cpu_to_le16(no_sgl_size);
		return;
	}

	*xfer_size = 0;
	datasg = &r->sg[0];
	r->iu_length = cpu_to_le16(no_sgl_size + sizeof(*datasg) * num_sg);

	for (i=0; i<num_sg; i++) {
		fill_sg_data_element(datasg, &sgl[i], xfer_size);
		datasg++;
	}
}

static int sop_scatter_gather(struct sop_device *h,
			struct queue_info *q,
			int num_sg,
			struct sop_limited_cmd_iu *r,
			struct scatterlist *sgl, u32 *xfer_size)
{
	int sg_block_number;
	int i;
	struct pqi_sgl_descriptor *datasg;
	static const u16 no_sgl_size =
			(u16) (sizeof(*r) - sizeof(r->sg[0]) * 2) - 4;

	BUG_ON(num_sg > MAX_SGLS);

	if (num_sg < 3) {
		fill_inline_sg_list(r, sgl, num_sg, xfer_size);
		return 0;
	}

	*xfer_size = 0;
	datasg = &r->sg[0];
	sg_block_number = (r->request_id & 0x0ff) * MAX_SGLS;
	r->iu_length = cpu_to_le16(no_sgl_size + sizeof(*datasg) * 2);

	for (i=0; i < num_sg; i++) {
		if (i == 1) {
			fill_sg_chain_element(datasg, q,
					sg_block_number, num_sg-1);
			datasg = &q->sg[sg_block_number];
		}
		fill_sg_data_element(datasg, &sgl[i], xfer_size);
		datasg++;
	}
	return 0;
}

static int sop_process_bio(struct sop_device *h, struct bio *bio,
			   struct queue_info *submitq, struct queue_info *replyq)
{
	struct sop_limited_cmd_iu *r;
	struct sop_request *ser;
	enum dma_data_direction dma_dir;
	struct scatterlist *sgl;
	u16 request_id;
	int prev_index, num_sg;
	int nsegs;

	request_id = alloc_request(h, submitq->pqiq->queue_id);
	if (request_id == (u16) -EBUSY) {
		/*
		dev_warn(&h->pdev->dev, "Failed to allocate request for bio %p!\n", bio);
		dev_warn(&h->pdev->dev, "SUBQ[%d] PI %d, CI %d \n", submitq->pqiq->queue_id,
			submitq->pqiq->unposted_index,  *(submitq->pqiq->ci));
			*/
		return -EBUSY;
	}

	r = pqi_alloc_elements(submitq->pqiq, 1);
	if (IS_ERR(r)) {
		dev_warn(&h->pdev->dev, "SUBQ[%d] pqi_alloc_elements for bio %p returned %ld\n", 
			submitq->pqiq->queue_id, bio, PTR_ERR(r));
		goto alloc_elem_fail;
	}
	nsegs = bio_phys_segments(h->rq, bio);

	r->iu_type = SOP_LIMITED_CMD_IU;
	r->compatible_features = 0;
	r->queue_id = cpu_to_le16(replyq->pqiq->queue_id);
	r->work_area = 0;
	r->request_id = request_id;
	sgl = &submitq->sgl[(request_id & 0x0ff) * MAX_SGLS];
	ser = &submitq->request[request_id & 0x0ff];
	ser->xfer_size = 0;
	ser->bio = bio;
	ser->waiting = NULL;

	/* It has to be a READ or WRITE for BIO */
	if (bio_data_dir(bio) == WRITE) {
		r->flags = SOP_DATA_DIR_TO_DEVICE;
		dma_dir = DMA_TO_DEVICE;
	}
	else {
		r->flags = SOP_DATA_DIR_FROM_DEVICE;
		dma_dir = DMA_FROM_DEVICE;
	}

	/* Prepare the CDB Now */
	sop_prepare_cdb(r->cdb, bio);

	/* Prepare the scatterlist */
	prev_index = bio->bi_idx;
	num_sg = sop_prepare_scatterlist(bio, ser, sgl, nsegs);

	/* Map the SG */
	num_sg = dma_map_sg(&h->pdev->dev, sgl, num_sg, dma_dir);
	if (num_sg < 0) {
		bio->bi_idx = prev_index;
		dev_warn(&h->pdev->dev, "dma_map failure bio %p, SQ[%d].\n",
			bio, submitq->pqiq->queue_id);
		goto sg_map_fail;
	}
	atomic_inc(&replyq->cur_qdepth);
	submitq->max_qdepth++;
	if (replyq->max_qdepth < atomic_read(&replyq->cur_qdepth))
		replyq->max_qdepth = atomic_read(&replyq->cur_qdepth);
	atomic_inc(&h->cmd_pending);
	if (atomic_read(&h->cmd_pending) > h->max_cmd_pending)
		h->max_cmd_pending = atomic_read(&h->cmd_pending);
	ser->num_sg = num_sg;
#if 0
	dev_warn(&h->pdev->dev, "CDB: [0]%02x [1]%02x [2]%02x %02x %02x %02x"
		"  XFER Size: %d, Num Seg %d\n",
		r->cdb[0], r->cdb[1], r->cdb[2], r->cdb[3], r->cdb[4], r->cdb[5], 
		ser->xfer_size, num_sg);
#endif
	sop_scatter_gather(h, submitq, num_sg, r, sgl, &ser->xfer_size);

	r->xfer_size = cpu_to_le32(ser->xfer_size);
	
	/* Submit it to the device */
	writew(submitq->pqiq->unposted_index, submitq->pqiq->pi);

	return 0;

sg_map_fail:
	pqi_unalloc_elements(submitq->pqiq, 1);

alloc_elem_fail:
	free_request(h, submitq->pqiq->queue_id, request_id);
	return -EBUSY;
}

static MRFN_TYPE sop_make_request(struct request_queue *q, struct bio *bio)
{
	struct sop_device *h = q->queuedata;
	int result;
	int cpu;
	struct queue_info *submitq, *replyq;
	struct sop_wait_queue *wq;

	/* Prepare the SOP IU and fire */
	cpu = get_cpu();

	/* Get the queues */
	submitq = find_submission_queue(h, cpu);
	replyq = find_reply_queue(h, cpu);
	if (!submitq)
		dev_warn(&h->pdev->dev, "make_request: q is null!\n");
	if (!submitq->pqiq)
		dev_warn(&h->pdev->dev, "make_request: q->pqiq is null!\n");
	wq = submitq->wq;
	if (!wq) {
		dev_warn(&h->pdev->dev, "make_request: wq null for SubmitQ[%d], RQ[%d]!\n",
					submitq->pqiq->queue_id, replyq->pqiq->queue_id);
		return MRFN_RET;
	}

	spin_lock_irq(&submitq->qlock);

	result = -EBUSY;
	if (bio_list_empty(&wq->iq_cong))
		/* Try to submit the command */
		result = sop_process_bio(h, bio, submitq, replyq);

	if (unlikely(result)) {
		if (bio_list_empty(&wq->iq_cong))
			add_wait_queue(&wq->iq_full, &wq->iq_cong_wait);
		bio_list_add(&wq->iq_cong, bio);
	}

	spin_unlock_irq(&submitq->qlock);
	put_cpu();

	return MRFN_RET;
}


static int sop_add_disk(struct sop_device *h)
{
	struct gendisk *disk;
	struct request_queue *rq;

	dev_warn(&h->pdev->dev, "sop_add_disk 1\n");
	rq = blk_alloc_queue(GFP_KERNEL);
	if (IS_ERR(rq))
		return -ENOMEM;
	
	/* Save the field in device struct */
	h->rq = rq;

	rq->queue_flags = QUEUE_FLAG_DEFAULT;
/* #if (!defined(REDHAT5) && !defined(SUSE10)) */
	queue_flag_set_unlocked(QUEUE_FLAG_NOMERGES, rq);
	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, rq);
/* #endif */
	blk_queue_make_request(rq, sop_make_request);
	rq->queuedata = h;

	dev_warn(&h->pdev->dev, "sop_add_disk 2\n");
	disk = alloc_disk(SOP_MINORS);
	if (!disk)
		goto out_free_queue;

	h->disk = disk;
	dev_warn(&h->pdev->dev, "sop_add_disk 3\n");
	blk_queue_logical_block_size(rq, 512);	/* TODO: change hardcode */
	if (h->max_hw_sectors)
		blk_queue_max_hw_sectors(rq, h->max_hw_sectors);
	blk_queue_max_segments(rq, MAX_SGLS);
	/* TODO: Setup other blk queue parameters too - e.g for segment limit */
	disk->major = sop_major;
	disk->minors = SOP_MINORS;
	disk->first_minor = SOP_MINORS * h->instance;
	disk->fops = &sop_fops;
	disk->private_data = h;
	disk->queue = rq;
	disk->driverfs_dev = &h->pdev->dev;
	sprintf(disk->disk_name, SOP"%d", h->instance);
	set_capacity(disk, h->capacity);
	dev_warn(&h->pdev->dev, "Creating SOP drive '%s'- Capacity %d sectors\n", 
		disk->disk_name, (int)(h->capacity));
	add_disk(disk);

	return 0;

 out_free_queue:
	blk_cleanup_queue(rq);
	return -ENOSYS;
}


static void sop_remove_disk(struct sop_device *h)
{
	/* Cleanup the node info */
	spin_lock(&dev_list_lock);
	list_del(&h->node);
	spin_unlock(&dev_list_lock);

	/* First free the disk */
	del_gendisk(h->disk);

	/* Free the request queue */
	blk_cleanup_queue(h->rq);
}


#if 0
static int sop_abort_handler(struct scsi_cmnd *sc)
{
	struct sop_device *h;

	h = bdev_to_hba(sc->device);
	dev_warn(&h->pdev->dev, "sop_abort_handler called but not implemented\n");
	return 0;
}

static int sop_device_reset_handler(struct scsi_cmnd *sc)
{
	struct sop_device *h;

	h = bdev_to_hba(sc->device);
	dev_warn(&h->pdev->dev, "sop_device_reset_handler called but not implemented\n");
	return 0;
}

#endif

#ifdef CONFIG_COMPAT
static int sop_compat_ioctl(struct block_device *dev, fmode_t mode, unsigned int cmd, unsigned long arg)
{
	struct sop_device *h = bdev_to_hba(dev);

	dev_warn(&h->pdev->dev, "sop_compat_ioctl called but not implemented\n");
	return 0;
}
#endif

static int sop_ioctl(struct block_device *dev, fmode_t mode, unsigned int cmd, unsigned long arg)
{
	struct sop_device *h = bdev_to_hba(dev);

	dev_warn(&h->pdev->dev, "sop_ioctl called but not implemented\n");
	return 0;
}

static void sop_timeout_ios(struct queue_info *q, uint flag)
{
}

static void sop_timeout_admin(struct sop_device *h)
{
}

static void sop_resubmit_waitq(struct queue_info *rq)
{
	struct sop_wait_queue *wq;
	struct sop_device *h;
	struct queue_info *sq;
	int ret;

	/* Compute the reply Q from submission queue */
	sq = find_submission_queue_fm_replyq(rq);

	h = rq->h;
	wq = sq->wq;
	if (!wq) {
		dev_warn(&h->pdev->dev, "sop_resubmit_waitq: wq null for SubmitQ[%d]!\n",
					sq->pqiq->queue_id);
		return;
	}

	spin_lock_irq(&sq->qlock);
	while (bio_list_peek(&wq->iq_cong)) {
		struct bio *bio = bio_list_pop(&wq->iq_cong);

		/* dev_warn(&h->pdev->dev, "sop_resubmit_waitq: proc bio %p Q[%d], PI %d CI %d!\n",
			bio, sq->pqiq->queue_id, sq->pqiq->unposted_index, *(sq->pqiq->ci)); */

		if ((ret = sop_process_bio(h, bio, sq, rq))) {
			/* dev_warn(&h->pdev->dev, "sop_resubmit_waitq: Error %d for [%d]!\n",
						ret, sq->pqiq->queue_id); */
			bio_list_add_head(&wq->iq_cong, bio);
			break;
		}
		if (bio_list_empty(&wq->iq_cong))
			remove_wait_queue(&wq->iq_full, &wq->iq_cong_wait);

		/* dev_warn(&h->pdev->dev, "sop_resubmit_waitq: done bio %p Q[%d], PI %d CI %d!\n",
			bio, sq->pqiq->queue_id, sq->pqiq->unposted_index, *(sq->pqiq->ci)); */
	}
	spin_unlock_irq(&sq->qlock);
}

static int sop_thread_proc(void *data)
{
	struct sop_device *h;

	while (!kthread_should_stop()) {
		__set_current_state(TASK_RUNNING);
		spin_lock(&dev_list_lock);
		list_for_each_entry(h, &dev_list, node) {
			int i;
			struct queue_info *q = &h->qinfo[0];

			/* Admin queue */
			spin_lock_irq(&q->qlock);
			sop_msix_handle_adminq(q);
			sop_timeout_admin(h);
			spin_unlock_irq(&q->qlock);

			/* Io Queue */
			for (i = 2; i < h->noqs+1; i++) {
				q = &h->qinfo[i];

				if (!q)
					continue;
				
				spin_lock_irq(&q->qlock);

				/* Process any pending ISR */
				sop_msix_handle_ioq(q);

				/* Handle errors */
				sop_timeout_ios(q, true);
				spin_unlock_irq(&q->qlock);

				/* Process wait queue */
				sop_resubmit_waitq(q);

			}

			if (sop_dbg_lvl == 1) {
				int i;

				sop_dbg_lvl = 0;
				printk("@@@@ Total CMD Pending= %d Max = %d @@@@\n", atomic_read(&h->cmd_pending),
						h->max_cmd_pending);
				for (i = 0; i < h->noqs - 1; i++) {
					printk("        OQ depth[%d] = %d, Max %d, Total = %d\n", i,
						atomic_read(&h->qinfo[h->io_q_from_dev[i].queue_id].cur_qdepth),
						h->qinfo[h->io_q_from_dev[i].queue_id].max_qdepth,
						h->qinfo[h->io_q_to_dev[i].queue_id].max_qdepth);
				}
			}
		}
		spin_unlock(&dev_list_lock);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);
	}
	return 0;
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
	VERIFY_OFFSET(reserved, 4);
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
        VERIFY_OFFSET(reserved, 4);
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
	VERIFY_OFFSET(reserved, 4);
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

}

module_init(sop_init);
module_exit(sop_exit);

