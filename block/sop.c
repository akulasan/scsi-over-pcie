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
#include <scsi/scsi.h>
#include "sop_kernel_compat.h"
#include "sop.h"

#define DRIVER_VERSION "1.0.1.2"
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

static DEFINE_SPINLOCK(dev_list_lock);
static LIST_HEAD(dev_list);
static struct task_struct *sop_thread;

static int sop_get_disk_params(struct sop_device *h);
static int sop_add_disk(struct sop_device *h);
static void sop_remove_disk(struct sop_device *h);
static int sop_thread_proc(void *data);
static int sop_add_timeout(struct queue_info *q, uint timeout);
static void sop_rem_timeout(struct queue_info *q, uint tmo_slot);
static void sop_fail_all_outstanding_io(struct sop_device *h);

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
	q->sgl = kmalloc(q->qdepth * MAX_SGLS * sizeof(struct scatterlist), GFP_KERNEL);
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

static void free_wait_queue(struct queue_info *q)
{
	struct sop_wait_queue *wq = q->wq;

	if (wq)
		kfree(wq);
}

static int pqi_device_queue_alloc(struct sop_device *h,
		struct pqi_device_queue **xq,
		u16 n_q_elements, u8 q_element_size_over_16,
		int queue_direction, int queue_pair_index)
{
	void *vaddr = NULL;
	dma_addr_t dhandle;

	int total_size = (n_q_elements * q_element_size_over_16 * 16) +
				sizeof(u64);
#if 0
	int remainder = total_size % 64;

	/* this adjustment we think is unnecessary because we now allocate queues
	 * separately not in a big array, and pci_alloc_consistent returns
	 * page aligned memory.
	 */
	total_size += remainder ? 64 - remainder : 0;
#endif

	*xq = kzalloc(sizeof(**xq), GFP_KERNEL);
	if (!*xq) {
		dev_warn(&h->pdev->dev, "Failed to alloc pqi struct #%d, dir %d\n", 
			queue_pair_index, queue_direction);
		goto bailout;
	}
	vaddr = pci_alloc_consistent(h->pdev, total_size, &dhandle);
	if (!vaddr) {
		dev_warn(&h->pdev->dev, "Failed to alloc PCI buffer #%d, dir %d\n", 
			queue_pair_index, queue_direction);
		goto bailout;
	}
	(*xq)->dhandle = dhandle;
	(*xq)->vaddr = vaddr;
	(*xq)->queue_vaddr = vaddr;

	if (queue_direction == PQI_DIR_TO_DEVICE) {
		(*xq)->ci = vaddr + q_element_size_over_16 * 16 * n_q_elements;
		/* TODO probably do not need to store this - calculate from qinfo address */
		(*xq)->queue_id = queue_pair_index * 2 + 1;
		/* (*xq)->pi is unknown now, hardware will tell us later */
	} else {
		(*xq)->pi = vaddr + q_element_size_over_16 * 16 * n_q_elements;
		(*xq)->queue_id = queue_pair_index * 2; /* TODO: may remove this later */
		/* (*xq)->ci is unknown now, hardware will tell us later */
	}
	(*xq)->unposted_index = 0;
	(*xq)->element_size = q_element_size_over_16 * 16;
	(*xq)->nelements = n_q_elements;

	return 0;

bailout:
	dev_warn(&h->pdev->dev, "Problem allocing queues\n");
	if (vaddr)
		pci_free_consistent(h->pdev, total_size, vaddr, dhandle);
	if (*xq)
		kfree(*xq);
	*xq = NULL;
	return -ENOMEM;
}

static void pqi_device_queue_init(struct pqi_device_queue *q,
		__iomem u16 *pi, __iomem u16 *ci)
{	
	q->pi = pi;
	q->ci = ci;
	q->unposted_index = 0;
	spin_lock_init(&q->qlock);
	spin_lock_init(&q->index_lock);
}

static void pqi_device_queue_free(struct sop_device *h, struct pqi_device_queue *q)
{
	size_t total_size, n_q_elements, element_size;

	if (q == NULL)
		return;

	n_q_elements = q->nelements;
	element_size = q->element_size;
	total_size = n_q_elements * element_size + sizeof(u64);
	pci_free_consistent(h->pdev, total_size, q->vaddr, q->dhandle);

	kfree(q);
}

static void pqi_iq_buffer_free(struct sop_device *h, struct queue_info *qinfo)
{
	free_q_request_buffers(qinfo);
	free_wait_queue(qinfo);
	free_sgl_area(h, qinfo);
}

static int pqi_iq_data_alloc(struct sop_device *h, struct queue_info *qinfo)
{
	int queue_pair_index = qinfo_to_qid(qinfo);
	int n_q_elements = qinfo->iq->nelements;

	if (allocate_q_request_buffers(qinfo, n_q_elements,
		sizeof(struct sop_request))) {
		dev_warn(&h->pdev->dev, "Failed to alloc rq buffers #%d\n", 
			queue_pair_index);
		goto bailout_iq;
	}

	if (allocate_wait_queue(qinfo)) {
		dev_warn(&h->pdev->dev, "Failed to alloc waitq #%d\n",
			queue_pair_index);
		goto bailout_iq;
	}

	/* Allocate SGL area for submission queue */
	if (allocate_sgl_area(h, qinfo)) {
		dev_warn(&h->pdev->dev, "Failed to alloc SGL #%d\n",
			queue_pair_index);
		goto bailout_iq;
	}
	return 0;

bailout_iq:
	pqi_iq_buffer_free(h, qinfo);
	return -ENOMEM;
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

static void *pqi_alloc_elements(struct pqi_device_queue *q, int nelements)
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

static void pqi_notify_device_queue_written(struct pqi_device_queue *q)
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

#define ADMIN_SLEEP_INTERVAL_MIN	100 /* microseconds */
#define ADMIN_SLEEP_INTERVAL_MAX	150 /* microseconds */
#define ADMIN_SLEEP_INTERATIONS		1000 /* total of 100 milliseconds */
#define ADMIN_SLEEP_TMO_MS		100 /* total of 100 milliseconds */

	do {
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
		paf = readq(&h->pqireg->process_admin_function);
		function_and_status = paf & 0xff;
		if (function_and_status == 0x00)
			return 0;
		count++;
	} while (count < ADMIN_SLEEP_INTERATIONS);
	return -1;
}

static int wait_for_admin_queues_to_become_idle(struct sop_device *h, 
						int timeout_ms,
						u8 device_state)
{
	int i;
	u64 paf;
	u32 status;
	u8 pqi_device_state, function_and_status;
	__iomem void *sig = &h->pqireg->signature;
	int tmo_count = timeout_ms * 10;	/* Each loop is 100us */

	for (i = 0; i < tmo_count; i++) {
		usleep_range(ADMIN_SLEEP_INTERVAL_MIN,
				ADMIN_SLEEP_INTERVAL_MAX);
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
			pqi_device_state == device_state)
			return 0;
		if (i == 0)
			dev_warn(&h->pdev->dev,
				"Waiting for admin queues to become idle (FnSt=0x%x, DevSt=0x%x\n",
				function_and_status, pqi_device_state);
	}
	dev_warn(&h->pdev->dev,
			"Failed waiting for admin queues to become idle and device state %d.",
			device_state);
	return -1;
}

static inline int sop_admin_queue_buflen(struct sop_device *h, int nelements)
{
	return (((h->pqicap.admin_iq_element_length * 16) +
		(h->pqicap.admin_oq_element_length * 16)) *
		nelements + 32);
}

static void sop_free_admin_queues(struct sop_device *h)
{
	struct queue_info *adminq = &h->qinfo[0];

	free_q_request_buffers(adminq);
	pqi_device_queue_free(h, adminq->iq);
	pqi_device_queue_free(h, adminq->oq);
	adminq->iq = adminq->oq = NULL;
}

#define ADMIN_QUEUE_ELEMENT_COUNT 64
static int __devinit sop_alloc_admin_queues(struct sop_device *h)
{
	u64 pqicap;
	u8 admin_iq_elem_count, admin_oq_elem_count;
	__iomem void *sig = &h->pqireg->signature;
	char *msg = "";

	if (safe_readq(sig, &pqicap, &h->pqireg->capability)) {
		dev_warn(&h->pdev->dev,  "Unable to read pqi capability register\n");
		return -1;
	}
	pqicap = readq(&h->pqireg->capability);
	memcpy(&h->pqicap, &pqicap, sizeof(h->pqicap));

	admin_iq_elem_count = admin_oq_elem_count = ADMIN_QUEUE_ELEMENT_COUNT;

	if (h->pqicap.max_admin_iq_elements < admin_iq_elem_count)
		admin_iq_elem_count = h->pqicap.max_admin_iq_elements;
	if (h->pqicap.max_admin_oq_elements < admin_oq_elem_count)
		admin_oq_elem_count = h->pqicap.max_admin_oq_elements;
	if (admin_oq_elem_count == 0 || admin_iq_elem_count == 0) {
		dev_warn(&h->pdev->dev, "Invalid Admin Q elerment count %d in PQI caps\n",
				ADMIN_QUEUE_ELEMENT_COUNT);
		return -1;
	}

	if (pqi_device_queue_alloc(h, &h->qinfo[0].oq, admin_oq_elem_count,
			h->pqicap.admin_iq_element_length, PQI_DIR_FROM_DEVICE, 0))
		return -1;

	if (pqi_device_queue_alloc(h, &h->qinfo[0].iq, admin_iq_elem_count,
			h->pqicap.admin_iq_element_length, PQI_DIR_TO_DEVICE, 0))
		goto bailout;

#define PQI_REG_ALIGNMENT 16

	if (h->qinfo[0].iq->dhandle % PQI_REG_ALIGNMENT != 0 ||
		h->qinfo[0].oq->dhandle % PQI_REG_ALIGNMENT != 0) {
		dev_warn(&h->pdev->dev, "Admin queues are not properly aligned.\n");
		dev_warn(&h->pdev->dev, "admin_iq_busaddr = %p\n",
				(void *) h->qinfo[0].iq->dhandle);
		dev_warn(&h->pdev->dev, "admin_oq_busaddr = %p\n",
				(void *) h->qinfo[0].oq->dhandle);
	}

	/* Allocate request buffers for admin queues */
	if (allocate_q_request_buffers(&h->qinfo[0],
				ADMIN_QUEUE_ELEMENT_COUNT,
				sizeof(struct sop_request))) {
		msg = "Failed to allocate admin request queue buffer";
		goto bailout;
	}

	return 0;

bailout:
	sop_free_admin_queues(h);

	dev_warn(&h->pdev->dev, "%s: %s\n", __func__, msg);
	return -1;	
}

static int sop_create_admin_queues(struct sop_device *h)
{
	u64 paf, admin_iq_pi_offset, admin_oq_ci_offset;
	u32 status, admin_queue_param;
	u8 function_and_status;
	u8 pqi_device_state;
	struct pqi_device_queue *admin_iq, *admin_oq;
	u16 *admin_iq_ci, *admin_oq_pi, *admin_iq_pi, *admin_oq_ci;
	dma_addr_t admin_iq_ci_busaddr, admin_oq_pi_busaddr;
	u16 msix_vector;
	__iomem void *sig = &h->pqireg->signature;
	int rc;
	char *msg = "";

	/* Check that device is ready to be set up */
	if (wait_for_admin_queues_to_become_idle(h, ADMIN_SLEEP_TMO_MS,
					PQI_READY_FOR_ADMIN_FUNCTION))
		return -1;

	admin_iq = h->qinfo[0].iq;
	admin_oq = h->qinfo[0].oq;

	admin_iq_ci = admin_iq->ci;
	admin_oq_pi = admin_oq->pi;

	admin_iq_ci_busaddr = admin_iq->dhandle +
				(h->pqicap.admin_iq_element_length * 16) *
				admin_iq->nelements;
	admin_oq_pi_busaddr = admin_oq->dhandle +
				(h->pqicap.admin_oq_element_length * 16) *
				admin_oq->nelements;

	msix_vector = 0; /* Admin Queue always uses vector [0] */
	admin_queue_param = admin_iq->nelements |
			(admin_oq->nelements << 8) |
			(msix_vector << 16);

	/* Tell the hardware about the admin queues */
	writeq(admin_iq->dhandle, &h->pqireg->admin_iq_addr);
	writeq(admin_oq->dhandle, &h->pqireg->admin_oq_addr);
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
		msg = "Failed to create admin queue";
		goto bailout;
	}

	/* Get the offsets of the hardware updated producer/consumer indices */
	admin_iq_pi_offset = readq(&h->pqireg->admin_iq_pi_offset);
	admin_oq_ci_offset = readq(&h->pqireg->admin_oq_ci_offset);
	admin_iq_pi = ((void *) h->pqireg) + admin_iq_pi_offset;
	admin_oq_ci = ((void *) h->pqireg) + admin_oq_ci_offset;

	status = readl(&h->pqireg->pqi_device_status);
	function_and_status = paf & 0xff;
	pqi_device_state = status & 0xff;

	pqi_device_queue_init(admin_oq, admin_oq_pi, admin_oq_ci);
	pqi_device_queue_init(admin_iq, admin_iq_pi, admin_iq_ci);

	dev_warn(&h->pdev->dev, "Successfully created admin queues\n");
	return 0;

bailout:
	dev_warn(&h->pdev->dev, "%s: %s\n", __func__, msg);
	return -1;	
}

static int sop_delete_admin_queues(struct sop_device *h)
{
	u64 paf;
	u32 status;
	u8 function_and_status;

	if (wait_for_admin_queues_to_become_idle(h, ADMIN_SLEEP_TMO_MS,
						PQI_READY_FOR_IO))
		return -1;
	writeq(PQI_DELETE_ADMIN_QUEUES, &h->pqireg->process_admin_function);
	if (wait_for_admin_command_ack(h) == 0) {
		sop_free_admin_queues(h);
		return 0;
	}

	/* Try to get some clues about why it failed. */
	paf = readq(&h->pqireg->process_admin_function);
	function_and_status = paf & 0xff;
	dev_warn(&h->pdev->dev,
		"Failed to delete admin queues: function_and_status = 0x%02x\n",
		function_and_status);
	if (function_and_status == 0)
		return -1;
	status = readl(&h->pqireg->pqi_device_status);
	dev_warn(&h->pdev->dev, "Device status = 0x%08x\n",
			status);

	dev_warn(&h->pdev->dev, "Device status = 0x%08x\n", status);
	return -1;
}

static int sop_setup_msix(struct sop_device *h)
{
	int i, err;

	struct msix_entry msix_entry[MAX_TOTAL_QUEUE_PAIRS];

	h->nr_queue_pairs = (num_online_cpus() + 1);
	if (h->nr_queue_pairs > MAX_TOTAL_QUEUE_PAIRS)
		h->nr_queue_pairs = MAX_TOTAL_QUEUE_PAIRS;

	/*
	 * Set up (h->nr_queue_pairs - 1) msix vectors. -1 because
	 * outbound admin queue shares with io queue 0
	 */
	for (i = 0; i < h->nr_queue_pairs - 1; i++) {
		msix_entry[i].vector = 0;
		msix_entry[i].entry = i;
	}

	if (!pci_find_capability(h->pdev, PCI_CAP_ID_MSIX))
		goto default_int_mode;
	/* TODO: fall back if we get fewer msix vectors */
	err = pci_enable_msix(h->pdev, msix_entry, h->nr_queue_pairs - 1);
	if (err > 0)
		h->nr_queue_pairs = err;

	if (err == 0) {
		for (i = 0; i < h->nr_queue_pairs; i++) {
			/* vid makes admin q share with io q 0 */
			int vid = i ? i - 1 : 0; 
			h->qinfo[i].msix_entry = msix_entry[vid].entry;
			h->qinfo[i].msix_vector = msix_entry[vid].vector;
			/*
			dev_warn(&h->pdev->dev, "q[%d] msix_entry[%d] = %d\n",
				i, vid, msix_entry[vid].vector);
			*/
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

static void free_request(struct sop_device *h, u8 queue_pair_index, u16 request_id);

static void sop_complete_bio(struct sop_device *h, struct queue_info *qinfo,
				struct sop_request *r)
{
	struct sop_cmd_response *scr;
	u16 sense_data_len;
	u16 response_data_len;
	u8 xfer_result;
	u32 data_xferred;
	enum dma_data_direction dma_dir;
	struct scatterlist *sgl;
	int result;

	sop_rem_timeout(qinfo, r->tmo_slot);
	sgl = &qinfo->sgl[(r->request_id & 0x0ff) * MAX_SGLS];

	if (bio_data_dir(r->bio) == WRITE)
		dma_dir = DMA_TO_DEVICE;
	else
		dma_dir = DMA_FROM_DEVICE;
	/* undo the DMA mappings */
	dma_unmap_sg(&h->pdev->dev, sgl, r->num_sg, dma_dir);

	result = 0;

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
			/* Nowhere to transfer sense data in block */
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

	case SOP_RESPONSE_INTERNAL_CMD_FAIL_IU_TYPE:
		result = -EIO;
		break;

	default:
		result = -EIO;
		dev_warn(&h->pdev->dev, "got UNKNOWN response type...\n");
		break;
	}

        bio_endio(r->bio, result);
	free_request(h, qinfo_to_qid(qinfo), r->request_id);
}

int sop_msix_handle_ioq(struct queue_info *q)
{
	u16 request_id;
	u8 iu_type;
	int rc;
	struct sop_device *h = q->h;
	int ncmd=0;
	struct sop_request *r;

	if (pqi_from_device_queue_is_empty(q->oq)) {
		return IRQ_NONE;
	}

	r = q->oq->cur_req;
	do {
		if (r == NULL) {
			/* Receiving completion of a new request */ 
			iu_type = pqi_peek_ui_type_from_device(q->oq);
			request_id = pqi_peek_request_id_from_device(q->oq);
			r = q->oq->cur_req = &q->request[request_id & 0x00ff];
			r->request_id = request_id;
			r->response_accumulated = 0;
		}
		rc = pqi_dequeue_from_device(q->oq,
				&r->response[r->response_accumulated]); 
		ncmd++;
		if (rc) { /* queue is empty */
			dev_warn(&h->pdev->dev, "=-=-=- io OQ[%hhu] PI %d CI %d empty(rc=%d)\n", 
				q->oq->queue_id, *(q->oq->pi), q->oq->unposted_index, rc);
			break;
		}
		r->response_accumulated += q->oq->element_size;
		if (sop_response_accumulated(r)) {
			q->oq->cur_req = NULL;
			wmb();
			if (likely(r->bio)) {
				sop_complete_bio(h, q, r);
				r = NULL;
			} else {
				if (likely(r->waiting)) {
					complete(r->waiting);
					r = NULL;
				} else {
					dev_warn(&h->pdev->dev, "r->bio and r->waiting both null\n");
				}
			}
			atomic_dec(&h->cmd_pending);
			atomic_dec(&q->cur_qdepth);
			pqi_notify_device_queue_read(q->oq);
		}
		else
			dev_warn(&h->pdev->dev, "Multiple entry completion Q[%d] CI %d\n",
				q->oq->queue_id, q->oq->unposted_index);
	} while (!pqi_from_device_queue_is_empty(q->oq));

	return IRQ_HANDLED;
}

int sop_msix_handle_adminq(struct queue_info *q)
{
	u8 iu_type;
	u16 request_id;
	int rc;

	if (pqi_from_device_queue_is_empty(q->oq))
		return IRQ_NONE;

	do {
		struct sop_request *r = q->oq->cur_req;

		if (r == NULL) {
			/* Receiving completion of a new request */ 
			iu_type = pqi_peek_ui_type_from_device(q->oq);
			request_id = pqi_peek_request_id_from_device(q->oq);
			r = q->oq->cur_req = &q->request[request_id & 0x00ff];
			r->response_accumulated = 0;
		}
		rc = pqi_dequeue_from_device(q->oq, &r->response[r->response_accumulated]); 
		if (rc)
			break;
		r->response_accumulated += q->oq->element_size;
		if (sop_response_accumulated(r)) {
			q->oq->cur_req = NULL;
			wmb();
			complete(r->waiting);
			pqi_notify_device_queue_read(q->oq);
		}
	} while (!pqi_from_device_queue_is_empty(q->oq));

	return IRQ_HANDLED;
}

irqreturn_t sop_ioq_msix_handler(int irq, void *devid)
{
	struct queue_info *q = devid;
	int ret;

	spin_lock(&q->oq->qlock);
	ret = sop_msix_handle_ioq(q);
	spin_unlock(&q->oq->qlock);

	return ret;
}

irqreturn_t sop_adminq_msix_handler(int irq, void *devid)
{
	struct queue_info *q = devid;
	int ret;

	spin_lock(&q->oq->qlock);
	ret = sop_msix_handle_adminq(q);
	spin_unlock(&q->oq->qlock);

	return ret;
}

static void sop_irq_affinity_hints(struct sop_device *h)
{
	int i, cpu, ret;

	cpu = cpumask_first(cpu_online_mask);
	ret = irq_set_affinity_hint(h->qinfo[0].msix_vector, get_cpu_mask(cpu));

	for (i = 1; i < h->nr_queue_pairs; i++) {
		ret = irq_set_affinity_hint(h->qinfo[i].msix_vector,
					get_cpu_mask(cpu));
		cpu = cpumask_next(cpu, cpu_online_mask);
	}
}

static int sop_request_irq(struct sop_device *h, int queue_index,
				irq_handler_t msix_handler)
{
	int rc;

	rc = request_irq(h->qinfo[queue_index].msix_vector, msix_handler,
				IRQF_SHARED, h->devname, &h->qinfo[queue_index]);
	if (rc != 0)
		dev_warn(&h->pdev->dev, "Request_irq failed, queue_index = %d\n",
				queue_index);
	return rc;
}

static int sop_request_io_irqs(struct sop_device *h,
				irq_handler_t msix_handler)
{
	int i;

	for (i = 1; i < h->nr_queue_pairs; i++) {
		if (sop_request_irq(h, i, msix_handler))
			goto irq_fail;
	}
	sop_irq_affinity_hints(h);
	return 0;

irq_fail:
	/* Free all the irqs already allocated */
	while (--i >= 0)
		free_irq(h->qinfo[i].msix_vector, &h->qinfo[i]);
	return -1;
}

static void sop_free_irq(struct sop_device *h, int qinfo_ind)
{
	int vector;

	vector = h->qinfo[qinfo_ind].msix_vector; 
	irq_set_affinity_hint(vector, NULL);
	free_irq(vector, &h->qinfo[qinfo_ind]);
}

static void sop_free_irqs_and_disable_msix(
		struct sop_device *h)
{
	int i;

	for (i = 0; i < h->nr_queue_pairs; i++) {
		sop_free_irq(h, i);
	}
#ifdef CONFIG_PCI_MSI
	if (h->intr_mode == INTR_MODE_MSIX && h->pdev->msix_enabled)
		pci_disable_msix(h->pdev);
#endif /* CONFIG_PCI_MSI */
}

/* FIXME: maybe there's a better way to do this */
static u16 alloc_request(struct sop_device *h, u8 queue_pair_index)
{
	u16 rc;

	struct queue_info *qinfo = &h->qinfo[queue_pair_index];

	/* I am encoding q number in high bight of request id */
	BUG_ON(qinfo->qdepth > MAX_CMDS);
	BUG_ON(queue_pair_index > 127); /* high bit reserved for error reporting */

        do {
                rc = (u16) find_first_zero_bit(qinfo->request_bits, qinfo->qdepth);
                if (rc >= qinfo->qdepth-1) {
			/*
			int i, lim;

			dev_warn(&h->pdev->dev, "Q[%d] alloc_request failed. Bit=%d Qdepth=%d.\n",
				q, rc, qinfo->qdepth);
			lim = BITS_TO_LONGS(qinfo->qdepth) + 1;
			for (i=0; i < lim; i++)
				dev_warn(&h->pdev->dev, "Bits [%02d]: %08lx\n", i, 
					qinfo->request_bits[i]);
			*/
                        return (u16) -EBUSY;
		}
        } while (test_and_set_bit((int) rc, qinfo->request_bits));

	return rc | (queue_pair_index << 8);
}

static void free_request(struct sop_device *h, u8 queue_pair_index, u16 request_id)
{
	if((request_id >> 8) != queue_pair_index) {
		dev_warn(&h->pdev->dev, "Non matching QID %x in request id 0x%x.\n",
			queue_pair_index, request_id);
		return;

	}
	if((request_id & 0x00ff) >= h->qinfo[queue_pair_index].qdepth) {
		dev_warn(&h->pdev->dev, "Q[%d] Request id %d exceeds qdepth %d.\n",
			queue_pair_index, request_id, h->qinfo[queue_pair_index].qdepth);
		return;
	}
	clear_bit(request_id & 0x00ff, h->qinfo[queue_pair_index].request_bits);
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
	DECLARE_COMPLETION_ONSTACK(wait);

	request = &h->qinfo[0].request[request_id & 0x00ff];
	request->waiting = &wait;
	request->response_accumulated = 0;
	pqi_notify_device_queue_written(h->qinfo[0].iq);
	wait_for_completion(&wait);
}

static void send_sop_command(struct sop_device *h, struct queue_info *qinfo,
				u16 request_id)
{
	struct sop_request *sopr;
	DECLARE_COMPLETION_ONSTACK(wait);

	sopr = &qinfo->request[request_id & 0x00ff];
	memset(sopr, 0, sizeof(*sopr));
	sopr->request_id = request_id;
	sopr->waiting = &wait;
	sopr->response_accumulated = 0;
	atomic_inc(&qinfo->cur_qdepth);
	atomic_inc(&h->cmd_pending);
	pqi_notify_device_queue_written(qinfo->iq);
	put_cpu();
	wait_for_completion(&wait);
}

static int sop_create_io_queue(struct sop_device *h, struct queue_info *q,
				int queue_pair_index, int direction)
{
	struct pqi_device_queue *aq = h->qinfo[0].iq;
	struct pqi_device_queue *ioq;
	struct pqi_create_operational_queue_request *r;
	int request_id;
	volatile struct pqi_create_operational_queue_response *resp;
	__iomem u16 *pi_or_ci;

	if (direction == PQI_DIR_FROM_DEVICE) {
		ioq = q->oq;
	} else {
		ioq = q->iq;
	}
	spin_lock_init(&ioq->index_lock);
	spin_lock_init(&ioq->qlock);
	r = pqi_alloc_elements(aq, 1);
	request_id = alloc_request(h, 0);
	if (request_id < 0) {
		dev_warn(&h->pdev->dev, "Requests exhausted for create Q #%d\n",
			queue_pair_index);
		goto bail_out;
	}
	fill_create_io_queue_request(h, r, ioq,
					direction == PQI_DIR_TO_DEVICE,
					request_id, q->msix_entry);
	send_admin_command(h, request_id);
	resp = (volatile struct pqi_create_operational_queue_response *)
		h->qinfo[0].request[request_id & 0x00ff].response;	
	if (resp->status != 0) {
		dev_warn(&h->pdev->dev, "Failed to create up OQ #%d\n",
			queue_pair_index);
		free_request(h, 0, request_id);
		goto bail_out;
	}

	pi_or_ci = ((void *) h->pqireg) + le64_to_cpu(resp->index_offset);
	if (direction == PQI_DIR_TO_DEVICE)
		pqi_device_queue_init(ioq, pi_or_ci, ioq->ci);
	else
		pqi_device_queue_init(ioq, ioq->pi, pi_or_ci);
	free_request(h, 0, request_id);
	return 0;
bail_out:
	return -1;
}

static void sop_free_io_queues(struct sop_device *h)
{
	int i;

	for (i = 1; i < h->nr_queue_pairs; i++) {
		struct queue_info *qinfo = &h->qinfo[i];

		pqi_device_queue_free(h, qinfo->iq);
		pqi_device_queue_free(h, qinfo->oq);
		pqi_iq_buffer_free(h, qinfo);
	}
}

static int sop_setup_io_queue_pairs(struct sop_device *h)
{
	int i;

	/* From 1, not 0, to skip admin oq, which was already set up */
	for (i = 1; i < h->nr_queue_pairs; i++) {
		if (pqi_device_queue_alloc(h, &h->qinfo[i].oq,
				IQ_NELEMENTS, IQ_IU_SIZE / 16, PQI_DIR_FROM_DEVICE, i))
			goto bail_out;
		if (pqi_device_queue_alloc(h, &h->qinfo[i].iq,
				OQ_NELEMENTS, OQ_IU_SIZE / 16, PQI_DIR_TO_DEVICE, i))
			goto bail_out;
		if (pqi_iq_data_alloc(h, &h->qinfo[i]))
			goto bail_out;
		if (sop_create_io_queue(h, &h->qinfo[i], i, PQI_DIR_FROM_DEVICE))
			goto bail_out;
		if (sop_create_io_queue(h, &h->qinfo[i], i, PQI_DIR_TO_DEVICE))
			goto bail_out;
	}
	return 0;

bail_out:
	sop_free_io_queues(h);
	return -1;
}

static int sop_delete_io_queue(struct sop_device *h, int qpindex, int to_device)
{
	struct pqi_delete_operational_queue_request *r;
	struct pqi_device_queue *aq = h->qinfo[0].iq;
	u16 request_id;
	volatile struct pqi_delete_operational_queue_response *resp;
	u16 qid;
	int err=0;

	/* Check to see if the Admin queue is ready for taking commands */
	if (wait_for_admin_queues_to_become_idle(h, ADMIN_SLEEP_TMO_MS,
							PQI_READY_FOR_IO))
		return -ENODEV;

	r = pqi_alloc_elements(aq, 1);
	request_id = alloc_request(h, 0);
	if (request_id < 0) {
		dev_warn(&h->pdev->dev, "Requests unexpectedly exhausted\n");
		err = -ENOMEM;
		goto bail_out;
	}
	qid = qpindex_to_qid(qpindex, to_device);
	fill_delete_io_queue_request(h, r, qid, to_device, request_id);
	send_admin_command(h, request_id);
	resp = (volatile struct pqi_delete_operational_queue_response *)
		h->qinfo[0].request[request_id & 0x00ff].response;	
	if (resp->status != 0) {
		dev_warn(&h->pdev->dev, "Failed to tear down queue #%d (to=%d)\n",
			qpindex, to_device);
		err = -EIO;
	}

	free_request(h, 0, request_id);

bail_out:
	return err;
}

static int sop_delete_io_queues(struct sop_device *h)
{
	int i;

	for (i = 1; i < h->nr_queue_pairs; i++) {
		if (sop_delete_io_queue(h, i, 1))
			break;
		if (sop_delete_io_queue(h, i, 0))
			break;
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

#define PQI_RESET_ACTION_SHIFT 5
#define PQI_RESET_ACTION_MASK (0x07 << PQI_RESET_ACTION_SHIFT)
#define PQI_START_RESET (1 << PQI_RESET_ACTION_SHIFT)
#define PQI_SOFT_RESET (1)
#define PQI_START_RESET_COMPLETED (2 << PQI_RESET_ACTION_SHIFT)

static int sop_wait_for_host_reset_ack(struct sop_device *h, uint tmo_ms)
{
	u32 reset_register, prev;
	int count = 0;
	int tmo_iter = tmo_ms * 10;	/* Each iteration is 100 us */

	prev = -1;
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
		if ((reset_register & PQI_RESET_ACTION_MASK) ==
					PQI_START_RESET_COMPLETED)
			return 0;
		count++;
	} while (count < tmo_iter);

	return -1;
}

#define ADMIN_RESET_TMO_MS		5000
static int sop_init_time_host_reset(struct sop_device *h)
{
	dev_warn(&h->pdev->dev, "Resetting host\n");
	writel(PQI_START_RESET | PQI_SOFT_RESET, &h->pqireg->reset);
	if (sop_wait_for_host_reset_ack(h, ADMIN_RESET_TMO_MS))
		return -1;

	dev_warn(&h->pdev->dev, "Host reset initiated.\n");

	if (wait_for_admin_queues_to_become_idle(h, ADMIN_RESET_TMO_MS,
					PQI_READY_FOR_ADMIN_FUNCTION))
		return -1;

	dev_warn(&h->pdev->dev, "Host reset completed.\n");
	return 0;
}

static int __devinit sop_probe(struct pci_dev *pdev,
			const struct pci_device_id *pci_id)
{
	struct sop_device *h;
	u64 signature;
	int i, rc;

	dev_warn(&pdev->dev, SOP ": Found device: %04x:%04x/%04x:%04x\n",
			pdev->vendor, pdev->device,
			pdev->subsystem_vendor, pdev->subsystem_device);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	rc = sop_set_instance(h);
	if (rc) {
		dev_warn(&h->pdev->dev, "Cannot set driver instance\n");
		goto bail_alloc_drvdata;
	}

	/* Initialize device structure */
	for (i = 0; i < MAX_TOTAL_QUEUE_PAIRS; i++)
		h->qinfo[i].h = h;
	sprintf(h->devname, SOP"%d", h->instance);
	INIT_DELAYED_WORK(&h->dwork, NULL);
	h->flags = 0;

	h->pdev = pdev;
	pci_set_drvdata(pdev, h);

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_warn(&h->pdev->dev, "Unable to enable PCI device\n");
		goto bail_set_drvdata;
	}

	/* Enable bus mastering (pci_disable_device may disable this) */
	pci_set_master(h->pdev);

	rc = pci_request_regions(h->pdev, SOP);
	if (rc) {
		dev_err(&h->pdev->dev,
			"Cannot obtain PCI resources, aborting\n");
		goto bail_pci_enable;
	}

	h->pqireg = pci_ioremap_bar(pdev, 0);
	if (!h->pqireg) {
		rc = -ENOMEM;
		goto bail_request_regions;
	}
	rc = sop_init_time_host_reset(h);
	if (rc) {
		dev_err(&pdev->dev, "Failed to Reset Device\n");
		goto bail_remap_bar;
	}

	if (sop_set_dma_mask(pdev)) {
		dev_err(&pdev->dev, "Failed to set DMA mask\n");
		goto bail_remap_bar;
	}

	signature = readq(&h->pqireg->signature);
	if (memcmp(SOP_SIGNATURE_STR, &signature, sizeof(signature)) != 0) {
		dev_warn(&pdev->dev, "Device does not appear to be a PQI device\n");
		goto bail_remap_bar;
	}

	rc = sop_setup_msix(h);
	if (rc != 0)
		goto bail_remap_bar;

	rc = sop_alloc_admin_queues(h);
	if (rc)
		goto bail_enable_msix;

	rc = sop_create_admin_queues(h);
	if (rc)
		goto bail_admin_allocated;

	rc = sop_request_irq(h, 0, sop_adminq_msix_handler);
	if (rc != 0) {
	dev_warn(&h->pdev->dev, "Bailing out in probe - requesting IRQ[0]\n");
		goto bail_admin_created;
	}

	rc = sop_setup_io_queue_pairs(h);
	if (rc) {
		dev_warn(&h->pdev->dev, "Bailing out in probe - Creating i/o queues\n");
		goto bail_admin_irq;
	}

	rc = sop_request_io_irqs(h, sop_ioq_msix_handler);
	if (rc)
		goto bail_io_q_created;

	h->max_hw_sectors = 2048;	/* TODO: For now hard code it */
	rc = sop_get_disk_params(h);
	if (rc) {
		dev_warn(&h->pdev->dev, "Bailing out in probe - can't get disk param\n");
		goto bail_io_irq;
	}

	rc = sop_add_disk(h);
	if (rc) {
		dev_warn(&h->pdev->dev, "Bailing out in probe - Cannot add disk\n");
		goto bail_io_irq;
	}

	spin_lock(&dev_list_lock);
	list_add(&h->node, &dev_list);
	spin_unlock(&dev_list_lock);

	return 0;

bail_io_irq:
	for (i=1; i< h->nr_queue_pairs; i++)
		sop_free_irq(h, i);
bail_io_q_created:
	sop_delete_io_queues(h);
bail_admin_irq:
	sop_free_irq(h, 0);
bail_admin_created:
	sop_delete_admin_queues(h);
bail_admin_allocated:
	sop_free_admin_queues(h);
bail_enable_msix:
	pci_disable_msix(pdev);
bail_remap_bar:
	if (h && h->pqireg)
		iounmap(h->pqireg);
bail_request_regions:
	pci_release_regions(pdev);
bail_pci_enable:
	pci_disable_device(pdev);
bail_set_drvdata:
	pci_set_drvdata(pdev, NULL);
	sop_release_instance(h);
bail_alloc_drvdata:
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

	dev_warn(&pdev->dev, "Remove called.\n");
	h = pci_get_drvdata(pdev);
	cancel_delayed_work_sync(&h->dwork);
	sop_fail_all_outstanding_io(h);
	sop_remove_disk(h);
	sop_delete_io_queues(h);
	sop_free_irqs_and_disable_msix(h);
	sop_delete_admin_queues(h);
	if (h && h->pqireg)
		iounmap(h->pqireg);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	sop_release_instance(h);
	dev_warn(&pdev->dev, "Device Removed\n");
	kfree(h);
}

static void sop_shutdown(struct pci_dev *pdev)
{
	dev_warn(&pdev->dev, "Shutdown called.\n");
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
	unregister_blkdev(sop_major, SOP);
	kthread_stop(sop_thread);
	printk("%s: Driver unloaded\n", SOP);
}

static inline struct sop_device *bdev_to_hba(struct block_device *bdev)
{
	unsigned long *priv = bdev->bd_disk->private_data;
	return (struct sop_device *)priv;
}

static inline int find_sop_queue(struct sop_device *h, int cpu)
{
	return 1 + (cpu % (h->nr_queue_pairs - 1));
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
			   struct queue_info *qinfo)
{
	struct sop_limited_cmd_iu *r;
	struct sop_request *ser;
	enum dma_data_direction dma_dir;
	struct scatterlist *sgl;
	u16 request_id;
	int prev_index, num_sg;
	int nsegs;

	/* FIXME: Temporarily limit the outstanding FW commands to 128 */
	if (atomic_read(&h->cmd_pending) >= 128)
		return -EBUSY;

	request_id = alloc_request(h, qinfo_to_qid(qinfo));
	if (request_id == (u16) -EBUSY)
		return -EBUSY;

	r = pqi_alloc_elements(qinfo->iq, 1);
	if (IS_ERR(r)) {
		dev_warn(&h->pdev->dev, "SUBQ[%d] pqi_alloc_elements for bio %p returned %ld\n", 
			qinfo_to_qid(qinfo), bio, PTR_ERR(r));
		goto alloc_elem_fail;
	}
	nsegs = bio_phys_segments(h->rq, bio);

	r->iu_type = SOP_LIMITED_CMD_IU;
	r->compatible_features = 0;
	r->queue_id = cpu_to_le16(qinfo->oq->queue_id);
	r->work_area = 0;
	r->request_id = request_id;
	sgl = &qinfo->sgl[(request_id & 0x0ff) * MAX_SGLS];
	ser = &qinfo->request[request_id & 0x00ff];
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
			bio, qinfo_to_qid(qinfo));
		goto sg_map_fail;
	}
	atomic_inc(&qinfo->cur_qdepth);
	if (qinfo->max_qdepth < atomic_read(&qinfo->cur_qdepth))
		qinfo->max_qdepth = atomic_read(&qinfo->cur_qdepth);
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
	sop_scatter_gather(h, qinfo, num_sg, r, sgl, &ser->xfer_size);

	r->xfer_size = cpu_to_le32(ser->xfer_size);
	ser->tmo_slot = sop_add_timeout(qinfo, DEF_IO_TIMEOUT);

	/* Submit it to the device */
	writew(qinfo->iq->unposted_index, qinfo->iq->pi);

	return 0;

sg_map_fail:
	pqi_unalloc_elements(qinfo->iq, 1);

alloc_elem_fail:
	free_request(h, qinfo_to_qid(qinfo), request_id);
	return -EBUSY;
}

static void sop_queue_cmd(struct sop_wait_queue *wq, struct bio *bio)
{
	if (bio_list_empty(&wq->iq_cong))
		add_wait_queue(&wq->iq_full, &wq->iq_cong_wait);
	bio_list_add(&wq->iq_cong, bio);
}

static MRFN_TYPE sop_make_request(struct request_queue *q, struct bio *bio)
{
	struct sop_device *h = q->queuedata;
	int result;
	int cpu;
	int qpindex;
	struct queue_info *qinfo;
	struct sop_wait_queue *wq;

	/* Prepare the SOP IU and fire */
	cpu = get_cpu();

	/* Get the queues */
	qpindex = find_sop_queue(h, cpu);
	qinfo = &h->qinfo[qpindex];
	wq = qinfo->wq;
	BUG_ON(!wq);

	spin_lock_irq(&qinfo->iq->qlock);

	result = -EBUSY;
	if (bio_list_empty(&wq->iq_cong))
		/* Try to submit the command */
		result = sop_process_bio(h, bio, qinfo);

	if (unlikely(result))
		sop_queue_cmd(wq, bio);

	spin_unlock_irq(&qinfo->iq->qlock);

	put_cpu();

	return MRFN_RET;
}

static void fill_send_cdb_request(struct sop_limited_cmd_iu *r,
		u16 queue_id, u16 request_id, char *cdb, 
		dma_addr_t phy_addr, int data_len, u8 data_dir)
{
	int cdb_len;
	u16 sgl_size;

	memset(r, 0, sizeof(*r));

	/* Prepare the entry for SOP */
	r->iu_type = SOP_LIMITED_CMD_IU;
	r->compatible_features = 0;
	r->queue_id = cpu_to_le16(queue_id);
	r->work_area = 0;
	r->request_id = request_id;
	sgl_size = (u16) (sizeof(*r) - sizeof(r->sg[0]) * 2) - 4;
	if (data_len)
		/* We use only max one descriptor */
		sgl_size += sizeof(struct pqi_sgl_descriptor);	
	r->iu_length = cpu_to_le16(sgl_size);

	/* Prepare the CDB */
	cdb_len = COMMAND_SIZE(cdb[0]);
	memcpy(r->cdb, cdb, cdb_len);

	r->flags = data_dir;
	r->xfer_size = data_len;

	/* Prepare SG */
	r->sg[0].address = cpu_to_le64(phy_addr);
	r->sg[0].length = cpu_to_le32(data_len);
	r->sg[0].descriptor_type = PQI_SGL_DATA_BLOCK;
}

static int process_direct_cdb_response(struct sop_device *h, u8 opcode,
			struct queue_info *qinfo, u16 request_id)
{
	struct sop_request *sopr = &qinfo->request[request_id & 0x00ff];
	volatile struct sop_cmd_response *resp = 
		(volatile struct sop_cmd_response *)sopr->response;
	u16 status, sq;
	u8 iu_type;

	iu_type = resp->iu_type;
	status = resp->status;
	sq = resp->status_qualifier;
	free_request(h, qinfo_to_qid(qinfo), request_id);

	if (iu_type == SOP_RESPONSE_CMD_SUCCESS_IU_TYPE)
		return 0;

	if (iu_type != SOP_RESPONSE_CMD_RESPONSE_IU_TYPE) {
		dev_warn(&h->pdev->dev, "Unexpected IU type 0x%x in %s\n",
				resp->iu_type, __func__);
		return -1;
	}

	switch (status & STATUS_MASK) {
	case SAM_STAT_GOOD:
	case SAM_STAT_CONDITION_MET:
	case SAM_STAT_INTERMEDIATE:
	case SAM_STAT_INTERMEDIATE_CONDITION_MET:
		return 0;

	case SAM_STAT_BUSY:
	case SAM_STAT_RESERVATION_CONFLICT:
	case SAM_STAT_COMMAND_TERMINATED:
	case SAM_STAT_TASK_SET_FULL:
	case SAM_STAT_ACA_ACTIVE:
	case SAM_STAT_TASK_ABORTED:
		dev_warn(&h->pdev->dev, "Sync command cdb=0x%x failed with status 0x%x\n",
			opcode, status);
		return (status << 16);

	case SAM_STAT_CHECK_CONDITION:
		dev_warn(&h->pdev->dev, "Sync command cdb=0x%x Check Condition SQ 0x%x\n",
			opcode, sq);
		return ((status << 16) | sq);
	}

	dev_warn(&h->pdev->dev, "Sync command cdb=0x%x failed with unknown status 0x%x\n",
		opcode, status);
	return -1;
}

static int send_sync_cdb(struct sop_device *h, char *cdb, dma_addr_t phy_addr, 
			 int data_len, u8 data_dir)
{
	int queue_pair_index;	
	struct queue_info *qinfo;

	struct sop_limited_cmd_iu *r;
	int request_id, cpu;
	int retval = -EBUSY;

	cpu = get_cpu();
	queue_pair_index = find_sop_queue(h, cpu);
	qinfo = &h->qinfo[queue_pair_index];
	request_id = alloc_request(h, queue_pair_index);
	if (request_id < 0) {
		dev_warn(&h->pdev->dev, "%s: Failed to allocate request\n",
					__func__);
		retval = -EBUSY;
		goto sync_req_id_fail;
	}
	r = pqi_alloc_elements(qinfo->iq, 1);
	if (IS_ERR(r)) {
		dev_warn(&h->pdev->dev, "SUBQ[%d] pqi_alloc_elements for CDB 0x%x returned %ld\n", 
			queue_pair_index, cdb[0], PTR_ERR(r));
		goto sync_alloc_elem_fail;
	}
	fill_send_cdb_request(r, queue_pair_index * 2, request_id, cdb, phy_addr, 
				data_len, data_dir);

	send_sop_command(h, qinfo, request_id);
	return process_direct_cdb_response(h, cdb[0], qinfo, request_id);

sync_alloc_elem_fail:
	free_request(h, qinfo_to_qid(qinfo), request_id);

sync_req_id_fail:
	put_cpu();
	return retval;
}

#define	IO_SLEEP_INTERVAL_MIN	2000
#define	IO_SLEEP_INTERVAL_MAX	2500
#define TUR_MAX_RETRY_COUNT	10

#define	MAX_CDB_SIZE	16
static int sop_get_disk_params(struct sop_device *h)
{
	int ret;
	u8 cdb[MAX_CDB_SIZE];
	dma_addr_t phy_addr;
	void *vaddr;
	int size, total_size;
	u32 *data;
	int retry_count;

	/* 0. Allocate memory */
	total_size = 1024;
	vaddr = pci_alloc_consistent(h->pdev, total_size, &phy_addr);
	if (!vaddr)
		return -ENOMEM;

	/* 0.1. Send Inquiry */
	memset(cdb, 0, MAX_CDB_SIZE);
	cdb[0] = INQUIRY;		/* Rest all remains 0 */
	size = 36;
	data = (u32 *)vaddr;
	cdb[4] = size;
	ret = send_sync_cdb(h, cdb, phy_addr, size, SOP_DATA_DIR_FROM_DEVICE);
	if (ret != 0)
		goto disk_param_err;

	retry_count = 0;
sync_send_tur:
	/* 1. send TUR */
	memset(cdb, 0, MAX_CDB_SIZE);
	cdb[0] = TEST_UNIT_READY;
	ret = send_sync_cdb(h, cdb, 0, 0, SOP_DATA_DIR_NONE);
	if (((ret >> 16) == SAM_STAT_CHECK_CONDITION) && 
		(retry_count < TUR_MAX_RETRY_COUNT)) {
		msleep(IO_SLEEP_INTERVAL_MIN);
		/*  Retry */
		retry_count++;
		goto sync_send_tur;
	}

	if (ret)
		goto disk_param_err;

	/* 2. Send Read Capacity */
	memset(cdb, 0, MAX_CDB_SIZE);
	cdb[0] = READ_CAPACITY;		/* Rest all remains 0 */
	size = 2 * sizeof(u32);
	data = (u32 *)vaddr;
	ret = send_sync_cdb(h, cdb, phy_addr, size, SOP_DATA_DIR_FROM_DEVICE);
	if (ret != 0)
		goto disk_param_err;
	/* Process the Read Cap data */
	h->capacity = be32_to_cpu(data[0]);
	h->block_size = be32_to_cpu(data[1]);

	pci_free_consistent(h->pdev, total_size, vaddr, phy_addr);
	return 0;

disk_param_err:
	dev_warn(&h->pdev->dev, "Error Getting Disk param (CDB0=0x%x return status 0x%x)\n",
		cdb[0], ret);
	pci_free_consistent(h->pdev, total_size, vaddr, phy_addr);

	/* Set Capacity to 0 and continue as degraded */
	h->capacity = 0;
	h->block_size = 0x200;
	return 0;
}


static int sop_add_disk(struct sop_device *h)
{
	struct gendisk *disk;
	struct request_queue *rq;

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

	disk = alloc_disk(SOP_MINORS);
	if (!disk)
		goto out_free_queue;

	h->disk = disk;

	blk_queue_logical_block_size(rq, h->block_size);
	if (h->max_hw_sectors)
		blk_queue_max_hw_sectors(rq, h->max_hw_sectors);
	blk_queue_max_segments(rq, MAX_SGLS);

	disk->major = sop_major;
	disk->minors = SOP_MINORS;
	disk->first_minor = SOP_MINORS * h->instance;
	disk->fops = &sop_fops;
	disk->private_data = h;
	disk->queue = rq;
	disk->driverfs_dev = &h->pdev->dev;
	strcpy(disk->disk_name, h->devname);
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

static int sop_add_timeout(struct queue_info *q, uint timeout)
{
	int tmo_slot;

	tmo_slot = ((atomic_read(&q->tmo.cur_slot) + timeout) % MAX_SOP_TIMEOUT);
	atomic_inc(&q->tmo.time_slot[tmo_slot]);
	return tmo_slot;
}

static void sop_rem_timeout(struct queue_info *q, uint tmo_slot)
{
	if(atomic_read(&q->tmo.time_slot[tmo_slot]) == 0) {
		printk(KERN_ERR "Q[%d] TM slot[%d] count is 0, cur_slot=%d\n",
			qinfo_to_qid(q), tmo_slot, 
			atomic_read(&q->tmo.cur_slot));
		return;
	}
	atomic_dec(&q->tmo.time_slot[tmo_slot]);
}

static void sop_fail_cmd(struct queue_info *q, struct sop_request *r)
{
	/* Fill a fake error completion in r->response */
	r->response[0] = SOP_RESPONSE_INTERNAL_CMD_FAIL_IU_TYPE;
	r->response_accumulated = 1;

	/* Call complete bio with this parameter */
	sop_complete_bio(q->h, q, r);
}

/* To be called instead of sop_process_bio in case of abort */
static int sop_fail_bio(struct sop_device *h, struct bio *bio, 
			struct queue_info *q)
{
	/* Call complete bio with error */
	bio_endio(bio, -EIO);

	return 0;
}

#define SOP_ERR_NONE		0
#define	SOP_ERR_DEV_REM		-1
#define SOP_ERR_DEV_RESET	-2
/* Return 0 if no global reason found */
static int sop_device_error_state(struct sop_device *h)
{
	__iomem void *sig = &h->pqireg->signature;
	u64 signature = 0;
	u16 state = 0;

	/* Do not check HW if any action is pending */
	if (SOP_DEVICE_BUSY(h))
		return SOP_ERR_NONE;

	/* Detect Surprise removal */
	if (safe_readq(sig, &signature, &h->pqireg->signature)) {
		return SOP_ERR_DEV_REM;
	}

	if (memcmp(SOP_SIGNATURE_STR, &signature, sizeof(signature)) != 0) {
		return SOP_ERR_DEV_REM;
	}

	/* Detect for surprise reset */
	if (safe_readw(sig, &state, &h->pqireg->pqi_device_status)) {
		return SOP_ERR_DEV_REM;
	}
	if (state == PQI_ERROR) {
		h->flags |= SOP_FLAGS_MASK_DO_RESET;
		return SOP_ERR_DEV_RESET;
	}


	/* No other global reason found */
	return SOP_ERR_NONE;
}

/* tmo_slot is negative if all commands are to be failed */
static int sop_timeout_queued_cmds(struct queue_info *q, int tmo_slot, int action)
{
	int rqid, maxid;
	int count = 0;
	struct sop_request *ser;

	/* Update time slot to include all commands for error cases */
	if (action != SOP_ERR_NONE)
		tmo_slot = -1;

	/* Process timeout by linear search of request bits in qinfo*/
	maxid = q->qdepth - 1;
	for_each_set_bit(rqid, q->request_bits, maxid) {
		ser = &q->request[rqid & 0x00ff];
		if ((tmo_slot > 0) && (ser->tmo_slot != tmo_slot))
			continue;

		/* Found a timed out command, process timeout */
		count++;

		switch (action) {
		case SOP_ERR_DEV_REM:
			sop_fail_cmd(q, ser);
			break;

		case SOP_ERR_NONE:
			/* TODO: Need to abort this command */
			/* For now, fall thru and reset as below */
			q->h->flags |= SOP_FLAGS_MASK_DO_RESET;

		case SOP_ERR_DEV_RESET:
			/*
			 * Requeue this command and 
			 * reset the controller at end 
			 */
			sop_queue_cmd(q->wq, ser->bio);
			break;
		}
	}

	return count;
}

static void sop_timeout_ios(struct queue_info *q, int action)
{
	int cmd_pend;
	u8 cur_slot;

	/* Update the current slot */
	cur_slot = (atomic_read(&q->tmo.cur_slot) + 1) % MAX_SOP_TIMEOUT;
	atomic_set(&q->tmo.cur_slot, cur_slot);

	/* Check if the curremnt slot is empty, otherwise... */
	cmd_pend = atomic_read(&q->tmo.time_slot[cur_slot]);
	if (cmd_pend == 0)
		return;

	dev_warn(&q->h->pdev->dev, "SOP timeout!! %d cmds left on slot[%d]\n",
				cmd_pend, cur_slot);

	/* Process timeout */
	cmd_pend -= sop_timeout_queued_cmds(q, cur_slot, action);

	if ((action == SOP_ERR_NONE) && cmd_pend)
		dev_warn(&q->h->pdev->dev, "SOP timeout!! Cannot account for %d cmds\n",
				cmd_pend);
}

static void sop_timeout_admin(struct sop_device *h)
{
}

static void sop_resubmit_waitq(struct queue_info *qinfo, int fail)
{
	struct sop_wait_queue *wq;
	struct sop_device *h;
	int ret;
	int (*bio_process)(struct sop_device *h, struct bio *bio,
			   struct queue_info *qinfo);

	h = qinfo->h;
	wq = qinfo->wq;
	if (!wq) {
		dev_warn(&h->pdev->dev, "sop_resubmit_waitq: wq null for SubmitQ[%d]!\n",
					qinfo->iq->queue_id);
		return;
	}

	if (fail)
		bio_process = sop_fail_bio;
	else
		bio_process = sop_process_bio;
	spin_lock_irq(&qinfo->iq->qlock);
	while (bio_list_peek(&wq->iq_cong)) {
		struct bio *bio = bio_list_pop(&wq->iq_cong);

		/* dev_warn(&h->pdev->dev, "sop_resubmit_waitq: proc bio %p Q[%d], PI %d CI %d!\n",
			bio, qinfo->iq->queue_id, qinfo->iq->unposted_index, *(qinfo->iq->ci)); */

		if ((ret = bio_process(h, bio, qinfo))) {
			/* dev_warn(&h->pdev->dev, "sop_resubmit_waitq: Error %d for [%d]!\n",
						ret, qinfo->iq->queue_id); */
			bio_list_add_head(&wq->iq_cong, bio);
			break;
		}
		if (bio_list_empty(&wq->iq_cong))
			remove_wait_queue(&wq->iq_full, &wq->iq_cong_wait);

		/* dev_warn(&h->pdev->dev, "sop_resubmit_waitq: done bio %p Q[%d], PI %d CI %d!\n",
			bio, qinfo->iq->queue_id, qinfo->iq->unposted_index, *(qinfo->iq->ci)); */
	}
	spin_unlock_irq(&qinfo->iq->qlock);
}

static void sop_requeue_all_outstanding_io(struct sop_device *h)
{
	int i;
	struct queue_info *q;

	/* Io Queue */
	for (i = 1; i < h->nr_queue_pairs; i++) {
		q = &h->qinfo[i];

		if (!q)
			continue;
		
		spin_lock_irq(&q->oq->qlock);

		/* Process any pending ISR */
		sop_msix_handle_ioq(q);
		/* Requeue all outstanding commands given to this HW queue */
		sop_timeout_queued_cmds(q, -1, SOP_ERR_DEV_RESET);

		spin_unlock_irq(&q->oq->qlock);
	}
}

static void sop_reinit_all_ioq(struct sop_device *h)
{
	int i;
	struct queue_info *q;

	/* Io Queue */
	for (i = 1; i < h->nr_queue_pairs; i++) {
		q = &h->qinfo[i];

		*(q->oq->pi) = q->oq->unposted_index = 0;
		*(q->iq->ci) = q->iq->unposted_index = 0;
		q->iq->local_pi = 0;
	}
}

#define	MAX_RESET_COUNT	3

/* Run time controller reset */
static void sop_reset_controller(struct work_struct *work)
{
	int rc;
	int i;
	int reset_count=0;
	struct sop_device *h;

	h =  container_of(work, struct sop_device, dwork.work);

start_reset:
	dev_warn(&h->pdev->dev, "%s: Starting Reset\n", h->devname);
	rc = sop_init_time_host_reset(h);
	if (rc)
		goto reset_err;

	sop_requeue_all_outstanding_io(h);
	sop_reinit_all_ioq(h);

	rc = sop_create_admin_queues(h);
	if (rc)
		goto reset_err;

	dev_warn(&h->pdev->dev, "Re creating %d I/O queue pairs\n", 
		h->nr_queue_pairs-1);
	/* Re create all the queue pairs */
	for (i = 1; i < h->nr_queue_pairs; i++) {
		if (sop_create_io_queue(h, &h->qinfo[i], i, PQI_DIR_FROM_DEVICE))
			goto reset_err;
		if (sop_create_io_queue(h, &h->qinfo[i], i, PQI_DIR_TO_DEVICE))
			goto reset_err;
	}

	dev_warn(&h->pdev->dev, "I/O queue created - Resubmitting pending commands\n");
	/* Next: sop_resubmit_waitq for all Q */
	for (i = 1; i < h->nr_queue_pairs; i++) {
		sop_resubmit_waitq(&h->qinfo[i], false);
	}

	dev_warn(&h->pdev->dev, "Reset Complete\n");
	clear_bit(SOP_FLAGS_BITPOS_RESET_PEND, (volatile unsigned long *)&h->flags);
	return;

reset_err:
	reset_count++;
	dev_warn(&h->pdev->dev, "Reset failed, attempt #%d of %d\n", reset_count,
		MAX_RESET_COUNT);
	if (reset_count < MAX_RESET_COUNT)
		goto start_reset;

	/* TODO: Error handling: Mark Dead? */
	clear_bit(SOP_FLAGS_BITPOS_RESET_PEND, (volatile unsigned long *)&h->flags);
	return;

}

static void sop_process_driver_debug(struct sop_device *h)
{
	if (sop_dbg_lvl == 1) {
		int i;

		sop_dbg_lvl = 0;
		printk("@@@@ Total CMD Pending= %d Max = %d @@@@\n", 
				atomic_read(&h->cmd_pending),
				h->max_cmd_pending);
		for (i = 1; i < h->nr_queue_pairs; i++) {
			printk("        OQ depth[%d] = %d, Max %d\n", i,
				atomic_read(&h->qinfo[i].cur_qdepth),
				h->qinfo[i].max_qdepth);
		}
	}

	if (sop_dbg_lvl == 2) {
		/* Reset the level */
		sop_dbg_lvl = 0;

		printk("@@@@ Trying to ASSERT the FW...\n");
		/* Perform illegal write to BAR */
		writel(0x10, &h->pqireg->error_data);
	}

	if (sop_dbg_lvl == 3) {
		/* Reset the level */
		sop_dbg_lvl = 0;

		h->flags |= SOP_FLAGS_MASK_DO_RESET;
	}

}

static void sop_process_dev_timer(struct sop_device *h)
{
	int i;
	struct queue_info *q = &h->qinfo[0];
	int action;

	/* Decide if there is any global errofr */
	action = sop_device_error_state(h);

	/* Admin queue */
	spin_lock_irq(&q->oq->qlock);
	sop_msix_handle_adminq(q);
	sop_timeout_admin(h);
	spin_unlock_irq(&q->oq->qlock);

	/* Io Queue */
	for (i = 1; i < h->nr_queue_pairs; i++) {
		q = &h->qinfo[i];

		if (!q)
			continue;
		
		spin_lock_irq(&q->oq->qlock);

		/* Process any pending ISR */
		sop_msix_handle_ioq(q);

		/* Handle errors */
		sop_timeout_ios(q, action);
		spin_unlock_irq(&q->oq->qlock);

		if (!SOP_DEVICE_BUSY(h)) {
			/* Process wait queue */
			sop_resubmit_waitq(q, false);
		}
	}

	sop_process_driver_debug(h);

	if ((h->flags & SOP_FLAGS_MASK_DO_RESET)) {

		/* Reset is being handled - clear the flag */
		h->flags &= ~SOP_FLAGS_MASK_DO_RESET;

		if (!test_and_set_bit(SOP_FLAGS_BITPOS_RESET_PEND,
			(volatile unsigned long *)&h->flags)) {

			/* Reset the controller */
			printk("Scheduling a reset for the controller...\n");
			PREPARE_DELAYED_WORK(&h->dwork, sop_reset_controller);
			schedule_delayed_work(&h->dwork, HZ);
		}
	}
}

static int sop_thread_proc(void *data)
{
	struct sop_device *h;

	while (!kthread_should_stop()) {
		__set_current_state(TASK_RUNNING);
		spin_lock(&dev_list_lock);
		list_for_each_entry(h, &dev_list, node)
			sop_process_dev_timer(h);
		spin_unlock(&dev_list_lock);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);
	}

	return 0;
}

static void sop_fail_all_outstanding_io(struct sop_device *h)
{
	int i;
	struct queue_info *q = &h->qinfo[0];

	spin_lock_irq(&q->oq->qlock);
	/* TODO: Handle Admin commands */
	spin_unlock_irq(&q->oq->qlock);

	/* Io Queue */
	for (i = 1; i < h->nr_queue_pairs; i++) {
		q = &h->qinfo[i];

		if (!q)
			continue;
		
		spin_lock_irq(&q->oq->qlock);
		/* Process any pending ISR */
		sop_msix_handle_ioq(q);
		/* Fail all outstanding commands given to HW queue */
		sop_timeout_queued_cmds(q, -1, SOP_ERR_DEV_REM);
		spin_unlock_irq(&q->oq->qlock);

		/* Fail all commands waiting in internal queue */
		sop_resubmit_waitq(q, true);
	}
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
	VERIFY_OFFSET(device_error, 0x80);
	VERIFY_OFFSET(error_data, 0x88);
	VERIFY_OFFSET(reset, 0x90);
	VERIFY_OFFSET(power_action, 0x94);

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

