/*
 *    SCSI Express driver
 *    Copyright 2012 Hewlett-Packard Development Company, L.P.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; version 2 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *    NON INFRINGEMENT.  See the GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
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
#include <linux/version.h>
#include <linux/completion.h>
#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>

#include "scsi_express_kernel_compat.h"
#include "scsi_express.h"

#define DRIVER_VERSION "1.0.0"
#define DRIVER_NAME "SCSI Express (v " DRIVER_VERSION ")"
#define SCSI_EXPRESS "SCSI Express"

MODULE_AUTHOR("Hewlett-Packard Company");
MODULE_DESCRIPTION("SCSI Express driver" DRIVER_VERSION);
MODULE_SUPPORTED_DEVICE("SCSI Express devices");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#ifndef PCI_VENDOR_SANDISK
#define PCI_VENDOR_SANDISK 0x15b7
#endif

DEFINE_PCI_DEVICE_TABLE(scsi_express_id_table) = {
	{ PCI_VENDOR_SANDISK, 0x0012, PCI_VENDOR_SANDISK, 0x0000 },
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, scsi_express_id_table);

static int controller_num;

static int scsi_express_queuecommand(struct Scsi_Host *h, struct scsi_cmnd *sc);
static int scsi_express_change_queue_depth(struct scsi_device *sdev,
        int qdepth, int reason);
static int scsi_express_abort_handler(struct scsi_cmnd *sc);
static int scsi_express_device_reset_handler(struct scsi_cmnd *sc);
static int scsi_express_slave_alloc(struct scsi_device *sdev);
static void scsi_express_slave_destroy(struct scsi_device *sdev);
static int scsi_express_compat_ioctl(struct scsi_device *dev, int cmd, void *arg);
static int scsi_express_ioctl(struct scsi_device *dev, int cmd, void *arg);

static struct scsi_host_template scsi_express_template = {
	.module				= THIS_MODULE,
	.name				= DRIVER_NAME,
	.proc_name			= DRIVER_NAME,
	.queuecommand			= scsi_express_queuecommand,
	.change_queue_depth		= scsi_express_change_queue_depth,
	.this_id			= -1,
	.use_clustering			= ENABLE_CLUSTERING,
	.eh_abort_handler		= scsi_express_abort_handler,
	.eh_device_reset_handler	= scsi_express_device_reset_handler,
	.ioctl				= scsi_express_ioctl,
	.slave_alloc			= scsi_express_slave_alloc,
	.slave_destroy			= scsi_express_slave_destroy,
#ifdef CONFIG_COMPAT
	.compat_ioctl			= scsi_express_compat_ioctl,
#endif
#if 0
	.sdev_attrs			= scsi_express_sdev_attrs,
	.host_attrs			= scsi_express_host_attrs,
#endif
	.max_sectors			= 8192, /* FIXME: is this correct? */
};

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

static int allocate_q_request_buffers(struct queue_info *q,
	int nbuffers, int buffersize)
{
	BUG_ON(nbuffers > 256);
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

static void free_all_q_request_buffers(struct scsi_express_device *h)
{
	int i;
	for (i = 0; i < h->nr_queues; i++)
		free_q_request_buffers(&h->qinfo[i]);
}

static int pqi_device_queue_array_alloc(struct scsi_express_device *h,
		struct pqi_device_queue **xq, int num_queues,
		u16 n_q_elements, u8 q_element_size_over_16,
		int queue_direction, int starting_queue_id)
{
	void *vaddr = NULL;
	dma_addr_t dhandle;
	int i, nqs_alloced = 0, err = 0;
	int total_size = (n_q_elements * q_element_size_over_16 * 16) +
				sizeof(u64);

	dev_warn(&h->pdev->dev, "Allocating %d queues %s device...\n", 
		num_queues,
		queue_direction == PQI_DIR_TO_DEVICE ? "TO" : "FROM");

	err = -ENOMEM;
	*xq = kzalloc(sizeof(**xq) * num_queues, GFP_KERNEL);
	if (!*xq)
		goto bailout;
	vaddr = pci_alloc_consistent(h->pdev, total_size * num_queues, &dhandle);
	if (!vaddr)
		goto bailout;

	for (i = 0; i < num_queues; i++) {
		int q = i + starting_queue_id;
		if (allocate_q_request_buffers(&h->qinfo[q], n_q_elements,
				sizeof(struct scsi_express_request)))
			goto bailout;
	}

	dev_warn(&h->pdev->dev, "Memory alloc'ed.\n");
	err = 0;

	for (i = 0; i < num_queues; i++) {
		(*xq)[i].queue_vaddr = vaddr;
		(*xq)[i].dhandle = dhandle;
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
	return nqs_alloced;

bailout:
	kfree(*xq);
	free_all_q_request_buffers(h);
	if (vaddr)
		pci_free_consistent(h->pdev, total_size, vaddr, dhandle);
	return err;
}

static void pqi_device_queue_init(struct pqi_device_queue *q,
		__iomem void *queue_vaddr, __iomem u16 *pi,
		__iomem u16 *ci,
		u16 element_size, u8 nelements,
		dma_addr_t dhandle)
{
	q->queue_vaddr = queue_vaddr;
	q->pi = pi;
	q->ci = ci;
	q->unposted_index = 0;
	q->element_size = element_size;
	q->nelements = nelements;
	q->dhandle = dhandle;
}

static int pqi_to_device_queue_is_full(struct pqi_device_queue *q,
				int nelements)
{
	u16 qci;
	u32 nfree;

	/* FIXME: shouldn't have to read this every time
	 * should be able to cache it */
	qci = readw(q->ci);

	if (q->unposted_index > qci)
		nfree = q->nelements - q->unposted_index + qci;
	else
		nfree = qci - q->unposted_index - 1;
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

	if (pqi_to_device_queue_is_full(q, nelements))
		return ERR_PTR(-ENOMEM);

	/* If the requested number of elements would wrap around the
	 * end of the ring buffer, insert NULL IUs to the end of the
	 * ring buffer.  This simplifies the code which has to fill
	 * in the IUs as it doesn't have to deal with wrapping
	 */
	if (q->nelements - q->unposted_index < nelements) {
		int extra_elements = q->nelements - q->unposted_index;
		if (pqi_to_device_queue_is_full(q, nelements + extra_elements))
			return ERR_PTR(-ENOMEM);
		p = q->queue_vaddr + q->unposted_index * q->element_size;
		memset(p, 0, (q->nelements - q->unposted_index) *
						q->element_size);
		q->unposted_index = 0;
	}
	p = q->queue_vaddr + q->unposted_index * q->element_size;
	q->unposted_index = (q->unposted_index + nelements) % q->nelements;
	return p;
}

static int pqi_enqueue_to_device(struct pqi_device_queue *q, void *element)
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
	printk(KERN_WARNING "DQ: p = %p, q->unposted_index = %hu, n = %hu\n",
				p, q->unposted_index, q->nelements);
	memcpy(element, p, q->element_size);
	q->unposted_index = (q->unposted_index + 1) % q->nelements;
	printk(KERN_WARNING "After DQ: q->unposted_index = %hu\n",
				q->unposted_index);
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

static void pqi_notify_device_queue_written(struct pqi_device_queue *q)
{
	/*
	 * Notify the device that the host has produced data for the device
	 */
	writew(q->unposted_index, q->pi);
}

static void pqi_notify_device_queue_read(struct pqi_device_queue *q)
{
	/*
	 * Notify the device that the host has consumed data from the device
	 */
	writew(q->unposted_index, q->ci);
}

static int wait_for_admin_command_ack(struct scsi_express_device *h)
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

static int __devinit scsi_express_create_admin_queues(struct scsi_express_device *h)
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

#define ADMIN_QUEUE_ELEMENT_COUNT ((MAX_IO_QUEUES + 1) * 2)

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
				sizeof(struct scsi_express_request)))
		goto bailout;
	if (allocate_q_request_buffers(&h->qinfo[1],
				ADMIN_QUEUE_ELEMENT_COUNT,
				sizeof(struct scsi_express_request)))
		goto bailout;
	return 0;

bailout:
	if (admin_iq)
		pci_free_consistent(h->pdev, total_admin_queue_size,
					admin_iq, admin_iq_busaddr);
	free_all_q_request_buffers(h);
	return -1;	
}

static int scsi_express_delete_admin_queues(struct scsi_express_device *h)
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

static int scsi_express_setup_msix(struct scsi_express_device *h)
{
	int i, err;

	struct msix_entry msix_entry[MAX_TOTAL_QUEUES];

	h->nr_queues = num_online_cpus();
	if (h->nr_queues > MAX_TOTAL_QUEUES)
		h->nr_queues = MAX_TOTAL_QUEUES;

	for (i = 0; i < MAX_TOTAL_QUEUES; i++) {
		msix_entry[i].vector = 0;
		msix_entry[i].entry = i;
	}

	/* this needs work */
	h->noqs = h->nr_queues - 2;
	if (h->noqs < 1)
		h->noqs = 1;
	h->niqs = 2;

	if (!pci_find_capability(h->pdev, PCI_CAP_ID_MSIX))
		goto default_int_mode;
	err = pci_enable_msix(h->pdev, msix_entry, h->noqs);
	if (err > 0)
		h->noqs = err;

	if (err >= 0) {
		for (i = 0; i < h->noqs; i++) {
			int idx = i ? i + 1 : i;
			h->qinfo[idx].msix_vector = msix_entry[i].vector;
			dev_warn(&h->pdev->dev, "q[%d] msix_entry[%d] = %d\n",
				idx, i, msix_entry[i].vector);
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

irqreturn_t scsi_express_ioq_msix_handler(int irq, void *devid)
{
	struct queue_info *q = devid;
	/* struct scsi_express_device __attribute__((unused)) *h = queue_to_hba(q); */

	printk(KERN_WARNING "Got ioq interrupt, q = %p (%d) vector = %d\n",
			q, q->pqiq->queue_id, q->msix_vector);

	return IRQ_HANDLED;
}

/* function to determine whether a complete response has been accumulated */
static int scsi_express_response_accumulated(struct scsi_express_request *r)
{
	u16 iu_length;

	if (r->response_accumulated == 0)
		return 0;
	iu_length = le16_to_cpu(*(u16 *) &r->response[2]);
	printk(KERN_WARNING "r->response_accumulated = %hu, iu_length = %hu\n", 
			r->response_accumulated, iu_length);
	return (r->response_accumulated >= iu_length);
}

irqreturn_t scsi_express_adminq_msix_handler(int irq, void *devid)
{
	struct queue_info *q = devid;
	u8 iu_type;
	u16 request_id;
	int rc;
	struct scsi_express_device *h = q->h;
	u8 sq;

	printk(KERN_WARNING "Got admin oq interrupt, q = %p\n", q);
	printk(KERN_WARNING "Got admin oq interrupt, q = %p (%d)\n", q, q->pqiq->queue_id);

	do {
		struct scsi_express_request *r = h->admin_q_from_dev.request;

		dev_warn(&h->pdev->dev, "admin intr, r = %p\n", r);

		if (pqi_from_device_queue_is_empty(&h->admin_q_from_dev)) {
			dev_warn(&h->pdev->dev, "admin OQ %p is empty\n", q);
			break;
		}

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
			return IRQ_HANDLED;
		}
		r->response_accumulated += h->admin_q_from_dev.element_size;
		dev_warn(&h->pdev->dev, "accumulated %d bytes\n", r->response_accumulated);
		if (scsi_express_response_accumulated(r)) {
			dev_warn(&h->pdev->dev, "accumlated response\n");
			h->admin_q_from_dev.request = NULL;
			wmb();
			complete(r->waiting);
			pqi_notify_device_queue_read(&h->admin_q_from_dev);
		}
	} while (1);

	return IRQ_HANDLED;
}

static void scsi_express_irq_affinity_hints(struct scsi_express_device *h)
{
	int i, cpu;

	cpu = cpumask_first(cpu_online_mask);
	for (i = 0; i < h->noqs; i++) {
		int idx = i ? i + 1 : i;
		irq_set_affinity_hint(h->qinfo[idx].msix_vector,
					get_cpu_mask(cpu));
		cpu = cpumask_next(cpu, cpu_online_mask);
	}
}

static int scsi_express_request_irqs(struct scsi_express_device *h,
					irq_handler_t msix_adminq_handler,
					irq_handler_t msix_ioq_handler)
{
	u8 i;
	int rc;

	dev_warn(&h->pdev->dev, "Requesting irq %d for msix vector %d (admin)\n",
			h->qinfo[0].msix_vector, 0);
	rc = request_irq(h->qinfo[0].msix_vector, msix_adminq_handler, 0,
					h->devname, &h->qinfo[0]);
	if (rc != 0) {
		dev_warn(&h->pdev->dev, "request_irq failed, i = %d\n", 0);
	}
		

	/* for loop starts at 1 to skip over the admin iq */
	for (i = 2; i < h->noqs + 1; i++) {
		dev_warn(&h->pdev->dev, "Requesting irq %d for msix vector %d (oq)\n",
				h->qinfo[i].msix_vector, i - 1);
		rc = request_irq(h->qinfo[i].msix_vector, msix_ioq_handler, 0,
					h->devname, &h->qinfo[i]);
		if (rc != 0) {
			dev_warn(&h->pdev->dev,
					"request_irq failed, i = %d\n", i);
			/* FIXME release irq's 0 through i - 1 */
			goto default_int_mode;
		}
	}
	scsi_express_irq_affinity_hints(h);
	return 0;

default_int_mode:
	dev_warn(&h->pdev->dev, "intx mode not implemented.\n");
	return -1;
}

static void scsi_express_free_irqs(struct scsi_express_device *h)
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

static void scsi_express_free_irqs_and_disable_msix(
		struct scsi_express_device *h)
{
	scsi_express_free_irqs(h);
#ifdef CONFIG_PCI_MSI
	if (h->intr_mode == INTR_MODE_MSIX && h->pdev->msix_enabled)
		pci_disable_msix(h->pdev);
#endif /* CONFIG_PCI_MSI */
}

/* FIXME: maybe there's a better way to do this */
static u16 alloc_request(struct scsi_express_device *h, u8 q)
{
	u16 rc;
	unsigned long flags;

	/* I am encoding q number in high bight of request id */
	BUG_ON(h->qinfo[q].qdepth > 256);
	BUG_ON(q > 127); /* high bit reserved for error reporting */

	/* FIXME, may be able to get rid of these spin locks */
	spin_lock_irqsave(&h->qinfo[q].qlock, flags);
        do {
                rc = (u16) find_first_zero_bit(h->qinfo[q].request_bits,
						h->qinfo[q].qdepth);
                if (rc >= h->qinfo[q].qdepth)
                        return (u16) -EBUSY;
        } while (test_and_set_bit((int) rc, h->qinfo[q].request_bits));
	spin_unlock_irqrestore(&h->qinfo[q].qlock, flags);
	return rc | (q << 8);
}

static void free_request(struct scsi_express_device *h, u8 q, u16 request_id)
{
	BUG_ON((request_id >> 8) != q);
	clear_bit(request_id & 0x00ff, h->qinfo[q].request_bits);
}

static void fill_create_io_queue_request(struct scsi_express_device *h,
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
	r->index_addr = cpu_to_le64(r->element_array_addr +
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

static void fill_delete_io_queue_request(struct scsi_express_device *h,
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

static void send_admin_command(struct scsi_express_device *h, u16 request_id)
{
	struct scsi_express_request *request;
	struct pqi_device_queue *aq = &h->admin_q_to_dev;
	DECLARE_COMPLETION_ONSTACK(wait);

	request = &h->qinfo[aq->queue_id].request[request_id & 0x00ff];
	dev_warn(&h->pdev->dev, "Sending request %p\n", request);
	request->waiting = &wait;
	request->response_accumulated = 0;
	dev_warn(&h->pdev->dev, "sending request %hu\n", request_id);
	pqi_notify_device_queue_written(aq);
	dev_warn(&h->pdev->dev, "waiting for completion\n");
	wait_for_completion(&wait);
	dev_warn(&h->pdev->dev, "wait_for_completion returned\n");
}

static int scsi_express_setup_io_queues(struct scsi_express_device *h)
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
		fill_create_io_queue_request(h, r, q, 0, request_id, q->queue_id - 1);
		send_admin_command(h, request_id);
		resp = (volatile struct pqi_create_operational_queue_response *)
			h->qinfo[aq->queue_id].request[request_id & 0x00ff].response;	
		dev_warn(&h->pdev->dev, "resp.status = %hhu, resp.index_offset = %llu\n",
			resp->status, le64_to_cpu(resp->index_offset));
		if (resp->status != 0)
			dev_warn(&h->pdev->dev, "Failed to set up OQ... now what?\n");
		h->io_q_from_dev[i].pi = h->io_q_from_dev[i].queue_vaddr +
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
		h->io_q_to_dev[i].ci = h->io_q_to_dev[i].queue_vaddr +
						le64_to_cpu(resp->index_offset);
		free_request(h, aq->queue_id, request_id);
	}

	return 0;

bail_out:
	return -1;
}

static int scsi_express_delete_io_queues(struct scsi_express_device *h)
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
	return 0;
}


static int scsi_express_set_dma_mask(struct pci_dev * pdev)
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

static int scsi_express_register_host(struct scsi_express_device *h)
{
	struct Scsi_Host *sh;
	int rc;

	dev_warn(&h->pdev->dev, "zzz scsi_express_register_host 1\n");

	sh = scsi_host_alloc(&scsi_express_template, sizeof(h));
	if (!sh)
		goto bail;
	sh->io_port = 0;
	sh->n_io_port = 0;
	sh->this_id = -1;
	sh->max_channel = 1;
	sh->max_cmd_len = MAX_COMMAND_SIZE;
	sh->max_lun = 255; /* FIXME are these correct? */
	sh->max_id = 255;
	sh->can_queue = 64; /* FIXME: get this from device if possible. */
	sh->cmd_per_lun = 64;
	sh->sg_tablesize = 2; /* FIXME make this bigger */
	sh->hostdata[0] = (unsigned long) h;
	sh->irq = h->qinfo[0].msix_vector;
	sh->unique_id = sh->irq; /* really? */
	dev_warn(&h->pdev->dev, "zzz scsi_express_register_host 2\n");
	rc = scsi_add_host(sh, &h->pdev->dev);
	if (rc)
		goto add_host_failed;
	dev_warn(&h->pdev->dev, "zzz scsi_express_register_host 3\n");
	scsi_scan_host(sh);
	dev_warn(&h->pdev->dev, "zzz scsi_express_register_host 4\n");
	return 0;

add_host_failed:
	dev_err(&h->pdev->dev, "scsi_add_host failed.\n");
	scsi_host_put(sh);
	return rc;
bail:
	dev_err(&h->pdev->dev, "scsi_host_alloc failed.\n");
	return -ENOMEM;
}

static int __devinit scsi_express_probe(struct pci_dev *pdev,
			const struct pci_device_id *pci_id)
{
	struct scsi_express_device *h;
	u64 signature;
	int i, rc;

	dev_warn(&pdev->dev, SCSI_EXPRESS "found device: %04x:%04x/%04x:%04x\n",
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
	sprintf(h->devname, "scsi_express-%d\n", h->ctlr);

	h->pdev = pdev;
	pci_set_drvdata(pdev, h);

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_warn(&h->pdev->dev, "unable to enable PCI device\n");
		return rc;
	}

	/* Enable bus mastering (pci_disable_device may disable this) */
	pci_set_master(h->pdev);

	rc = pci_request_regions(h->pdev, SCSI_EXPRESS);
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

	if (scsi_express_set_dma_mask(pdev)) {
		dev_err(&pdev->dev, "failed to set DMA mask\n");
		goto bail;
	}

	signature = readq(&h->pqireg->signature);
	if (memcmp("PQI DREG", &signature, sizeof(signature)) != 0) {
		dev_warn(&pdev->dev, "device does not appear to be a PQI device\n");
		goto bail;
	}
	dev_warn(&pdev->dev, "device does appear to be a PQI device\n");

	rc = scsi_express_setup_msix(h);
	if (rc != 0)
		goto bail;

	rc = scsi_express_create_admin_queues(h);
	if (rc)
		goto bail;

	rc = scsi_express_request_irqs(h, scsi_express_adminq_msix_handler,
					scsi_express_ioq_msix_handler);
	if (rc != 0)
		goto bail;

	dev_warn(&h->pdev->dev, "Setting up i/o queues\n");
	rc = scsi_express_setup_io_queues(h);
	if (rc)
		goto bail;
	dev_warn(&h->pdev->dev, "Finished Setting up i/o queues, rc = %d\n", rc);
	rc = scsi_express_register_host(h);
	if (rc)
		goto bail;

	return 0;
bail:
	if (h && h->pqireg)
		iounmap(h->pqireg);
	kfree(h);
	return 0;
}

static int scsi_express_suspend(__attribute__((unused)) struct pci_dev *pdev,
				__attribute__((unused)) pm_message_t state)
{
	return -ENOSYS;
}

static int scsi_express_resume(__attribute__((unused)) struct pci_dev *pdev)
{
	return -ENOSYS;
}

static void __devexit scsi_express_remove(struct pci_dev *pdev)
{
	struct scsi_express_device *h;

	h = pci_get_drvdata(pdev);
	dev_warn(&pdev->dev, "remove called.\n");
	scsi_express_delete_io_queues(h);
	scsi_express_free_irqs_and_disable_msix(h);
	dev_warn(&pdev->dev, "irqs freed, msix disabled\n");
	scsi_express_delete_admin_queues(h);
	if (h && h->pqireg)
		iounmap(h->pqireg);
	pci_disable_device(pdev);
	pci_release_regions(pdev);
	pci_set_drvdata(pdev, NULL);
	kfree(h);
}

static void scsi_express_shutdown(struct pci_dev *pdev)
{
	dev_warn(&pdev->dev, "shutdown called.\n");
}

static struct pci_driver scsi_express_pci_driver = {
	.name = SCSI_EXPRESS,
	.probe = scsi_express_probe,
	.remove = __devexit_p(scsi_express_remove),
	.id_table = scsi_express_id_table,
	.shutdown = scsi_express_shutdown,
	.suspend = scsi_express_suspend,
	.resume = scsi_express_resume,
};

static int __init scsi_express_init(void)
{
	return pci_register_driver(&scsi_express_pci_driver);
}

static void __exit scsi_express_exit(void)
{
	pci_unregister_driver(&scsi_express_pci_driver);
}

static inline struct scsi_express_device *sdev_to_hba(struct scsi_device *sdev)
{
	unsigned long *priv = shost_priv(sdev->host);
	return (struct scsi_express_device *) *priv;
}

static int scsi_express_queuecommand_lck(struct scsi_cmnd *sc,
        void (*done)(struct scsi_cmnd *))
{
	struct scsi_express_device *h;
	struct scsi_device *sdev = sc->device;

	h = sdev_to_hba(sc->device);

	dev_warn(&h->pdev->dev, "scsi_express_queuecommand called\n");

	dev_warn(&h->pdev->dev, "h%db%dt%dl%d: "
		"CDB = 0x%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		sdev->host->host_no, sdev_channel(sc->device), sdev_id(sc->device), sdev->lun,
		sc->cmnd[0], sc->cmnd[1], sc->cmnd[2], sc->cmnd[3],
		sc->cmnd[4], sc->cmnd[5], sc->cmnd[6], sc->cmnd[7],
		sc->cmnd[8], sc->cmnd[9], sc->cmnd[10], sc->cmnd[11],
		sc->cmnd[12], sc->cmnd[13], sc->cmnd[14], sc->cmnd[15]);

	return 0;
}

static DEF_SCSI_QCMD(scsi_express_queuecommand);

static int scsi_express_change_queue_depth(struct scsi_device *sdev,
        int qdepth, int reason)
{
	struct scsi_express_device *h = sdev_to_hba(sdev);

	dev_warn(&h->pdev->dev, "scsi_express_change_queue_depth called but not implemented\n");
	return 0;
}

static int scsi_express_abort_handler(struct scsi_cmnd *sc)
{
	struct scsi_express_device *h;

	h = sdev_to_hba(sc->device);
	dev_warn(&h->pdev->dev, "scsi_express_abort_handler called but not implemented\n");
	return 0;
}

static int scsi_express_device_reset_handler(struct scsi_cmnd *sc)
{
	struct scsi_express_device *h;

	h = sdev_to_hba(sc->device);
	dev_warn(&h->pdev->dev, "scsi_express_device_reset_handler called but not implemented\n");
	return 0;
}

static int scsi_express_slave_alloc(struct scsi_device *sdev)
{
	struct scsi_express_device *h = sdev_to_hba(sdev);

	dev_warn(&h->pdev->dev, "scsi_express_slave_alloc called but not implemented\n");
	return 0;
}

static void scsi_express_slave_destroy(struct scsi_device *sdev)
{
	struct scsi_express_device *h = sdev_to_hba(sdev);

	dev_warn(&h->pdev->dev, "scsi_express_slave_destroy called but not implemented\n");
	return;
}

static int scsi_express_compat_ioctl(struct scsi_device *sdev,
						int cmd, void *arg)
{
	struct scsi_express_device *h = sdev_to_hba(sdev);

	dev_warn(&h->pdev->dev, "scsi_express_compat_ioctl called but not implemented\n");
	return 0;
}

static int scsi_express_ioctl(struct scsi_device *sdev, int cmd, void *arg)
{
	struct scsi_express_device *h = sdev_to_hba(sdev);

	dev_warn(&h->pdev->dev, "scsi_express_ioctl called but not implemented\n");
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
}

module_init(scsi_express_init);
module_exit(scsi_express_exit);

