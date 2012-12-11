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

static int pqi_device_queue_array_alloc(struct pci_dev *pdev,
		struct pqi_device_queue **xq, int num_queues,
		u16 n_q_elements, u8 q_element_size_over_16,
		int queue_direction, int starting_queue_id)
{
	void *vaddr = NULL;
	dma_addr_t dhandle;
	int i, nqs_alloced = 0, err = 0;
	int total_size = (n_q_elements * q_element_size_over_16 * 16) +
				sizeof(u64);

	dev_warn(&pdev->dev, "Allocating %d queues %s device...\n", 
		num_queues,
		queue_direction == PQI_DIR_TO_DEVICE ? "TO" : "FROM");

	*xq = kzalloc(sizeof(**xq) * num_queues, GFP_KERNEL);
	if (!*xq) {
		err = -ENOMEM;
		goto bailout;
	}
	vaddr = pci_alloc_consistent(pdev, total_size * num_queues, &dhandle);
	if (!vaddr) {
		err = -ENOMEM;
		goto bailout;
	}
	dev_warn(&pdev->dev, "Memory alloc'ed.\n");

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
		dev_warn(&pdev->dev, "zzz bottom of loop, i = %d\n", i);
	}
	return nqs_alloced;

bailout:
	kfree(*xq);
	if (vaddr)
		pci_free_consistent(pdev, total_size, vaddr, dhandle);
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
	qpi = readw(q->pi);
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
	memcpy(element, p, q->element_size);
	q->unposted_index = (q->unposted_index + 1) % q->nelements;
	return 0;
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

	return 0;

bailout:
	if (admin_iq)
		pci_free_consistent(h->pdev, total_admin_queue_size,
					admin_iq, admin_iq_busaddr);
	return -1;	
	

#if 0
	/*
	 * If the FUNCTION AND STATUS CODE field in the Process Administrator
	 * Function register is set to 00h (i.e., IDLE) (see 5.2.5) and the
	 * Status register PQI DEVICE STATE field is set to 02h (see 5.2.10)
	 * (i.e., S2:Ready_For_Administrator_Function), then the PQI host may
	 * create the administrator queues using the following steps:
	 * 1) read the Capability register to determine the element length and
	 *    the maximum number of the element in each of the administrator
	 *    queues; 2) allocates the PCI memory spaces using a mechanism outside
	 * the scope of this standard for the
	 *	 * following:
	 *	 * A) Administrator IQ element array;
	 *	 * B) Administrator OQ element array;
	 *	 * C) Administrator IQ CI; and
	 *	 * D) Administrator OQ PI;
	 * 3) set the fields as shown in table 15;
	 * 4) set the value of the FUNCTION AND STATUS CODE field in the
	 *    Process Administrator Function register to 01h (i.e., CREATE
	 *    ADMINISTRATOR QUEUE FUNCTION); 5) read the Process Administrator
	 *    Function register until:
	 *	 * A) the FUNCTION AND STATUS CODE field is set to 00h (i.e., IDLE); or
	 *	 * B) 100 ms has elapsed;
	 * 6) if the FUNCTION AND STATUS CODE field is not set to 00h and 100
	 *    ms has elapsed then read the Process Administrator Function
	 *    register; 7) if the FUNCTION AND STATUS CODE field is set to 00h,
	 *    then:
	 *	 * A) read the administrator IQ PI offset from the
	 *	 Administrator IQ PI Offset register and save the value in
	 *	 the PQI host local memory; and
	 *	 * B) read the administrator OQ CI offset from the
	 *	 Administrator OQ CI Offset register and save the value in
	 *	 the PQI host local memory;
	 * and
	 * 8) if the FUNCTION AND STATUS CODE field is not set to 00h, then
	 *    read the Status register and report the status in a vendor specific
	 *    manner.
	 */
#endif

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

	struct msix_entry msix_entry[TOTAL_QUEUES];

	for (i = 0; i < TOTAL_QUEUES; i++) {
		msix_entry[i].vector = 0;
		msix_entry[i].entry = i;
	}
	if (!pci_find_capability(h->pdev, PCI_CAP_ID_MSIX))
		goto default_int_mode;
	err = pci_enable_msix(h->pdev, msix_entry, TOTAL_QUEUES);
	if (!err) {
		for (i = 0; i < TOTAL_QUEUES; i++) {
			h->intr[i] = msix_entry[i].vector;
			dev_warn(&h->pdev->dev, "msix_entry[%d] = %d\n",
				i, msix_entry[i].vector);
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

/*
 *  * Convert &h->q[x] (passed to interrupt handlers) back to h.
 *   * Relies on (h-q[x] == x) being true for x such that
 *    * 0 <= x < TOTAL_QUEUES.
 *     */
static struct scsi_express_device *queue_to_hba(u8 *queue)
{
	return container_of((queue - *queue),
				struct scsi_express_device, q[0]);
}

irqreturn_t scsi_express_msix_handler(int irq, void *devid)
{
	u8 *q = devid;
	struct scsi_express_device __attribute__((unused)) *h = queue_to_hba(q);

	printk(KERN_WARNING "Got interrupt, q = %hhu\n", *q);
	return IRQ_HANDLED;
}

static int scsi_express_request_irqs(struct scsi_express_device *h,
					irq_handler_t msix_handler)
{
	u8 i;
	int rc;

	for (i = 0; i < TOTAL_QUEUES; i++)
		h->q[i] = i;
	for (i = 0; i < TOTAL_QUEUES; i++) {
		rc = request_irq(h->intr[i], msix_handler, 0,
					h->devname, &h->q[i]);
		if (rc != 0) {
			dev_warn(&h->pdev->dev,
					"request_irq failed, i = %d\n", i);
			/* FIXME release irq's 0 through i - 1 */
			goto default_int_mode;
		}
	}
	return 0;

default_int_mode:
	dev_warn(&h->pdev->dev, "intx mode not implemented.\n");
	return -1;
}

static void scsi_express_free_irqs(struct scsi_express_device *h)
{
	int i;

	for (i = 0; i < TOTAL_QUEUES; i++)
		free_irq(h->intr[i], &h->q[i]);
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
static u16 get_request_id(struct scsi_express_device *h)
{
	u16 rc;
	unsigned long flags;

	spin_lock_irqsave(&h->id_lock, flags);
	/* set high bit because we use an array index in other contexts */
	rc = h->current_id | 0x8000;
	h->current_id++;
	spin_unlock_irqrestore(&h->id_lock, flags);
	return rc;
}

static void fill_create_io_queue_request(struct scsi_express_device *h,
	struct pqi_create_operational_queue_request *r,
	struct pqi_device_queue *q, int to_device)
{
	u8 function_code = to_device ? 0x10 : 0x11; /* FIXME magic */

	memset(r, 0, sizeof(*r));
	r->iu_type = 0x60; /* FIXME, magic */
	r->iu_length = cpu_to_le16(0x003c);
	r->response_oq = 0;
	r->request_id = cpu_to_le16(get_request_id(h));
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
		r->oqp.interrupt_message_number = PQI_VECTOR_FM_Q_ID(q->queue_id);
		/* Coalascing is not supported yet */
		r->oqp.operational_queue_protocol = 0;
		dev_warn(&h->pdev->dev,
				"Create OQ, QID[%d], MSIX=%d\n", r->queue_id,
				r->oqp.interrupt_message_number);
	}
}

static int scsi_express_setup_io_queues(struct scsi_express_device *h)
{

	int i, niqs, noqs;
	struct pqi_create_operational_queue_request *r;

	dev_warn(&h->pdev->dev,
		"sizeof struct pqi_create_operational_queue_request is %lu\n",
		(unsigned long) sizeof(struct pqi_create_operational_queue_request));

	niqs = pqi_device_queue_array_alloc(h->pdev, &h->io_q_to_dev,
			MAX_TO_DEVICE_QUEUES, IQ_NELEMENTS, IQ_IU_SIZE / 16,
			PQI_DIR_TO_DEVICE, 2);
	if (niqs < 0)
		goto bail_out;
	noqs = pqi_device_queue_array_alloc(h->pdev, &h->io_q_from_dev,
			MAX_FROM_DEVICE_QUEUES, OQ_NELEMENTS, OQ_IU_SIZE / 16,
			PQI_DIR_FROM_DEVICE, 2 + niqs);
	if (noqs < 0)
		goto bail_out;

	dev_warn(&h->pdev->dev,
		"Setting up %d submission queues and %d reply queues\n",
			niqs, noqs);

	for (i = 0; i < noqs; i++) {
		struct pqi_device_queue *q;

		dev_warn(&h->pdev->dev,
			"Setting up io queue %d Qid = %d from device\n", i,
			h->io_q_from_dev[i].queue_id);
		/* Set up i/o queue #i from device */
		q = &h->io_q_from_dev[i];
		r = pqi_alloc_elements(&h->admin_q_to_dev, 1);
		dev_warn(&h->pdev->dev, "xxx2 i = %d, r = %p, q = %p\n",
				i, r, q);
		fill_create_io_queue_request(h, r, q, 0);
		pqi_notify_device_queue_written(&h->admin_q_to_dev);
	}

	for (i = 0; i < niqs; i++) {
		struct pqi_device_queue *q;

		dev_warn(&h->pdev->dev,
			"Setting up io queue %d Qid = %d to device\n", i,
			h->io_q_to_dev[i].queue_id);
		/* Set up i/o queue #i to device */
		r = pqi_alloc_elements(&h->admin_q_to_dev, 1);
		q = &h->io_q_to_dev[i];
		dev_warn(&h->pdev->dev, "xxx1 i = %d, r = %p, q = %p\n",
				i, r, q);
		fill_create_io_queue_request(h, r, q, 1);
		pqi_notify_device_queue_written(&h->admin_q_to_dev);
	}

	return 0;

bail_out:
	return -1;
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

static int __devinit scsi_express_probe(struct pci_dev *pdev,
			const struct pci_device_id *pci_id)
{
	struct scsi_express_device *h;
	u64 signature;
	int rc;

	dev_warn(&pdev->dev, SCSI_EXPRESS "found device: %04x:%04x/%04x:%04x\n",
			pdev->vendor, pdev->device,
			pdev->subsystem_vendor, pdev->subsystem_device);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	h->ctlr = controller_num;
	spin_lock_init(&h->id_lock);
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

	rc = scsi_express_request_irqs(h, scsi_express_msix_handler);
	if (rc != 0)
		goto bail;

	rc = scsi_express_create_admin_queues(h);
	if (rc)
		goto bail;

	rc = scsi_express_setup_io_queues(h);
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
	scsi_express_free_irqs_and_disable_msix(h);
	if (h && h->pqireg)
		iounmap(h->pqireg);
	scsi_express_delete_admin_queues(h);
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

}

module_init(scsi_express_init);
module_exit(scsi_express_exit);

