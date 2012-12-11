#ifndef _SCSI_EXPRESS_H
#define _SCSI_EXPRESS_H

struct pqi_device_queue {
	__iomem void *queue_vaddr;
	__iomem u16 *pi;		/* producer index */
	__iomem u16 *ci;		/* consumer index */
	u16 unposted_index;		/* temporary host copy of pi or ci,
					 * depending on direction of queue.
					 */
	u16 element_size;		/* must be multiple of 16 */
	u8 nelements;
	dma_addr_t dhandle;
	u16 queue_id;
	u8 direction;
#define PQI_DIR_TO_DEVICE 0
#define PQI_DIR_FROM_DEVICE 1
};

#define PQI_QUEUE_FULL (-1)
#define PQI_QUEUE_EMPTY (-2)


#pragma pack(1)

struct pqi_oq_extra_params {
	u16 interrupt_message_number;
	u16 wait_for_rearm;
	u16 coalesce_count;
	u16 min_coalesce_time;
	u16 max_coalesce_time;
	u8 operational_queue_protocol;
};

struct pqi_iq_extra_params {
	u8 operational_queue_protocol;
	u8 reserved[10];
};

struct pqi_create_operational_queue_request {
	u8 iu_type;
	u8 compatible_features;
	u16 iu_length;
	u16 response_oq;
	u16 work_area;
	u16 request_id;
	u8 function_code;
	u8 reserved2;
	u16 queue_id;
	u8 reserved3[2];
	u64 element_array_addr;
	u64 index_addr;
	u16 nelements;
	u16 element_length;
	union {
		struct pqi_iq_extra_params iqp;
		struct pqi_oq_extra_params oqp;
	};
	u8 reserved4[17];
};

struct pqi_create_operational_queue_response {
	u8 ui_type;
	u8 compatible_features;
	u16 ui_length;
	u8 reserved[4];
	u16 request_id;
	u8 function_code;
	u8 status;
	u8 reserved2[4];
	u64 index_offset;
	u8 reserved3[40];
};

/*
 * A note about variable names:
 *
 * "iq" == "inbound queue"
 * "oq" == "outbound queue"
 * "pi" == "producer index"
 * "ci" == "consumer index"
 *
 * "inbound" and "outbound" are from the point of view of the device,
 * so "inbound" means "from the host to the device" and "outbound"
 * means "from the device to the host".
 *
 */

struct pqi_device_register_set {
	u64 signature;
	u64 process_admin_function;
#define PQI_IDLE 0
#define PQI_CREATE_ADMIN_QUEUES 0x01ULL
#define PQI_DELETE_ADMIN_QUEUES 0x02ULL
	u64 capability;
	u32 legacy_intx_status;
	u32 legacy_intx_mask_set;
	u32 legacy_intx_mask_clear;
	u8  reserved1[28];
	u32 pqi_device_status;
#define PQI_READY_FOR_ADMIN_FUNCTION 0x02
#define PQI_READY_FOR_IO 0x03
	u8 reserved2[4];
	u64 admin_iq_pi_offset;
	u64 admin_oq_ci_offset;
	u64 admin_iq_addr;
	u64 admin_oq_addr;
	u64 admin_iq_ci_addr;
	u64 admin_oq_pi_addr;
	u32 admin_queue_param;
	u8  reserved3[4];
	u64 error_data;
	u32 reset;
	u8  reserved4[4];
	u32 power_action;
};

struct pqi_capability {
	u8 max_admin_iq_elements;
	u8 max_admin_oq_elements;
	u8 admin_iq_element_length; /* length in 16 byte units */
	u8 admin_oq_element_length; /* length in 16 byte units */
	u8 reserved[4];
};
#pragma pack()

#define IQ_IU_SIZE 64
#define OQ_IU_SIZE 16
#define IQ_NELEMENTS 64
#define OQ_NELEMENTS 64

struct scsi_express_device {
	struct pci_dev *pdev;
	struct pqi_capability pqicap;
	__iomem struct pqi_device_register_set *pqireg;
#define MAX_IO_QUEUES 6
#define MAX_TO_DEVICE_QUEUES 1
#define MAX_FROM_DEVICE_QUEUES 6
#define MAX_TOTAL_QUEUES (MAX_TO_DEVICE_QUEUES + MAX_FROM_DEVICE_QUEUES + 2)
	u8 q[MAX_TOTAL_QUEUES];
	int intr[MAX_TOTAL_QUEUES];
	int msix_vector[MAX_TOTAL_QUEUES];
	int nr_queues, niqs, noqs; /* total, inbound and outbound queues */
#define INTR_MODE_MSIX 1
#define INTR_MODE_MSI  2
#define INTR_MODE_INTX 3
	int intr_mode;
	char devname[20];
	int ctlr;
	struct pqi_device_queue admin_q_to_dev, admin_q_from_dev;
	struct pqi_device_queue *io_q_to_dev;
	struct pqi_device_queue *io_q_from_dev;
	u16 current_id;
	spinlock_t id_lock;
};

#define PQI_IQ_ID_FM_INDEX(_i)  (((_i) + 2) * 2)
#define PQI_OQ_ID_FM_INDEX(_i)  (((_i) + 2) * 2 + 1)

/* Works for both IQ and OQ ID, although valid only for OQ  */
#define PQI_VECTOR_FM_Q_ID(_q)  (((_q) / 2) - 1)

#pragma pack()

#endif
