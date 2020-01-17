/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_COMMON_MA_USB_H__
#define __MAUSB_COMMON_MA_USB_H__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <types.h>
#endif /* __KERNEL__ */

#define _MA_USB_SET_FIELD(_m, _v) (((~((_m) - 1) & (_m)) * (_v)) & (_m))
#define _MA_USB_GET_FIELD(_m, _v) (((_v) & (_m)) / (~((_m) - 1) & (_m)))
#define MA_USB_SET_FIELD_(_m_, _v) _MA_USB_SET_FIELD(MA_USB_##_m_##_MASK, _v)
#define MA_USB_GET_FIELD_(_m_, _v) _MA_USB_GET_FIELD(MA_USB_##_m_##_MASK, _v)
#define MA_USB_SET_FIELD(_m_, _v) _MA_USB_SET_FIELD(MA_USB_##_m_##_MASK, \
						    MA_USB_##_v)
#define MA_USB_GET_FIELD(_m_, _v) _MA_USB_GET_FIELD(MA_USB_##_m_##_MASK, \
						    MA_USB_##_v)

#define MA_USB_MGMT_TOKEN_RESERVED  0
#define MA_USB_MGMT_TOKEN_MIN       1
#define MA_USB_MGMT_TOKEN_MAX       ((1 << 10) - 1)

#define MA_USB_DATA_EPS_UNASSIGNED  0
#define MA_USB_DATA_EPS_ACTIVE      1
#define MA_USB_DATA_EPS_INACTIVE    2
#define MA_USB_DATA_EPS_HALTED      3

#define MA_USB_DATA_TFLAGS_ARQ      1
#define MA_USB_DATA_TFLAGS_NEG      2
#define MA_USB_DATA_TFLAGS_EOT      4
#define MA_USB_DATA_TFLAGS_TRANSFER_TYPE_CTRL   0
#define MA_USB_DATA_TFLAGS_TRANSFER_TYPE_ISOCH  8
#define MA_USB_DATA_TFLAGS_TRANSFER_TYPE_BULK   16
#define MA_USB_DATA_TFLAGS_TRANSFER_TYPE_INTR   24

#define MA_USB_DATA_TFLAGS_TRANSFER_TYPE_MASK   0x18

#define MA_USB_DATA_IFLAGS_MTD_VALID      1
#define MA_USB_DATA_IFLAGS_HDR_FMT_SHORT  0
#define MA_USB_DATA_IFLAGS_HDR_FMT_STD    2
#define MA_USB_DATA_IFLAGS_HDR_FMT_LONG   4
#define MA_USB_DATA_IFLAGS_IRS_FMT_STD    0
#define MA_USB_DATA_IFLAGS_IRS_FMT_LONG   2
#define MA_USB_DATA_IFLAGS_ASAP           8

#define MA_USB_DATA_IFLAGS_FMT_MASK       0x6

/* version */

#define MA_USB_HDR_VERSION_1_0      0

/* flags */

#define MA_USB_HDR_FLAGS_HOST       1
#define MA_USB_HDR_FLAGS_RETRY      2
#define MA_USB_HDR_FLAGS_TIMESTAMP  4
#define MA_USB_HDR_FLAGS_RESERVED   8
#define MA_USB_HDR_FLAG(_f) MA_USB_HDR_FLAGS_##_f

/* type and subtype */

#define MA_USB_HDR_TYPE_TYPE_MASK     0xC0
#define MA_USB_HDR_TYPE_SUBTYPE_MASK  0x3F

#define MA_USB_HDR_TYPE_TYPE_MANAGEMENT 0
#define MA_USB_HDR_TYPE_TYPE_CONTROL    1
#define MA_USB_HDR_TYPE_TYPE_DATA       2

/* Management subtypes */

#define _MA_USB_HDR_TYPE_MANAGEMENT_REQ(_s) \
	MA_USB_SET_FIELD_(HDR_TYPE_SUBTYPE, (_s) * 2)
#define _MA_USB_HDR_TYPE_MANAGEMENT_RESP(_s) \
	MA_USB_SET_FIELD_(HDR_TYPE_SUBTYPE, (_s) * 2 + 1)

#define MA_USB_HDR_TYPE_MANAGEMENT_REQ(_s) \
	_MA_USB_HDR_TYPE_MANAGEMENT_REQ(MA_USB_HDR_TYPE_SUBTYPE_##_s)
#define MA_USB_HDR_TYPE_MANAGEMENT_RESP(_s) \
	_MA_USB_HDR_TYPE_MANAGEMENT_RESP(MA_USB_HDR_TYPE_SUBTYPE_##_s)
#define MA_USB_HDR_TYPE_IS_MANAGEMENT(_v) ( \
	MA_USB_GET_FIELD_(HDR_TYPE_TYPE, _v) \
	== MA_USB_HDR_TYPE_TYPE_MANAGEMENT)
#define MA_USB_HDR_TYPE_IS_MANAGEMENT_RESP(_v) ( \
	MA_USB_HDR_TYPE_IS_MANAGEMENT(_v) && \
	(MA_USB_GET_FIELD_(HDR_TYPE_SUBTYPE, _v) & 1) != 0)
#define MA_USB_HDR_TYPE_MANAGEMENT_GET_SUBTYPE(_v) ( \
	MA_USB_HDR_TYPE_IS_MANAGEMENT(_v) ? ((_v) >> 1) : -1)

#define MA_USB_HDR_TYPE_SUBTYPE_CAP               0
#define MA_USB_HDR_TYPE_SUBTYPE_USBDEVHANDLE      1
#define MA_USB_HDR_TYPE_SUBTYPE_EPHANDLE          2
#define MA_USB_HDR_TYPE_SUBTYPE_EPACTIVATE        3
#define MA_USB_HDR_TYPE_SUBTYPE_EPINACTIVATE      4
#define MA_USB_HDR_TYPE_SUBTYPE_EPRESET           5
#define MA_USB_HDR_TYPE_SUBTYPE_CLEARTRANSFERS    6
#define MA_USB_HDR_TYPE_SUBTYPE_EPHANDLEDELETE    7
#define MA_USB_HDR_TYPE_SUBTYPE_DEVRESET          8
#define MA_USB_HDR_TYPE_SUBTYPE_MODIFYEP0         9
#define MA_USB_HDR_TYPE_SUBTYPE_SETUSBDEVADDR     10
#define MA_USB_HDR_TYPE_SUBTYPE_UPDATEDEV         11
#define MA_USB_HDR_TYPE_SUBTYPE_USBDEVDISCONNECT  12
#define MA_USB_HDR_TYPE_SUBTYPE_USBSUSPEND        13
#define MA_USB_HDR_TYPE_SUBTYPE_USBRESUME         14
#define MA_USB_HDR_TYPE_SUBTYPE_REMOTEWAKE        15
#define MA_USB_HDR_TYPE_SUBTYPE_PING              16
#define MA_USB_HDR_TYPE_SUBTYPE_DEVDISCONNECT     17
#define MA_USB_HDR_TYPE_SUBTYPE_DEVINITDISCONNECT 18
#define MA_USB_HDR_TYPE_SUBTYPE_SYNCH             19
#define MA_USB_HDR_TYPE_SUBTYPE_CANCELTRANSFER    20
#define MA_USB_HDR_TYPE_SUBTYPE_EPOPENSTREAM      21
#define MA_USB_HDR_TYPE_SUBTYPE_EPCLOSESTREAM     22
#define MA_USB_HDR_TYPE_SUBTYPE_USBDEVRESET       23
#define MA_USB_HDR_TYPE_SUBTYPE_DEVNOTIFICATION   24
#define MA_USB_HDR_TYPE_SUBTYPE_EPSETKEEPALIVE    25
#define MA_USB_HDR_TYPE_SUBTYPE_GETPORTBW         26
#define MA_USB_HDR_TYPE_SUBTYPE_SLEEP             27
#define MA_USB_HDR_TYPE_SUBTYPE_WAKE              28
#define MA_USB_HDR_TYPE_SUBTYPE_VENDORSPECIFIC    31 /* Reserved */

/* Data subtypes */

#define _MA_USB_HDR_TYPE_DATA_REQ(_s) ( \
	MA_USB_SET_FIELD(HDR_TYPE_TYPE, HDR_TYPE_TYPE_DATA) | \
	MA_USB_SET_FIELD_(HDR_TYPE_SUBTYPE, (_s) * 2 \
	+ ((_s) > 0 ? 1 : 0)))
#define _MA_USB_HDR_TYPE_DATA_RESP(_s) ( \
	MA_USB_SET_FIELD(HDR_TYPE_TYPE, HDR_TYPE_TYPE_DATA) | \
	MA_USB_SET_FIELD_(HDR_TYPE_SUBTYPE, (_s) * 2 + 1 \
	+ ((_s) > 0 ? 1 : 0)))
#define _MA_USB_HDR_TYPE_DATA_ACK(_s) ( \
	MA_USB_SET_FIELD(HDR_TYPE_TYPE, HDR_TYPE_TYPE_DATA) | \
	MA_USB_SET_FIELD_(HDR_TYPE_SUBTYPE, (_s) * 2 + 2))

#define MA_USB_HDR_TYPE_DATA_REQ(_s) \
	_MA_USB_HDR_TYPE_DATA_REQ(MA_USB_HDR_TYPE_SUBTYPE_##_s)
#define MA_USB_HDR_TYPE_DATA_RESP(_s) \
	_MA_USB_HDR_TYPE_DATA_RESP(MA_USB_HDR_TYPE_SUBTYPE_##_s)
#define MA_USB_HDR_TYPE_DATA_ACK(_s) \
	_MA_USB_HDR_TYPE_DATA_ACK(MA_USB_HDR_TYPE_SUBTYPE_##_s)
#define MA_USB_HDR_TYPE_IS_DATA(_v) ( \
	MA_USB_GET_FIELD_(HDR_TYPE_TYPE, _v) \
	== MA_USB_HDR_TYPE_TYPE_DATA)

#define MA_USB_HDR_TYPE_SUBTYPE_TRANSFER          0
#define MA_USB_HDR_TYPE_SUBTYPE_ISOCHTRANSFER     1

/* EP/Device Handle */

#define MA_USB_DEVICE_HANDLE_RESERVED   0

#define MA_USB_EP_HANDLE_D_MASK     0x0001
#define MA_USB_EP_HANDLE_EP_N_MASK  0x001e
#define MA_USB_EP_HANDLE_ADDR_MASK  0x0fe0
#define MA_USB_EP_HANDLE_BUS_N_MASK 0xf000

#define MA_USB_EP_HANDLE(_b, _a, _e, _d) ( \
	MA_USB_SET_FIELD_(EP_HANDLE_BUS_N, _b)  | \
	MA_USB_SET_FIELD_(EP_HANDLE_ADDR, _a)   | \
	MA_USB_SET_FIELD_(EP_HANDLE_EP_N, _e)   | \
	MA_USB_SET_FIELD_(EP_HANDLE_D, _d))

#define MA_USB_EP_HANDLE_BUS_N_VIRTUAL  15
#define MA_USB_EP_HANDLE_ADDR_DEFAULT   0
#define MA_USB_EP_HANDLE_EP_N_DEFAULT   0
#define MA_USB_EP_HANDLE_D_OUT          0	/* See USB2.0 9.3, Table 9-2 */
#define MA_USB_EP_HANDLE_D_IN           1	/* See USB2.0 9.3, Table 9-2 */

/* Status codes */

#define MA_USB_HDR_STATUS_UNSUCCESSFUL                  -128
#define MA_USB_HDR_STATUS_INVALID_MA_USB_SESSION_STATE  -127
#define MA_USB_HDR_STATUS_INVALID_DEVICE_HANDLE         -126
#define MA_USB_HDR_STATUS_INVALID_EP_HANDLE             -125
#define MA_USB_HDR_STATUS_INVALID_EP_HANDLE_STATE       -124
#define MA_USB_HDR_STATUS_INVALID_REQUEST               -123
#define MA_USB_HDR_STATUS_MISSING_SEQUENCE_NUMBER       -122
#define MA_USB_HDR_STATUS_TRANSFER_PENDING              -121
#define MA_USB_HDR_STATUS_TRANSFER_EP_STALL             -120
#define MA_USB_HDR_STATUS_TRANSFER_SIZE_ERROR           -119
#define MA_USB_HDR_STATUS_TRANSFER_DATA_BUFFER_ERROR    -118
#define MA_USB_HDR_STATUS_TRANSFER_BABBLE_DETECTED      -117
#define MA_USB_HDR_STATUS_TRANSFER_TRANSACTION_ERROR    -116
#define MA_USB_HDR_STATUS_TRANSFER_SHORT_TRANSFER       -115
#define MA_USB_HDR_STATUS_TRANSFER_CANCELED             -114
#define MA_USB_HDR_STATUS_INSUFICIENT_RESOURCES         -113
#define MA_USB_HDR_STATUS_NOT_SUFFICIENT_BANDWIDTH      -112
#define MA_USB_HDR_STATUS_INTERNAL_ERROR                -111
#define MA_USB_HDR_STATUS_DATA_OVERRUN                  -110
#define MA_USB_HDR_STATUS_DEVICE_NOT_ACCESSED           -109
#define MA_USB_HDR_STATUS_BUFFER_OVERRUN                -108
#define MA_USB_HDR_STATUS_BUSY                          -107
#define MA_USB_HDR_STATUS_DROPPED_PACKET                -106
#define MA_USB_HDR_STATUS_ISOCH_TIME_EXPIRED            -105
#define MA_USB_HDR_STATUS_ISOCH_TIME_INVALID            -104
#define MA_USB_HDR_STATUS_NO_USB_PING_RESPONSE          -103
#define MA_USB_HDR_STATUS_NOT_SUPPORTED                 -102
#define MA_USB_HDR_STATUS_REQUEST_DENIED                -101
#define MA_USB_HDR_STATUS_MISSING_REQUEST_ID            -100
#define MA_USB_HDR_STATUS_SUCCESS                       0	/* Reserved */
#define MA_USB_HDR_STATUS_NO_ERROR MA_USB_HDR_STATUS_SUCCESS	/* Reserved */

/* Speed values */

#define MA_USB_SPEED_LOW_SPEED         0
#define MA_USB_SPEED_FULL_SPEED        1
#define MA_USB_SPEED_HIGH_SPEED        2
#define MA_USB_SPEED_SUPER_SPEED       3
#define MA_USB_SPEED_SUPER_SPEED_PLUS  4

/* capreq extra hdr */

#define MA_USB_CAPREQ_DESC_SYNCHRONIZATION_LENGTH\
	(sizeof(struct ma_usb_desc) +\
	sizeof(struct ma_usb_capreq_desc_synchronization))
#define MA_USB_CAPREQ_DESC_LINK_SLEEP_LENGTH\
	(sizeof(struct ma_usb_desc) +\
	sizeof(struct ma_usb_capreq_desc_link_sleep))

#define MA_USB_CAPREQ_LENGTH\
	(sizeof(struct ma_usb_hdr_common) +\
	sizeof(struct ma_usb_hdr_capreq) +\
	MA_USB_CAPREQ_DESC_SYNCHRONIZATION_LENGTH +\
	MA_USB_CAPREQ_DESC_LINK_SLEEP_LENGTH)

/* capreq desc types */

#define MA_USB_CAPREQ_DESC_TYPE_SYNCHRONIZATION 3
#define MA_USB_CAPREQ_DESC_TYPE_LINK_SLEEP      5

/* capresp descriptors */

#define MA_USB_CAPRESP_DESC_TYPE_SPEED            0
#define MA_USB_CAPRESP_DESC_TYPE_P_MANAGED_OUT    1
#define MA_USB_CAPRESP_DESC_TYPE_ISOCHRONOUS      2
#define MA_USB_CAPRESP_DESC_TYPE_SYNCHRONIZATION  3
#define MA_USB_CAPRESP_DESC_TYPE_CONTAINER_ID     4
#define MA_USB_CAPRESP_DESC_TYPE_LINK_SLEEP       5
#define MA_USB_CAPRESP_DESC_TYPE_HUB_LATENCY      6

/* Request ID and sequence number values */

#define MA_USB_TRANSFER_RESERVED      0
#define MA_USB_TRANSFER_REQID_MIN     0
#define MA_USB_TRANSFER_REQID_MAX     ((1 <<  8) - 1)
#define MA_USB_TRANSFER_SEQN_MIN      0
#define MA_USB_TRANSFER_SEQN_MAX      ((1 << 24) - 2)
#define MA_USB_TRANSFER_SEQN_INVALID  ((1 << 24) - 1)

#define MA_USB_ISOCH_SFLAGS_FRAGMENT      0x1
#define MA_USB_ISOCH_SFLAGS_LAST_FRAGMENT 0x2

#define MAUSB_MAX_MGMT_SIZE 50

#define MAUSB_TRANSFER_HDR_SIZE (sizeof(struct ma_usb_hdr_common) +\
				 sizeof(struct ma_usb_hdr_transfer))

#define MAUSB_ISOCH_TRANSFER_HDR_SIZE (sizeof(struct ma_usb_hdr_common) +\
			sizeof(struct ma_usb_hdr_isochtransfer) +\
			sizeof(struct ma_usb_hdr_isochtransfer_optional))

#define MAX_ISOCH_ASAP_PACKET_SIZE (150000 /* Network MTU */ -\
	MAUSB_ISOCH_TRANSFER_HDR_SIZE - 20 /* IP header size */ -\
	8 /* UDP header size*/)

#define shift_ptr(ptr, offset) ((uint8_t *)(ptr) + (offset))

/* USB descriptor */
struct ma_usb_desc {
	uint8_t length;
	uint8_t type;
	uint8_t value[0];
} __packed;

struct ma_usb_ep_handle {
	uint16_t d         :1,
		 ep_n      :4,
		 addr      :7,
		 bus_n     :4;
};

struct ma_usb_hdr_mgmt {
	uint32_t status    :8,
		 token     :10,  /* requestor originator allocated */
		 reserved  :14;
} __aligned(4);

struct ma_usb_hdr_ctrl {	/* used in all req/resp/conf operations */
	int8_t status;
	uint8_t link_type;
	union {
		uint8_t tid;	/* ieee 802.11 */
	} connection_id;
} __aligned(4);

struct ma_usb_hdr_data {
	int8_t status;
	uint8_t eps	:2,
		t_flags	:6;
	union {
		uint16_t stream_id;
		struct {
			uint16_t headers  :12,
				 i_flags  :4;
		};
	};
} __aligned(4);

struct ma_usb_hdr_common {
	uint8_t version	:4,
		flags	:4;
	uint8_t  type;
	uint16_t length;
	union {
		uint16_t dev;
		uint16_t epv;
		struct ma_usb_ep_handle eph;
	} handle;
	uint8_t dev_addr;
	uint8_t ssid;
	union {
		int8_t status;
		struct ma_usb_hdr_mgmt mgmt;
		struct ma_usb_hdr_ctrl ctrl;
		struct ma_usb_hdr_data data;
	};
} __aligned(4);

/* capreq extra hdr */

struct ma_usb_hdr_capreq {
	uint32_t out_mgmt_reqs	:12,
		 reserved	:20;
} __aligned(4);

struct ma_usb_capreq_desc_synchronization {
	uint8_t media_time_available  :1,
		reserved              :7;
} __packed;

struct ma_usb_capreq_desc_link_sleep {
	uint8_t link_sleep_capable    :1,
		reserved              :7;
} __packed;

/* capresp extra hdr */

struct ma_usb_hdr_capresp {
	uint16_t endpoints;
	uint8_t devices;
	uint8_t streams		:5,
		dev_type	:3;
	uint32_t descs		:8,
		 descs_length	:24;
	uint16_t out_transfer_reqs;
	uint16_t out_mgmt_reqs	:12,
		 reserved	:4;
} __aligned(4);

struct ma_usb_capresp_desc_speed {
	uint8_t reserved1	:4,
		speed		:4;
	uint8_t reserved2	:4,
		lse		:2,	/* USB3.1 8.5.6.7, Table 8-22 */
		reserved3	:2;
} __packed;

struct ma_usb_capresp_desc_p_managed_out {
	uint8_t elastic_buffer		:1,
		drop_notification	:1,
		reserved		:6;
} __packed;

struct ma_usb_capresp_desc_isochronous {
	uint8_t payload_dword_aligned	:1,
		reserved		:7;
} __packed;

struct ma_usb_capresp_desc_synchronization {
	uint8_t media_time_available	:1,
		time_stamp_required	:1,/* hubs need this set */
		reserved		:6;
} __packed;

struct ma_usb_capresp_desc_container_id {
	uint8_t container_id[16];	/* UUID IETF RFC 4122 */
} __packed;

struct ma_usb_capresp_desc_link_sleep {
	uint8_t link_sleep_capable	:1,
		reserved		:7;
} __packed;

struct ma_usb_capresp_desc_hub_latency {
	uint16_t latency;		/* USB3.1 */
} __packed;

/* usbdevhandlereq extra hdr */
struct ma_usb_hdr_usbdevhandlereq {
	uint32_t route_string	:20,
		 speed		:4,
		 reserved1	:8;
	uint16_t hub_dev_handle;
	uint16_t reserved2;
	uint16_t parent_hs_hub_dev_handle;
	uint16_t parent_hs_hub_port	:4,
		 mtt			:1,	/* USB2.0 11.14, 11.14.1.3 */
		 lse			:2,	/* USB3.1 8.5.6.7, Table 8-22 */
		 reserved3		:9;
} __aligned(4);

/* usbdevhandleresp extra hdr */
struct ma_usb_hdr_usbdevhandleresp {
	uint16_t dev_handle;
	uint16_t reserved;
} __aligned(4);

/* ephandlereq extra hdr */
struct ma_usb_hdr_ephandlereq {
	uint32_t ep_descs	:5,
		 ep_desc_size	:6,
		 reserved	:21;
} __aligned(4);

/*
 * Restricted USB2.0 ep desc limited to 6 bytes, isolating further changes.
 * See USB2.0 9.6.6, Table 9-13
 */
struct usb_ep_desc {
	uint8_t bLength;
	/* USB2.0 9.4, Table 9-5 (5) usb/ch9.h: USB_DT_ENDPOINT */
	uint8_t bDescriptorType;
	uint8_t  bEndpointAddress;
	uint8_t  bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t  bInterval;
} __packed;

/*
 * Restricted USB3.1 ep comp desc isolating further changes in usb/ch9.h
 * See USB3.1 9.6.7, Table 9-26
 */
struct usb_ss_ep_comp_desc {
	uint8_t bLength;
	/* USB3.1 9.4, Table 9-6 (48) usb/ch9.h: USB_DT_SS_ENDPOINT_COMP */
	uint8_t  bDescriptorType;
	uint8_t  bMaxBurst;
	uint8_t  bmAttributes;
	uint16_t wBytesPerInterval;
} __packed;

/*
 * USB3.1 ss_plus_isoch_ep_comp_desc
 * See USB3.1 9.6.8, Table 9-27
 */
struct usb_ss_plus_isoch_ep_comp_desc {
	uint8_t bLength;
	/* USB3.1 9.4, Table 9-6 (49) usb/ch9.h: not yet defined! */
	uint8_t bDescriptorType;
	uint16_t wReserved;
	uint32_t dwBytesPerInterval;
} __packed;

struct ma_usb_ephandlereq_desc_std {
	struct usb_ep_desc usb20;
} __aligned(4);

struct ma_usb_ephandlereq_desc_ss {
	struct usb_ep_desc	   usb20;
	struct usb_ss_ep_comp_desc usb31;
} __aligned(4);

struct ma_usb_ephandlereq_desc_ss_plus {
	struct usb_ep_desc		      usb20;
	struct usb_ss_ep_comp_desc	      usb31;
	struct usb_ss_plus_isoch_ep_comp_desc usb31_isoch;
} __aligned(4);

struct ma_usb_dev_context {
	struct usb_ep_desc usb;
};

/* ephandleresp extra hdr */
struct ma_usb_hdr_ephandleresp {
	uint32_t ep_descs :5,
		 reserved :27;
} __aligned(4);

/* ephandleresp descriptor */
struct ma_usb_ephandleresp_desc {
	union {
		struct ma_usb_ep_handle eph;
		uint16_t		epv;
	} ep_handle;
	uint16_t d	   :1,		/* non-control or non-OUT */
		 isoch	   :1,
		 l_managed :1,		/* control or non-isoch OUT */
		 invalid   :1,
		 reserved1 :12;
	uint16_t ccu;			/* control or non-isoch OUT */
	uint16_t reserved2;
	uint32_t buffer_size;		/* control or OUT */
	uint16_t isoch_prog_delay;	/* in us. */
	uint16_t isoch_resp_delay;	/* in us. */
} __aligned(4);

/* epactivatereq extra hdr */
struct ma_usb_hdr_epactivatereq {
	uint32_t ep_handles :5,
		 reserved   :27;
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle[0];
} __aligned(4);

/* epactivateresp extra hdr */
struct ma_usb_hdr_epactivateresp {
	uint32_t ep_errors :5,
		 reserved  :27;
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle[0];
} __aligned(4);

/* epinactivatereq extra hdr */
struct ma_usb_hdr_epinactivatereq {
	uint32_t ep_handles	:5,
		 suspend	:1,
		 reserved	:26;
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle[0];
} __aligned(4);

/* epinactivateresp extra hdr */
struct ma_usb_hdr_epinactivateresp {
	uint32_t ep_errors	:5,
		 reserved	:27;
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle[0];
} __aligned(4);

/* epresetreq extra hdr */
struct ma_usb_hdr_epresetreq {
	uint32_t ep_reset_blocks :5,
		 reserved	 :27;
} __aligned(4);

/* epresetreq reset block */
struct ma_usb_epresetreq_block {
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle;
	uint16_t tsp	  :1,
		 reserved :15;
} __aligned(4);

/* epresetresp extra hdr */
struct ma_usb_hdr_epresetresp {
	uint32_t ep_errors :5,
		 reserved  :27;
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle[0];
} __aligned(4);

/* cleartransfersreq extra hdr */
struct ma_usb_hdr_cleartransfersreq {
	uint32_t info_blocks	:8,
		 reserved	:24;
} __aligned(4);

/* cleartransfersreq info block */
struct ma_usb_cleartransfersreq_block {
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle;
	uint16_t stream_id;	/* ss stream eps only */
	uint32_t start_req_id	:8,
		 reserved	:24;
} __aligned(4);

/* cleartransfersresp extra hdr */
struct ma_usb_hdr_cleartransfersresp {
	uint32_t status_blocks	:8,
		 reserved	:24;
} __aligned(4);

/* cleartransfersresp status block */
struct ma_usb_cleartransfersresp_block {
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle;
	uint16_t stream_id;	/* ss stream eps only */
	uint32_t cancel_success	  :1,
		 partial_delivery :1,
		 reserved	  :30;
	uint32_t last_req_id	 :8,
		 delivered_seq_n :24;	/* OUT w/partial_delivery only */
	uint32_t delivered_byte_offset;	/* OUT w/partial_delivery only */
} __aligned(4);

/* ephandledeletereq extra hdr */
struct ma_usb_hdr_ephandledeletereq {
	uint32_t ep_handles :5,
		 reserved   :27;
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle[0];
} __aligned(4);

/* ephandledeleteresp extra hdr */
struct ma_usb_hdr_ephandledeleteresp {
	uint32_t ep_errors :5,
		 reserved  :27;
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle[0];
} __aligned(4);

/* modifyep0req extra hdr */
struct ma_usb_hdr_modifyep0req {
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle;
	uint16_t max_packet_size;
} __aligned(4);

/*
 * modifyep0resp extra hdr
 * Only if req ep0 handle addr was 0 and req dev is in the addressed state
 * or  if req ep0 handle addr != 0 and req dev is in default state
 */
struct ma_usb_hdr_modifyep0resp {
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle;

	uint16_t reserved;
} __aligned(4);

/* setusbdevaddrreq extra hdr */
struct ma_usb_hdr_setusbdevaddrreq {
	uint16_t response_timeout;	/* in ms. */
	uint16_t reserved;
} __aligned(4);

/* setusbdevaddrresp extra hdr */
struct ma_usb_hdr_setusbdevaddrresp {
	uint32_t addr	  :7,
		 reserved :25;
} __aligned(4);

/* updatedevreq extra hdr */
struct ma_usb_hdr_updatedevreq {
	uint16_t max_exit_latency;	/* hubs only */
	uint8_t hub	:1,
		ports	:4,
		mtt	:1,
		ttt	:2;
	uint8_t integrated_hub_latency	:1,
		reserved		:7;
} __aligned(4);

/*
 * USB2.0 dev desc, isolating further changes in usb/ch9.h
 * See USB2.0 9.6.6, Table 9-13
 */
struct usb_dev_desc {
	uint8_t bLength;
	uint8_t bDescriptorType;/* USB2.0 9.4, Table 9-5 (1)
				 * usb/ch9.h: USB_DT_DEVICE
				 */
	uint16_t bcdUSB;
	uint8_t  bDeviceClass;
	uint8_t  bDeviceSubClass;
	uint8_t  bDeviceProtocol;
	uint8_t  bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t  iManufacturer;
	uint8_t  iProduct;
	uint8_t  iSerialNumber;
	uint8_t  bNumConfigurations;
} __packed;

struct ma_usb_updatedevreq_desc {
	struct usb_dev_desc usb20;
} __aligned(4);

/* remotewakereq extra hdr */
struct ma_usb_hdr_remotewakereq {
	uint32_t resumed  :1,
		 reserved :31;
} __aligned(4);

/* synchreq/resp extra hdr */
struct ma_usb_hdr_synch {
	uint32_t mtd_valid	:1,	/* MA-USB1.0b 6.5.1.8, Table 66 */
		 resp_required	:1,
		 reserved	:30;
	union {
		uint32_t timestamp;	/* MA-USB1.0b 6.5.1.11 */
		struct {
			uint32_t delta		:13,
				 bus_interval	:19;
		};		/* MA-USB1.0b 6.6.1, Table 69 */
	};
	uint32_t mtd;		/* MA-USB1.0b 6.5.1.12 */
} __aligned(4);

/* canceltransferreq extra hdr */
struct ma_usb_hdr_canceltransferreq {
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle;
	uint16_t stream_id;
	uint32_t req_id	  :8,
		 reserved :24;
} __aligned(4);

/* canceltransferresp extra hdr */
struct ma_usb_hdr_canceltransferresp {
	union {
		uint16_t		epv;
		struct ma_usb_ep_handle eph;
	} handle;
	uint16_t stream_id;
	uint32_t req_id		:8,
		 cancel_status	:3,
		 reserved1	:21;
	uint32_t delivered_seq_n :24,
		 reserved2	 :8;
	uint32_t delivered_byte_offset;
} __aligned(4);

/* transferreq/resp/ack extra hdr */
struct ma_usb_hdr_transfer {
	uint32_t seq_n	:24,
		 req_id	:8;
	union {
		uint32_t rem_size_credit;
		/* ISOCH aliased fields added for convenience. */
		struct {
			uint32_t presentation_time :20,
				 segments	   :12;
		};
	};
} __aligned(4);

/* isochtransferreq/resp extra hdr */
struct ma_usb_hdr_isochtransfer {
	uint32_t seq_n	:24,
		 req_id	:8;
	uint32_t presentation_time :20,
		 segments	   :12;
} __aligned(4);

/* isochtransferreq/resp optional hdr */
struct ma_usb_hdr_isochtransfer_optional {
	union {
		uint32_t timestamp;	/* MA-USB1.0b 6.5.1.11 */
		struct {
			uint32_t delta		:13,
				 bus_interval	:19;
		};		/* MA-USB1.0b 6.6.1, Table 69 */
	};
	uint32_t mtd;		/* MA-USB1.0b 6.5.1.12 */
} __aligned(4);

/* isochdatablock hdrs */

struct ma_usb_hdr_isochdatablock_short {
	uint16_t block_length;
	uint16_t segment_number	:12,
		 s_flags	:4;
} __aligned(4);

struct ma_usb_hdr_isochdatablock_std {
	uint16_t block_length;
	uint16_t segment_number	:12,
		 s_flags	:4;
	uint16_t segment_length;
	uint16_t fragment_offset;
} __aligned(4);

struct ma_usb_hdr_isochdatablock_long {
	uint16_t block_length;
	uint16_t segment_number	:12,
		 s_flags	:4;
	uint32_t segment_length;
	uint32_t fragment_offset;
} __aligned(4);

/* isochreadsizeblock hdrs */

struct ma_usb_hdr_isochreadsizeblock_std {
	uint32_t service_intervals	:12,
		 max_segment_length	:20;
} __aligned(4);

struct ma_usb_hdr_isochreadsizeblock_long {
	uint32_t service_intervals	:12,
		 reserved		:20;
	uint32_t max_segment_length;
} __aligned(4);

static inline struct ma_usb_hdr_transfer *mausb_get_data_transfer_hdr(
	struct ma_usb_hdr_common *hdr)
{
	return (struct ma_usb_hdr_transfer *)shift_ptr(hdr, sizeof(*hdr));
}

static inline struct ma_usb_hdr_isochtransfer *mausb_get_isochtransfer_hdr(
	struct ma_usb_hdr_common *hdr)
{
	return (struct ma_usb_hdr_isochtransfer *)shift_ptr(hdr, sizeof(*hdr));
}

static inline
struct ma_usb_hdr_isochtransfer_optional *mausb_hdr_isochtransfer_optional_hdr(
						struct ma_usb_hdr_common *hdr)
{
	return (struct ma_usb_hdr_isochtransfer_optional *)
			shift_ptr(hdr, sizeof(struct ma_usb_hdr_common) +
				       sizeof(struct ma_usb_hdr_isochtransfer));
}

#endif	/* __MAUSB_COMMON_MA_USB_H__ */
