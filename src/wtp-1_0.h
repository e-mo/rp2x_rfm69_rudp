#ifndef WTP_1_0_H
#define WTP_1_0_H
//| This file contains definitions for implementing the WTP 
//| (Wisdom Transmission Protocol) 1.0 specification.
//| See WTP_specification.txt included with these drivers.

#include <stdint.h>

// Max value of packet size field.
// Actual packet size is always 1 byte large due to the packet size byte not
// being counted in the calculation. 
#define WTP_PKT_SIZE_MAX     (255)
#define WTP_PKT_SIZE_MAX_AES (65)

// Size of packet header in bytes
#define WTP_HEADER_SIZE (8)
#define WTP_HEADER_SIZE_EFFECTIVE (WTP_HEADER_SIZE - 1)

// Header offsets from beginning of packet
// uint8_t packet_size = header[WTP_HEADER_OFFSET_PKT_SIZE];
#define WTP_HEADER_PKT_SIZE_OFFSET (0) // 1 byte
#define WTP_HEADER_RX_ADDR_OFFSET  (1) // 1 byte
#define WTP_HEADER_TX_ADDR_OFFSET  (2) // 1 byte
#define WTP_HEADER_FLAGS_OFFSET    (3) // 1 byte
#define WTP_HEADER_SEQ_NUM_OFFSET  (4) // 2 bytes
#define WTP_HEADER_ACK_NUM_OFFSET  (6) // 2 bytes

// Flag masks
#define WTP_FLAG_SYN (0x01)
#define WTP_FLAG_ACK (0x02)
#define WTP_FLAG_FIN (0x04)
#define WTP_FLAG_RTR (0x08)

// Data segment is directly after header
#define WTP_DATA_SEGMENT_OFFSET (WTP_HEADER_SIZE)

// +1 here comes from the fact that the first byte of the header (packet size)
// is not included included in the packet size. (i.e. packet size byte does not
// count itself).
#define WTP_PKT_DATA_MAX (WTP_PKT_SIZE_MAX - WTP_HEADER_SIZE_EFFECTIVE)
#define WTP_PKT_DATA_MAX_AES (WTP_PKT_SIZE_MAX_AES - WTP_HEADER_SIZE_EFFECTIVE)

// Special broadcast address
#define WTP_BROADCAST_ADDRESS (0xFF)

struct wtp_header_index {
	uint8_t *pkt_size;
	uint8_t *rx_addr;
	uint8_t *tx_addr;
	uint8_t *flags;
	uint16_t *seq_num;
	uint16_t *ack_num;
	uint8_t *data;
};

void wtp_index_init(struct wtp_header_index *index, uint8_t *buffer);

#endif // WTP_1_0_H
