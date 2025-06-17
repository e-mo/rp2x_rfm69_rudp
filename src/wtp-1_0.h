#ifndef WTP_1_0_H
#define WTP_1_0_H
//| This file contains definitions for implementing the WTP 
//| (Wisdom Transmission Protocol) 1.0 specification.
//| See WTP_specification.txt included with these drivers.

#include <stdint.h>
#include <assert.h>
#include <stdbool.h>

// Max value of packet size field.
// Actual packet size is always 1 byte large due to the packet size byte not
// being counted in the calculation. 
#define WTP_PKT_SIZE_MAX     (255)
#define WTP_PKT_SIZE_MAX_AES (65)

#define WTP_PKT_TX_SIZE_MAX (WTP_PKT_SIZE_MAX + 1)
#define WTP_PKT_TX_SIZE_MAX_AES (WTP_PKT_SIZE_MAX_AES + 1)


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
#define WTP_TRX_ADDR_MAX   (0xFE)
#define WTP_BROADCAST_ADDR (0xFF)

static inline void wtp_pkt_size_set(uint8_t *wtp_header, uint8_t data_size) {
	assert(data_size <= WTP_PKT_DATA_MAX);

	wtp_header[WTP_HEADER_PKT_SIZE_OFFSET] = 
		WTP_HEADER_SIZE_EFFECTIVE + data_size;
}

static inline uint8_t wtp_pkt_size_get(const uint8_t *wtp_header) {
	return wtp_header[WTP_HEADER_PKT_SIZE_OFFSET];
}

static inline uint8_t wtp_data_size_get(const uint8_t *wtp_header) {
	return wtp_header[WTP_HEADER_PKT_SIZE_OFFSET] - WTP_HEADER_SIZE_EFFECTIVE;
}

static inline void wtp_tx_addr_set(uint8_t *wtp_header, uint8_t tx_addr) {
	wtp_header[WTP_HEADER_TX_ADDR_OFFSET] = tx_addr;
}
static inline uint8_t wtp_tx_addr_get(const uint8_t *wtp_header) {
	return wtp_header[WTP_HEADER_TX_ADDR_OFFSET];
}
static inline void wtp_rx_addr_set(uint8_t *wtp_header, uint8_t rx_addr) {
	wtp_header[WTP_HEADER_RX_ADDR_OFFSET] = rx_addr;
}
static inline uint8_t wtp_rx_addr_get(const uint8_t *wtp_header) {
	return wtp_header[WTP_HEADER_RX_ADDR_OFFSET];
}
static inline void wtp_seq_num_set(uint8_t *wtp_header, uint16_t seq_num) {
	wtp_header[WTP_HEADER_SEQ_NUM_OFFSET] = seq_num & 0xFF;
	wtp_header[WTP_HEADER_SEQ_NUM_OFFSET + 1] = seq_num >> 8;
}
static inline uint16_t wtp_seq_num_get(const uint8_t *wtp_header) {
	uint16_t seq_num = wtp_header[WTP_HEADER_SEQ_NUM_OFFSET];
	seq_num |= wtp_header[WTP_HEADER_SEQ_NUM_OFFSET + 1] << 8;
	return seq_num;
}

static inline void wtp_ack_num_set(uint8_t *wtp_header, uint16_t ack_num) {
	wtp_header[WTP_HEADER_ACK_NUM_OFFSET] = ack_num & 0xFF;
	wtp_header[WTP_HEADER_ACK_NUM_OFFSET + 1] = ack_num >> 8;
}

static inline uint16_t wtp_ack_num_get(const uint8_t *wtp_header) {
	uint16_t ack_num = wtp_header[WTP_HEADER_ACK_NUM_OFFSET];
	ack_num |= wtp_header[WTP_HEADER_ACK_NUM_OFFSET + 1] << 8;
	return ack_num;
}

static inline void wtp_flags_set(uint8_t *wtp_header, uint8_t flags) {
	wtp_header[WTP_HEADER_FLAGS_OFFSET] |= flags;
}

static inline void wtp_flags_clear(uint8_t *wtp_header, uint8_t flags) {
	wtp_header[WTP_HEADER_FLAGS_OFFSET] &= ~flags;
}

static inline void wtp_flags_clear_all(uint8_t *wtp_header) {
	wtp_flags_clear(wtp_header, 0xFF);
}

static inline uint8_t wtp_flags_get(const uint8_t *wtp_header) {
	return wtp_header[WTP_HEADER_FLAGS_OFFSET];
}

static inline bool wtp_flags_are_set(const uint8_t *wtp_header, uint8_t flags) {
	return (wtp_flags_get(wtp_header) & flags) == flags;
}

static inline void wtp_seq_num_inc(uint8_t *wtp_header) {
	wtp_seq_num_set(wtp_header, wtp_seq_num_get(wtp_header)+1);	
}

static inline void wtp_ack_num_inc(uint8_t *wtp_header) {
	wtp_ack_num_set(wtp_header, wtp_ack_num_get(wtp_header)+1);	
}

static inline uint8_t wtp_data_size_calc(ptrdiff_t bytes_remaining) {
	if (bytes_remaining > WTP_PKT_DATA_MAX)
		return WTP_PKT_DATA_MAX;
		
	return bytes_remaining;
}

static inline bool wtp_rx_flags_valid(
		const uint8_t *rx_header, 
		const uint8_t *tx_packet
) {
	bool rx_ack = wtp_flags_are_set(rx_header, WTP_FLAG_ACK);
	bool rx_syn = wtp_flags_are_set(rx_header, WTP_FLAG_SYN);
	bool rx_fin = wtp_flags_are_set(rx_header, WTP_FLAG_FIN);

	bool tx_ack = wtp_flags_are_set(tx_packet, WTP_FLAG_ACK);
	bool tx_syn = wtp_flags_are_set(tx_packet, WTP_FLAG_SYN);
	bool tx_rtr = wtp_flags_are_set(tx_packet, WTP_FLAG_RTR);
	bool tx_fin = wtp_flags_are_set(tx_packet, WTP_FLAG_FIN);

	// If Rx FIN is set, Tx FIN must also be.
	if (rx_fin && !tx_fin)
		return false;

	// If Rx ACK is not set, Rx flags should be clear.
	if (!rx_ack && wtp_flags_get(rx_header))
		return false;
	// We can assume Rx ACK is set if any other flags are set.

	if (tx_syn) { 
		// Tx SYN and Tx ACK are mutually exclusive.
		if (tx_ack)
			return false;

		if (rx_syn)
			return tx_rtr;

		// !rx_syn
		// tx_syn
		// Valid if Rx ACK is not set, otherwise invalid.
		return !rx_ack;

	// If Tx SYN is not set, both sides should have ACK set.
	} else if (!tx_ack || !rx_ack) {
		return false;
	}
	
	// !tx_syn
	// rx_ack
	// tx_ack
	// Valid state.
	return true;
}

enum wtp_pkt_state {
	WTP_PKT_STATE_SENTRY = -1,

	// General
	WTP_PKT_STATE_INVALID,

	// TX
	WTP_PKT_STATE_VALID_ACK,

	// RX
	WTP_PKT_STATE_VALID_FIRST,
	WTP_PKT_STATE_VALID_PREV,
	WTP_PKT_STATE_VALID_NEXT,

	WTP_PKT_STATE_BOOKEND
};

static inline int wtp_rx_pkt_state(
		const uint8_t *rx_header, 
		const uint8_t *tx_packet
) {
	// If headers are invalid.
	if (!wtp_rx_flags_valid(rx_header, tx_packet))
		return WTP_PKT_STATE_INVALID;

	// If Rx ACK is set, we have received at least one packet and can
	// check for valid Tx ADDR.
	if (wtp_flags_are_set(rx_header, WTP_FLAG_ACK)) {
		uint8_t old_addr = wtp_rx_addr_get(rx_header);
		uint8_t new_addr = wtp_tx_addr_get(tx_packet);
		if (new_addr != old_addr)
			return WTP_PKT_STATE_INVALID;
	}

	uint8_t rx_ack = wtp_ack_num_get(rx_header);
	uint8_t rx_seq = wtp_seq_num_get(rx_header);
	uint8_t tx_ack = wtp_ack_num_get(tx_packet);
	uint8_t tx_seq = wtp_seq_num_get(tx_packet);

	bool tx_syn = wtp_flags_are_set(tx_packet, WTP_FLAG_SYN);
	bool rx_syn = wtp_flags_are_set(rx_header, WTP_FLAG_SYN);

	// First packet?
	if (tx_syn && !rx_syn)
		return WTP_PKT_STATE_VALID_FIRST;

	int packet_state = WTP_PKT_STATE_SENTRY;

	// Is this the next packet?
	if (rx_ack == tx_seq) {
		// This is a Tx SYN|RTR packet, which is not valid.
		if (tx_syn)
			return WTP_PKT_STATE_INVALID;

		if (tx_ack != (rx_seq+1))
			return WTP_PKT_STATE_INVALID;

		packet_state = WTP_PKT_STATE_VALID_NEXT;

	// Is this the previous packet?
	} else if (rx_ack == (tx_seq+1)) {

		if (!tx_syn && (tx_ack != rx_seq))
			return WTP_PKT_STATE_INVALID;

		packet_state = WTP_PKT_STATE_VALID_PREV;

	// If neither, packet is invalid.
	} else {
		return WTP_PKT_STATE_INVALID;
	}

	if (packet_state == WTP_PKT_STATE_VALID_PREV) {
		int tx_flags = wtp_flags_get(tx_packet) | WTP_FLAG_ACK;
		int rx_flags = wtp_flags_get(rx_header) | WTP_FLAG_RTR;

		if (tx_flags != rx_flags)
			return WTP_PKT_STATE_INVALID;
	}

	return packet_state;
}

// Generic TRX Function Prototypes.


#endif // WTP_1_0_H
