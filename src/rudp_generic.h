#ifndef RUDP_GENERIC_H
#define RUDP_GENERIC_H
// A generic RUDP interface

#include <stdbool.h>
#include <stdint.h>

typedef struct _rudp_context rudp_context_t;

enum {
	RX_STATE_WAITING,
	RX_STATE_RECEIVING,
	RX_STATE_FIN_UNCONFIRMED,
	RX_STATE_FIN
};

typedef enum  _rudp_tx_status_enum {
	RUDP_TX_SUCCESS,
	RUDP_TX_HARDWARE_ERROR,
	RUDP_TX_RESEND_MAX_REACHED
} RUDP_TX_STATUS_T;

typedef enum  _rudp_rx_status_enum {
	RUDP_RX_SUCCESS,
	RUDP_RX_SUCCESS_UNCONFIRMED,
	RUDP_RX_HARDWARE_ERROR,
	RUDP_RX_BUFFER_OVERFLOW,
	RUDP_RX_TIMEOUT
} RUDP_RX_STATUS_T;

bool rudp_init(rudp_context_t *rudp, const struct rudp_config_s *config);

void rudp_config(rfm69_context_t *rfm);

RUDP_TX_STATUS_T rudp_tx(
		rudp_context_t *rudp,
		uint8_t rx_address, 
		uint8_t *payload_buffer,
		uint32_t payload_size
);

RUDP_RX_STATUS_T rudp_rx(
		rudp_context_t *rudp,
		uint8_t *payload_buffer,
		uint32_t buffer_size,
		uint32_t *received
);

bool rudp_tx_broadcast(
		rfm69_context_t *rfm,
		uint8_t *payload_buffer,
		uint32_t payload_size
);

bool rudp_rx_broadcast(
		rfm69_context_t *rfm,
		uint8_t *payload_buffer,
		uint32_t buffer_size,
		uint32_t timeout_ms
);

#endif // RFM69_RUDP_WTP_H
