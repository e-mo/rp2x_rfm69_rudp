#ifndef RUDP_GENERIC_H
#define RUDP_GENERIC_H
// A generic RUDP interface

#include <stdbool.h>
#include <stdint.h>

enum {
	RX_STATE_WAITING,
	RX_STATE_RECEIVING,
	RX_STATE_FIN_UNCONFIRMED,
	RX_STATE_FIN
};

enum rudp_trx_error {
	// GENERAL
	RUDP_OK,
	RUDP_HARDWARE_ERROR,
	
	// INIT
	RUDP_UNINITIALIZED,
	RUDP_INIT_SUCCESS,
	RUDP_INIT_FAILURE,

	// TX
	RUDP_TX_SUCCESS,
	RUDP_TX_HARDWARE_ERROR,
	RUDP_TX_RESEND_MAX_REACHED,

	// RX
	RUDP_RX_SUCCESS,
	RUDP_RX_HARDWARE_ERROR,
	RUDP_RX_SUCCESS_UNCONFIRMED,
	RUDP_RX_BUFFER_OVERFLOW,
	RUDP_RX_TIMEOUT,

	// bookend
	RUDP_TRX_ERROR_MAX
};

bool rudp_init(struct rudp_context *rudp, const struct rudp_config_s *config);

void rudp_config(rfm69_context_t *rfm);

int rudp_tx(
		struct rudp_context *rudp,
		uint8_t rx_address, 
		uint8_t *payload_buffer,
		uint32_t payload_size
);

int rudp_rx(
		struct rudp_context *rudp,
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

#endif // RFM69_GENERIC_H
