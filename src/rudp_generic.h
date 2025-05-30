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
	RUDP_UNINITIALIZED, // 2
	RUDP_INIT_SUCCESS,
	RUDP_INIT_FAILURE,

	// TX
	RUDP_TX_SUCCESS, // 5
	RUDP_TX_HARDWARE_ERROR,
	RUDP_TX_RESEND_MAX_REACHED,

	// RX
	RUDP_RX_SUCCESS, // 8
	RUDP_RX_HARDWARE_ERROR,
	RUDP_RX_SUCCESS_UNCONFIRMED,
	RUDP_RX_BUFFER_OVERFLOW,
	RUDP_RX_TIMEOUT,

	// bookend
	RUDP_TRX_ERROR_MAX
};

struct rudp_context;
struct rudp_config_s;

bool rudp_init(struct rudp_context *rudp, const struct rudp_config_s *config);

void rudp_config(void *rfm_ctx);

int rudp_tx(
		struct rudp_context *rudp,
		int rx_address, 
		uint8_t *payload_buffer,
		ptrdiff_t payload_size
);

int rudp_rx(
		struct rudp_context *rudp,
		uint8_t *payload_buffer,
		ptrdiff_t buffer_size,
		ptrdiff_t *received,
		int *tx_address
);

bool rudp_tx_broadcast(
		struct rudp_context *rudp,
		uint8_t *payload_buffer,
		ptrdiff_t *payload_size
);

bool rudp_rx_broadcast(
		rfm69_context_t *rfm,
		uint8_t *payload_buffer,
		ptrdiff_t *buffer_size,
		ptrdiff_t *received,
		int *tx_address
);

#endif // RFM69_GENERIC_H
