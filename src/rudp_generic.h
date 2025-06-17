#ifndef RUDP_GENERIC_H
#define RUDP_GENERIC_H
// A generic RUDP interface

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

enum rudp_trx_error {
	RUDP_UNKNOWN_RETURN = -1,
	RUDP_UNINITIALIZED,
	RUDP_INIT_SUCCESS,
	RUDP_INIT_FAILURE,

	RUDP_OK,
	RUDP_TIMEOUT,
	RUDP_BAD_PACKET,
	RUDP_HARDWARE_ERROR,
	RUDP_BUFFER_OVERFLOW,
	RUDP_RESEND_MAX_REACHED,

	// bookend
	RUDP_TRX_ERROR_MAX
};

struct rudp_context;
struct rudp_config_s;

struct trx_report {
	int tx_addr;
	int rx_addr;
	int last_rssi;
	int rtr_count;
	int bad_pkt_count;
	int payload_size;
};

bool rudp_init(struct rudp_context *rudp, const struct rudp_config_s *config);

void rudp_config(void *rfm_ctx);

int rudp_tx(
		struct rudp_context *rudp,
		int rx_address, 
		const uint8_t *payload_buffer,
		ptrdiff_t payload_size
);

int rudp_rx(
		struct rudp_context *rudp,
		uint8_t *payload_buffer,
		ptrdiff_t buffer_size
);

bool rudp_tx_broadcast(
		struct rudp_context *rudp,
		uint8_t *payload_buffer,
		ptrdiff_t *payload_size
);

bool rudp_rx_broadcast(
		struct rudp_context *rfm,
		uint8_t *payload_buffer,
		ptrdiff_t *buffer_size
);

#endif // RFM69_GENERIC_H
