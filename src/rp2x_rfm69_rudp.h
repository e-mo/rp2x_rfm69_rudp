#ifndef RUDP_RFM69_H
#define RUDP_RFM69_H

#include "rp2x_rfm69_interface.h"

struct rudp_config_s {
	rfm69_context_t *rfm;
	struct rfm69_config_s *rfm_config;
	// Number of retries of a packet before failure in tx
	uint32_t tx_resend_max;
	// How long the TX side waits for a response before resending
	uint32_t tx_resend_timeout;
	// How long RX side will wait for first packet
	uint32_t rx_wait_timeout;
	// How long RX will wait for next packet before declaring
	// transmission dropped
	uint32_t rx_drop_timeout;
};

struct rudp_context {
	rfm69_context_t *rfm;		
	uint32_t tx_resend_max;
	uint32_t tx_resend_timeout;
	uint32_t rx_wait_timeout;
	uint32_t rx_drop_timeout;
};

#include "rudp_generic.h"

#endif // RUDP_RFM69_H
