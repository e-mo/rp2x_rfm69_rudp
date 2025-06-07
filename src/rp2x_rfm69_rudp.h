#ifndef RUDP_RFM69_H
#define RUDP_RFM69_H

#include "rudp_generic.h"
#include "rp2x_rfm69_interface.h"

struct rudp_config_s {
	rfm69_context_t *rfm;
	struct rfm69_config_s *rfm_config;
	// Number of retries of a packet before failure in tx
	int tx_resend_max;
	// How long the TX side waits for a response before resending
	int tx_resend_timeout;
	// How long RX side will wait for first packet
	int rx_wait_timeout;
	// How long RX will wait for next packet before declaring
	// transmission dropped
	int rx_drop_timeout;
};


struct rudp_context {
	rfm69_context_t *rfm;		

	int tx_resend_max;
	int tx_resend_timeout;
	int rx_wait_timeout;
	int rx_drop_timeout;

	struct trx_report trx_report;
};

#endif // RUDP_RFM69_H
