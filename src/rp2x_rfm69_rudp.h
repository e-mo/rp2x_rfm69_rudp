#ifndef RFM69_RUDP_WTP_H
#define RFM69_RUDP_WTP_H


#include <stdbool.h>
#include <stdint.h>

#include "wtp-1_0.h"
#include "rfm69_rp2040_interface.h"

// 255 - address byte
#define VP_TX_PAYLOAD_MAX (254)
// VP_TX_PAYLOAD_MAX + size byte + address byte
#define VP_RX_BUFFER_MAX (VP_TX_PAYLOAD_MAX + 2)

bool rfm69_rudp_init(rfm69_context_t *rfm, const struct rfm69_config_s *config);

bool rfm69_tx_variable_packet(
		rfm69_context_t *rfm,
		uint8_t rx_address, 
		uint8_t *payload_buffer,
		uint8_t payload_size
);

typedef enum _VP_RX_ERRORS {
	VP_RX_OK,
	VP_RX_TIMEOUT,
	VP_RX_FIFO_READ_FAILURE,
	VP_RX_BUFFER_TOO_SMALL,
	VP_RX_CRC_FAILURE
} VP_RX_ERROR_T;

VP_RX_ERROR_T rfm69_rx_variable_packet(
		rfm69_context_t *rfm,
		uint8_t *rx_buffer,
		uint16_t buffer_size,
		uint16_t timeout_ms
);

bool rfm69_rudp_tx(
		rfm69_context_t *rfm,
		uint8_t rx_address, 
		uint8_t *payload_buffer,
		uint32_t payload_size
);

#endif // RFM69_RUDP_WTP_H
