#ifndef RFM69_VP_H
#define RFM69_VP_H

#include "rp2x_rfm69_interface.h"

typedef enum _VP_RX_ERRORS {
	VP_RX_OK,
	VP_RX_TIMEOUT,
	VP_RX_FIFO_READ_FAILURE,
	VP_RX_CRC_FAILURE
} VP_RX_ERROR_T;

typedef enum _VP_TX_ERRORS {
	VP_TX_OK,
	VP_TX_FIFO_WRITE_FAILURE
} VP_TX_ERROR_T;

VP_TX_ERROR_T rfm69_vp_tx(
		rfm69_context_t *rfm,
		uint8_t *header, 
		uint8_t *payload_buffer,
		uint8_t payload_size
);

VP_RX_ERROR_T rfm69_vp_rx(
		rfm69_context_t *rfm,
		uint8_t *rx_buffer,
		uint16_t buffer_size,
		int timeout_ms,
		int *rssi
);

#endif  // RFM69_VP_H
