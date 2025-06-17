#ifndef RFM69_LP_H
#define RFM69_LP_H

#include "rp2x_rfm69_interface.h"

#define LP_UNKNOWN_RETURN (-1)

typedef enum _LP_RX_ERRORS {
	LP_RX_OK,
	LP_RX_TIMEOUT,
	LP_RX_FIFO_READ_FAILURE,
	LP_RX_CRC_FAILURE,
	LP_RX_PKT_SIZE_ERROR,
	LP_RX_BUFFER_OVERFLOW,
} LP_RX_ERROR_T;

typedef enum _LP_TX_ERRORS {
	LP_TX_OK,
	LP_TX_HARDWARE_FAILURE
} LP_TX_ERROR_T;

LP_TX_ERROR_T rfm69_lp_tx(
		rfm69_context_t *rfm,
		const uint8_t *wtp_header,
		const uint8_t *payload_buffer,
		uint8_t payload_size
);

LP_RX_ERROR_T rfm69_lp_rx(
		rfm69_context_t *rfm,
		uint8_t *rx_buffer,
		uint16_t max_size,
		int filter_addr,
		uint32_t timeout_ms,
		int *rssi
);

#endif  // RFM69_LP_H
