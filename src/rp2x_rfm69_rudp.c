// rfm69_pico_rudp.c
// Interface implementation for rfm69_pico Reliable UDP

//	Copyright (C) 2024 
//	Evan Morse
//	Amelia Vlahogiannis

//	This program is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program.  If not, see <https://www.gnu.org/licenses/>.
#include <stdio.h>

#include "pico/rand.h"
#include "pico/time.h"


#include "wtp-1_0.h"
#include "rp2x_rfm69_rudp.h"

#include "rfm69_vp.h"
#include "rudp_validation.h"
#include "rp2x_gpio_irq.h"

// 255 - address byte
#define VP_TX_PAYLOAD_MAX (254)
// VP_TX_PAYLOAD_MAX + size byte + address byte
#define VP_RX_BUFFER_MAX (VP_TX_PAYLOAD_MAX + 2)

// Helper function to initialize rfm69 context for use with rudp interface
// Same as calling rfm69_init and then rfm69_rudp_config
bool rudp_init(struct rudp_context *rudp, const struct rudp_config_s *config) {
	if (rfm69_init(config->rfm, config->rfm_config) == false) return false; 

	rudp_config(config->rfm);

	rudp->rfm = config->rfm;
	rudp->tx_resend_max = config->tx_resend_max;
	// default: 10 resend max
	if (rudp->tx_resend_max == 0)
		rudp->tx_resend_max = 10;

	rudp->tx_resend_timeout = config->tx_resend_timeout;
	// default: 100ms resend timeout
	if (rudp->tx_resend_timeout == 0)
		rudp->tx_resend_timeout = 100;

	rudp->rx_wait_timeout = config->rx_wait_timeout;
	// default: 30s wait timeout
	if (rudp->rx_wait_timeout == 0)
		rudp->rx_wait_timeout = 30000;

	rudp->rx_drop_timeout = config->rx_drop_timeout;
	// default: 2s drop timeout
	if (rudp->rx_drop_timeout == 0)
		rudp->rx_drop_timeout = 2000;

	return true;
}

void rudp_config(void *rfm_ctx) {
	// Init rp2x gpio irq lib
	rp2x_gpio_irq_init();
	rfm69_context_t *rfm = (rfm69_context_t *)rfm_ctx; 

	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);

    rfm69_dcfree_set(rfm, RFM69_DCFREE_WHITENING);
	rfm69_packet_format_set(rfm, RFM69_PACKET_VARIABLE);
	rfm69_payload_length_set(rfm, WTP_PKT_SIZE_MAX);

	// RFM69_FIFO_SIZE/2 == 33
	rfm69_fifo_threshold_set(rfm, RFM69_FIFO_SIZE/2);
	rfm69_crc_autoclear_set(rfm, false);

	// DIO settings
	// DIO0 changes between RX and TX so isn't set here
	rfm69_dio1_config_set(rfm, RFM69_DIO1_PKT_TX_FIFO_LVL);
	rfm69_dio2_config_set(rfm, RFM69_DIO2_PKT_TX_FIFO_N_EMPTY);

	// TEMPORARY TESTING STUFF
	//rfm69_power_level_set(rfm, 0);
    //rfm69_bitrate_set(rfm, RFM69_MODEM_BITRATE_300);
	//rfm69_fdev_set(rfm, 300000);
	//rfm69_rxbw_set(rfm, RFM69_RXBW_MANTISSA_16, 0);

	// Make sure we are sleeping as a default state
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);
}

typedef enum _ack_status_enum {
	ACK_RECEIVED,
	ACK_TIMEOUT,
	ACK_HARDWARE_ERROR
} ACK_STATUS_T;

static ACK_STATUS_T _ack_received(
		rfm69_context_t *rfm, 
		uint8_t *tx_header,
		uint8_t *rx_header, 
		int ack_timeout
)
{
	ACK_STATUS_T ack_status = -1;

	for (;;) {
		absolute_time_t start_time = get_absolute_time();

		VP_RX_ERROR_T rx_rval = rfm69_vp_rx(
				rfm, 
				rx_header, 
				WTP_HEADER_SIZE,
				ack_timeout
		);

		// If OK
		if (rx_rval == VP_RX_OK) {
			if (ack_valid(tx_header, rx_header)) {
				ack_status = ACK_RECEIVED;
				break;
			}
		}

		// If timeout, break in a success = false state
		else if (rx_rval == VP_RX_TIMEOUT) {
			ack_status = ACK_TIMEOUT;
			break;
		} 

		// If we had a hardware failure, set return and go to cleanup
		else if (rx_rval == VP_RX_FIFO_READ_FAILURE) {
			ack_status = ACK_HARDWARE_ERROR;
			break;
		} 

		// get time elapsed here
		// subtract from ack_timeout
		int32_t t_diff = absolute_time_diff_us(
				start_time, get_absolute_time()) / 1000;

		ack_timeout = t_diff >= ack_timeout ? 
			0 : ack_timeout - t_diff;
	}

	return ack_status;
}

int rudp_tx(
		struct rudp_context *rudp,
		int rx_address, 
		uint8_t *payload_buffer,
		ptrdiff_t payload_size
)
{
	// Default value to catch errors in function
	enum rudp_trx_error return_status = -1;
	rfm69_context_t *rfm = rudp->rfm;

	// Start and end in sleep so we always knows our inital and ending state
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);
	// Make sure we are only filtering by node address and ignoring broadcast
	rfm69_address_filter_set(rfm, RFM69_FILTER_NODE);

	// Packet buffers and index structs
	uint8_t tx_header[WTP_HEADER_SIZE] = {0};
	struct wtp_header_index tx_index;
	wtp_index_init(&tx_index, tx_header);

	uint8_t rx_header[WTP_HEADER_SIZE];
	struct wtp_header_index rx_index;
	wtp_index_init(&rx_index, rx_header);

	// Syn always set in first packet
	*tx_index.flags |= WTP_FLAG_SYN;

	// Set RX/TX address here because they do not change
	*tx_index.rx_addr = rx_address;
	rfm69_node_address_get(rfm, tx_index.tx_addr);	

	// Generate sequence num for first packet
	*tx_index.seq_num = get_rand_32()/2;

	// TX loop
	uint8_t send_data_size = 0;
	uint8_t resend_count = 0;
	for (;;) {
		if (resend_count > rudp->tx_resend_max) {
			return_status = RUDP_TX_RESEND_MAX_REACHED;
			goto CLEANUP;
		}

		// If retry flag is not set we build a new packet
		if ((*tx_index.flags & WTP_FLAG_RTR) == 0) {
			send_data_size = payload_size > WTP_PKT_DATA_MAX ? 
				WTP_PKT_DATA_MAX : payload_size;

			// Set size in header
			*tx_index.pkt_size = WTP_HEADER_SIZE_EFFECTIVE + send_data_size;

			// Keep track of what we have left and set FIN flag if done
			payload_size -= send_data_size;
			if (payload_size == 0)
				*tx_index.flags |= WTP_FLAG_FIN;
		}

		// Send packet
		VP_TX_ERROR_T tx_rval = rfm69_vp_tx(
				rfm, tx_header, payload_buffer, send_data_size);

		// Only way a send can fail is if the transmitter itself returns
		// a hardware error. 
		if (tx_rval == VP_TX_FIFO_WRITE_FAILURE) {
			return_status = RUDP_TX_HARDWARE_ERROR;
			goto CLEANUP;
		}
		// If here tx_rval == VP_TX_OK

		// WAIT FOR ACK
		// There should be a total timeout in a loop here maybe?
		// 100 ms as testing value
		// TODO: Figure out what the fuck to do with this.
		// 	- Put it in config?
		// 	- Make it a constant?
		// 	- Perhaps set to good enough for slowest and be done with it?
		ACK_STATUS_T ack_status = _ack_received(
				rfm, tx_header, rx_header, rudp->tx_resend_timeout);

		// If ack not received,
		if (ack_status != ACK_RECEIVED) {
			// In case of hardware issue
			if (ack_status == ACK_HARDWARE_ERROR) {
				return_status = RUDP_TX_HARDWARE_ERROR;
				goto CLEANUP;
			}

			*tx_index.flags |= WTP_FLAG_RTR;
			resend_count++;
		 	continue;
		};
		resend_count = 0;

		// Update seq/ack nums
		*tx_index.seq_num += 1;
		*tx_index.ack_num = *rx_index.seq_num;
		*tx_index.ack_num += 1;
		// Ensure ACK flag is set from this point forward;
		*tx_index.flags |= WTP_FLAG_ACK;

		// Ensure RTR and SYN flags are cleared because SYN should
		// always be cleared after first ACK and RTR is no longer valid
		// if we just receieved a valid ACK.
		*tx_index.flags &= ~(WTP_FLAG_RTR | WTP_FLAG_SYN);

		// If payload_size == 0 and we have received ACK
		// we are ready to send our final goodbye packet and close connection.
		if (payload_size == 0)
			break;

		// Move payload_buffer forward if we are going to send another
		// data packet
		payload_buffer += send_data_size;

	}

	*tx_index.pkt_size = WTP_HEADER_SIZE_EFFECTIVE;
	// I don't error check here because I would not expect a hardware error
	// that wouldn't have presented earlier and already generated an error.
	rfm69_vp_tx(rfm, tx_header, NULL, 0);

	return_status = RUDP_TX_SUCCESS;
CLEANUP:;
	// Ensure we are in sleep mode
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	return return_status;
}



int rudp_rx(
		struct rudp_context *rudp,
		uint8_t *payload_buffer,
		ptrdiff_t buffer_size,
		ptrdiff_t *received,
		int *tx_address
)
{
	enum rudp_trx_error return_status = -1;
	uint8_t *buffer_p = payload_buffer;

	rfm69_context_t *rfm = rudp->rfm;
	// Start and end in sleep so we always knows our inital and ending state
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);
	// Make sure we are only filtering by node address and ignoring broadcast
	rfm69_address_filter_set(rfm, RFM69_FILTER_NODE);

	// Packet buffers and index structs
	uint8_t tx_packet[WTP_HEADER_SIZE + WTP_PKT_DATA_MAX];
	struct wtp_header_index tx_index;
	wtp_index_init(&tx_index, tx_packet);

	uint8_t rx_header[WTP_HEADER_SIZE];
	struct wtp_header_index rx_index;
	wtp_index_init(&rx_index, rx_header);

	// Get our address here because it does not change
	rfm69_node_address_get(rfm, rx_index.tx_addr);	

	// Generate sequence num for first packet
	*rx_index.seq_num = get_rand_32()/2;

	// Packet size always effective header size
	*rx_index.pkt_size = WTP_HEADER_SIZE_EFFECTIVE;

	// RX loop vars
	int rx_state = RX_STATE_WAITING;
	*received = 0;
	uint32_t data_size;
	bool success = false;
	uint32_t rx_timeout = rudp->rx_wait_timeout;
	int64_t time_elapsed_us = 0;
	for (;;) {

		// Track how much time has been spent waiting for packets for timeout
		// Start time

		absolute_time_t start_time = get_absolute_time();

		uint32_t elapsed_ms = time_elapsed_us / 1000;
		uint32_t timeout_ms = elapsed_ms >= rx_timeout ?
			0 : rx_timeout - elapsed_ms;

		VP_RX_ERROR_T rx_rval = rfm69_vp_rx(
			rfm, tx_packet, WTP_HEADER_SIZE + WTP_PKT_DATA_MAX, timeout_ms);

		time_elapsed_us += absolute_time_diff_us(start_time, get_absolute_time());


		// End time

		// Radio hardware error. GTFO
		if (rx_rval == VP_RX_FIFO_READ_FAILURE) {
			return_status = RUDP_RX_HARDWARE_ERROR;
			goto CLEANUP;
		}

		// Timeout reached. GTFO
		if (rx_rval == VP_RX_TIMEOUT) {
			if (rx_state == RX_STATE_FIN_UNCONFIRMED) {
				break;
			}

			return_status = RUDP_RX_TIMEOUT;
			goto CLEANUP;
		}


		// Bad packet, go back to waiting
		if (rx_rval == VP_RX_CRC_FAILURE) {
			continue;
		}


		// Good CRC, check packet
		PACKET_STATE_T p_state = packet_check(rx_header, tx_packet, rx_state);

		// Check for invalid states
		switch (p_state) {
		// Packet invalid states. Go back to waiting for a good packet. 
		case PACKET_INVALID_ADDR:
			//printf("invalid addr\n");
			continue;
		case PACKET_INVALID_SYN:
			//printf("invalid syn\n");
			continue;
		case PACKET_INVALID_ACK:
			//printf("invalid ack\n");
			continue;
		case PACKET_INVALID_SEQ:
			//printf("invalid seq\n");
			continue;
		case PACKET_INVALID_FLAGS:
			//printf("invalid flags\n");
			continue;
		// If tx has restarted, reset all necessary vars
		case PACKET_VALID_TX_RESTART:
			buffer_p = payload_buffer;
			*received = 0;
			rx_state = RX_STATE_WAITING;
			p_state = PACKET_VALID_NEW;
		}

		// We have received a packet and it is valid, so we can reset our
		// elapsed time
		time_elapsed_us = 0;
		
		// If this is our first packet we need to set a few things
		if (rx_state == RX_STATE_WAITING) {
			// We have received our packet and thus should no longer
			// use the wait timeout.
			rx_timeout = rudp->rx_drop_timeout;
			*(rx_index.rx_addr) = *(tx_index.tx_addr);
			// save tx address to output variable
			*tx_address = *(tx_index.tx_addr);
			*(rx_index.ack_num) = *(tx_index.seq_num);
			rx_state = RX_STATE_RECEIVING;
		} 

		// If this is the next packet in the sequence (or the first)
		// we can update our ack header and copy data segment into buffer
		if (p_state == PACKET_VALID_NEW) {

			// If FIN is set
			if ((*tx_index.flags & WTP_FLAG_FIN) != 0) {
				if (rx_state == RX_STATE_FIN_UNCONFIRMED) {
					rx_state = RX_STATE_FIN;
					break;
				} else {
					rx_state = RX_STATE_FIN_UNCONFIRMED;
				}
			}

			data_size = *(tx_index.pkt_size) - WTP_HEADER_SIZE_EFFECTIVE;
			*received += data_size;
			// This prevents a buffer overflow
			if (*received > buffer_size) {
				// Change received back to reflect how much was actually
				// received.
				*received -= data_size;
				return_status = RUDP_RX_BUFFER_OVERFLOW;
				goto CLEANUP;
			}

			// Copy data into buffer
			for (int i = 0; i < data_size; i++) {
				*buffer_p = tx_index.data[i];
				buffer_p++;
			}

			// Update seq and ack nums
			*(rx_index.seq_num) += 1;
			*(rx_index.ack_num) += 1;
		}

		// Set proper flags (same as TX and always ACK)
		*(rx_index.flags) = *(tx_index.flags) | WTP_FLAG_ACK;		

		rfm69_vp_tx(rfm, rx_header, NULL, 0);

	}
	
	// Only two possible states here:
	//  RX_STATE_FIN or RX_STATE_FIN_UNCONFIRMED
	//  Either way, the transmission was a success, we just can't be 100%
	//  sure that the TX side KNOWS it was 100% a success.
	//  Hence potential success unconfirmed state.
	return_status = rx_state == RX_STATE_FIN ?
		RUDP_RX_SUCCESS : RUDP_RX_SUCCESS_UNCONFIRMED;

CLEANUP:;
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	return return_status;
}
