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
// #include <stdio.h>

#include "pico/rand.h"
#include "pico/time.h"
#include "hardware/dma.h"

#include "wtp-1_0.h"
#include "rp2x_rfm69_rudp.h"

#include "rfm69_lp.h"
#include "rp2x_gpio_irq.h"

static void trx_report_reset(struct trx_report *report) {
	report->tx_addr       = -1;
	report->rx_addr       = -1;
	report->last_rssi     = 0x1AC;
	report->rtr_count     = 0;
	report->bad_pkt_count = 0;
	report->payload_size  = 0;
}

static uint16_t seq_num_generate(void) {
	return (uint16_t)(get_rand_32()/2);
}

// Helper function to initialize rfm69 context for use with rudp interface
// Same as calling rfm69_init and then rfm69_rudp_config
bool rudp_init(struct rudp_context *rudp, const struct rudp_config_s *config) {
	//* INITIALIZED HARDWARE *//

	// Call init for the underlying rfm69 drivers.
	// Only fail point in function.
	if (rfm69_init(config->rfm, config->rfm_config) == false) 
		return false; 

	// Apply rudp configuration to radio.
	rudp_config(config->rfm);

	// Cache pointer to radio hardware context.
	rudp->rfm = config->rfm;

	//* SET UNINTIALIZED TRX CONFIG VALUES *//

	// How many retransmission attempts on a packet before giving up.
	rudp->tx_resend_max = config->tx_resend_max;
	// default: 10 resend max
	if (rudp->tx_resend_max == 0)
		rudp->tx_resend_max = 10;

	// How long (ms) TX will wait for ACK before resending last packet.
	rudp->tx_resend_timeout = config->tx_resend_timeout;
	// default: 100ms resend timeout
	if (rudp->tx_resend_timeout == 0)
		rudp->tx_resend_timeout = 100;

	// How long (ms) RX will wait for first packet before timing out.
	// After first packet has been received, rx_drop_timeout becomes the
	// timeout.
	rudp->rx_wait_timeout = config->rx_wait_timeout;
	// default: 10s wait timeout
	if (rudp->rx_wait_timeout == 0)
		rudp->rx_wait_timeout = 10000;

	// How long (ms) RX will wait for any valid packet after the first before 
	// timing out. This defaults to the same value as rx_wait_timeout.
	// Timeout resets after any valid packet is received.
	rudp->rx_drop_timeout = config->rx_drop_timeout;
	// default: rx_wait_timeout
	if (rudp->rx_drop_timeout == 0)
		rudp->rx_drop_timeout = rudp->rx_wait_timeout;

	// The rudp context caches info about the last TRX that is valid
	// if TRX is successful.
	trx_report_reset(&rudp->trx_report);

	// Only fail point is the rfm69 hardware init.
	return true;
}

void rudp_config(void *rfm_ctx) {
	// Init rp2x gpio irq lib.
	rp2x_gpio_irq_init();
	rfm69_context_t *rfm = rfm_ctx; 

	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);
	rfm69_fifo_threshold_set(rfm, RFM69_FIFO_SIZE/2);

    rfm69_dcfree_set(rfm, RFM69_DCFREE_WHITENING);
	rfm69_packet_format_set(rfm, RFM69_PACKET_VARIABLE);
	rfm69_payload_length_set(rfm, WTP_PKT_SIZE_MAX);

	rfm69_crc_autoclear_set(rfm, false);

	// DIO settings.
	// DIO0 changes between RX and TX so isn't set here.
	// Set TX specific DIO0
	rfm69_dio1_config_set(rfm, RFM69_DIO1_PKT_TX_FIFO_LVL);
	rfm69_dio2_config_set(rfm, RFM69_DIO2_PKT_TX_FIFO_N_EMPTY);
	rfm69_dio3_config_set(rfm, RFM69_DIO3_PKT_RX_SYNC_ADDRESS);

	// Make sure we are sleeping as a default state
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);
}

static int rudp_wtp_packet_tx(
	struct rudp_context *rudp_ctx,
	const uint8_t *wtp_header, 
	const uint8_t *data_buffer,
	ptrdiff_t data_size
) {
	assert(data_size <= WTP_PKT_DATA_MAX);

	rfm69_mode_set(rudp_ctx->rfm, RFM69_OP_MODE_STDBY);

	// Push header into FIFO
	rfm69_fifo_write(rudp_ctx->rfm, (uint8_t *)wtp_header, WTP_HEADER_SIZE);

	// Start variable packet TX with data_buffer
	int lp_rval = rfm69_lp_tx(rudp_ctx->rfm, data_buffer, data_size);

	rfm69_mode_set(rudp_ctx->rfm, RFM69_OP_MODE_SLEEP);

	if (lp_rval != LP_TX_OK)
		return RUDP_HARDWARE_ERROR;

	return RUDP_OK;
}

static int rudp_lp_rx(
	rfm69_context_t *rfm_ctx,
	uint8_t *rx_buffer,
	ptrdiff_t buffer_size,
	int rx_timeout,	
	int *rssi
) {
	rfm69_mode_set(rfm_ctx, RFM69_OP_MODE_STDBY);

	// Start variable packet RX
	int lp_rval = rfm69_lp_rx(
			rfm_ctx,
			rx_buffer,
			buffer_size,
			-1,
			rx_timeout,
			rssi
	);

	rfm69_mode_set(rfm_ctx, RFM69_OP_MODE_SLEEP);

	return lp_rval;
}

static int rudp_wtp_packet_rx(
	struct rudp_context *rudp_ctx,
	uint8_t *wtp_packet,
	ptrdiff_t packet_size,
	int timeout
) {
	int rudp_rval = RUDP_UNKNOWN_RETURN;

	printf("pre_rx\n");
	int lp_rval = rudp_lp_rx(
			rudp_ctx->rfm,
			wtp_packet,
			packet_size,
			timeout,
			&rudp_ctx->trx_report.last_rssi
	);
	printf("post_rx\n");
	fflush(stdout);

	// Handle any fatal lp_rx errors.
	if (lp_rval != LP_RX_OK) {
		switch (lp_rval) {
		// Bad CRC.
		case LP_RX_CRC_FAILURE:
			printf("CRC Failure\n");
			rudp_rval = RUDP_BAD_PACKET;
			goto CLEANUP;
		// Timeout.
		case LP_RX_TIMEOUT:
			printf("Timeout\n");
			rudp_rval = RUDP_TIMEOUT;
			goto CLEANUP;
		// Hardware Failure.
		case LP_RX_FIFO_READ_FAILURE:
			rudp_rval = RUDP_HARDWARE_ERROR;
			goto CLEANUP;
		case LP_RX_BUFFER_OVERFLOW:
			printf("Buffer Overflow\n");
			rudp_rval = RUDP_BUFFER_OVERFLOW;
			goto CLEANUP;
		// Impossible case state (hopefully).
		case LP_UNKNOWN_RETURN:
			rudp_rval = RUDP_UNKNOWN_RETURN;
			goto CLEANUP;
		}
	}

	rudp_rval = RUDP_OK;
CLEANUP:;

	return rudp_rval;
}

int rudp_tx(
		struct rudp_context *rudp,
		int rx_address,
		const uint8_t *payload_buffer,
		ptrdiff_t payload_size
) {
	// Set sentry return and state value.
	int tx_rval = RUDP_UNKNOWN_RETURN;	

	// Reset trx_report.
	trx_report_reset(&rudp->trx_report);

	// Make sure the radio is in a known state:
	// - FIFO is empty
	// - Op Mode = SLEEP
	rfm69_mode_set(rudp->rfm, RFM69_OP_MODE_STDBY);
	rfm69_fifo_clear(rudp->rfm);
	rfm69_mode_set(rudp->rfm, RFM69_OP_MODE_SLEEP);

	//* PROTOCOL SETUP: TX SIDE *//

	// TX/RX Buffers.
	uint8_t tx_header[WTP_HEADER_SIZE] = {0}; // <- OUT
	// RX packet never contains data and thus only
	// ever contains a header.
	uint8_t rx_packet[WTP_HEADER_SIZE] = {0}; // <- IN

	// Construct TX Header (we are TX).
	uint8_t node_address = {0};
	rfm69_node_address_get(rudp->rfm, &node_address);
	wtp_tx_addr_set(tx_header, node_address);
	wtp_rx_addr_set(tx_header, rx_address);

	// Set trx_report info.
	rudp->trx_report.tx_addr = node_address;
	rudp->trx_report.rx_addr = rx_address;

	// Generate initial seq num.
	wtp_seq_num_set(tx_header, seq_num_generate());

	// Set inital flags.
	// First packet always contains SYN flag.
	wtp_flags_set(tx_header, WTP_FLAG_SYN);

	//* TX LOOP *//
	
	// Variables to track tx progress/state
	ptrdiff_t bytes_remaining = payload_size;
	const uint8_t *data_p = payload_buffer;
	uint8_t next_data_size = {0};
	int resend_count = {0};
	int fin_count = 0;
	for (;;) {

		printf("Top of TX loop\n");

		// If we have reached max resends.
		if (resend_count > rudp->tx_resend_max) {
			tx_rval = RUDP_RESEND_MAX_REACHED;
			goto CLEANUP;
		}

		// Set packet size if new packet.
		if (wtp_flags_are_set(tx_header, WTP_FLAG_RTR) == false) {
			printf("Building new packet\n");

			next_data_size = wtp_data_size_calc(bytes_remaining);
			// pkt_size_set helper function adds header size automatically.
			wtp_pkt_size_set(tx_header, next_data_size);

			// If this packet is sending the last data bytes, we must set the
			// FIN flag to signal TRX teardown.
			if (bytes_remaining == next_data_size) {
				wtp_flags_set(tx_header, WTP_FLAG_FIN);
			}
		} 

		printf("Sending...\n");
		int rudp_rval = rudp_wtp_packet_tx(rudp, tx_header, data_p, next_data_size);
		if (rudp_rval != RUDP_OK) {
			tx_rval = rudp_rval;
			goto CLEANUP;
		}

		printf("Waiting for ack...\n");
		// Packet sent. Wait for ACK.
		rudp_rval = rudp_wtp_packet_rx(
				rudp, 
				rx_packet, 
				WTP_HEADER_SIZE,
				rudp->tx_resend_timeout
		);

		// Check Rx errors.
		switch (rudp_rval) {
		case RUDP_OK:
			break;
		// Both back packet or timeout means resend.
		case RUDP_BAD_PACKET:
			// This is where we can keep track of bad packets.
			rudp->trx_report.bad_pkt_count++;
		case RUDP_BUFFER_OVERFLOW:
			printf("buffer overflow\n");
		case RUDP_TIMEOUT:
			resend_count++;
			wtp_flags_set(tx_header, WTP_FLAG_RTR);
			continue;
		// All other errors are fatal.
		default:
			tx_rval = rudp_rval;
			goto CLEANUP;
		}

		printf("Ack received!\n");
		// Ack received.
		resend_count = 0;

		// TEST BREAK
		break;

		// If that was the ACK to first packet, we need to set our ACK num
		// to RX SEQ num.
		if (wtp_flags_are_set(tx_header, WTP_FLAG_SYN)) {
			wtp_flags_clear(tx_header, WTP_FLAG_SYN);
			wtp_ack_num_set(tx_header, wtp_seq_num_get(rx_packet));
		}

		// Clear RTR flag and ensure set ACK flag.
		wtp_flags_clear(tx_header, WTP_FLAG_RTR);
		wtp_flags_set(tx_header, WTP_FLAG_ACK);

		// Increment SEQ and ACK.
		wtp_seq_num_inc(tx_header);
		wtp_ack_num_inc(tx_header);

		// If FIN flag is set, we can break from main TX loop and send
		// final teardown packet.
		if (wtp_flags_are_set(tx_header, WTP_FLAG_FIN))
			break;

		// Move data pointer forward number of data bytes sent.
		data_p += next_data_size;
		// Subtract data bytes sent from bytes_remaining.
		bytes_remaining -= next_data_size;

		// Back to top of TX loop.
	}

	// Send final teardown packet and don't wait for an ACK.
	rudp_wtp_packet_tx(rudp, tx_header, NULL, 0);

	tx_rval = RUDP_OK;
CLEANUP:;
	//* CLEANUP *//
	// We can't be certain of the state of the FIFO, so we leave it cleared
	// and then put the hardware to sleep.
	rfm69_mode_set(rudp->rfm, RFM69_OP_MODE_STDBY);
	rfm69_fifo_clear(rudp->rfm);
	rfm69_mode_set(rudp->rfm, RFM69_OP_MODE_SLEEP);
	return tx_rval;
}

// RX
int rudp_rx(
		struct rudp_context *rudp,
		uint8_t *payload_buffer,
		ptrdiff_t buffer_size
) {
	// Set sentry return value
	// The function should never return this.
	int rx_rval = RUDP_UNKNOWN_RETURN;	

	// Reset trx_report.
	trx_report_reset(&rudp->trx_report);

	// Make sure the radio is in a known state:
	// - FIFO is empty
	// - Op Mode = SLEEP
	rfm69_mode_set(rudp->rfm, RFM69_OP_MODE_STDBY);
	rfm69_fifo_clear(rudp->rfm);
	rfm69_mode_set(rudp->rfm, RFM69_OP_MODE_SLEEP);

	//* PROTOCOL SETUP: RX SIDE *//

	// TX/RX Buffers
	uint8_t rx_header[WTP_HEADER_SIZE] = {0};     // <- OUT
	// WTP_PKT_TX_SIZE_MAX = WTP_PKT_SIZE_MAX + 1
	// This is due to a quirk of underlying RFM69 hardware which
	// does not include the packet size byte itself in the packet
	// size calculation.
	uint8_t tx_packet[WTP_PKT_TX_SIZE_MAX] = {0}; // <- IN

	// Construct RX header (we are RX)
	uint8_t node_address = 0;
	rfm69_node_address_get(rudp->rfm, &node_address);
	wtp_tx_addr_set(rx_header, node_address);

	// Cache Rx ADDR for Trx report.
	rudp->trx_report.rx_addr = node_address;

	// Generate initial seq num
	wtp_seq_num_set(rx_header, seq_num_generate());

	//* RX Loop *//
	
	int bytes_received = 0;
	// A few things are handled differently for only the first packet
	// received.
	int timeout = rudp->rx_wait_timeout;
	int fin_count = 0;
	for (;;) {

		// Avoid possible timeout value race condition.
		if (timeout <= 0) {
			rx_rval = RUDP_TIMEOUT;
			goto CLEANUP;
		}

		absolute_time_t start_time = get_absolute_time();
		int rudp_rval = rudp_wtp_packet_rx(
				rudp, 
				tx_packet, 
				WTP_PKT_TX_SIZE_MAX,
				timeout
		);

		uint64_t time_diff_us = 
			absolute_time_diff_us(start_time, get_absolute_time());

		// Remaining timeout tracked.
		timeout -= time_diff_us/1000;

		// Some error occurred.
		if (rudp_rval != RUDP_OK) {
			
			// Only non-fatal error. Means we received a packet but it failed
			// the CRC check. Nothing to do but kick back into Rx ASAP.
			if (rudp_rval == RUDP_BAD_PACKET) {
				rudp->trx_report.bad_pkt_count++;
				continue;
			}

			// Fatal error (likely timeout).
			rx_rval = rudp_rval;
			goto CLEANUP;
		}

		// Validate packet.
		int packet_state = wtp_rx_pkt_state(rx_header, tx_packet);

		// Bad packet. Back to Rx.
		if (packet_state == WTP_PKT_STATE_INVALID)
			continue;

		printf("valid packet!\n");

		// Packet valid!
		// Reset timeout.
		timeout = rudp->rx_drop_timeout;

		// Track number of RTR packets we receive.
		if (wtp_flags_are_set(tx_packet, WTP_FLAG_RTR))
			rudp->trx_report.rtr_count++;

		switch (packet_state) {
		case WTP_PKT_STATE_VALID_PREV:
			break;
		case WTP_PKT_STATE_VALID_FIRST:
			printf("First!\n");
			wtp_ack_num_set(rx_header, wtp_seq_num_get(tx_packet));
			wtp_rx_addr_set(rx_header, wtp_tx_addr_get(tx_packet));

			// Cache Tx ADDR for Trx Report.
			rudp->trx_report.tx_addr = wtp_tx_addr_get(tx_packet);

			// VALID_FIRST falls through to VALID_NEXT since both should be handled
			// as a new packet.
		case WTP_PKT_STATE_VALID_NEXT:
			printf("Next!\n");

			// Check if packet contained data.
			int data_size = wtp_data_size_get(tx_packet);
			if (data_size > 0) {
				// Check for potential buffer overflow.
				if ((bytes_received + data_size) > buffer_size) {
					rx_rval = RUDP_BUFFER_OVERFLOW;
					goto CLEANUP;
				}

				// Start DMA data copying from packet to buffer.
				// TODO

				// Update received bytes.
				bytes_received += data_size;
			}

			// If this is a FIN packet.
			if (wtp_flags_are_set(tx_packet, WTP_FLAG_FIN))
				fin_count++;

			// Increment SEQ and ACK.
			wtp_seq_num_inc(rx_header);
			wtp_ack_num_inc(rx_header);
		}

		// We have received our final FIN packet and can break from RX loop.
		if (fin_count == 2) 
			goto CLEANUP;

		//* SEND ACK *//

		// Copy TX flags + always ACK.
		wtp_flags_clear_all(rx_header);
		wtp_flags_set(rx_header, wtp_flags_get(tx_packet));
		wtp_flags_set(rx_header, WTP_FLAG_ACK);

		printf("Sending Ack\n");
		rudp_rval = rudp_wtp_packet_tx(rudp, rx_header, NULL, 0);
		if (rudp_rval != RUDP_OK) {
			rx_rval = rudp_rval;
			goto CLEANUP;
		}

		break;
	}

	// If we successfully received at least the first FIN packet, we have the
	// entire payload and TRX was a success.
	
	// We should not be able to hit this point if fin_count < 1.
	if (fin_count > 0)
		rx_rval = RUDP_OK;


CLEANUP:;
	//* CLEANUP *//

	// We can't be certain of the state of the FIFO, so we clear it as a
	// kindness and then put the hardware to sleep.
	rfm69_mode_set(rudp->rfm, RFM69_OP_MODE_STDBY);
	rfm69_fifo_clear(rudp->rfm);
	rfm69_mode_set(rudp->rfm, RFM69_OP_MODE_SLEEP);

	return rx_rval;
}
