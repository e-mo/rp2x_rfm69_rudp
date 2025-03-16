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
//#include <stdio.h>

#include "pico/sync.h"

#include "rp2x_gpio_irq.h"

#include "rp2x_rfm69_rudp.h"


// Helper function to initialize rfm69 context for use with rudp interface
bool rfm69_rudp_init(rfm69_context_t *rfm, const struct rfm69_config_s *config) {
	// Init rp2x gpio irq lib
	rp2x_gpio_irq_init();
	if (rfm69_init(rfm, config) == false) return false; 

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
    //rfm69_bitrate_set(rfm, RFM69_MODEM_BITRATE_1_2);
	//rfm69_fdev_set(rfm, 300000);
	//rfm69_rxbw_set(rfm, RFM69_RXBW_MANTISSA_16, 0);

	// Make sure we are sleeping as a default state
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	return true;
}

static void semaphore_release_cb(uint gpio, uint32_t event_mask, void *data) {
	semaphore_t *sem = (semaphore_t *)data;	
	sem_release(sem);
}

bool rfm69_tx_variable_packet(
		rfm69_context_t *rfm,
		uint8_t rx_address, 
		uint8_t *payload_buffer,
		uint8_t payload_size
)
{
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	// Set TX specific DIO0
	rfm69_dio0_config_set(rfm, RFM69_DIO0_PKT_TX_PACKET_SENT);

	semaphore_t packet_sent;
	semaphore_t fifo_level;

	sem_init(&packet_sent, 0, 1);
	sem_init(&fifo_level, 0, 1);

	rp2x_gpio_irq_enable(
			rfm->pin_dio0, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_release_cb,
			&packet_sent
	);
	// fifo_level triggers when fifo_level flag clears
	rp2x_gpio_irq_enable(
			rfm->pin_dio1, 
			GPIO_IRQ_EDGE_FALL, 
			semaphore_release_cb,
			&fifo_level
	);

	uint8_t header[] = {payload_size + 1, rx_address};
	if (!rfm69_write(rfm, RFM69_REG_FIFO, header, 2))
		return false;

	// Set first write size
	uint8_t write_size = payload_size > RFM69_FIFO_SIZE - 2 ? 
						 RFM69_FIFO_SIZE - 2 : payload_size;

	rfm69_write(rfm, RFM69_REG_FIFO, payload_buffer, write_size);
	payload_size -= write_size;

	rfm69_mode_set(rfm, RFM69_OP_MODE_TX);

	while (payload_size) {
		payload_buffer += write_size;

		// When fifo_level clears, calculate new write size
		// 20 is a number I found to work for all bitrates during fill.
		// There seems to be some actual delay in the interrupt firing
		// and bytes actually being ready to pull from the fifo and things
		// can get strange. You'll see this same 20 in the RX when pulling
		// from an active FIFO as well. 
		write_size = payload_size > 20 ? 
					 20 : payload_size;

		// Wait for fifo_level flag to clear
		sem_acquire_blocking(&fifo_level);

		rfm69_write(rfm, RFM69_REG_FIFO, payload_buffer, write_size);
		payload_size -= write_size;
	}
	// Wait for packet to send
	sem_acquire_blocking(&packet_sent);

	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	// Disable IRQ
	rp2x_gpio_irq_disable(rfm->pin_dio0);
	rp2x_gpio_irq_disable(rfm->pin_dio1);

	return true;
}

VP_RX_ERROR_T rfm69_rx_variable_packet(
		rfm69_context_t *rfm,
		uint8_t *rx_buffer,
		uint16_t buffer_size,
		uint16_t timeout_ms
)
{
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	rfm69_dio0_config_set(rfm, RFM69_DIO0_PKT_RX_PAYLOAD_READY);

	semaphore_t payload_ready;
	semaphore_t fifo_level;
	semaphore_t fifo_not_empty;

	sem_init(&payload_ready, 0, 1);
	sem_init(&fifo_level, 0, 1);
	sem_init(&fifo_not_empty, 0, 1);

	// Payload ready means the last of the payload is in the buffer
	rp2x_gpio_irq_enable(
			rfm->pin_dio0, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_release_cb,
			&payload_ready
	);
	// fifo level flag rises meaning we need to pull from fifo
	rp2x_gpio_irq_enable(
			rfm->pin_dio1, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_release_cb,
			&fifo_level
	);
	// Fifo not empty tells us that we are receiving something
	rp2x_gpio_irq_enable(
			rfm->pin_dio2, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_release_cb,
			&fifo_not_empty
	);

	rfm69_mode_set(rfm, RFM69_OP_MODE_RX);

	// Wait to see if we start to receive anything
	if (sem_acquire_timeout_ms(&fifo_not_empty, timeout_ms) == false)
		return VP_RX_TIMEOUT;

	// Rx loop
	for (;;) {
		// Note magic 20 fill rate here. I found this to work for all
		// bitrates as the interrupt firing does not guarantee that there
		// are as many bytes ready to read from the buffer as you would hope.
		if (sem_try_acquire(&fifo_level)) {
			rfm69_read(rfm, RFM69_REG_FIFO, rx_buffer, 20);
			rx_buffer += 20;
		}

		// Check if we have hit a payload_ready
		if (sem_try_acquire(&payload_ready)) break;
	}

	// Check CRCOK flag to see if packet is OK
	bool success;
	rfm69_irq2_flag_state(rfm, RFM69_IRQ2_FLAG_CRC_OK, &success);

	VP_RX_ERROR_T rval = success ? VP_RX_OK : VP_RX_CRC_FAILURE;

	// Go to sleep AFTER checking CRC because leaving RX clears CRC flag
	// which is UNDOCUMENTED BEHAVIOR GOD DAMN IT HOPERF this one gave me 
	// a fucking headache
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	// Empty the FIFO if necessary
	bool not_empty;
	rfm69_irq2_flag_state(rfm, RFM69_IRQ2_FLAG_FIFO_NOT_EMPTY, &not_empty);
	if (not_empty) {
		rp2x_gpio_irq_enable(
				rfm->pin_dio2, 
				GPIO_IRQ_EDGE_FALL, 
				semaphore_release_cb,
				&fifo_not_empty
		);
		sem_reset(&fifo_not_empty, 0);
		rfm69_read(rfm, RFM69_REG_FIFO, rx_buffer, 1);
		rx_buffer++;

		// 1us timout to ensure fifo_not_empty pin settles and interrupt fires
		// if last byte has been pulled from FIFO
		while (sem_acquire_timeout_us(&fifo_not_empty, 1) == false) {
			rfm69_read(rfm, RFM69_REG_FIFO, rx_buffer, 1);
			rx_buffer++;
		}

	}
	
	// Breakdown
	rp2x_gpio_irq_disable(rfm->pin_dio0);
	rp2x_gpio_irq_disable(rfm->pin_dio1);
	rp2x_gpio_irq_disable(rfm->pin_dio2);

	return rval;
}


bool rfm69_rudp_tx(
		rfm69_context_t *rfm,
		uint8_t rx_address, 
		uint8_t *payload_buffer,
		uint32_t payload_size
)
{

}

bool rfm69_rudp_rx(
		rfm69_context_t *rfm,
		uint32_t buffer_size,
		uint8_t *payload_buffer
)
{

}
