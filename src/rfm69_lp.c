#include "pico/sync.h"

#include "rfm69_lp.h"
#include "wtp-1_0.h"
#include "rp2x_gpio_irq.h"

static void semaphore_and_level_cb(uint gpio, uint32_t event_mask, volatile void *data) {
	void *volatile *items = data;
	semaphore_t *sem = (semaphore_t *)items[0];	
	volatile bool *flag = items[1];

	
	if (event_mask == GPIO_IRQ_EDGE_RISE) {
		sem_release(sem);
		*flag = true;
	} else if (event_mask == GPIO_IRQ_EDGE_FALL) {
		*flag = false;
	}
}

static void semaphore_release_cb(uint gpio, uint32_t event_mask, volatile void *data) {
	semaphore_t *sem = (semaphore_t *)data;	
	sem_release(sem);
}

static void bool_level_cb(uint gpio, uint32_t event_mask, volatile void *data) {
	volatile bool *flag = data;

	if (event_mask == GPIO_IRQ_EDGE_RISE)
		*flag = true;
	else if (event_mask == GPIO_IRQ_EDGE_FALL)
		*flag = false;
}

void wtp_tx(
		rfm69_context_t *rfm,
		const uint8_t *wtp_header,
		const uint8_t *payload_buffer,
		uint8_t payload_size
) {
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	// Set TX specific DIO0.
	rfm69_dio0_config_set(rfm, RFM69_DIO0_PKT_TX_PACKET_SENT);

	semaphore_t packet_sent;
	volatile bool fifo_level = false;

	sem_init(&packet_sent, 0, 1);

	rp2x_gpio_irq_enable(
			rfm->pin_dio0, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_release_cb,
			&packet_sent
	);
	// fifo_level triggers when fifo_level flag clears.
	rp2x_gpio_irq_enable(
			rfm->pin_dio1, 
			GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
			bool_level_cb,
			(bool *)&fifo_level
	);

	rfm69_mode_set(rfm, RFM69_OP_MODE_TX);

	while (payload_size) {
		// If fifo level is low, write bits until it is high.
		if (!fifo_level) {
			rfm69_fifo_write(rfm, (uint8_t *)payload_buffer, 1);
			payload_buffer++;
			payload_size--;
		}
	}

	// Wait for packet to send
	sem_acquire_blocking(&packet_sent);

	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);
	
	// Disable function specific IRQs.
	rp2x_gpio_irq_disable(rfm->pin_dio0);
	rp2x_gpio_irq_disable(rfm->pin_dio1);
}

LP_RX_ERROR_T wtp_rx(
		rfm69_context_t *rfm,
		uint8_t *rx_buffer,
		uint16_t max_size,
		int filter_addr,
		uint32_t timeout_ms,
		int *rssi	
) {
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	// Set Rx specific DIO0.
	rfm69_dio0_config_set(rfm, RFM69_DIO0_PKT_RX_PAYLOAD_READY);
	rfm69_payload_length_set(rfm, max_size);

	volatile bool payload_ready  = false;
	//volatile bool fifo_level     = false;
	volatile bool fifo_not_empty = false;

	semaphore_t rx_event;
	sem_init(&rx_event, 0, 1);

	semaphore_t address_match;
	sem_init(&address_match, 0, 1);

	// Payload ready means the last of the payload is in the buffer.
	rp2x_gpio_irq_enable(
			rfm->pin_dio0, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_and_level_cb,
			(volatile void *[]) {&rx_event, &payload_ready}
	);

	//rp2x_gpio_irq_enable(
	//		rfm->pin_dio1, 
	//		GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
	//		semaphore_and_level_cb,
	//		(volatile void *[]) {&rx_event, &fifo_level}
	//);

	// Fifo not empty tells us that we are receiving something
	rp2x_gpio_irq_enable(
			rfm->pin_dio2, 
			GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
			semaphore_and_level_cb,
			(volatile void *[]) {&rx_event, &fifo_not_empty}
	);

	rp2x_gpio_irq_enable(
			rfm->pin_dio3, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_release_cb,
			(semaphore_t *)&address_match
	);

	rfm69_mode_set(rfm, RFM69_OP_MODE_RX);

	// Rx loop
	volatile int received = 0;
	for (;;) {

		if (received == 0) {
			if (sem_acquire_timeout_ms(&address_match, timeout_ms) == false) {
				lp_rx_rval = LP_RX_TIMEOUT;
				goto CLEANUP;
			}
		}

		if (!payload_ready)
			sem_acquire_blocking(&rx_event);
		else
			break;

		while(fifo_not_empty) {

			if (received == max_size) {
				lp_rx_rval = LP_RX_BUFFER_OVERFLOW;
				goto CLEANUP;
			}

			rfm69_fifo_read(rfm, rx_buffer+received, 1);
			received++;

			// We have the Tx Address.
			// If we have a filter_addr set, we have to check against it.
			if ((received == 3) && (filter_addr != -1)) {
				// Continue if correct Tx.
				if (rx_buffer[2] == filter_addr)
					continue;

				// Incorrect Tx.
				// Clear FIFO, reset flags, and restart RX.
				printf("Wrong tx!\n");
				rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);

				payload_ready = false;
				fifo_not_empty = false;

				sem_reset(&rx_event, 0);
				sem_reset(&address_match, 0);

				received = 0;
				rfm69_mode_set(rfm, RFM69_OP_MODE_RX);
			}
		}
	}

	// Check CRCOK flag to see if packet is OK.
	bool crc_ok = false;
	rfm69_irq2_flag_state(rfm, RFM69_IRQ2_FLAG_CRC_OK, &crc_ok);

	// Cache RSSI value if not NULL.
	if (rssi != NULL) {
		int16_t rssi_16 = 0;
		rfm69_rssi_measurement_get(rfm, &rssi_16);
		*rssi = rssi_16;
	}

	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);

	if (crc_ok) lp_rx_rval = LP_RX_OK;
	else {
		lp_rx_rval = LP_RX_CRC_FAILURE;
		goto CLEANUP;
	}

	// Finishing emptying FIFO if necessary.
	while (fifo_not_empty) {

		if (received == max_size) {
			lp_rx_rval = LP_RX_BUFFER_OVERFLOW;
			goto CLEANUP;
		}

		rfm69_fifo_read(rfm, rx_buffer+received, 1);
		received++;
	}

	lp_rx_rval = LP_RX_OK;
CLEANUP:;
	// Breakdown.
	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);
	rfm69_fifo_clear(rfm);
	// Retore prev mode.
	rfm69_mode_set(rfm, prev_mode);
	// Disable IRQs
	rp2x_gpio_irq_disable(rfm->pin_dio0);
	rp2x_gpio_irq_disable(rfm->pin_dio1);
	rp2x_gpio_irq_disable(rfm->pin_dio2);

	return lp_rx_rval;
}
