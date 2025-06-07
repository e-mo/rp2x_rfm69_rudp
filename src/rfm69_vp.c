#include "pico/sync.h"

#include "rfm69_vp.h"
#include "wtp-1_0.h"
#include "rp2x_gpio_irq.h"

#define TRX_LOOP_DELAY (10)

static void semaphore_release_cb(uint gpio, uint32_t event_mask, void *data) {
	semaphore_t *sem = data;	
	sem_release(sem);
}

static void bool_level_cb(uint gpio, uint32_t event_mask, void *data) {
	volatile bool *flag = data;
	*flag = !!(event_mask & GPIO_IRQ_EDGE_RISE);
}

VP_TX_ERROR_T rfm69_vp_tx(
		rfm69_context_t *rfm,
		uint8_t *header, 
		uint8_t *payload_buffer,
		uint8_t payload_size
) {
	VP_TX_ERROR_T tx_status = VP_TX_OK;
	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);

	// Set TX specific DIO0
	rfm69_dio0_config_set(rfm, RFM69_DIO0_PKT_TX_PACKET_SENT);
	rfm69_fifo_threshold_set(rfm, RFM69_FIFO_SIZE/2);

	semaphore_t packet_sent;
	volatile bool fifo_level = false;

	sem_init(&packet_sent, 0, 1);

	rp2x_gpio_irq_enable(
			rfm->pin_dio0, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_release_cb,
			&packet_sent
	);
	// fifo_level triggers when fifo_level flag clears
	rp2x_gpio_irq_enable(
			rfm->pin_dio1, 
			GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
			bool_level_cb,
			(bool *)&fifo_level
	);


	if (!rfm69_write(rfm, RFM69_REG_FIFO, header, WTP_HEADER_SIZE)) {
		tx_status = VP_TX_FIFO_WRITE_FAILURE;
		goto CLEANUP;
	}

	rfm69_mode_set(rfm, RFM69_OP_MODE_TX);

	while (payload_size) {

		// If fifo level is low, write bits until it is high
		if (!fifo_level) {
			rfm69_fifo_write(rfm, payload_buffer, 1);
			payload_buffer++;
			payload_size--;
		}

		sleep_us(TRX_LOOP_DELAY);
	}

	// Wait for packet to send
	sem_acquire_blocking(&packet_sent);

CLEANUP:;
	// Always ensure FIFO clear
	rfm69_fifo_clear(rfm);
	// Go to stdby
	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);

	// Breakdown
	rp2x_gpio_irq_disable(rfm->pin_dio0);
	rp2x_gpio_irq_disable(rfm->pin_dio1);

	return tx_status;
}

VP_RX_ERROR_T rfm69_vp_rx(
		rfm69_context_t *rfm,
		uint8_t *rx_buffer,
		uint16_t buffer_size,
		uint32_t timeout_ms,
		int *rssi	
) {
	VP_RX_ERROR_T rx_status = -1;
	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);

	rfm69_dio0_config_set(rfm, RFM69_DIO0_PKT_RX_PAYLOAD_READY);

	volatile bool payload_ready = false;
	volatile bool fifo_level = false;
	semaphore_t fifo_not_empty;

	//sem_init((semaphore_t *)&payload_ready, 0, 1);
	sem_init(&fifo_not_empty, 0, 1);

	// Payload ready means the last of the payload is in the buffer
	rp2x_gpio_irq_enable(
			rfm->pin_dio0, 
			GPIO_IRQ_EDGE_RISE, 
			bool_level_cb,
			(bool *)&payload_ready
	);

	// fifo level flag rises meaning we need to pull from fifo
	rp2x_gpio_irq_enable(
			rfm->pin_dio1, 
			GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
			bool_level_cb,
			(bool *)&fifo_level
	);

	// Fifo not empty tells us that we are receiving something
	rp2x_gpio_irq_enable(
			rfm->pin_dio2, 
			GPIO_IRQ_EDGE_RISE, 
			semaphore_release_cb,
			(semaphore_t *)&fifo_not_empty
	);


	rfm69_mode_set(rfm, RFM69_OP_MODE_RX);

	if (sem_acquire_timeout_ms(&fifo_not_empty, timeout_ms) == false) {
		rx_status = VP_RX_TIMEOUT;
		goto CLEANUP;
	}
	
	// Rx loop
	volatile int received = 0;
	uint32_t elapsed_ms = 0;
	absolute_time_t start_time = 0;
	int64_t time_elapsed_us = 0;
	for (;;) {

		start_time = get_absolute_time();

		if (payload_ready)
			break;

		if (fifo_level) {
			time_elapsed_us = 0;

			if (received == buffer_size) {
				rx_status = VP_RX_BUFFER_OVERFLOW;
				goto CLEANUP;
			}

			rfm69_fifo_read(rfm, rx_buffer, 1);
			rx_buffer++;
			received++;
		} 

		time_elapsed_us += absolute_time_diff_us(start_time, get_absolute_time());
		elapsed_ms = time_elapsed_us / 1000;

		if (elapsed_ms > timeout_ms) {
			rx_status = VP_RX_TIMEOUT;
			goto CLEANUP;
		}

		sleep_us(TRX_LOOP_DELAY);
	}

	// Check CRCOK flag to see if packet is OK
	bool success = false;
	rfm69_irq2_flag_state(rfm, RFM69_IRQ2_FLAG_CRC_OK, &success);

	int16_t rssi_16 = 0;
	rfm69_rssi_measurement_get(rfm, &rssi_16);
	*rssi = rssi_16;

	if (success) rx_status = VP_RX_OK;
	else rx_status = VP_RX_CRC_FAILURE;

	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);

	// Ensure empty FIFO
	bool not_empty = false;
	for (;;) {
		rfm69_irq2_flag_state(rfm, RFM69_IRQ2_FLAG_FIFO_NOT_EMPTY, &not_empty);
		// If empty
		if (not_empty == false)
			break;

		if (received == buffer_size) {
			rx_status = VP_RX_BUFFER_OVERFLOW;
			goto CLEANUP;
		}

		rfm69_fifo_read(rfm, rx_buffer, 1);
		rx_buffer++;
		received++;

		sleep_us(TRX_LOOP_DELAY);
	}

CLEANUP:;
	rfm69_fifo_clear(rfm);
	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);
	
	// Breakdown
	rp2x_gpio_irq_disable(rfm->pin_dio0);
	rp2x_gpio_irq_disable(rfm->pin_dio1);
	rp2x_gpio_irq_disable(rfm->pin_dio2);

	return rx_status;
}
