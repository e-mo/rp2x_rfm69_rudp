#include "pico/sync.h"

#include "rfm69_vp.h"
#include "wtp-1_0.h"
#include "rp2x_gpio_irq.h"

static void semaphore_release_cb(uint gpio, uint32_t event_mask, void *data) {
	semaphore_t *sem = (semaphore_t *)data;	
	sem_release(sem);
}

VP_TX_ERROR_T rfm69_vp_tx(
		rfm69_context_t *rfm,
		uint8_t *header, 
		uint8_t *payload_buffer,
		uint8_t payload_size
)
{
	VP_TX_ERROR_T tx_status = VP_TX_OK;
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


	if (!rfm69_write(rfm, RFM69_REG_FIFO, header, WTP_HEADER_SIZE)) {
		tx_status = VP_TX_FIFO_WRITE_FAILURE;
		goto CLEANUP;
	}

	// Set first write size
	uint8_t write_size = payload_size > RFM69_FIFO_SIZE - WTP_HEADER_SIZE ? 
						 RFM69_FIFO_SIZE - WTP_HEADER_SIZE : payload_size;

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

CLEANUP:;
	// Go to sleep
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);

	// Breakdown
	rp2x_gpio_irq_disable(rfm->pin_dio0);
	rp2x_gpio_irq_disable(rfm->pin_dio1);

	return tx_status;
}

VP_RX_ERROR_T rfm69_vp_rx(
		rfm69_context_t *rfm,
		uint8_t *rx_buffer,
		uint16_t buffer_size,
		uint16_t timeout_ms
)
{
	VP_RX_ERROR_T rx_status = -1;

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
	if (sem_acquire_timeout_ms(&fifo_not_empty, timeout_ms) == false) {
		rx_status = VP_RX_TIMEOUT;
		goto CLEANUP;
	}

	// Rx loop
	for (;;) {
		// Note magic 20 fill rate here. I found this to work for all
		// bitrates as the interrupt firing does not guarantee that there
		// are as many bytes ready to read from the buffer as you would hope.
		if (sem_try_acquire(&fifo_level)) {
			if (sem_acquire_timeout_ms(&payload_ready, 1)) break;

			if (!rfm69_read(rfm, RFM69_REG_FIFO, rx_buffer, 20)) {
				rx_status = VP_RX_FIFO_READ_FAILURE;
				goto CLEANUP;
			}
			rx_buffer += 20;
		}

		// Check if we have hit a payload_ready
		if (sem_try_acquire(&payload_ready)) break;
	}

	// Check CRCOK flag to see if packet is OK
	bool success;
	rfm69_irq2_flag_state(rfm, RFM69_IRQ2_FLAG_CRC_OK, &success);

	if (success) rx_status = VP_RX_OK;
	else rx_status = VP_RX_CRC_FAILURE;

	// Go to stdby AFTER checking CRC because leaving RX clears CRC flag
	// which is UNDOCUMENTED BEHAVIOR GOD DAMN IT HOPERF this one gave me 
	// a fucking headache
	rfm69_mode_set(rfm, RFM69_OP_MODE_STDBY);

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

CLEANUP:;
	rfm69_mode_set(rfm, RFM69_OP_MODE_SLEEP);
	
	// Breakdown
	rp2x_gpio_irq_disable(rfm->pin_dio0);
	rp2x_gpio_irq_disable(rfm->pin_dio1);
	rp2x_gpio_irq_disable(rfm->pin_dio2);

	return rx_status;
}
