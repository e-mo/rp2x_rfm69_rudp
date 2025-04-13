#ifndef RUDP_VALIDATION_H
#define RUDP_VALIDATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum _packet_validation_states {
	PACKET_STATE_UNKNOWN,
	PACKET_VALID_NEW,	
	PACKET_VALID_RTR,
	PACKET_VALID_TX_RESTART,
	PACKET_INVALID_SYN,
	PACKET_INVALID_ADDR,
	PACKET_INVALID_ACK,
	PACKET_INVALID_SEQ,
	PACKET_INVALID_FLAGS,
} PACKET_STATE_T;

// In TX function, makes sure ack packet received is valid
bool ack_valid(uint8_t *tx_header, uint8_t *rx_header);

// In RX, returns if TX packet is valid, or why it is not valid
PACKET_STATE_T packet_check(
		uint8_t *rx_header, 
		uint8_t *tx_header, 
		int state
); 

#endif // RUDP_VALIDATION_H
