#include "rudp_validation.h"

#include "wtp-1_0.h"
#include "rp2x_rfm69_rudp.h"

// Helper function for tx to check if rx packets are valid
bool ack_valid(uint8_t *tx_header, uint8_t *rx_header) {
	// Expected flags are TX flags from last packet sent except we are always
	// checking for ACK.
	uint8_t expected_flags = tx_header[WTP_HEADER_FLAGS_OFFSET];
	expected_flags |= WTP_FLAG_ACK;

	printf("%02X : %02X\n", expected_flags, rx_header[WTP_HEADER_FLAGS_OFFSET]);

	// Flags are wrong, go away
	if (rx_header[WTP_HEADER_FLAGS_OFFSET] != expected_flags)
		return false;

	// Check if ack is correct
	uint16_t expected_ack = tx_header[WTP_HEADER_SEQ_NUM_OFFSET];
	expected_ack |= tx_header[WTP_HEADER_SEQ_NUM_OFFSET + 1] << 8;
	expected_ack += 1;
	uint16_t rx_ack = rx_header[WTP_HEADER_ACK_NUM_OFFSET];
	rx_ack |= rx_header[WTP_HEADER_ACK_NUM_OFFSET + 1] << 8;

	printf("ea: %04X - ra: %04X\n", expected_ack, rx_ack);

	if (expected_ack != rx_ack) {
		return false;
	}
	
	// If both flags and ack_num are correct, this is a valid ack packet
	return true;
}


// Uses the current header state, the receieved header, and current transmission
// state to report whether the packet is valid or not and why. 
PACKET_STATE_T packet_check(
		uint8_t *rx_header, 
		uint8_t *tx_header, 
		int state
) 
{
	// Initial state. If this value is output there is something wrong with
	// the code.
	PACKET_STATE_T p_state = PACKET_STATE_UNKNOWN;

	// After first packet we filter by address
	if (state != RX_STATE_WAITING) {
		uint8_t tx_addr = tx_header[WTP_HEADER_TX_ADDR_OFFSET];
		uint8_t expected_addr = rx_header[WTP_HEADER_RX_ADDR_OFFSET];
		if (tx_addr != expected_addr) {
			p_state = PACKET_INVALID_ADDR;
			goto CLEANUP;
		}
	}

	uint8_t tx_flags = tx_header[WTP_HEADER_FLAGS_OFFSET];	
	bool syn_set = !!(tx_flags & WTP_FLAG_SYN);
	bool ack_set = !!(tx_flags & WTP_FLAG_ACK);
	bool rtr_set = !!(tx_flags & WTP_FLAG_RTR);
	if (syn_set) {

		bool syn_valid = 
			(tx_flags & ~(WTP_FLAG_RTR | WTP_FLAG_FIN)) == WTP_FLAG_SYN;
		if (!syn_valid)
			return false;

		// Goto CLEANUP because this is a valid syn and our first packet
		if (state == RX_STATE_WAITING) {
			p_state = PACKET_VALID_NEW;
			goto CLEANUP;
		}
		
		// HOWEVER!
		// If we get a SYN but have already received a valid first packet
		// (we are no longer in a waiting for first packet state)
		// This signals the beginning of a new transmission from our original
		// sender for some reason.
		// This is neither valid nor invalid
		if ((state != RX_STATE_WAITING) && !rtr_set) {
			p_state = PACKET_VALID_TX_RESTART;
			goto CLEANUP;
		}

		// Here we have a valid SYN packet but we were not in a waiting
		// state so it is not our first packet. However, RTR is set, so
		// we must now check the seq num of that packet to see if it is
		// a resend of the last packet.
			
	
	// If SYN is not set, ACK must be, or we have malformed flags
	} else if (!ack_set) {
		p_state = PACKET_INVALID_FLAGS;
		goto CLEANUP;
	// If SYN is not set, but we are still in a waiting state, this is not a
	// valid packet.
	} else if (state == RX_STATE_WAITING) {
		p_state = PACKET_INVALID_SYN;
		goto CLEANUP;
	}


	// Get SEQ and ACK values
	uint16_t rx_ack = *(uint16_t *)&rx_header[WTP_HEADER_ACK_NUM_OFFSET];
	uint16_t tx_seq = *(uint16_t *)&tx_header[WTP_HEADER_SEQ_NUM_OFFSET];

	// Initially assume the packet is new
	bool new_packet = true;
	// SEQ CHECK
	// Unlike ACK, we always check the SEQ number for validity unless
	// If this is not the next SEQ we expected
	if (rx_ack != tx_seq) {
		// If not the previous SEQ either
		if ((rx_ack - 1) != tx_seq) {
			p_state = PACKET_INVALID_SEQ;
			goto CLEANUP;
		}

		// It is the previous SEQ but is RTR set properly?
		if (!rtr_set) {
			p_state = PACKET_INVALID_FLAGS;
			goto CLEANUP;
		}

		// This is not a new packet.
		new_packet = false;
	}

	// We are done if ACK is not set
	if (!ack_set) goto CLEANUP;

	uint16_t rx_seq = *(uint16_t *)&rx_header[WTP_HEADER_SEQ_NUM_OFFSET];
	uint16_t tx_ack = *(uint16_t *)&tx_header[WTP_HEADER_ACK_NUM_OFFSET];

	// ACK CHECK
	// If we are expecting this to be the prev packet ack num
	if (new_packet == false) {
		if (rx_seq != tx_ack) {
			p_state = PACKET_INVALID_ACK;
			goto CLEANUP;
		}
	// Otherwise we expect this to be a new packet
	} else if ((rx_seq + 1) != tx_ack) {
		p_state = PACKET_INVALID_ACK;
		goto CLEANUP;
	}

	// Report whether this is a valid new or retransmitted packet
	p_state = new_packet ? PACKET_VALID_NEW : PACKET_VALID_RTR;

	// Return whatever status has been saved to p_state
	// The default is PACKET_STATE_UNKNOWN
CLEANUP:;
	return p_state;
}
