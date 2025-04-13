#include "wtp-1_0.h"

void wtp_index_init(struct wtp_header_index *index, uint8_t *buffer) {

	index->pkt_size = &buffer[WTP_HEADER_PKT_SIZE_OFFSET];
	index->rx_addr = &buffer[WTP_HEADER_RX_ADDR_OFFSET];
	index->tx_addr = &buffer[WTP_HEADER_TX_ADDR_OFFSET];
	index->flags = &buffer[WTP_HEADER_FLAGS_OFFSET];
	index->seq_num = (uint16_t *)&buffer[WTP_HEADER_SEQ_NUM_OFFSET];
	index->ack_num = (uint16_t *)&buffer[WTP_HEADER_ACK_NUM_OFFSET];
	index->data = &buffer[WTP_DATA_SEGMENT_OFFSET];

}
