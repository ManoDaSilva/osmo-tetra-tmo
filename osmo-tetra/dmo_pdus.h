
#include <stdint.h>

extern uint8_t dmac_sync_pdu_schs[8];	/* 60 bits */
extern uint8_t dmac_sync_pdu_schh[16];		/* 124 bits */
extern uint8_t dmac_data_pdu[16];       /* 124 bits */

void sync_schs_pdu(uint8_t fn, uint8_t tn);
void sync_schh_pdu(uint8_t mcc, uint16_t mnc);