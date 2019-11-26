
#include <stdint.h>

extern uint8_t pdu_sync[8];	/* 60 bits */
extern uint8_t pdu_sysinfo[16];	/* 124 bits */
extern uint8_t pdu_acc_ass[2];
extern uint8_t pdu_acc_ass_18[2];
extern uint8_t pdu_schf[268];
extern uint8_t pdu_null[124];

void sync_pdu(const uint16_t cc, const uint8_t mn, const uint8_t fn, const uint8_t tn, const uint8_t mcc, const uint16_t mnc);
void sysinfo_pdu(const uint16_t hn);
void null_pdu();
void acc_pdu();
void acc_pdu_18();
