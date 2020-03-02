
#include <stdint.h>

extern uint8_t pdu_sync[8];	        /* 60 bits */
extern uint8_t pdu_sysinfo[16];         /* 124 bits */
extern uint8_t pdu_acc_ass[2];
extern uint8_t pdu_acc_ass_18[2];
extern uint8_t pdu_schf[268];
extern uint8_t pdu_mac_data[16];        /* 124 bits */

void sync_pdu(uint16_t cc, uint8_t mn, uint8_t fn, uint8_t tn, uint8_t mcc, uint16_t mnc);
void sysinfo_pdu(uint16_t hn);
void mac_data_pdu();
void acc_pdu(uint8_t af1, uint8_t af2);
void acc_pdu_18();
