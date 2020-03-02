#ifndef TETRA_BURST_H
#define TETRA_BURST_H

#include <stdint.h>

enum tp_sap_data_type {
	TPSAP_T_SB1,
	TPSAP_T_SB2,
	TPSAP_T_NDB,
	TPSAP_T_BBK,
	TPSAP_T_SCH_HU,
	TPSAP_T_SCH_F,
};

extern void tp_sap_udata_ind(enum tp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv);

/* 9.4.3.2.3 Direct Mode Synchronization Burst */
int build_dm_sync_burst(uint8_t *buf, const uint8_t *sb, const uint8_t *bkn);

/* 9.4.3.2.1 Direct Mode Normal Burst */
int build_dm_norm_burst(uint8_t *buf, const uint8_t *bkn1, const uint8_t *bkn2, int two_log_chan);

/* 9.4.4 DM Device Multiple Slot Transmission */
int add_guard_bits(uint8_t *buf,int long_short);



#endif /* TETRA_BURST_H */
