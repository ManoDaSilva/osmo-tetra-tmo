/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
* Generate one hyperframe burst that can be sent to the modulator
*
*
*/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include <arpa/inet.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/bitvec.h>

#include "tetra_common.h"
#include <lower_mac/crc_simple.h>
#include <lower_mac/tetra_conv_enc.h>
#include <lower_mac/tetra_interleave.h>
#include <lower_mac/tetra_scramb.h>
#include <lower_mac/tetra_rm3014.h>
#include <lower_mac/viterbi.h>
#include <phy/dmo_tetra_burst.h>
#include "dmo_pdus.h"

#define BLEN 510 // burst length

/* Network info */
#define CC      0x29
#define MCC     901
#define MNC     16383

#define swap16(x) ((x)<<8)|((x)>>8)

#define verbose 1


/* incoming TP-SAP UNITDATA.ind  from PHY into lower MAC */
void tp_sap_udata_ind(enum tp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv)
{
}

/* Build a full 'Direct Mode Normal Burst'
*/
void build_dnb(uint8_t *buf)
{
	uint8_t sb1_type2[144];
	uint8_t sb1_master[216*4];
	uint8_t sb1_type3[216];
	uint8_t sb1_type4[216];
	//For now we're using a pre-made PDU
	uint8_t sb1_type5[216]={1,0,1,0,1,1,1,1,0,1,0,0,1,0,0,0,0,1,0,1,1,1,0,0,0,0,1,1,0,1,0,1,0,1,1,0,1,0,1,0,1,1,0,1,1,0,1,0,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,1,1,0,1,1,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,0,1,0,1,0,0,1,0,0,0,0,1,1,0,1,0,1,0,0,0,1,1,0,1,0,1,0,1,1,0,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,1,1,0,0,1,0,0,0,0,0,1,0,1,0,1,1,0,0,0,1,0,0,1,0,0,1,1,1,0,0,0,1,1,0,0,0,0,1,0,0,0,0,0,0,0,0,1,1,0,1,0,1,0,0};

	uint8_t sb2_type2[144];
	uint8_t sb2_master[216*4];
	uint8_t sb2_type3[216];
	uint8_t sb2_type4[216];
	uint8_t sb2_type5[216]={0,1,0,0,0,1,1,1,1,1,1,1,0,0,1,0,1,1,1,0,0,0,0,0,1,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,1,1,1,0,0,0,0,1,1,0,0,0,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,0,0,1,0,0,1,0,0,1,1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,1,0,0,0,1,1,1,1,0,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,0,0,1,1,1,0,0,0,0,1,1,1,0,0,1,0,1,0,0,0,0,1,1,1,1,1,0,1,1,1,1,0,1,0,0,0,1,0,0,0,1,1,1,0,1,0,0,0,0,1,1,0,0,1,0,1,0,1,0,0,1,1,0,0,0,1,1,1,0,1,0,1,1,1,0,1,0,0,0,1,1,0,0,1,1,0,0,1,0,1,1};

	uint16_t crc;
	uint8_t *cur;

	uint32_t scramb_init = tetra_scramb_get_init(MCC, MNC, CC);

	memset(sb1_type2, 0, sizeof(sb1_type2));
	cur = sb1_type2;

	// Use MAC-DATA PDU from pdus.c
	cur += osmo_pbit2ubit(sb1_type2, pdu_mac_data, 124);
	//printf("MAC-DATA PDU: %s\n", osmo_ubit_dump(sb1_type2, 124));

	// Run it through CRC16-CCITT
	crc = ~crc16_ccitt_bits(sb1_type2, 124);
	crc = swap16(crc);
	cur += osmo_pbit2ubit(cur, (uint8_t *) &crc, 16);

	// Append 4 tail bits: type-2 bits
	cur += 4;

	/* Run rate 2/3 RCPC code: type-3 bits */
	{
		struct conv_enc_state *ces = calloc(1, sizeof(*ces));
		conv_enc_init(ces);
		conv_enc_input(ces, sb1_type2, 144, sb1_master);
		get_punctured_rate(TETRA_RCPC_PUNCT_2_3, sb1_master, 216, sb1_type3);
		free(ces);
	}

	// Run (216,101) block interleaving: type-4 bits
	block_interleave(216, 101, sb1_type3, sb1_type4);

	//UNCOMMENT HERE FOR HOMEMADE PDU
	//memcpy(sb1_type5, sb1_type4, 216);
	// Run scrambling (all-zero): type-5 bits
	//tetra_scramb_bits(scramb_init, sb1_type5, 216);

	//printf("Scrambled block 1 bits (SCH/HD): %s\n", osmo_ubit_dump(sb1_type5, 216));

	memset(sb2_type2, 0, sizeof(sb2_type2));
	cur = sb2_type2;

	// Use pdu_sysinfo from pdus.c
	cur += osmo_pbit2ubit(sb2_type2, pdu_sysinfo, 124);
	//memcpy(sb2_type2,pdu_sysinfo_entropia,124);
	//cur +=124;
	// Run it through CRC16-CCITT
	crc = ~crc16_ccitt_bits(sb2_type2, 124);
	crc = swap16(crc);
	cur += osmo_pbit2ubit(cur, (uint8_t *) &crc, 16);

	// Append 4 tail bits: type-2 bits
	cur += 4;


	// Run rate 2/3 RCPC code: type-3 bits
	{
		struct conv_enc_state *ces = calloc(1, sizeof(*ces));
		conv_enc_init(ces);
		conv_enc_input(ces, sb2_type2, 144, sb2_master);
		get_punctured_rate(TETRA_RCPC_PUNCT_2_3, sb2_master, 216, sb2_type3);
		free(ces);
	}

	// Run (216,101) block interleaving: type-4 bits
	block_interleave(216, 101, sb2_type3, sb2_type4);
	//UNCOMMENT HERE FOR HOMEMADE PDU
	//memcpy(sb2_type5, sb2_type4, 216);
	// Run scrambling (all-zero): type-5 bits
	//tetra_scramb_bits(scramb_init, sb2_type5, 216);

	//printf("Scrambled block 2 bits (BNCH): %s\n", osmo_ubit_dump(sb2_type5, 216));

	//printf("Scrambled broadcast bits (AACH): %s\n", osmo_ubit_dump(bb_type5, 30));

	// Finally, hand it into the physical layer
	build_dm_norm_burst(buf, sb1_type5, sb2_type5, 0);

	//printf("Normal continuous downlink burst (NCDB): %s\n", osmo_ubit_dump(buf, BLEN));
}

/* Build a full 'Direct Mode Synchronisation Burst'
*/
void build_dsb(uint8_t *buf, const uint8_t fn)
{
	uint8_t sb_type2[80];
	uint8_t sb_master[80*4];
	uint8_t sb_type3[120];
	uint8_t sb_type4[120];
	//For now we're using a pre-made PDU. Supposed to be implemented in dmo_pdus once we get some time...
	uint8_t sb_type5[120]={1,1,0,1,1,1,1,1,1,1,0,0,1,1,1,0,1,1,1,1,1,1,1,1,0,1,0,1,1,0,1,0,1,1,0,1,0,0,0,0,0,0,1,0,1,1,0,1,1,0,1,0,1,0,1,1,1,1,1,0,1,1,1,1,0,0,0,0,1,0,1,1,1,0,0,0,0,0,1,0,1,1,1,1,0,0,0,0,0,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,0,1,0,1,0,0,1,1,0,0,1,1,1,0,1,1};

	uint8_t sb2_type2[144];
	uint8_t sb2_master[216*4];
	uint8_t sb2_type3[216];
	uint8_t sb2_type4[216];
	uint8_t sb2_type5[216]={1,1,0,1,1,0,0,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,0,0,0,1,1,1,1,0,0,1,0,0,0,0,1,0,1,0,1,1,1,1,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0,0,1,0,1,0,0,1,0,0,1,0,0,0,0,0,0,1,0,1,0,1,0,1,0,0,1,0,0,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,0,1,1,0,0,0,0,1,0,0,0,0,0,1,0,1,1,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,0,1,1,1,1,0,1,1,0,0,0,1,1,0,0,1,0,0,0,1,0,1,0,1,0,0,1,0,0,1,0,0,1,0,1,1,0,0,0,0,0,0,1,0,1,1,0,0,1,1,1,0,1,1,0,1,0,1,1,1,1,0,1,1,1,0,1,1,1,1,1,0,1,1};

	uint16_t crc;
	uint8_t *cur;
	uint32_t bb_rm3014, bb_rm3014_be;

	uint32_t scramb_init = tetra_scramb_get_init(MCC, MNC, CC);

	memset(sb_type2, 0, sizeof(sb_type2));
	cur = sb_type2;

	/* Use pdu_sync from pdus.c */
	cur += osmo_pbit2ubit(sb_type2, pdu_sync, 60);

	crc = ~crc16_ccitt_bits(sb_type2, 60);
	crc = swap16(crc);
	cur += osmo_pbit2ubit(cur, (uint8_t *) &crc, 16);

	/* Append 4 tail bits: type-2 bits */
	cur += 4;

	/* Run rate 2/3 RCPC code: type-3 bits*/
	{
		struct conv_enc_state *ces = calloc(1, sizeof(*ces));
		conv_enc_init(ces);
		conv_enc_input(ces, sb_type2, 80, sb_master);
		get_punctured_rate(TETRA_RCPC_PUNCT_2_3, sb_master, 120, sb_type3);
		free(ces);
	}

	/* Run (120,11) block interleaving: type-4 bits */
	block_interleave(120, 11, sb_type3, sb_type4);

	//UNCOMMENT HERE FOR HOMEMADE PDU
	//memcpy(sb_type5, sb_type4, 120);
	/* Run scrambling (all-zero): type-5 bits */
	//tetra_scramb_bits(SCRAMB_INIT, sb_type5, 120);

	//printf("Scrambled synchronization block 1 bits (BSCH): %s\n", osmo_ubit_dump(sb_type5, 120));

	memset(sb2_type2, 0, sizeof(sb2_type2));
	cur = sb2_type2;

	/* Use pdu_sysinfo from pdus.c */
	cur += osmo_pbit2ubit(sb2_type2, pdu_sysinfo, 124);
	//memcpy(si_type2,pdu_sysinfo_entropia,124);
	//cur +=124;

	/* Run it through CRC16-CCITT */
	crc = ~crc16_ccitt_bits(sb2_type2, 124);
	crc = swap16(crc);
	cur += osmo_pbit2ubit(cur, (uint8_t *) &crc, 16);

	/* Append 4 tail bits: type-2 bits */
	cur += 4;

	/* Run rate 2/3 RCPC code: type-3 bits */
	{
		struct conv_enc_state *ces = calloc(1, sizeof(*ces));
		conv_enc_init(ces);
		conv_enc_input(ces, sb2_type2, 144, sb2_master);
		get_punctured_rate(TETRA_RCPC_PUNCT_2_3, sb2_master, 216, sb2_type3);
		free(ces);
	}

	/* Run (216,101) block interleaving: type-4 bits */
	block_interleave(216, 101, sb2_type3, sb2_type4);

	//UNCOMMENT HERE FOR HOMEMADE PDU
	//memcpy(sb2_type5, sb2_type4, 216);
	/* Run scrambling (all-zero): type-5 bits */
	//tetra_scramb_bits(scramb_init, sb2_type5, 216);



	//printf("Scrambled block 2 bits (BNCH): %s\n", osmo_ubit_dump(sb2_type5, 216));

	/* Finally, hand it into the physical layer */
	build_dm_sync_burst(buf, sb_type5, sb2_type5);

}

int main(int argc, char **argv)
{
	uint8_t burst[BLEN];
	uint8_t *bp = burst;

	uint8_t cur_tn = 0; // timeslot
	uint8_t cur_fn = 1; // frame
	uint8_t cur_mn = 1; // multiframe

	tetra_rm3014_init();
	mac_data_pdu();

	add_guard_bits(bp,1);
	bp +=34;
	build_dsb(bp,1);
	bp +=470;
	add_guard_bits(bp,0);

	printf("%s", osmo_ubit_dump(burst, BLEN));


	exit(0);
}
