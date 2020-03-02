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

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/utils.h>

#include "dmo_pdus.h"

uint8_t dmac_sync_pdu_schs[8];	/* 60 bits */
uint8_t dmac_sync_pdu_schh[16];		/* 124 bits */
uint8_t dmac_data_pdu[16];       /* 124 bits */


void sync_schs_pdu(uint8_t fn, uint8_t tn){
	struct bitvec bv;
	memset(&bv, 0, sizeof(bv));
	bv.data = dmac_sync_pdu_schs;
	bv.data_len = sizeof(dmac_sync_pdu_schs);

	// According to Table 21: DMAC-SYNC PDU Contents
	bitvec_set_uint(&bv, 13, 4);	// System Code: ETS 300 392-2 ed. 1
	bitvec_set_uint(&bv, 0, 2);	// SYNC PDU Type
	bitvec_set_uint(&bv, 0, 2);	// Comms type: MS-MS
	bitvec_set_bit(&bv,0); //Reserved (if comms type = 0)
	bitvec_set_bit(&bv, 0); //Reserved (if comm type = 0)
	bitvec_set_uint(&bv, 0, 2);	// A/B Channel usage (A, normal mode)
	bitvec_set_uint(&bv, tn, 2);	// Slot Number
	bitvec_set_uint(&bv, fn, 5);	// Frame number
	bitvec_set_uint(&bv, 0, 2);	// Encryption type
	bitvec_set_uint(&bv, 0,39); //Empty if no encryption
	//bitvec_set_bit(&bv, 0);		// U-plane DTX: Discontinuous U-plane transmission is not allowed

	//printf("DMAC SYNC PDU (SCH/S) %s\n", osmo_hexdump(dmac_sync_pdu_schs, sizeof(dmac_sync_pdu_schs)));

}

void sync_schh_pdu(uint8_t mcc, uint16_t mnc){
	struct bitvec bv;
	memset(&bv, 0, sizeof(bv));
	bv.data = dmac_sync_pdu_schh;
	bv.data_len = sizeof(dmac_sync_pdu_schh);
	// According to Table 22: DMAC-SYNC PDU Contents
	bitvec_set_uint(&bv, 0, 10);	//If Comms = 0, all blank (Reserved)
	bitvec_set_bit(&bv,0); //Fill bit indication
	bitvec_set_bit(&bv,0); //Fragmentation flag
	bitvec_set_uint(&bv, 0, 2);	//Frame countdown
	bitvec_set_uint(&bv, 0, 2);	//Destination address type 
	bitvec_set_uint(&bv, 2065999, 24);//Destination address 
	bitvec_set_uint(&bv, 0, 2);	//Source address type 
	bitvec_set_uint(&bv, 2065998, 24);//Source address
	bitvec_set_uint(&bv, mcc, 10);//MNI MCC
	bitvec_set_uint(&bv, mnc, 14);//MNI MNC
	bitvec_set_uint(&bv, 0,5);//Message type
	//printf("DMAC SYNC PDU (SCH/H) %s\n", osmo_hexdump(dmac_sync_pdu_schh, sizeof(dmac_sync_pdu_schh)));

}
