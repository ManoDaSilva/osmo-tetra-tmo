/* Implementation of TETRA Physical Layer, i.e. what is _below_
 * CRC, FEC, Interleaving and Scrambling */

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


#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <phy/dmo_tetra_burst.h>

#define DQPSK4_BITS_PER_SYM	2


//TODO: APPLY CORRECT DMO OFFSETS
#define DNB_BLK1_OFFSET ((5+1+1)*DQPSK4_BITS_PER_SYM)
#define DNB_BLK2_OFFSET	((5+1+1+108+7+11+8)*DQPSK4_BITS_PER_SYM)
#define DNB_BLK_BITS	(108*DQPSK4_BITS_PER_SYM)


#define DSB_SBLK_OFFSET	((6+1+40)*DQPSK4_BITS_PER_SYM)
#define DSB_BLK2_OFFSET	((6+1+40+60+19+15)*DQPSK4_BITS_PER_SYM)
#define DSB_SBLK_BITS	(60*DQPSK4_BITS_PER_SYM)
#define DSB_BLK2_BITS	(108*DQPSK4_BITS_PER_SYM)


/* 9.4.4.3.1 Frequency Correction Field */
static const uint8_t f_bits[80] = {
	/* f1 .. f8 = 1 */
	1,1,1,1,1,1,1,1,
	/* f9 .. f72 = 0*/
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	/* f73 .. f80 = 1*/
	1,1,1,1,1,1,1,1
	};

/* 9.4.3.3.2 Inter-slot frequency correction field*/
static const uint8_t g_bits[40] = {
	/* g1 .. g6 = 0 */
	0,0,0,0,0,0,
	/* g7 .. g32 = 1 */
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	/* g33 .. g40 = 0 */
	0,0,0,0,0,0,0,0
};

/* 9.4.3.3.3 Normal Training Sequence */
static const uint8_t j_bits[12] = {0,0,1,1,0,0,1,0,0,0,1,1};
static const uint8_t k_bits[12] = {1,0,0,1,1,0,1,0,1,0,0,1};
static const uint8_t n_bits[22] = { 1,1, 0,1, 0,0, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,1, 0,0 };
static const uint8_t p_bits[22] = { 0,1, 1,1, 1,0, 1,0, 0,1, 0,0, 0,0, 1,1, 0,1, 1,1, 1,0 };
static const uint8_t q_bits[22] = { 1,0, 1,1, 0,1, 1,1, 0,0, 0,0, 0,1, 1,0, 1,0, 1,1, 0,1 };
static const uint8_t N_bits[33] = { 1,1,1, 0,0,1, 1,0,1, 1,1,1, 0,0,0, 1,1,1, 1,0,0, 0,1,1, 1,1,0, 0,0,0, 0,0,0 };
static const uint8_t P_bits[33] = { 1,0,1, 0,1,1, 1,1,1, 1,0,1, 0,1,0, 1,0,1, 1,1,0, 0,0,1, 1,0,0, 0,1,0, 0,1,0 };
static const uint8_t I_bits[12] = {0,0,0,1,0,1,0,0,0,1,1,1};

/* 9.4.4.3.3 Extended training sequence */
static const uint8_t x_bits[30] = { 1,0, 0,1, 1,1, 0,1, 0,0, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,1, 0,0, 0,0, 1,1 };
static const uint8_t X_bits[45] = { 0,1,1,1,0,0,1,1,0,1,0,0,0,0,1,0,0,0,1,1,1,0,1,1,0,1,0,1,0,1,1,1,1,1,0,1,0,0,0,0,0,1,1,1,0 };

/* 9.4.4.3.4 Synchronization training sequence */
static const uint8_t y_bits[38] = { 1,1, 0,0, 0,0, 0,1, 1,0, 0,1, 1,1, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,0, 0,0, 0,1, 1,0, 0,1, 1,1 };

/* 9.4.4.3.5 Tail bits */
static const uint8_t t_bits[2] = { 0, 0 };
static const uint8_t T_bits[6] = { 1, 1, 1, 0, 0, 0 };

/* 9.4.4.3.6 Phase adjustment bits */
enum phase_adj_bits { HA, HB, HC, HD, HE, HF, HG, HH, HI, HJ, HK,HL};
struct phase_adj_n {
	uint16_t n1;
	uint16_t n2;
};

/* Table 8.14 */
static const struct phase_adj_n phase_adj_n[] = {
	[HA] = { .n1 = 8,	.n2 = 122 },
	[HB] = { .n1 = 123,	.n2 = 249 },
	[HC] = { .n1 = 8,	.n2 = 108 },
	[HD] = { .n1 = 109,	.n2 = 249 },
	[HE] = { .n1 = 112,	.n2 = 230 },
	[HF] = { .n1 = 1,	.n2 = 111 },
	[HG] = { .n1 = 3,	.n2 = 117 },
	[HH] = { .n1 = 118,	.n2 = 224 },
	[HI] = { .n1 = 3,	.n2 = 103 },
	[HJ] = { .n1 = 104,	.n2 = 224 },
	[HK] = { .n1 = 8,	.n2 = 126 },
	[HL] = { .n1 = 8,	.n2 = 126 },
};

static const int8_t bits2phase[] = {
	[0]	= 1,		/* +pi/4 needs to become -pi/4 */
	[1]	= -1,		/* -pi/4 needs to become +pi/4 */
	[2]	= +3,		/* +3pi/4 needs to become -3pi/4 */
	[3]	= -3,		/* -3pi/4 needs to become +3pi/4 */
};

/* offset everything by 3 in order to get positive array index */
#define PHASE(x)	((x)+3)
struct phase2bits {
	int8_t phase;
	uint8_t bits[2];
};
static const struct phase2bits phase2bits[] = {
	[PHASE(-3)]	= { -3, {1, 1} },
	[PHASE(-1)]	= { -1, {0, 1} },
	[PHASE( 1)]	= {  1, {0, 0} },
	[PHASE( 3)]	= {  3, {1, 0} },
};

static int32_t calc_phase_adj(int32_t phase)
{
	int32_t adj_phase = -(phase % 8);

	/* 'wrap around' to get a value in the range between +3 / -3 */
	if (adj_phase > 3)
		adj_phase -= 8;
	else if (adj_phase < -3)
		adj_phase += 8;

	return adj_phase;
}

/* return the cumulative phase shift of all bits (in units of pi/4) */
int32_t sum_up_phase(const uint8_t *bits, unsigned int sym_count)
{
	uint8_t sym_in;
	int32_t sum_phase = 0;
	unsigned int n;

	for (n = 0; n < sym_count; n++) {
		/* offset '-1' due to array-index starting at 0 */
		uint32_t bn = 2*n;
		sym_in = bits[bn];
		sym_in |= bits[bn+1] << 1;

		sum_phase += bits2phase[sym_in];
	}

	//printf("phase sum over %u symbols: %dpi/4, mod 8 = %dpi/4, wrap = %dpi/4\n",
	//	sym_count, sum_phase, sum_phase % 8, calc_phase_adj(sum_phase));
	return sum_phase;
}

/* compute phase adjustment bits according to 'pa' and write them to {out, out+2} */
void put_phase_adj_bits(const uint8_t *bits, enum phase_adj_bits pa, uint8_t *out)
{
	int32_t sum_phase, adj_phase;
	const struct phase_adj_n *pan = &phase_adj_n[pa];
	const struct phase2bits *p2b;

	/* offset '-1' due to array-index starting at 0 */
	sum_phase = sum_up_phase(bits + 2*(pan->n1-1), 1 + pan->n2 - pan->n1);
	adj_phase = calc_phase_adj(sum_phase);

	p2b = &phase2bits[adj_phase];

	*out++ = p2b->bits[0];
	*out = p2b->bits[1];
}

/* 9.4.3.2.3 Direct Mode Synchronization Burst */
int build_dm_sync_burst(uint8_t *buf, const uint8_t *sb, const uint8_t *bkn)
{
	uint8_t *cur = buf;
	uint8_t *hl;

	/* Normal Training Sequence: q11 to q22 */
	memcpy(cur, I_bits+10, 12);
	cur += 12;

	/* Phase adjustment bits: hc1 to hc2 */
	hl = cur;
	cur += 2;

	/* Frequency correction: f1 to f80 */
	memcpy(cur, f_bits, 80);
	cur += 80;

	/* Scrambled synchronization block 1 bits: sb(1) to sb(120) */
	memcpy(cur, sb, 120);
	cur += 120;

	/* Synchronization training sequence: y1 to y38 */
	memcpy(cur, y_bits, 38);
	cur+= 38;

	/* Scrambled block2 bits: bkn2(1) to bkn2(216) */
	memcpy(cur, bkn, 216);
	cur += 216;

	/* Normal training sequence 3: q1 to q10 */
	memcpy(cur, t_bits, 2);
	cur += 2;

	/* put in the phase adjustment bits */
	put_phase_adj_bits(buf, HL, hl);

	return cur - buf;
}

/* 9.4.3.2.3 Direct Mode Synchronization Burst */
int build_dm_norm_burst(uint8_t *buf, const uint8_t *bkn1, const uint8_t *bkn2, int two_log_chan)
{
	uint8_t *cur = buf;
	uint8_t *hk;

	/* Preamble: j1 to j12 or k1 to k12 */
	if (two_log_chan)
		memcpy(cur, k_bits, 12);
	else
		memcpy(cur, j_bits, 12);
	cur += 12;

	/* Phase adjustment bits: hc1 to hc2 */
	hk = cur;
	cur += 2;

	/* Scrambled block 1 bits: bkn1(1) to bkn1(216) */
	memcpy(cur, bkn1, 216);
	cur += 216;

	/* Normal training sequence: n1 to n22 or p1 to p22 */
	if (two_log_chan)
		memcpy(cur, p_bits, 22);
	else
		memcpy(cur, n_bits, 22);
	cur += 22;

	/* Scrambled block2 bits: bkn2(1) to bkn2(216) */
	memcpy(cur, bkn2, 216);
	cur += 216;

	/* Normal training sequence 3: q1 to q10 */
	memcpy(cur, t_bits, 2);
	cur += 2;

	/* put in the phase adjustment bits */
	put_phase_adj_bits(buf, HK, hk);

	return cur - buf;
}
