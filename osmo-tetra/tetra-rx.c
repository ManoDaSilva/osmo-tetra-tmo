/* Test program for tetra burst synchronizer */

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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/stat.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/talloc.h>

#include "tetra_common.h"
#include <phy/tetra_burst.h>
#include <phy/tetra_burst_sync.h>
#include "tetra_gsmtap.h"

void *tetra_tall_ctx;


/* the following was taken from float_to_bits.c (C) 2011 by Harald Welte <laforge@gnumonks.org> --sq5bpf */
static int process_sym_fl(float fl)
{
	int ret;

	/* very simplistic scheme */
	if (fl > 2)
		ret = 3;
	else if (fl > 0)
		ret = 1;
	else if (fl < -2)
		ret = -3;
	else
		ret = -1;

	return ret;
}

static void sym_int2bits(int sym, uint8_t *ret0,uint8_t *ret1)
{
	switch (sym) {
		case -3:
			*ret0 = 1;
			*ret1 = 1;
			break;
		case 1:
			*ret0 = 0;
			*ret1 = 0;
			break;
			//case -1:
		case 3:
			*ret0 = 0;
			*ret1 = 1;
			break;
			//case 3:
		case -1:
			*ret0 = 1;
			*ret1 = 0;
			break;
	}
}

int main(int argc, char **argv)
{
	int fd;
	int opt;
	struct tetra_rx_state *trs;
	struct tetra_mac_state *tms;

	tms = talloc_zero(tetra_tall_ctx, struct tetra_mac_state);
	tetra_mac_state_init(tms);

	int accept_float = 0;
	float filter=0;
	float filter_val=0.0001;
	float filter_goal=0;

	trs = talloc_zero(tetra_tall_ctx, struct tetra_rx_state);
	trs->burst_cb_priv = tms;

	while ((opt = getopt(argc, argv, "d:i")) != -1) {
		switch (opt) {
		case 'd':
			tms->dumpdir = strdup(optarg);
			break;
		case 'i':
			accept_float = 1;
			break;
		default:
			fprintf(stderr, "Unknown option %c\n", opt);
		}
	}

	if (argc <= optind) {
		fprintf(stderr, "Usage: %s <-i> [-d DUMPDIR] <file_with_1_byte_per_bit>\n", argv[0]);
		fprintf(stderr, "-i accept float values (internal float_to_bits)\n");
		exit(1);
	}

	fd = open(argv[optind], O_RDONLY);
	if (fd < 0) {
		perror("open");
		exit(2);
	}

	tetra_gsmtap_init("localhost", 0);

#define BUFLEN 64
#define MAXVAL 5.0

	while (1) {
		uint8_t buf[64];
		int len;
		int i;
		if (accept_float) {
			int rc;
			int rc2;
			float fl[64];
			rc = read(fd, &fl, sizeof(fl));
			if (rc < 0) {
				perror("read");
				exit(1);
			} else if (rc == 0)
				break;
			rc2=rc/sizeof(float);
			for(i=0;i<rc2;i++) {	

				if ((fl[i]>-MAXVAL)&&(fl[i]<MAXVAL)) 
					filter=filter*(1.0-filter_val)+(fl[i]-filter_goal)*filter_val;
					rc = process_sym_fl(fl[i]);
				
				sym_int2bits(rc, &buf[2*i],&buf[2*i+1]);

			}		
			len=rc2*2;

		}
		else
		{
			len = read(fd, buf, sizeof(buf));
			if (len < 0) {
				perror("read");
				exit(1);
			} else if (len == 0) {
				printf("EOF");
				break;
			}
		}
		tetra_burst_sync_in(trs, buf, len);
	}

	free(tms->dumpdir);
	talloc_free(trs);
	talloc_free(tms);

	exit(0);
}
