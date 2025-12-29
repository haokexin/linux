// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>

struct time_sync_parm {
	unsigned int latch_gtc_hicnt;
	unsigned int latch_gtc_lwcnt;
	long long phc_utc_sec;
	long phc_utc_nsec;
	unsigned int gtc_hicnt;
	unsigned int gtc_lwcnt;
};

struct gtc_freq {
	unsigned int clk_freq;
	unsigned int clk_div;
};

#define GTC_IOC_MAGIC 'g'
#define GTC_IOC_LATCH_CFG    _IOW(GTC_IOC_MAGIC, 1, int)
#define GTC_IOC_INTR_CFG     _IOW(GTC_IOC_MAGIC, 2, int)
#define GTC_IOC_MUX_CFG      _IOW(GTC_IOC_MAGIC, 3, int)
#define GTC_IOC_GET_PARM     _IOR(GTC_IOC_MAGIC, 4, struct time_sync_parm)
#define GTC_IOC_GET_FREQ     _IOR(GTC_IOC_MAGIC, 5, struct gtc_freq)
#define GTC_IOC_LOG          _IOW(GTC_IOC_MAGIC, 6, int)
#define GTC_IOC_TEST_KTIME   _IOW(GTC_IOC_MAGIC, 7, long long)

#define GTC_IOC_MAXNR    8

static void usage(char *progname)
{
	fprintf(stderr,
			"usage: %s [options]\n"
			" -f			get gtc freq info\n"
			" -g			get gtc time sync record\n"
			" -l 255|[0-28]	clear|set latch index\n"
			" -i 0|1		disable/enable soc intr\n"
			" -m 255|[0-28]	disable/set mux\n"
			" -d 0|1        log on/off\n"
			" -k hi32|lw32  gtc counter\n",
			progname);
}

int main(int argc, char *argv[])
{
	char *progname;
	int c, gtc_fd, latch_parm, intr_parm, mux_parm, ret = 0, log_parm;
	long long  cnt_parm = 0;
	bool query = false, latch = false, intr = false, mux = false, freq = false, log = false, cnt = false;
	struct time_sync_parm rec;
	struct gtc_freq freq_info;

	progname = strrchr(argv[0], '/');
	progname = progname ? 1+progname : argv[0];
	while (EOF != (c = getopt(argc, argv, "fgl:i:m:d:k:"))) {
		switch (c) {
		case 'f':
			freq = true;
			printf("get gtc freq\n");
			break;
		case 'g':
			query = true;
			printf("query time sync info\n");
			break;
		case 'l':
			latch_parm = atoi(optarg);
			latch = true;
			printf("latch parm = %d\n", latch_parm);
			break;
		case 'i':
			intr_parm = atoi(optarg);
			intr = true;
			printf("intr parm = %d\n", intr_parm);
			break;
		case 'm':
			mux_parm = atoi(optarg);
			mux = true;
			printf("mux parm = %d\n", mux_parm);
			break;
		case 'd':
			log_parm = atoi(optarg);
			log = true;
			printf("log parm = %d\n", log_parm);
			break;
		case 'k':
			cnt_parm = atol(optarg);
			cnt = true;
			printf("cnt parm = %llu\n", cnt_parm);
			break;
		default:
			usage(progname);
			return -1;
		}
	}

	if (argc < 2) {
		usage(progname);
		return -1;
	}

	gtc_fd = open("/dev/gtc", O_RDWR);
	if (gtc_fd < 0) {
		printf("Failed to open /dev/gtc  %d\n", gtc_fd);
		return -1;
	}

	if (freq) {
		if (ioctl(gtc_fd, GTC_IOC_GET_FREQ, &freq_info) < 0) {
			perror("GTC_IOC_GET_FREQ");
		} else {
		printf("gtc freq:\n"
		    "\tclk_freq :%d\n"
		    "\tclk_div :%d\n",
			freq_info.clk_freq,
		    freq_info.clk_div);
		}
	}

	if (query) {
		if (ioctl(gtc_fd, GTC_IOC_GET_PARM, &rec) < 0) {
			perror("GTC_IOC_GET_PARM");
		} else {
		printf("gtc record:\n"
		    "\tlatch_gtc_hicnt :%d\n"
		    "\tlatch_gtc_lwcnt :%d\n"
		    "\tphc_sec :%llu\n"
		    "\tphc_nsec  :%lu\n"
			"\tgtc_hicnt :%d\n"
		    "\tgtc_lwcnt  :%d\n\n",
			rec.latch_gtc_hicnt,
		    rec.latch_gtc_lwcnt,
		    rec.phc_utc_sec,
		    rec.phc_utc_nsec,
			rec.gtc_hicnt,
		    rec.gtc_lwcnt);
		}
	}

	if (latch) {
		printf("1-1 intr_parm = %d\n", latch_parm);
		ret = ioctl(gtc_fd, GTC_IOC_LATCH_CFG, &latch_parm);
		if (ret < 0)
			perror("GTC_IOC_LATCH_CFG Error");
		else
		 	printf("GTC_IOC_LATCH_CFG Success\n");
	}

	if (intr) {
		printf("1-2 intr_parm = %d\n", intr_parm);
		ret = ioctl(gtc_fd, GTC_IOC_INTR_CFG, &intr_parm);
		if (ret < 0)
			perror("GTC_IOC_INTR_CFG Error");
		else
		 	printf("GTC_IOC_INTR_CFG Success\n");
	}

	if (mux) {
		printf("1-3 mux_parm = %d\n", mux_parm);
		ret = ioctl(gtc_fd, GTC_IOC_MUX_CFG, &mux_parm);
		if (ret < 0)
			perror("GTC_IOC_MUX_CFG Error");
		else
		 	printf("GTC_IOC_MUX_CFG Success\n");
	}

	if (log) {
		printf("0|1 log_parm = %d\n", log_parm);
		ret = ioctl(gtc_fd, GTC_IOC_LOG, &log_parm);
		if (ret < 0)
			perror("GTC_IOC_LOG Error");
		else
		 	printf("GTC_IOC_LOG Success\n");
	}

	if (cnt) {
		printf("hi32|lw32 = %llu\n", cnt_parm);
		ret = ioctl(gtc_fd, GTC_IOC_TEST_KTIME, &cnt_parm);
		if (ret < 0)
			perror("GTC_IOC_TEST_KTIME Error");
		else
		 	printf("GTC_IOC_TEST_KTIME Success\n");
	}
	close(gtc_fd);

	return ret;
}
