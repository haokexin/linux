//#define _GNU_SOURCE
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "xrp_kernel_defs.h"
#include "test_data_user.h"

static int fails;

static void test_aec_in(int fd)
{
	char buf[5];
	struct xrp_sync_cmd cmd = {0};
	struct xrp_alg_param alg = {0};
	int rc;

	cmd.cmd_addr = 0x90000000;
	cmd.cmd_size = 0;

	rc = ioctl(fd, XRP_IOCTL_ALLOC_SYNC_CMD, &cmd);
	if (rc == -1) {
		perror("XFAIL in_data 1");
	} else {
		++fails;
		fprintf(stderr, "FAIL in_data 1\n");
	}

	
	rc = ioctl(fd, XRP_IOCTL_SYNC_CMD, &cmd);
	if (rc == -1) {
		perror("XFAIL in_data 2");
	} else {
		++fails;
		fprintf(stderr, "FAIL in_data 2\n");
	}

	alg.alg_flg = 0x3;
	alg.mask = 0xff;
	rc = ioctl(fd, XRP_IOCTL_DEF_ALG_SET, &alg);
	if (rc == -1) {
		++fails;
		perror("FAIL in_data 3");
	} else {
		fprintf(stderr, "PASS in_data 3\n");
	}
}

static void test_aec_out(int fd)
{
	char buf[5];
	char ns_id[16];
	struct xrp_output out = {0};
	struct xrp_nsid nsid = {0};	
	
	int rc;

	out.output_addr = 0xa0003000;
	out.outsize = 0x3000;

	rc = ioctl(fd, XRP_IOCTL_ALG_FLUSH, &out);
	if (rc == -1) {
		perror("XFAIL out_data 1");
	} else {
		++fails;
		fprintf(stderr, "FAIL out_data 1\n");
	}

	rc = ioctl(fd, XRP_IOCTL_ALG_RESULT_GET, &out);
	if (rc == -1) {
		perror("XFAIL out_data 2");
	} else {
		++fails;
		fprintf(stderr, "FAIL out_data 2\n");
	}

	nsid.nsid_addr = (uint32_t) ns_id;
	nsid.nsid_size = 16;
	rc = ioctl(fd, XRP_IOCTL_ALLOC_NSID, &nsid);
	if (rc == -1) {
		++fails;
		perror("FAIL out_data 3");
	} else {
		fprintf(stderr, "PASS out_data 3\n");
	}
}
static void test_aec_in_pio(int fd)
{
	char buf[5];
	struct xrp_sync_cmd cmd = {0};
	struct xrp_alg_param alg = {0};
	int rc;

	cmd.cmd_addr = 0x90000000;
	cmd.cmd_size = 0;

	rc = ioctl(fd, XRP_IOCTL_ALLOC_SYNC_CMD, &cmd);
	if (rc == -1) {
		perror("XFAIL in_data 1");
	} else {
		++fails;
		fprintf(stderr, "FAIL in_data 1\n");
	}

	
	rc = ioctl(fd, XRP_IOCTL_SYNC_CMD_PIO, &cmd);
	if (rc == -1) {
		perror("XFAIL in_data 2");
	} else {
		++fails;
		fprintf(stderr, "FAIL in_data 2\n");
	}

	alg.alg_flg = 0x3;
	alg.mask = 0xff;
	rc = ioctl(fd, XRP_IOCTL_DEF_ALG_SET_PIO, &alg);
	if (rc == -1) {
		++fails;
		perror("FAIL in_data 3");
	} else {
		fprintf(stderr, "PASS in_data 3\n");
	}
}

static void test_aec_out_pio(int fd)
{
	char buf[5];
	char ns_id[16];
	struct xrp_output out = {0};
	struct xrp_nsid nsid = {0};	
	
	int rc;

	out.output_addr = 0xa0003000;
	out.outsize = 0x3000;

	rc = ioctl(fd, XRP_IOCTL_ALG_FLUSH, &out);
	if (rc == -1) {
		perror("XFAIL out_data 1");
	} else {
		++fails;
		fprintf(stderr, "FAIL out_data 1\n");
	}

	rc = ioctl(fd, XRP_IOCTL_ALG_RESULT_GET_PIO, &out);
	if (rc == -1) {
		perror("XFAIL out_data 2");
	} else {
		++fails;
		fprintf(stderr, "FAIL out_data 2\n");
	}

	nsid.nsid_addr = ( uint32_t) ns_id;
	nsid.nsid_size = 16;
	rc = ioctl(fd, XRP_IOCTL_ALLOC_NSID, &nsid);
	if (rc == -1) {
		++fails;
		perror("FAIL out_data 3");
	} else {
		fprintf(stderr, "PASS out_data 3\n");
	}
}


int main()
{
	int fd = open("/dev/xvp0", O_RDWR);
	test_aec_in_pio(fd);
	test_aec_out_pio(fd);
	//test_queue_in(fd);
	//test_queue_out(fd);	

	return fails;
}
