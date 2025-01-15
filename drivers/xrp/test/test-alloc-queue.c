//#define _GNU_SOURCE
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>

#include "xrp_kernel_defs.h"
#include "test_data_user.h"

static int fails;
extern char in_data_test[4096*4];

static void test_queue_in(int fd)
{
	char buf[5];

	struct xrp_ioctl_alloc alloc = {
		.size = 4096 * 4,
	};
	int rc = ioctl(fd, XRP_IOCTL_ALLOC, &alloc);

	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_ALLOC)");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_ALLOC), addr = %p\n", alloc.addr);
	}


	struct xrp_ioctl_queue q = {0};
	q.in_data_addr = alloc.addr;
 	q.in_data_size = alloc.size;

	for (int i = 0; i < 4096*4; i++) {

		in_data_test[i] = (i%255);
		#if 0
		if (i < 257)	
			printf("%d ", in_data_test[i]);
		#endif
	}

//	printf("\n");

	memcpy((void *)q.in_data_addr, (void *)in_data_test, q.in_data_size);

	rc = ioctl(fd, XRP_IOCTL_QUEUE, &q);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_QUEUE)");
	} else {
		printf("PASS ioctl(XRP_IOCTL_QUEUE)\n");
	}


	rc = ioctl(fd, XRP_IOCTL_FREE, &alloc);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_FREE)");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_FREE) \n");
	}
}



static void test_queue_out(int fd)
{
	char buf[5];

	struct xrp_ioctl_alloc alloc = {
		.size = 4096 * 4,
	};
	int rc = ioctl(fd, XRP_IOCTL_ALLOC, &alloc);

	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_ALLOC)");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_ALLOC), addr = %p\n", alloc.addr);
	}


	struct xrp_ioctl_queue q = {0};
	q.out_data_addr = alloc.addr;
 	q.out_data_size = alloc.size;


	rc = ioctl(fd, XRP_IOCTL_QUEUE, &q);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_QUEUE)");
	} else {
		printf("PASS ioctl(XRP_IOCTL_QUEUE)\n");
	}


	rc = ioctl(fd, XRP_IOCTL_FREE, &alloc);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_FREE)");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_FREE) \n");
	}
}

static void test_queue_in_out(int fd)
{

	struct xrp_ioctl_alloc in_alloc = {
		.size = 4096 * 4,
	};
	int rc = ioctl(fd, XRP_IOCTL_ALLOC, &in_alloc);

	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_ALLOC) IN ALLOC\n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_ALLOC) IN ALLOC, addr = %p\n", in_alloc.addr);
	}


	struct xrp_ioctl_queue q = {0};
	q.in_data_addr = in_alloc.addr;
 	q.in_data_size = in_alloc.size;

	for (int i = 0; i < 4096*4; i++) {

		in_data_test[i] = 1;
		#if 1
		if (i < 16)	
			printf("%d ", in_data_test[i]);
		#endif
	}

	printf("\n");

	memcpy((void *)q.in_data_addr, (void *)in_data_test, q.in_data_size);


	struct xrp_ioctl_alloc out_alloc = {
		.size = 4096 * 4,
	};
	rc = ioctl(fd, XRP_IOCTL_ALLOC, &out_alloc);

	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_ALLOC) OUT ALLOC \n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_ALLOC) OUT ALLOC, addr = %p\n", out_alloc.addr);
	}

	q.out_data_addr = out_alloc.addr;
 	q.out_data_size = out_alloc.size;

	rc = ioctl(fd, XRP_IOCTL_QUEUE, &q);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_QUEUE)");
	} else {
		printf("PASS ioctl(XRP_IOCTL_QUEUE)\n");
	}

	char *out_data_test = (char *)out_alloc.addr;
	for (int j = 0; j < 4096*4; j++) {

		if (j < 16)	
			printf("%d ", out_data_test[j]);
	}

	printf("\n");


	rc = ioctl(fd, XRP_IOCTL_FREE, &in_alloc);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_FREE) IN ALLOC\n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_FREE) IN ALLOC\n");
	}

	rc = ioctl(fd, XRP_IOCTL_FREE, &out_alloc);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_FREE) OUT ALLOC \n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_FREE) OUT ALLOC \n");
	}
}

static void test_queue_buf(int fd)
{
	struct xrp_ioctl_buffer buf[2] = {0};
	struct xrp_ioctl_queue q = {0};
	int rc;


	struct xrp_ioctl_alloc in_alloc = {
		.size = 4096 * 4,
	};
	rc = ioctl(fd, XRP_IOCTL_ALLOC, &in_alloc);

	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_ALLOC) IN ALLOC\n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_ALLOC) IN ALLOC, addr = %p\n", in_alloc.addr);
	}


	for (int i = 0; i < 4096*4; i++) {

		in_data_test[i] = (i%256);
		#if 1
		if (i < 16)	
			printf("%d ", in_data_test[i]);
		#endif
	}

	printf("\n");

	q.buffer_addr = (__u64)(uintptr_t)&buf[0];
	q.buffer_size = 2*sizeof(struct xrp_ioctl_buffer);

	buf[0].flags = XRP_FLAG_READ;
	buf[0].size = in_alloc.size;
	buf[0].addr = in_alloc.addr;

	memcpy((void *)buf[0].addr, (void *)in_data_test, buf[0].size);

	struct xrp_ioctl_alloc out_alloc = {
		.size = 4096 * 4,
	};
	rc = ioctl(fd, XRP_IOCTL_ALLOC, &out_alloc);

	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_ALLOC) OUT ALLOC \n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_ALLOC) OUT ALLOC, addr = %p\n", out_alloc.addr);
	}

	buf[1].flags = XRP_FLAG_WRITE;
	buf[1].size = out_alloc.size;
	buf[1].addr = out_alloc.addr;

	rc = ioctl(fd, XRP_IOCTL_QUEUE, &q);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_QUEUE)");
	} else {
		printf("PASS ioctl(XRP_IOCTL_QUEUE)\n");
	}

	char *out_data_test = (char *)out_alloc.addr;
	for (int j = 0; j < 4096*4; j++) {

		if (j < 16)	
			printf("%d ", out_data_test[j]);
	}

	printf("\n");

	rc = ioctl(fd, XRP_IOCTL_FREE, &in_alloc);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_FREE) IN ALLOC\n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_FREE) IN ALLOC\n");
	}

	rc = ioctl(fd, XRP_IOCTL_FREE, &out_alloc);
	if (rc == -1) {
		++fails;
		printf("FAIL ioctl(XRP_IOCTL_FREE) OUT ALLOC \n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_FREE) OUT ALLOC \n");
	}

}

int main()
{
	int fd = open("/dev/xvp0", O_RDWR);

//	test_queue_in(fd);
//	test_queue_out(fd);
//	test_queue_in_out(fd);
	test_queue_buf(fd);

	return fails;
}
