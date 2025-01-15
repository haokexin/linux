//#define _GNU_SOURCE
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include "xrp_kernel_defs.h"

#define KWS_FRAME_LEN 16000
char* audio_buf = NULL;

static void test_kws_by_queue_buf(int fd)
{
	struct xrp_ioctl_buffer buf[2] = {0};
	struct xrp_ioctl_queue q = {0};
	int rc;
	int audio_len = KWS_FRAME_LEN* 2;  
	char *in_test_data;

	struct xrp_ioctl_alloc in_alloc = {
		.size = audio_len,
	};
	rc = ioctl(fd, XRP_IOCTL_ALLOC, &in_alloc);

	if (rc == -1) {
		printf("FAIL ioctl(XRP_IOCTL_ALLOC) IN ALLOC\n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_ALLOC) IN ALLOC, addr = %p\n", in_alloc.addr);
	}


	q.buffer_addr = (__u64)(uintptr_t)&buf[0];
	q.buffer_size = 2*sizeof(struct xrp_ioctl_buffer);

	buf[0].flags = XRP_FLAG_READ;
	buf[0].size = in_alloc.size;
	buf[0].addr = in_alloc.addr;

	memcpy((void *)buf[0].addr, (void *)audio_buf, buf[0].size);

	in_test_data = (char *)buf[0].addr;
	for (int i = 0; i < 16; i++) {
			printf("%d ", in_test_data[i]);
	}
	printf("\n");

	struct xrp_ioctl_alloc out_alloc = {
		.size = 4096 * 4,
	};
	rc = ioctl(fd, XRP_IOCTL_ALLOC, &out_alloc);

	if (rc == -1) {
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
		printf("FAIL ioctl(XRP_IOCTL_QUEUE)");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_QUEUE)\n");
	}

	char *kws_res = (char *)buf[1].addr;
	printf("kws result ( %s )\n", kws_res);


	rc = ioctl(fd, XRP_IOCTL_FREE, &in_alloc);
	if (rc == -1) {
		printf("FAIL ioctl(XRP_IOCTL_FREE) IN ALLOC\n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_FREE) IN ALLOC\n");
	}

	rc = ioctl(fd, XRP_IOCTL_FREE, &out_alloc);
	if (rc == -1) {
		printf("FAIL ioctl(XRP_IOCTL_FREE) OUT ALLOC \n");
		return;
	} else {
		printf("PASS ioctl(XRP_IOCTL_FREE) OUT ALLOC \n");
	}

}

int main(int argc,char *argv[])
{
	if(argc != 2) 
    {
        printf("Please enter one params");
        return -1;
    }
	
	FILE *fp;   
	char file_0[] =  "./0.pcm";
	char file_1[] =  "./1.pcm";
	int audio_len = KWS_FRAME_LEN * 2; 

	switch (*argv[1]) {
	case '0':
		fp = fopen(file_0, "r");
		break;
	case '1':
		fp = fopen(file_1, "r");
		break;
	default:
		printf("invalid params : %s\n", argv[1]);
		return -1;
	}

	audio_buf = (char*) malloc(audio_len);
	fread(audio_buf, sizeof(char), audio_len, fp);
	fclose(fp);	

	int fd = open("/dev/xvp0", O_RDWR);
	if (fd < 0) {
		printf("open xvp0 failed: %d\n", fd);
		free(audio_buf);
		return -1;
	} 

	test_kws_by_queue_buf(fd);

	free(audio_buf);
	return 0;
}
