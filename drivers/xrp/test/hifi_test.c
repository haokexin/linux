#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <sys/ioctl.h>


struct xrp_sync_cmd {
	char input_str[64];
	char out_str[64];
	int cmd_addr;
	int cmd_size;
};

#define  XRP_IOCTL_SYNC_CMD 29190

int main(int argc,char *argv[])
{

    if(argc != 2) 
    {
        printf("Please enter one params");
        return -1;
    }
	
	int fd = open("/dev/xvp0", O_RDWR);
	if (fd < 0) {
		printf("open xvp0 failed: %d\n", fd);
		return -1;
	} 
	
	struct xrp_sync_cmd cmd = {0};
	int ret;

	cmd.cmd_addr = 0x90000000;
	cmd.cmd_size = 0x1000;;
	strcpy(cmd.input_str,argv[1]);

	printf("ivi to hifi: %s \n", cmd.input_str);
	
	ret = ioctl(fd, XRP_IOCTL_SYNC_CMD, &cmd);
	if (ret < 0) {
		printf("XRP_IOCTL_SYNC_CMD failed: %d\n", ret);
		return -1;
	} 

	printf("hifi reply: %s \n", cmd.out_str);
	return 0;
}
