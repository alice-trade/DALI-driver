#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#define DALI_drv "/dev/dali"

static u_int8_t	seq = 0;
static u_int8_t ShortAddr = 0;

void 
send_command(int fd, char *command) {
    char buffer[7];
    sprintf(buffer, "%02x%4s", seq, command);
    write(fd, buffer, 6);
    seq+=1;
}

void 
init_search(int fd) {
    send_command(fd, "2000");
    usleep(90000);

    send_command(fd, "2000");
    usleep(90000);

    send_command(fd, "A500");
    usleep(90000);

    send_command(fd, "A500");
    usleep(90000);

    send_command(fd, "A700");
    usleep(90000);

    send_command(fd, "A700");
    usleep(90000);
}
 
int 
SearchAndCompare(int fd, u_int32_t addr) {
    u_int8_t  h, m, l;
    char buffer[7];
    struct pollfd fds;
    int ret;
    int res = 0;

    h = addr >> 16;
    h = h & 0xff;

    m = addr >> 8;
    m = m & 0xff;

    l = addr & 0xff;

    sprintf(buffer, "B1%02x", h);
    send_command(fd, &buffer[0]);
    usleep(34194);

    sprintf(buffer, "B3%02x", m);
    send_command(fd, &buffer[0]);
    usleep(34194);

    sprintf(buffer, "B5%02x", l);
    send_command(fd, &buffer[0]);
    usleep(34194);

    send_command(fd, "A900");
    usleep(34194);

    fds.fd	= fd;
    fds.events	= POLLIN;

    ret = poll( &fds, 1, 34);

    switch (ret) {
	case -1:
    	    break;
	case 0:
    	    break;
	default:
	    read(fd, buffer, 7);
	    res = 1;
//	    printf("%s", buffer);
	    break;
    }
    return res;
}

int 
main(int argc, char* argv[]) {
    struct timeval timeout;
    char buffer[7];
    int fd;
    int len = 7;
    int ret;
    u_int32_t t;
    fd = open(DALI_drv, O_RDWR );

    if( fd < 0 ) {
	perror("Cannot open device \t");
	printf(" fd = %d \n",fd);
	return 0;
    }

    init_search(fd);

    while (ShortAddr < 64) {
	u_int32_t SearchAddr = 0xFFFFFF;
	u_int8_t  Response = 0;
	u_int32_t LowLimit = 0;
	u_int32_t HighLimit = 0x1000000;
	
	Response = SearchAndCompare(fd, SearchAddr);
	if(Response) {
	    printf("Device detected, address searching...\n");
	    if(!SearchAndCompare(fd, SearchAddr - 1)) {
		SearchAndCompare(fd, SearchAddr);
		sprintf(buffer, "B7%02x", ((ShortAddr << 1) | 1));
		send_command(fd, &buffer[0]);
		usleep(102582);
		send_command(fd, "AB00");
		usleep(34194);
		printf("24-bit address found %06x - assigning short address %d\n", SearchAddr, ShortAddr);
		break;
	    }
	}else{
	    printf("No devices detected\n");
	    break;
	}

	while(1) {
	    SearchAddr = (long)((LowLimit + HighLimit) / 2);
	    Response = SearchAndCompare(fd, SearchAddr);

	    if (Response) {
		if ((SearchAddr == 0) || (!SearchAndCompare(fd, SearchAddr - 1))) break;
    		HighLimit = SearchAddr;
    	    }else LowLimit = SearchAddr;
	}
	SearchAndCompare(fd, SearchAddr);
	sprintf(buffer, "B7%02x", ((ShortAddr << 1) | 1));
	send_command(fd, &buffer[0]);
	usleep(102582);
	send_command(fd, "AB00");
	usleep(34194);
	printf("24-bit address found %06x - assigning short address %d\n", SearchAddr, ShortAddr);
	
	ShortAddr++;
    }

    send_command(fd, "A100");
    printf("Init complete\n");

    if( 0 != close(fd) ){
	perror("Could not close device\n");
    }

    return 0;
}
