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

#define DALI_drv "/dev/dali"
 
int 
main(int argc, char* argv[]) {
    char buffer[7];
    int fd;
    int len = 7;
    int ret;
    struct pollfd fds;
    int t;

    if (argc!=2) {
	printf("Usage: %s command <seq: 1-byte> <data: 2-bytes>\n", argv[0]);
	printf("Example: 0700fe\n");
	printf("xxyyyy, xx - hex seq, yyyy - hex data\n\n");
	exit(0);
    }

    fd = open(DALI_drv, O_RDWR );

    if( fd < 0 ) {
	perror("Cannot open device \t");
	printf(" fd = %d \n",fd);
	return 0;
    }

    write(fd, argv[1], strlen(argv[1]));
    bzero(buffer, len);

    fds.fd	= fd;
    fds.events	= POLLIN;
l:
    ret = poll( &fds, 1, 34);

    switch (ret) {
	case -1:
    	    // Error
    	    break;
	case 0:
    	    // Timeout 
    	    break;
	default:
	    read(fd, buffer, len);
	    if (buffer[0]=='f' && buffer[1]=='f') goto l;
	    printf("%s", buffer);
	    break;
    }

    if( 0 != close(fd) ){
	perror("Could not close device\n");
    }

    return 0;
}
