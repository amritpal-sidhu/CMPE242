#include "pa6h.h"

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>


static int uart_fd;

// static unsigned int bad_checksum_counter;


int PA6H_jetson_nano_init(void) {

    struct termios tty;
    int retval;

    if (tcgetattr(uart_fd, &tty) != 0) {
        printf("tcgetattr failed\n");
        retval = -1;
    }
    else {
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            printf("tcsetattr failed\n");
            retval = -1;
        }
    }

    if (retval != -1 && (uart_fd = open(JETSON_NANO_LINUX_UART, O_RDWR)) < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        retval = -1;
    }

    return retval;
}

int PA6H_read(char *buf, const int buf_size) {
    return read(uart_fd, buf, buf_size*sizeof(char));
}

int PA6H_write(const char *buf, const int buf_size) {
    return write(uart_fd, buf, buf_size*sizeof(char));
}

void PA6H_jetson_nano_deinit(void) {
    
    close(uart_fd);
}
