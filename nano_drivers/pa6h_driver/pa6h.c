#include "pa6h.h"

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>


static int uart_fd;

// static unsigned int bad_checksum_counter;


int PA6H_jetson_nano_init(void) {

    struct termios tty;
    int retval;

    if ((uart_fd = open(JETSON_NANO_LINUX_UART, O_RDWR)) < 0) {
        retval = -1;
    }

    if (tcgetattr(uart_fd, &tty) != 0) {
        retval = -1;
    }
    else {
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;

        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            retval = -1;
        }
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