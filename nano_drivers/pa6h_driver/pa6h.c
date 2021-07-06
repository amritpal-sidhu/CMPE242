#include "pa6h.h"

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>

#define MAX_PACKET_BUF_SIZE     256


static int uart_fd = -1;

// static unsigned int bad_checksum_counter;


static int linux_uart_set_baud(const unsigned baud_rate);

static int PA6H_set_updaterate(const PAH6_config config);
static int PA6H_set_baudrate(const PAH6_config config);

static int PA6H_set_sbas_enabled(void);
static void PA6H_query_sbas_enabled(void);

static int PA6H_set_dgps_mode(const PAH6_config config);
static void PA6H_query_dgps_mode(void);

static int PA6H_set_output(const PAH6_config config);
static void PA6H_query_output(void);

static unsigned nema_checksum(const char *nema_packet);


int PA6H_jetson_nano_init(const PAH6_config config_data) {

    struct termios tty;
    int retval;

    if ((uart_fd = open(JETSON_NANO_LINUX_UART, O_RDWR)) < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        retval = -1;
    }
    else {

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
    }

    // PA6H_query_sbas_enabled();
    // PA6H_query_dgps_mode();
    // PA6H_query_output();

    // PA6H_set_baudrate(config_data);
    // linux_uart_set_baud(config_data.baud_rate);
    // PA6H_set_updaterate(config_data);
    PA6H_set_dgps_mode(config_data);
    PA6H_set_output(config_data);

    return retval;
}

int PA6H_read(char *buf, const int buf_size) {
    int retval;
    char byte_buf;
    int write_i = 0;

    while (write_i < buf_size-1 && (retval=read(uart_fd, &byte_buf, sizeof(char))) > 0 && byte_buf != '\n') {

        buf[write_i++] = byte_buf;
    }

    buf[write_i] = '\0';

    return retval;
}

int PA6H_write(const char *buf, const int buf_size) {
    return write(uart_fd, buf, buf_size*sizeof(char));
}

void PA6H_jetson_nano_deinit(void) {
    
    close(uart_fd);
}

/**
 * Private local function definitions below
 */

static int linux_uart_set_baud(const unsigned baud_rate) {

    struct termios tty;
    int retval;

    if (tcgetattr(uart_fd, &tty) != 0) {
        printf("tcgetattr failed when linux_uart_set_baud() called\n");
        retval = -1;
    }
    else {
        
        switch (baud_rate)
        {
            case PA6H_BAUD_4800:
                cfsetispeed(&tty, B4800);
                cfsetospeed(&tty, B4800);
                break;

            case PA6H_BAUD_9600:
                cfsetispeed(&tty, B9600);
                cfsetospeed(&tty, B9600);
                break;

            case PA6H_BAUD_19200:
                cfsetispeed(&tty, B19200);
                cfsetospeed(&tty, B19200);
                break;

            case PA6H_BAUD_38400:
                cfsetispeed(&tty, B38400);
                cfsetospeed(&tty, B38400);
                break;

            case PA6H_BAUD_57600:
                cfsetispeed(&tty, B57600);
                cfsetospeed(&tty, B57600);
                break;

            case PA6H_BAUD_115200:
                cfsetispeed(&tty, B115200);
                cfsetospeed(&tty, B115200);
                break;

            default:
                cfsetispeed(&tty, B9600);
                cfsetospeed(&tty, B9600);
                break;
        }

        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            printf("tcsetattr failed when linux_uart_set_baud() called with baud_rate = %i\n", baud_rate);
            retval = -1;
        }
    }

    return retval;
}

static int PA6H_set_updaterate(const PAH6_config config) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,220,3";

    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK220,%i*", config.update_rate) < 0) {
        retval = -1;
    }
    else {
        if (snprintf(tx_buf+strlen(tx_buf), MAX_PACKET_BUF_SIZE, "%02X\r\n", nema_checksum(tx_buf)) < 0)
            retval = -1;
        else {
            const clock_t begin_time = clock();

            printf("DEBUG SET UPDATERATE TX:  %s", tx_buf);
            
            do {
                PA6H_write(tx_buf, strlen(tx_buf));
                PA6H_read(rx_buf, sizeof(rx_buf));
                if (!strncmp(rx_buf, ack_msg, strlen(ack_msg)-2))
                    printf("DEBUG SET UPDATERATE RX:  %s\n", rx_buf);

            } while (strncmp(rx_buf, ack_msg, strlen(ack_msg)));

            printf("DEBUG SET UPDATERATE:  It took %0.6f milliseconds for an ACK\n", 1000*((float)clock() - begin_time)/CLOCKS_PER_SEC);
        }
    }

    return retval;
}

static int PA6H_set_baudrate(const PAH6_config config) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,251,3";

    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK251,%i*", config.baud_rate) < 0) {
        retval = -1;
    }
    else {
        if (snprintf(tx_buf+strlen(tx_buf), MAX_PACKET_BUF_SIZE, "%02X\r\n", nema_checksum(tx_buf)) < 0)
            retval = -1;
        else{
            const clock_t begin_time = clock();

            printf("DEBUG SET BAUDRATE TX:  %s", tx_buf);

            do {
                PA6H_write(tx_buf, strlen(tx_buf));
                PA6H_read(rx_buf, sizeof(rx_buf));
                if (!strncmp(rx_buf, ack_msg, strlen(ack_msg)-2))
                    printf("DEBUG SET BAUDRATE RX:  %s\n", rx_buf);

            } while (strncmp(rx_buf, ack_msg, strlen(ack_msg)));

            printf("DEBUG SET BAUDRATE:  It took %0.6f milliseconds for an ACK\n", 1000*((float)clock() - begin_time)/CLOCKS_PER_SEC);
        }
    }

    return retval;
}

static int PA6H_set_sbas_enabled(void) {
    
    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,313,3";
    
    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK313,1*2E\r\n") < 0)
        retval = -1;
    else {
        const clock_t begin_time = clock();

        printf("DEBUG SBAS ENABLED TX:  %s", tx_buf);

        do {
            PA6H_write(tx_buf, strlen(tx_buf));
            PA6H_read(rx_buf, sizeof(rx_buf));
            if (!strncmp(rx_buf, ack_msg, strlen(ack_msg)-2))
                printf("DEBUG SBAS ENABLED RX:  %s\n", rx_buf);

        } while (strncmp(rx_buf, ack_msg, strlen(ack_msg)));

        printf("DEBUG SBAS ENABLED:  It took %0.6f milliseconds for an ACK\n", 1000*((float)clock() - begin_time)/CLOCKS_PER_SEC);
    }
    
    return retval;
}

static void PA6H_query_sbas_enabled(void) {

    char tx_buf[MAX_PACKET_BUF_SIZE] = "$PMTK413*34\r\n";
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char reply_header[32] = "$PMTK513";
    const clock_t begin_time = clock();

    printf("DEBUG QUERY SBAS ENABLED TX:  %s", tx_buf);

    do {
        PA6H_write(tx_buf, strlen(tx_buf));
        PA6H_read(rx_buf, sizeof(rx_buf));
    } while (strncmp(rx_buf, reply_header, strlen(reply_header)));

    printf("DEBUG QUERY SBAS ENABLED RX:  %s\n", rx_buf);
    printf("DEBUG QUERY SBAS ENABLED:  It took %0.6f milliseconds for an ACK\n", 1000*((float)clock() - begin_time)/CLOCKS_PER_SEC);
}

static int PA6H_set_dgps_mode(const PAH6_config config) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,301,3";

    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK301,%i*", config.dgps_mode) < 0) {
        retval = -1;
    }
    else {
        if (snprintf(tx_buf+strlen(tx_buf), MAX_PACKET_BUF_SIZE, "%02X\r\n", nema_checksum(tx_buf)) < 0)
            retval = -1;
        else {
            /* SBAS needs to be enabled when WAAS is used */
            if (config.dgps_mode == WAAS_DGPS)
                PA6H_set_sbas_enabled();
            
            const clock_t begin_time = clock();
            
            printf("DEBUG SET DGPS MODE TX:  %s", tx_buf);

            do {
                PA6H_write(tx_buf, strlen(tx_buf));
                PA6H_read(rx_buf, sizeof(rx_buf));
                if (!strncmp(rx_buf, ack_msg, strlen(ack_msg)-2))
                    printf("DEBUG SET DGPS MODE RX:  %s\n", rx_buf);

            } while (strncmp(rx_buf, ack_msg, strlen(ack_msg)));

            printf("DEBUG SET DGPS MODE:  It took %0.6f milliseconds for an ACK\n", 1000*((float)clock() - begin_time)/CLOCKS_PER_SEC);
        }
    }

    return retval;
}

static void PA6H_query_dgps_mode(void) {

    char tx_buf[MAX_PACKET_BUF_SIZE] = "$PMTK401*37\r\n";
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char reply_header[32] = "$PMTK501";
    const clock_t begin_time = clock();

    printf("DEBUG QUERY SET DGPS MODE TX:  %s", tx_buf);

    do {
        PA6H_write(tx_buf, strlen(tx_buf));
        PA6H_read(rx_buf, sizeof(rx_buf));
    } while (strncmp(rx_buf, reply_header, strlen(reply_header)));

    printf("DEBUG QUERY SET DGPS MODE RX:  %s\n", rx_buf);
    printf("DEBUG QUERY SET DGPS MODE:  It took %0.6f milliseconds for an ACK\n", 1000*((float)clock() - begin_time)/CLOCKS_PER_SEC);
}

static int PA6H_set_output(const PAH6_config config) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,314,3";

    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK314,%i,%i,%i,%i,%i,%i,0,0,0,0,0,0,0,0,0,0,0,0,%i*", 
    config.sen_output_rates.gll, config.sen_output_rates.rmc, config.sen_output_rates.vtg, config.sen_output_rates.gga,
    config.sen_output_rates.gsa, config.sen_output_rates.gsv, config.sen_output_rates.mchn) < 0) {
        retval = -1;
    }
    else {
        if (snprintf(tx_buf+strlen(tx_buf), MAX_PACKET_BUF_SIZE, "%02X\r\n", nema_checksum(tx_buf)) < 0)
            retval = -1;
        else {
            const clock_t begin_time = clock();

            printf("DEBUG SET OUTPUT TX:  %s", tx_buf);

            do {
                PA6H_write(tx_buf, strlen(tx_buf));
                PA6H_read(rx_buf, sizeof(rx_buf));
                if (!strncmp(rx_buf, ack_msg, strlen(ack_msg)-2))
                    printf("DEBUG SET OUTPUT RX:  %s\n", rx_buf);

            } while (strncmp(rx_buf, ack_msg, strlen(ack_msg)));

            printf("DEBUG SET OUTPUT:  It took %0.6f milliseconds for an ACK\n", 1000*((float)clock() - begin_time)/CLOCKS_PER_SEC);
        }
    }

    return retval;
}

static void PA6H_query_output(void) {

    char tx_buf[MAX_PACKET_BUF_SIZE] = "$PMTK414*33\r\n";
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char reply_header[32] = "$PMTK514";
    const clock_t begin_time = clock();

    printf("DEBUG QUERY OUTPUT TX:  %s", tx_buf);

    do {
        PA6H_write(tx_buf, strlen(tx_buf));
        PA6H_read(rx_buf, sizeof(rx_buf));
    } while (strncmp(rx_buf, reply_header, strlen(reply_header)));

    printf("DEBUG QUERY OUTPUT RX:  %s\n", rx_buf);
    printf("DEBUG QUERY OUTPUT:  It took %0.6f milliseconds for an ACK\n", 1000*((float)clock() - begin_time)/CLOCKS_PER_SEC);
}

static unsigned nema_checksum(const char *nema_packet) {

    unsigned checksum = 0;
    int i = 0;

    if (nema_packet[i] == '$') {
        
        while (nema_packet[++i] != '*') {
            // if (i > strlen(nema_packet)-5) {
            //     checksum = 0;
            //     break;
            // }
            checksum ^= nema_packet[i];
        }
    }

    return checksum;
}
