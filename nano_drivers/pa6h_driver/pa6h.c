#include "pa6h.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

#define MAX_PACKET_BUF_SIZE     128


static int uart_fd = -1;

static unsigned previous_baud_rate = PA6H_BAUD_9600;
// static unsigned int bad_checksum_counter;


static int linux_uart_set_baud(const unsigned baud_rate);

static int PA6H_read_sentence(char *buf, const int buf_size);
static int PA6H_write_sentence(const char *buf, const int buf_size);

static int PA6H_config_wrapper(const char *config_cmd_name, const char *pmtk_sen, const char *ack_sen, const unsigned timeout_ms);

static int PA6H_set_updaterate(const PAH6_config config, const unsigned timeout_ms);
static int PA6H_set_baudrate(const PAH6_config config, const unsigned timeout_ms);

static int PA6H_set_sbas_enabled(const unsigned timeout_ms);
static int PA6H_query_sbas_enabled(const unsigned timeout_ms);

static int PA6H_set_dgps_mode(const PAH6_config config, const unsigned timeout_ms);
static int PA6H_query_dgps_mode(const unsigned timeout_ms);

static int PA6H_set_output(const PAH6_config config, const unsigned timeout_ms);
static int PA6H_query_output(const unsigned timeout_ms);

static int nema_checksum(const char *nema_packet);

static float coordinate_strtof(const char *str);


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

    // PA6H_query_sbas_enabled(5000);
    // PA6H_query_dgps_mode(5000);
    // PA6H_query_output(5000);

    // PA6H_set_baudrate(config_data, 5000);
    PA6H_set_updaterate(config_data, 5000);
    PA6H_set_dgps_mode(config_data, 5000);
    PA6H_set_output(config_data, 5000);

    return retval;
}

void PA6H_jetson_nano_deinit(void) {
    
    close(uart_fd);
}

int PA6H_read_GP_sentence(char *buf, const int buf_size) {

    int checksum = 0;
    char GP_header[8] = "$GP";

    while (PA6H_read_sentence(buf, buf_size) && strncmp(buf, GP_header, strlen(GP_header)));
    if ((checksum=nema_checksum(buf)) != -1) {
        char rx_checksum[3];
        strncpy(rx_checksum, strchr(buf, '*')+1, 2*sizeof(char));
        rx_checksum[2] = '\0';
        if (checksum != strtol(rx_checksum, NULL, 16))
            checksum = -1;
    }

    return checksum;
}

int PA6H_parse_coordinate(char *GP_sentence, PAH6_gps_coordinate *result_coordinate) {

    int retval = 0;
    char *delim_ptr;

    delim_ptr = strtok(GP_sentence, ",");

    if (!strcmp(delim_ptr, "$GPGLL")) {
        
        delim_ptr = strtok(NULL, ",");
    }
    else if (!strcmp(delim_ptr, "$GPGGA")) {

        for (size_t i = 0; i < 2; ++i)
            delim_ptr = strtok(NULL, ",");
    }
    else if (!strcmp(delim_ptr, "$GPRMC")) {

        for (size_t i = 0; i < 3; ++i)
            delim_ptr = strtok(NULL, ",");
    }
    else {
        retval = -1;
    }

    if (!retval) {

        result_coordinate->latitude = coordinate_strtof(delim_ptr);
        delim_ptr = strtok(NULL, ",");
        if (!strcmp(delim_ptr, "S"))
            result_coordinate->latitude *= -1;
        
        delim_ptr = strtok(NULL, ",");
        result_coordinate->longitude = coordinate_strtof(delim_ptr);
        delim_ptr = strtok(NULL, ",");
        if (!strcmp(delim_ptr, "W"))
            result_coordinate->longitude *= -1;
    }

    return retval;
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

int PA6H_read_sentence(char *buf, const int buf_size) {
    int retval;
    char byte_buf;
    int write_i = 0;

    while (write_i < buf_size-1 && (retval=read(uart_fd, &byte_buf, sizeof(char))) > 0 && byte_buf != '\n') {

        buf[write_i++] = byte_buf;
    }

    buf[write_i] = '\0';

    return retval;
}

int PA6H_write_sentence(const char *buf, const int buf_size) {
    return write(uart_fd, buf, buf_size*sizeof(char));
}

static int PA6H_config_wrapper(const char *config_cmd_name, const char *pmtk_sen, const char *ack_sen, const unsigned timeout_ms) {
    
    int retval = 0;
    char rx_buf[MAX_PACKET_BUF_SIZE];
    const clock_t begin_tick = clock();
    const clock_t end_tick = (((float)timeout_ms/1000)*CLOCKS_PER_SEC) + begin_tick;

    printf("TX %s MSG:  %s", config_cmd_name, pmtk_sen);

    do {
        PA6H_write_sentence(pmtk_sen, strlen(pmtk_sen));
        PA6H_read_sentence(rx_buf, sizeof(rx_buf));
        if (!strncmp(rx_buf, ack_sen, strlen(ack_sen)-2))
            printf("RX %s MSG:  %s\n", config_cmd_name, rx_buf);

        if (timeout_ms && clock() >= end_tick)
            retval = ETIMEDOUT;

    } while (!retval && strncmp(rx_buf, ack_sen, strlen(ack_sen)));

    if (!retval)
        printf("%s:  It took %0.6f milliseconds for an ACK\n", config_cmd_name, 1000*(((float)clock()-begin_tick)/CLOCKS_PER_SEC));
    else
        printf("%s:  Timeout of %i milliseconds reached\n", config_cmd_name, timeout_ms);

    return retval;
}

static int PA6H_set_updaterate(const PAH6_config config, const unsigned timeout_ms) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char rx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,220,3";
    const clock_t begin_tick = clock();
    const clock_t end_tick = (((float)timeout_ms/1000)*CLOCKS_PER_SEC) + begin_tick;

    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK220,%i*", config.update_rate) < 0
        || snprintf(tx_buf+strlen(tx_buf), MAX_PACKET_BUF_SIZE, "%02X\r\n", nema_checksum(tx_buf)) < 0) {
        retval = -1;
    }
    else {
        printf("TX %s MSG:  %s", "SET UPDATERATE", tx_buf);

        PA6H_write_sentence(tx_buf, strlen(tx_buf));
        linux_uart_set_baud(config.update_rate);

        do {
            PA6H_read_sentence(rx_buf, sizeof(rx_buf));
            if (!strncmp(rx_buf, ack_msg, strlen(ack_msg)-2))
                printf("RX %s MSG:  %s\n", "SET UPDATERATE", rx_buf);

            if (timeout_ms && clock() >= end_tick)
                retval = ETIMEDOUT;

        } while (!retval && strncmp(rx_buf, ack_msg, strlen(ack_msg)));

        if (!retval) {
            printf("%s:  It took %0.6f milliseconds for an ACK\n", "SET UPDATERATE", 1000*(((float)clock()-begin_tick)/CLOCKS_PER_SEC));
            previous_baud_rate = config.update_rate;
        }
        else {
            printf("%s:  Timeout of %i milliseconds reached\n", "SET UPDATERATE", timeout_ms);
            linux_uart_set_baud(previous_baud_rate);
        }
    }

    return retval;
}

static int PA6H_set_baudrate(const PAH6_config config, const unsigned timeout_ms) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,251,3";

    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK251,%i*", config.baud_rate) < 0
        || snprintf(tx_buf+strlen(tx_buf), MAX_PACKET_BUF_SIZE, "%02X\r\n", nema_checksum(tx_buf)) < 0) {
        retval = -1;
    }
    else {
        retval = PA6H_config_wrapper("SET BAUDRATE", tx_buf, ack_msg, timeout_ms);
    }

    return retval;
}

static int PA6H_set_sbas_enabled(const unsigned timeout_ms) {
    
    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,313,3";
    
    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK313,1*2E\r\n") < 0)
        retval = -1;
    else {
        retval = PA6H_config_wrapper("SBAS ENABLE", tx_buf, ack_msg, timeout_ms);
    }
    
    return retval;
}

static int PA6H_query_sbas_enabled(const unsigned timeout_ms) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE] = "$PMTK413*34\r\n";
    char reply_header[32] = "$PMTK513";
    
    retval = PA6H_config_wrapper("QUERY SBAS", tx_buf, reply_header, timeout_ms);

    return retval;
}

static int PA6H_set_dgps_mode(const PAH6_config config, const unsigned timeout_ms) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,301,3";

    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK301,%i*", config.dgps_mode) < 0
        || snprintf(tx_buf+strlen(tx_buf), MAX_PACKET_BUF_SIZE, "%02X\r\n", nema_checksum(tx_buf)) < 0) {
        retval = -1;
    }
    else {
        /* SBAS needs to be enabled when WAAS is used */
        if (config.dgps_mode == WAAS_DGPS)
            PA6H_set_sbas_enabled(timeout_ms);
        
        retval = PA6H_config_wrapper("SET DGPS MODE", tx_buf, ack_msg, timeout_ms);
    }

    return retval;
}

static int PA6H_query_dgps_mode(const unsigned timeout_ms) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE] = "$PMTK401*37\r\n";
    char reply_header[32] = "$PMTK501";
    
    retval = PA6H_config_wrapper("QUERY DGPS MODE", tx_buf, reply_header, timeout_ms);

    return retval;
}

static int PA6H_set_output(const PAH6_config config, const unsigned timeout_ms) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE];
    char ack_msg[32] = "$PMTK001,314,3";

    if (snprintf(tx_buf, MAX_PACKET_BUF_SIZE, "$PMTK314,%i,%i,%i,%i,%i,%i,0,0,0,0,0,0,0,0,0,0,0,0,%i*", 
        config.sen_output_rates.gll, config.sen_output_rates.rmc, config.sen_output_rates.vtg, config.sen_output_rates.gga,
        config.sen_output_rates.gsa, config.sen_output_rates.gsv, config.sen_output_rates.mchn) < 0
        || snprintf(tx_buf+strlen(tx_buf), MAX_PACKET_BUF_SIZE, "%02X\r\n", nema_checksum(tx_buf)) < 0) {
        retval = -1;
    }
    else {
        retval = PA6H_config_wrapper("SET OUTPUT", tx_buf, ack_msg, timeout_ms);
    }

    return retval;
}

static int PA6H_query_output(const unsigned timeout_ms) {

    int retval = 0;
    char tx_buf[MAX_PACKET_BUF_SIZE] = "$PMTK414*33\r\n";
    char reply_header[32] = "$PMTK514";
    
    retval = PA6H_config_wrapper("QUERY OUTPUT", tx_buf, reply_header, timeout_ms);

    return retval;
}

static int nema_checksum(const char *nema_packet) {

    int checksum = 0;
    size_t i = 0;

    if (nema_packet[i] == '$') {
        
        while (nema_packet[++i] != '*') {
            /* check if packet possibly corrupt */
            if (i > strlen(nema_packet)-1) {
                checksum = -1;
                break;
            }
            checksum ^= nema_packet[i];
        }
    }
    else {
        checksum = -1;
    }

    return checksum;
}

static float coordinate_strtof(const char *str) {

    /* TODO: add error checking */
    float coordinate;
    char *ptr = strchr(str, '.')-2; // points to start of degree portion
    char previous_value = *ptr;

    coordinate = strtof(ptr, NULL)/60;
    *ptr = '\0';
    coordinate += strtof(str, NULL);
    *ptr = previous_value;

    return coordinate;
}
