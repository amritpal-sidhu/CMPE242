#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>

#include <sys/socket.h>
#include <linux/netlink.h>

#include "ads1015.h"

#define MAX_STRING_SIZE 32
#define NETLINK_ID 18

sig_atomic_t stop_signal = 0;


// netlink wrapper
int adc_value_to_pwm_control(int16_t adc_value);
void netlink_send_string(char *payload);

void stop_handler(int param) {
    if (param) {
        /* ignore param */
    }
    stop_signal = 1;
}


int main() {

    const int duty_cycle = 50;
    char str_buf[MAX_STRING_SIZE];

    ADS1015_adc_config adc_config = {
        .mode = ADS1015_CONTINUOUS,
        .mux_config = ADS1015_MUX_AIN0_SINGLE,
        .data_rate = ADS1015_RATE_920SPS,
        .pga_fsr = ADS1015_PGA_4_096
    };
    
    if (ADS1015_jetson_nano_i2c_init() == -1) {
        printf("I2C initialization failed, exiting program\n");
        return -1;
    }

    ADS1015_config_adc(adc_config);

	netlink_send_string("sync");
    netlink_send_string("dut");
    sprintf(str_buf, "%i", duty_cycle);
    netlink_send_string(str_buf);
	
    signal(SIGINT, stop_handler);

	while (!stop_signal) {

        netlink_send_string("frq");
		sprintf(str_buf, "%i", adc_value_to_pwm_control(ADS1015_get_data()));
        netlink_send_string(str_buf);
	}

    ADS1015_jetson_nano_i2c_deinit();

    return 0;
}



int adc_value_to_pwm_control(int16_t adc_value) {
    /**
     * Based on ADC characterization I'm creating these
     * 10 discrete points.
     * 
     * Sometimes my potentiometer will change to Vdd, which
     * is why I have the OR condition
     */
    int freq_value;

    if (adc_value < 128 || adc_value > 3500) {
        freq_value = 0;
    }
    else if (adc_value < 255) {
        freq_value = 100;
    }
    else if (adc_value < 386) {
        freq_value = 200;
    }
    else if (adc_value < 510) {
        freq_value = 300;
    }
    else if (adc_value < 638) {
        freq_value = 400;
    }
    else if (adc_value < 765) {
        freq_value = 500;
    }
    else if (adc_value < 893) {
        freq_value = 600;
    }
    else if (adc_value < 1020) {
        freq_value = 700;
    }
    else if (adc_value < 1148) {
        freq_value = 800;
    }
    else if (adc_value < 1275) {
        freq_value = 900;
    }
    else {
        freq_value = 1000;
    }

    return freq_value;
}

/**
 * @brief  Wrapper for transmitting/receiving between
 *         userspace and kernel space.
 * @param  sock_fd: File descriptor of the socket.
 * @param  nlh: Pointer to netlink message handler.
 * @param  payload: C string to send to Kernel space.
 * @retval None
 */
void netlink_send_string(char *payload) {

	struct sockaddr_nl src_addr;
	struct sockaddr_nl dest_addr;
    struct nlmsghdr *nlh;
	struct iovec iov;
	struct msghdr msg;
	int sock_fd;
	int rc;
	
	/* Create netlink socket */
    sock_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ID);
    if (sock_fd < 0) {
        printf("socket: %s\n", strerror(errno));
        exit(sock_fd);
    }

	/* set family as netlink and bind socket for userspace */
    memset(&src_addr, 0, sizeof(src_addr));
    src_addr.nl_family = AF_NETLINK;
    src_addr.nl_pid = getpid();  /* self pid */
    src_addr.nl_groups = 0;  /* not in mcast groups */
    bind(sock_fd, (struct sockaddr*)&src_addr, sizeof(src_addr));

	/* allocate memory for the netlink message handler object */
    nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(MAX_STRING_SIZE));

    /* Fill the netlink message header */
    nlh->nlmsg_len = NLMSG_SPACE(MAX_STRING_SIZE);
    nlh->nlmsg_pid = getpid();  /* self pid */
    nlh->nlmsg_flags = 0;
	
	/* create kernel sockaddr_nl object */
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.nl_family = AF_NETLINK;
    dest_addr.nl_pid = 0;   /* For Linux Kernel */
    dest_addr.nl_groups = 0; /* unicast */

    /* Fill in the netlink message payload */
    strcpy(NLMSG_DATA(nlh), payload);

	/* Populate the I/O vector object */
    memset(&iov, 0, sizeof(iov));
    iov.iov_base = (void *)nlh;
    iov.iov_len = nlh->nlmsg_len;

	/* Populate message handler object */
    memset(&msg, 0, sizeof(msg));
    msg.msg_name = (void *)&dest_addr;
    msg.msg_namelen = sizeof(dest_addr);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;

    printf("Attempting to send to kernel: %s\n", payload);

	/* send payload */
    rc = sendmsg(sock_fd, &msg, 0);
    if (rc < 0) {
        printf("sendmsg(): %s\n", strerror(errno));
        close(sock_fd);
        exit(rc);
    }

    /* Read message from kernel */
    memset(nlh, 0, NLMSG_SPACE(MAX_STRING_SIZE));

	/* get a response from Kernel */
    rc = recvmsg(sock_fd, &msg, 0);
    if (rc < 0) {
        printf("recvmsg(): %s\n", strerror(errno));
        close(sock_fd);
        exit(rc);
    }

    printf("Received from kernel: %s\n", (char *)NLMSG_DATA(nlh));
    
    close(sock_fd);
}
