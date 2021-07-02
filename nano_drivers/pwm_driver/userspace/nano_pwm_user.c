#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <sys/socket.h>
#include <linux/netlink.h>

#define MAX_STRING_SIZE 48
#define NETLINK_ID 18

// parsing wrappers
int parse_number(char *str_start_ptr, int is_float);
void parse_sub_string(char *str_start_ptr, char *result_str);

// netlink wrapper
void netlink_send_string(char *payload);


int main() {

	/* Note: Datasheet says max frequency is 50kHz, but I'm using 10000 temporarily */
	const int max_freq = 10000;
    char str_buf[MAX_STRING_SIZE];
    char *str_ptr;
    int temp_frq, temp_dut, something_sent;
	char temp_dir[16];
    
	netlink_send_string("sync");
	
	printf("Enter your message in the following format: \"FRQ=x, DUT=y, DIR=z\"\n");
	printf("FRQ is your desired PWM frequency in the range [0, %u]\n", max_freq);
	printf("DUT is your desired duty cycle percent in the range [0.00, 100.00]\n");
	printf("DIR is your desired direction (CW or CCW)\n");
	printf("You may omit values you do not wish to change, and parameter names are not case sensitive\n");
	printf("Enter 'q' to quit program.\n");

	for (;;) {
	
		printf("Enter your command: ");
		fgets(str_buf, MAX_STRING_SIZE, stdin);
		if (!strcmp(str_buf, "q\n")) {
			break;
		}

		temp_frq = -1;
		temp_dut = -1;
		strcpy(temp_dir, "");
		something_sent = 0;

		if ((str_ptr=strstr(str_buf, "FRQ")) || (str_ptr=strstr(str_buf, "frq"))) {
			
			temp_frq = parse_number(str_ptr, 0);
			if (temp_frq < 0 || temp_frq > max_freq) {
				printf("Parsed frequency value out of range\n");
				continue;
			}
		}

		if ((str_ptr=strstr(str_buf, "DUT")) || (str_ptr=strstr(str_buf, "dut"))) {
			
			temp_dut = parse_number(str_ptr, 1);
			if (temp_dut < 0 || temp_dut > 10000) {
				printf("Parsed duty cycle value out of range\n");
				continue;
			}
		}

		if ((str_ptr=strstr(str_buf, "DIR")) || (str_ptr=strstr(str_buf, "dir"))) {
			
			parse_sub_string(str_ptr, temp_dir);
			if (strcmp(temp_dir, "CW") && strcmp(temp_dir, "CCW")) {
				printf("Parsed direction string is invalid\n");
				continue;
			}
		}

		// send payloads if available
		if (temp_frq > 0) {
			netlink_send_string("frq");
			sprintf(str_buf, "%i", temp_frq);
			netlink_send_string(str_buf);
			++something_sent;
		}

		if (temp_dut > 0) {
			netlink_send_string("dut");
			sprintf(str_buf, "%i", temp_dut);
			netlink_send_string(str_buf);
			++something_sent;
		}

		if (strlen(temp_dir) > 1) {
			netlink_send_string("dir");
			netlink_send_string(temp_dir);
			++something_sent;
		}

		if (!something_sent) {
			printf("Input string did not contain valid fields\n");
		}
	}
}



/**
 * @brief  Simple string parsing wrapper to take care of
 *         extracting number value once start of 'frq' or
 *         'dut' field is found.
 * @param  str_start_ptr: Start of 'frq' or 'dut' field,
 *                        Including the field name.
 * @param  is_float: 0 if parsing 'frq', 1 if parsing 'dut'
 * @retval Parsed integer value
 */
int parse_number(char *str_start_ptr, int is_float) {

	int retval = -1;
	size_t sub_str_size;
	char *str_ptr;
	char result_string[16];

	// jump over 3 character field
	str_start_ptr += 3;
	// ignore empty space
	while (*str_start_ptr == '=' || *str_start_ptr == ' ')
		++str_start_ptr;

	if ((str_ptr=strchr(str_start_ptr, ',')) ||
	    (str_ptr=strchr(str_start_ptr, ' ')) ||
		(str_ptr=strchr(str_start_ptr, '\n'))) {

			sub_str_size = strlen(str_start_ptr) - strlen(str_ptr);
			strncpy(result_string, str_start_ptr, sub_str_size);
			result_string[sub_str_size] = '\0'; // strncpy() doesn't do this for you.
			if (is_float) {
				retval = (int)(atof(result_string)*100);
			}
			else {
				retval = atoi(result_string);
			}
	}

	return retval;
}

/**
 * @brief  Simple string parsing wrapper to take care of
 *         extracting string from 'dir' field.
 * @param  str_start_ptr: Start of 'dir'field,
 *                        Including the field name.
 * @param  result_str: Reference of where to put result.
 * @retval None
 */
void parse_sub_string(char *str_start_ptr, char *result_str) {

	size_t sub_str_size;
	char *str_ptr;

	// jump over 3 character field
	str_start_ptr += 3;
	// ignore empty space
	while (*str_start_ptr == '=' || *str_start_ptr == ' ')
		++str_start_ptr;

	if ((str_ptr=strchr(str_start_ptr, ',')) ||
	    (str_ptr=strchr(str_start_ptr, ' ')) ||
		(str_ptr=strchr(str_start_ptr, '\n'))) {

			sub_str_size = strlen(str_start_ptr) - strlen(str_ptr);
			strncpy(result_str, str_start_ptr, sub_str_size);
			result_str[sub_str_size] = '\0'; // strncpy() doesn't do this for you.
	}
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

