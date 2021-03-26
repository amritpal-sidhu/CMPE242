#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <sys/socket.h>
#include <linux/netlink.h>

#define MAX_PAYLOAD 32  /* maximum payload size */
#define NETLINK_ID 18

void comm_handler(int sock_fd, struct nlmsghdr *nlh, char *payload);


int main()
{
    struct sockaddr_nl src_addr;
    struct nlmsghdr *nlh;
    int sock_fd;
    char input_string[MAX_PAYLOAD];
    int temp_num;
  
    /* Create netlink socket */
    sock_fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_ID);
    if (sock_fd < 0) {
        printf("socket: %s\n", strerror(errno));
        return sock_fd;
    }

	/* set family as netlink and bind socket for userspace */
    memset(&src_addr, 0, sizeof(src_addr));
    src_addr.nl_family = AF_NETLINK;
    src_addr.nl_pid = getpid();  /* self pid */
    src_addr.nl_groups = 0;  /* not in mcast groups */
    bind(sock_fd, (struct sockaddr*)&src_addr, sizeof(src_addr));

	/* allocate memory for the netlink message handler object */
    nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(MAX_PAYLOAD));

    /* Fill the netlink message header */
    nlh->nlmsg_len = NLMSG_SPACE(MAX_PAYLOAD);
    nlh->nlmsg_pid = getpid();  /* self pid */
    nlh->nlmsg_flags = 0;

	printf("Enter 'q' to quit program.\n");
	
	for (;;) {
	
		do {

			printf("Enter your desired duty cycle percent in the range [0.00:100.00]: ");
			fgets(input_string, MAX_PAYLOAD, stdin);
			if (!strcmp(input_string, "q"))
			    break;
			/* I'm using 2 decimal points in fixed point format */
			temp_num = (int)(atof(input_string)*100);
		
		} while (temp_num < 0 || temp_num > 10000);
		
		if (sprintf(input_string, "%ui", temp_num)) {
		    printf("String to int conversion failed for %ui\n", temp_num);
		    close(sock_fd);
        	exit(1);
		}
		comm_handler(sock_fd, nlh, input_string);
		
		do {
		
			/* Note: Datasheet says max frequency is 50kHz, but I'm using 1000 temporarily */
			printf("Enter your desired PWM frequency in the range [0:1000]: ");
			fgets(input_string, MAX_PAYLOAD, stdin);
			if (!strcmp(input_string, "q"))
			    break;
			temp_num = atoi(input_string);
		
		} while (temp_num < 0 || temp_num > 1000);
		
		comm_handler(sock_fd, nlh, input_string);
		
		do {
		
			printf("Enter your desired direction (CW or CCW): ");
			fgets(input_string, MAX_PAYLOAD, stdin);
			if (!strcmp(input_string, "q"))
			    break;
		
		} while (strcmp(input_string, "CW") || strcmp(input_string, "CCW"));
		
		comm_handler(sock_fd, nlh, input_string);
	}

    /* Close Netlink Socket */
    close(sock_fd);

    return 0;
}



/**
 * @brief  Wrapper for transmitting/receiving between
 *         userspace and kernel space.
 * @param  sock_fd: File descriptor of the socket.
 * @param  nlh: Pointer to netlink message handler.
 * @param  payload: C string to send to Kernel space.
 * @retval None
 */
void comm_handler(int sock_fd, struct nlmsghdr *nlh, char *payload) {

	struct sockaddr_nl dest_addr;
	struct iovec iov;
	struct msghdr msg;
	int rc;
	
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
    memset(nlh, 0, NLMSG_SPACE(MAX_PAYLOAD));

	/* get a response from Kernel */
    rc = recvmsg(sock_fd, &msg, 0);
    if (rc < 0) {
        printf("recvmsg(): %s\n", strerror(errno));
        close(sock_fd);
        exit(rc);
    }

    printf("Received from kernel: %s\n", (char *)NLMSG_DATA(nlh));
}

