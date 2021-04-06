#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/io.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>

// Pin 33 on the 40-pin header can be configured for PWM2
// Pin 33 maps to GPIO_PE.06
/* GPIO configuration and output addresses and offsets for port E */
#define GPIO_BASE                (unsigned long)0x6000d000
#define PE_CNF_OFFSET            (unsigned long)0x100
#define PE_OE_OFFSET             (unsigned long)0x110
#define PE_OUT_OFFSET            (unsigned long)0x120
#define PIN_6_OFFSET             (unsigned)6
// Pin 31 GPIO_PZ.00 is for controlling stepper direction
#define PZ_CNF_OFFSET            (unsigned long)0x604
#define PZ_OE_OFFSET             (unsigned long)0x614
#define PZ_OUT_OFFSET            (unsigned long)0x624
#define PIN_0_OFFSET             (unsigned)0

/* Pinmux configuration and control addresses and offsets for PE6 */
#define PINMUX_AUX_BASE          (unsigned long)0x70000000
#define PINMUX_PE6_OFFSET        (unsigned long)0x3248 // see page 368 of Tegra X1 manual
#define PINMUX_PZ0_OFFSET        (unsigned long)0x327c
#define INPUT_BIT_OFFSET         (unsigned)6
#define TRISTATE_BIT_OFFSET      (unsigned)4
#define PUPD_BITS_OFFSET         (unsigned)2
#define PM_BITS_OFFSET           (unsigned)0

/* For verifying PWM clock source */
#define CLK_RST_BASE             (unsigned long)0x60006000
#define CLK_RST_DEVICES_L_0      (unsigned long)0x4
#define CLK_RST_OUT_ENB_L_0      (unsigned long)0x10
#define CLK_RST_PWM_SOURCE       (unsigned long)0x110

/* PWM configuration and control addresses and offsets */
#define PWM_CSR2_BASE            (unsigned long)0x7000a020
#define PWM_EN_BIT_OFFSET        (unsigned)31
#define PWM_PULSE_WIDTH_OFFSET   (unsigned)16
#define PWM_FREQ_OFFSET          (unsigned)0

/* netlink id */
#define NETLINK_ID  18


enum pwm_state_e {
    PWM_ENABLE,
    PWM_DISABLE
};

enum {
	FRQ,
	DUT,
	DIR,
	CMD_STRING
} msg_type;


/* For kernel space to-from user space comm */
static struct sock *nl_sock = NULL;

/**
 * Local globals needed to communicate between callback and
 * this module's scope.
 *
 * Default values are also assigned.
 * Note: 20Hz seems to be the minimum frequency
 */
static unsigned pwm_freq_Hz = 20;
static unsigned duty_cycle_percent = 50;

/* functions for configuring PWM */
static unsigned get_min(unsigned a, unsigned b) { return a < b ? a : b; }
static unsigned get_max(unsigned a, unsigned b) { return a > b ? a : b; }
static unsigned round_divide(unsigned dividend, unsigned divisor) { 
	return (dividend + (divisor/2)) / divisor;
}

static void netlink_rx_msg(struct sk_buff *skb);

static void print_PWM_clock_info(void);
static void GPIO_init(void);
static void PWM2_config(enum pwm_state_e state, unsigned desired_freq_Hz, unsigned duty_cycle_fixed);

static void update_message_state_machine(char *msg);


/**
 * @note On the NEMA 17 stepper motor
 *       
 *    Step angle = 1.8 degrees
 *    Thus, 200 steps in a revolution
 *    To convert to RPM one could do the following:
 *    RPM = freq(steps/sec) * 60(sec/min) * (1/200)(rev/step)
 *    RPM = 0.3 * freq
 */

int nano_pwm_driver_init(void) {

	struct netlink_kernel_cfg nl_cfg;

	printk(KERN_INFO "nano_pwm_driver: Loading CMPE242 nano pwm kernel module\n");

	GPIO_init();
	PWM2_config(PWM_DISABLE, pwm_freq_Hz, duty_cycle_percent);
	
	// start up state
	msg_type = CMD_STRING;
	
	nl_cfg.input = netlink_rx_msg;
	
	nl_sock = netlink_kernel_create(&init_net, NETLINK_ID, &nl_cfg);
	if (!nl_sock) {
		printk(KERN_ALERT "nano_pwm_driver: Failed to create netlink from nano_pwm_driver\n");
		return -1;
	}

	return 0;
}

void nano_pwm_driver_exit(void) {

	void __iomem *reg_ptr;
	unsigned reg_val;

	printk(KERN_INFO "nano_pwm_driver: Clearing PWM2 registers and setting PE6 to default\n");	
	
	reg_val = 0x00000000;
	reg_ptr = ioremap(PWM_CSR2_BASE, 4);
	iowrite32(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	// configure PINMUX PE6 to default values
	reg_ptr = ioremap(PINMUX_AUX_BASE+PINMUX_PE6_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val &= ~(3U << PM_BITS_OFFSET);
	reg_val |= ((1U << INPUT_BIT_OFFSET) | (1U << TRISTATE_BIT_OFFSET) | (1U << PUPD_BITS_OFFSET));
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	// configure GPIO PE.06 as GPIO
	reg_ptr = ioremap(GPIO_BASE+PE_CNF_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val |= (1U << PIN_6_OFFSET);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);

	netlink_kernel_release(nl_sock);

	printk(KERN_INFO "nano_pwm_driver: Removing CMPE242 nano pwm kernel module\n");
}


static void netlink_rx_msg(struct sk_buff *skb) {
	struct sk_buff *skb_out;
    struct nlmsghdr *nlh;
    int msg_size;
    char *msg;
    int pid;
    int res;

    nlh = (struct nlmsghdr *)skb->data;
    pid = nlh->nlmsg_pid; /* pid of sending process */
    msg = (char *)nlmsg_data(nlh);
    msg_size = strlen(msg);

    printk(KERN_INFO "nano_pwm_driver: Netlink received from pid %d: %s\n", pid, msg);

	update_message_state_machine(msg);

    // create reply
    skb_out = nlmsg_new(msg_size, 0);
    if (!skb_out) {
      printk(KERN_ERR "nano_pwm_driver: Netlink failed to allocate new skb\n");
      return;
    }

    // put received message into reply
    nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, msg_size, 0);
    NETLINK_CB(skb_out).dst_group = 0; /* not in mcast group */
    strncpy(nlmsg_data(nlh), msg, msg_size);

    printk(KERN_INFO "nano_pwm_driver: Netlink attempting to send %s\n", msg);

    res = nlmsg_unicast(nl_sock, skb_out, pid);
    if (res < 0)
      printk(KERN_INFO "nano_pwm_driver: Netlink error while sending skb to user\n");
}

static void print_PWM_clock_info(void) {

	void __iomem *reg_ptr;
	unsigned reg_val;

	reg_ptr = ioremap(CLK_RST_BASE+CLK_RST_PWM_SOURCE, 4);
	reg_val = ioread32(reg_ptr);
	// bits [31:29] are CLK_M, bits [7:0] are the divisor
	printk(KERN_INFO "nano_pwm_driver: PWM source CLK_M = %x, CLK_DIVISOR = %x\n", reg_val&0xe0000000, reg_val&0x0ff);
	iounmap(reg_ptr);
}

/**
 *  @brief Configure GPIO PE6 as SPIO for PWM2
 */
static void GPIO_init(void) {

	void __iomem *reg_ptr;
	unsigned reg_val;

	// configure GPIO PE.06 as SPIO (Special Purpose IO)
	reg_ptr = ioremap(GPIO_BASE+PE_CNF_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val &= ~(1U << PIN_6_OFFSET);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	// configure PINMUX PE6 for PWM2
	reg_ptr = ioremap(PINMUX_AUX_BASE+PINMUX_PE6_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val &= ~((1U << INPUT_BIT_OFFSET) | (1U << TRISTATE_BIT_OFFSET) | (3U << PUPD_BITS_OFFSET) | (3U << PM_BITS_OFFSET));
	reg_val |= (2U << PM_BITS_OFFSET);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	// configure GPIO PZ.00
	reg_ptr = ioremap(GPIO_BASE+PZ_CNF_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val |= (1U << PIN_0_OFFSET);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	reg_ptr = ioremap(GPIO_BASE+PZ_OE_OFFSET, 1);
	reg_val = ioread8(reg_ptr);
	reg_val |= (1U << PIN_0_OFFSET);
	iowrite8(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	reg_ptr = ioremap(PINMUX_AUX_BASE+PINMUX_PZ0_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val &= ~((1U << INPUT_BIT_OFFSET) | (1U << TRISTATE_BIT_OFFSET) | (3U << PUPD_BITS_OFFSET));
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);

	// configure stepper to move CCW by default
	reg_ptr = ioremap(GPIO_BASE+PZ_OUT_OFFSET, 1);
	reg_val = ioread8(reg_ptr);
	reg_val |= (1U << PIN_0_OFFSET);
	iowrite8(reg_val, reg_ptr);
	iounmap(reg_ptr);
}

/**
 *  @param state: 1 PWM enable, 0 PWM disable
 *  @param desired_freq_Hz: The desired frequency in Hertz
 *  @param duty_cycle_fixed: Fixed point representation of the duty cycle
 *                           with 2 decimal places. (i.e. multiply by 100)
 */
static void PWM2_config(enum pwm_state_e state, unsigned desired_freq_Hz, unsigned duty_cycle_fixed) {

	// PWM is set up to use the peripheral pllP_out0 which is fixed at 408MHz
	// PWM also uses a divisor of 0xf
	const unsigned pwm_src_clk_freq = 408000000U / ((0xfU/2)+1);

	void __iomem *reg_ptr;
	unsigned reg_val = 0X00000000;
	
	// EN is 1 bit
	// PWM is 15 bits
	// PFM is 13 bits
	unsigned EN, PWM, PFM;


	if (state == PWM_DISABLE) {
		reg_ptr = ioremap(PWM_CSR2_BASE, 4);
		iowrite32(reg_val, reg_ptr);
		iounmap(reg_ptr);
		return;
	}
	
	EN = 1U;
	// dividing by 10000 to convert percent and fixed point value
	PWM = get_max(get_min(round_divide(duty_cycle_fixed*256, 10000), 256), 0);
	PFM = get_max(get_min(round_divide(round_divide(pwm_src_clk_freq, 256), desired_freq_Hz)-1, 0x1fff), 0x0);
	
	reg_val = (EN << PWM_EN_BIT_OFFSET) | (PWM << PWM_PULSE_WIDTH_OFFSET) | (PFM << PWM_FREQ_OFFSET);
	
	reg_ptr = ioremap(PWM_CSR2_BASE, 4);
	iowrite32(reg_val, reg_ptr);
	iounmap(reg_ptr);
}

static void update_message_state_machine(char *msg) {

	void __iomem *reg_ptr;
	unsigned reg_val;

	if (!strcmp(msg, "sync")) {
		msg_type = CMD_STRING;
		return;
	}
	
	if (msg_type == CMD_STRING) {
		if (!strcmp(msg, "frq")) {
			msg_type = FRQ;
		}
		else if (!strcmp(msg, "dut")) {
			msg_type = DUT;
		}
		else if (!strcmp(msg, "dir")) {
			msg_type = DIR;
		}
		else {
			printk(KERN_ALERT "nano_pwm_driver: Unexpected message string when msg_type = CMD_STRING\n");
		}
	}
	else if (msg_type == FRQ) {
		if (kstrtouint(msg, 10, &pwm_freq_Hz)) {
			printk(KERN_ALERT "nano_pwm_dirver: kstrtouint() failed\n");
		}
		if (pwm_freq_Hz > 20) {
		    PWM2_config(PWM_ENABLE, pwm_freq_Hz, duty_cycle_percent);
		}
		else {
		    PWM2_config(PWM_DISABLE, pwm_freq_Hz, duty_cycle_percent);
		}
		printk(KERN_INFO "nano_pwm_driver: Set PWM2 frequency to = %u\n", pwm_freq_Hz);
		msg_type = CMD_STRING;
	}
	else if (msg_type == DUT) {
		if (kstrtouint(msg, 10, &duty_cycle_percent)) {
			printk(KERN_ALERT "nano_pwm_dirver: kstrtouint() failed\n");
		}
		if (pwm_freq_Hz > 20) {
		    PWM2_config(PWM_ENABLE, pwm_freq_Hz, duty_cycle_percent);
		}
		else {
		    PWM2_config(PWM_DISABLE, pwm_freq_Hz, duty_cycle_percent);
		}
		PWM2_config(PWM_ENABLE, pwm_freq_Hz, duty_cycle_percent);
		printk(KERN_INFO "nano_pwm_driver: Set PWM2 duty cycle to = %u\n", duty_cycle_percent);
		msg_type = CMD_STRING;
	}
	else if (msg_type == DIR) {
		reg_ptr = ioremap(GPIO_BASE+PZ_OUT_OFFSET, 1);
		reg_val = ioread8(reg_ptr);
		if (!strcmp(msg, "CCW")) {
			reg_val |= (1U << PIN_0_OFFSET);
		}
		else {
			reg_val &= ~(1U << PIN_0_OFFSET);
		}
		iowrite8(reg_val, reg_ptr);
		iounmap(reg_ptr);
		printk(KERN_INFO "nano_pwm_driver: Set GPIO_PZ.00 so stepper rotates %s\n", msg);
		msg_type = CMD_STRING;
	}
	else {
		printk(KERN_ALERT "nano_pwm_driver: msg_type enum incorrect\n");
	}
}

module_init(nano_pwm_driver_init);
module_exit(nano_pwm_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nano PWM driver for CMPE242");
MODULE_AUTHOR("APS");

