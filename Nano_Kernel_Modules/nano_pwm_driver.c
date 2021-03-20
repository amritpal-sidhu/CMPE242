#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/io.h>

// Pin 33 on the 40-pin header can be configured for PWM2
// Pin 33 maps to GPIO_PE.06
/* GPIO configuration and output addresses and offsets for port E */
#define GPIO_BASE                (unsigned long)0x6000d000
#define PE_CNF_OFFSET            (unsigned long)0x100
#define PE_OE_OFFSET             (unsigned long)0x110
#define PE_OUT_OFFSET            (unsigned long)0x120
#define PIN_6_OFFSET             (unsigned)6

/* Pinmux configuration and control addresses and offsets for PE6 */
#define PINMUX_AUX_BASE          (unsigned long)0x70000000
#define PINMUX_PE6_OFFSET        (unsigned long)0x3248 // see page 368 of Tegra X1 manual
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

/* functions for configuring PWM */
static unsigned get_min(unsigned a, unsigned b) { return a < b ? a : b; }
static unsigned get_max(unsigned a, unsigned b) { return a > b ? a : b; }
static unsigned round_divide(unsigned dividend, unsigned divisor) { 
	return (dividend + (divisor/2)) / divisor;
}


void print_PWM_clock_info(void) {

	void __iomem *reg_ptr;
	unsigned reg_val;

	reg_ptr = ioremap(CLK_RST_BASE+CLK_RST_PWM_SOURCE, 4);
	reg_val = ioread32(reg_ptr);
	// bits [31:29] are CLK_M, bits [7:0] are the divisor
	printk(KERN_INFO "PWM source CLK_M = %x, CLK_DIVISOR = %x\n", reg_val&0xe0000000, reg_val&0x0ff);
	iounmap(reg_ptr);
}

/**
 *  @brief Configure GPIO PE6 as SPIO for PWM2
 */
void GPIO_PE6_config(void) {

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
	printk(KERN_INFO "PE6 pinmux = %x\n", reg_val);
	reg_val &= ~((1U << INPUT_BIT_OFFSET) | (1U << TRISTATE_BIT_OFFSET) | (3U << PUPD_BITS_OFFSET) | (3U << PM_BITS_OFFSET));
	reg_val |= (2U << PM_BITS_OFFSET);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
}

/**
 *
 *  @param desired_freq_Hz: The desired frequency in Hertz
 *  @param duty_cycle_fixed: Fixed point representation of the duty cycle
 *                           with 2 decimal places. (i.e. multiply by 100)
 */
void PWM2_init(unsigned desired_freq_Hz, unsigned duty_cycle_fixed) {

	void __iomem *reg_ptr;
	unsigned reg_val = 0X00000000;
	
	// PWM is set up to use the peripheral pllP_out0 which is fixed at 408MHz
	// PWM also uses a divisor of 0xf
	const unsigned pwm_src_clk_freq = 408000000U / ((0xfU/2)+1);

	// EN is 1 bit
	// PWM is 15 bits
	// PFM is 13 bits
	unsigned EN, PWM, PFM;
	
	EN = 1U;
	// dividing by 10000 to convert percent and fixed point value
	PWM = get_max(get_min(round_divide(duty_cycle_fixed*256, 10000), 256), 0);
	PFM = get_max(get_min(round_divide(round_divide(pwm_src_clk_freq, 256), desired_freq_Hz)-1, 0x1fff), 0x0);
	
	reg_val = (EN << PWM_EN_BIT_OFFSET) | (PWM << PWM_PULSE_WIDTH_OFFSET) | (PFM << PWM_FREQ_OFFSET);
	
	reg_ptr = ioremap(PWM_CSR2_BASE, 4);
	iowrite32(reg_val, reg_ptr);
	iounmap(reg_ptr);
}

void module_deinit(void) {

	void __iomem *reg_ptr;
	unsigned reg_val;
	
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
}

int nano_pwm_driver_init(void) {

	printk(KERN_INFO "Loading CMPE242 nano pwm kernel module\n");

	GPIO_PE6_config();
	PWM2_init(75, 5000); // duty cycle needs to be multipled by 100 for fixed point

	return 0;
}

void nano_pwm_driver_exit(void) {

	printk(KERN_INFO "Removing CMPE242 nano pwm kernel module\n");
	module_deinit();
}

module_init(nano_pwm_driver_init);
module_exit(nano_pwm_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nano PWM driver for CMPE242");
MODULE_AUTHOR("APS");

