#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/io.h>

// Pin 33 on the 40-pin header can be configured for PWM2
// Pin 33 maps to GPIO_PE.06
#define GPIO_BASE                (unsigned long)0x6000d000
#define PE_CNF_OFFSET            (unsigned long)0x100 // bit 6 should be 0 for SPIO mode
#define PE_OE_OFFSET             (unsigned long)0x110 // bit 6 should be 1 for output enabled
#define PE_OUT_OFFSET            (unsigned long)0x120

#define PINMUX_AUX_BASE          (unsigned long)0x70003000
#define PINMUX_PE6_OFFSET        (unsigned long)0x3248 // see page 368 of Tegra X1 manual
#define INPUT_BIT_OFFSET         (unsigned)0x06
#define TRISTATE_BIT_OFFSET      (unsigned)0x04
#define PUPD_BITS_OFFSET         (unsigned)0x02
#define PM_BITS_OFFSET           (unsigned)0x00

#define PWM2_CTL_BASE             (unsigned long)0x7000a020
#define PWM_EN_BIT_OFFSET         (unsigned)0x1F
#define PWM_PULSE_WIDTH_OFFSET    (unsigned)0x10
#define PWM_FREQ_OFFSET           (unsigned)0x00


static void __iomem *reg_ptr;


/**
 *  @brief Configure GPIO PE6 as SPIO for PWM2
 */
void GPIO_PE6_config(void) {

	unsigned reg_val;

	// configure GPIO PE.06 as SPIO (Special Purpose IO)
	reg_ptr = ioremap(GPIO_BASE+PE_CNF_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val &= ~(1U << 6);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	// configure PINMUX PE6 for PWM2
	reg_ptr = ioremap(PINMUX_AUX_BASE+PINMUX_PE6_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val &= ~(  (1U << INPUT_BIT_OFFSET) 
	             | (1U << TRISTATE_BIT_OFFSET)
	             | (3U << PUPD_BITS_OFFSET)
	             | (3U << PM_BITS_OFFSET) );
	// configure pin mode for PWM2
	reg_val |= (2U << PM_BITS_OFFSET);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
}

void PWM2_init(unsigned freq) {

	// PFM is 13 bits
	// PWM is 15 bits
	// EN is 1 bit
	unsigned PFM, PWM, EN;
	unsigned reg_val;
	
	PFM = freq;
	PFM = 0x1000; // freq is not actually used yet.
	PWM = 4U;
	EN = 1U;
	
	reg_val = (EN << PWM_EN_BIT_OFFSET) | (PWM << PWM_PULSE_WIDTH_OFFSET) | (PFM << PWM_FREQ_OFFSET);
	
	reg_ptr = ioremap(PWM2_CTL_BASE, 4);
	iowrite32(reg_val, reg_ptr);
	iounmap(reg_ptr);
}

void module_deinit(void) {

	unsigned reg_val;
	
	reg_val = 0x00000000;
	reg_ptr = ioremap(PWM2_CTL_BASE, 4);
	iowrite32(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	// configure GPIO PE.06 as GPIO
	reg_ptr = ioremap(GPIO_BASE+PE_CNF_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val |= (1U << 6);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
	
	// configure PINMUX PE6 to default values
	reg_ptr = ioremap(PINMUX_AUX_BASE+PINMUX_PE6_OFFSET, 2);
	reg_val = ioread16(reg_ptr);
	reg_val &= ~(3U << PM_BITS_OFFSET);
	reg_val |= (1U << INPUT_BIT_OFFSET) | (1U << TRISTATE_BIT_OFFSET) | (2U << PUPD_BITS_OFFSET);
	iowrite16(reg_val, reg_ptr);
	iounmap(reg_ptr);
}

int nano_pwm_driver_init(void) {

	printk(KERN_INFO "Loading CMPE242 nano pwm kernel module\n");

	GPIO_PE6_config();
	PWM2_init(100);

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

