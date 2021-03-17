#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

// Pin 33 on the 40-pin header can be configured for PWM
// Pin 33 maps to GPIO3_PE.06
#define GPIO3_BASE                (uint32_t*)0x6000d200;
#define GPIO3_E_CNF_OFFSET        (uint32_t)0x100; // bit 6 should be 1 for GPIO mode
#define GPIO3_E_OE_OFFSET         (uint32_t)0x110; // bit 6 should be 1 for output enabled
#define GPIO3_E_OUT_OFFSET        (uint32_t)0x120;
#define GPIO3_E_IN_OFFSET         (uint32_t)0x130;

#define PWM_CTL_BASE              (uint32_t*)0x7000a000;

int CMPE242_nano_init(void) {

	printk(KERN_INFO "Loading CMPE242 kernel module\n");
	return 0;
}

void CMPE242_nano_exit(void) {

	printk(KERN_INFO "Removing CMPE242 kernel module\n");
}

module_init(CMPE242_nano_init);
module_exit(CMPE242_nano_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nano kernel module for CMPE242");
MODULE_AUTHOR("APS");

