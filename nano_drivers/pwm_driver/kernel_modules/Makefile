obj-m = nano_pwm_driver.o
KVERSION = $(shell uname -r)
CROSS_COMPILE_BUILD_DIR = $(HOME)/l4t/Linux_for_Tegra/kernel/linux-headers-4.9.201-tegra-linux_x86_64/kernel-4.9

# used when building on Ubuntu host
all:
	make ARCH=arm64 -C $(CROSS_COMPILE_BUILD_DIR) M=$(PWD) modules

clean:
	make ARCH=arm64 -C $(CROSS_COMPILE_BUILD_DIR) M=$(PWD) clean

# used when building on Jetson Nano
# all:
# 	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
# clean:
# 	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
