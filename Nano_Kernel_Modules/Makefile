obj-m = nano_pwm_driver.o
KVERSION = $(shell uname -r)

# used when building on Ubuntu host
# all:
# 	make ARCH=arm64 -C ~/l4t/Linux_for_Tegra/kernel_build M=$(PWD) modules
# clean:
# 	make ARCH=arm64 -C ~/l4t/Linux_for_Tegra/kernel_build M=$(PWD) clean

# used when building on Jetson Nano
all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
