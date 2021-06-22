/**
 * This is a driver for the PA6H GPS module for use
 * with the Jetson Nano developer board.  This driver
 * assumes you have linux installed on the Jetson nano
 * as it does not directly write to the Tegra SoC's
 * registers.
 * 
 * UART2 is used and the pin numbers on the pin header are:
 *   Tx - Pin number 8
 *   Rx - Pin number 10
 * 
 * @date 2021-06-13
 * @author Amritpal Sidhu
 */
#pragma once

#define JETSON_NANO_LINUX_UART  "/dev/ttyTHS1"

int PA6H_jetson_nano_init(void);
void PA6H_jetson_nano_deinit(void);

int PA6H_read(char *buf, const int buf_size);
int PA6H_write(const char *buf, const int buf_size);
