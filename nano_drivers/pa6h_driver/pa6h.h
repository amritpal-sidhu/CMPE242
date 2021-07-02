/**
 * This is a driver for the PA6H GPS module for use
 * with the Jetson Nano developer board.  This driver
 * assumes you have linux installed on the Jetson nano
 * as it does not directly write to the Tegra SoC's
 * registers.
 * 
 * @date Created: 2021-06-13
 *       Modified: 2021-07-01
 * @author Amritpal Sidhu
 */
#pragma once

typedef enum {
    NO_DGPS,
    RTCM_DGPS,
    WAAS_DGPS
} PA6H_dgps_mode;

typedef enum {
    DISABLE_OUTPUT,
    ONCE_EVERY_FIX,
    ONCE_EVERY_TWO_FIXES,
    ONCE_EVERY_THREE_FIXES,
    ONCE_EVERY_FOUR_FIXES,
    ONCE_EVERY_FIVE_FIXES
} PA6H_output_freq;

typedef struct {
    unsigned baud_rate;             /* See accepted baud rates below */
    unsigned update_rate;           /* valid range is [100-10000] in ms */
    PA6H_dgps_mode dgps_mode;
    PA6H_output_freq gll_output;
    PA6H_output_freq rmc_output;
    PA6H_output_freq vtg_output;
    PA6H_output_freq gga_output;
    PA6H_output_freq gsa_output;
    PA6H_output_freq gsv_output;
    PA6H_output_freq mchn_output;
    unsigned nav_speed_threshold;   /* See below for nav speed thresholds */
} PAH6_config;

/**
 * Jetson nano UART2 on J41 pin header:
 *   UART_2_TX - pin 8  - P
 *   UART_2_RX - pin 10 - P
 * This program is using the linux device drivers to interface
 * with the PA6H
 * 
 * As a reminder, for UART Jetson Nano TX/RX pins should
 * connect to the PA6H RX/TX pins respectively.
 */
#define JETSON_NANO_LINUX_UART  "/dev/ttyTHS1"

/**
 * Accepted PA6H baud rates
 */
#define PA6H_BAUD_4800                  4800
#define PA6H_BAUD_9600                  9600
#define PA6H_BAUD_19200                 19200
#define PA6H_BAUD_38400                 38400
#define PA6H_BAUD_57600                 57600
#define PA6H_BAUD_115200                115200

/**
 * NAV Speed Thresholds
 */
#define PA6H_NAV_SPEED_THRESH_DISABLE   0
#define PA6H_NAV_SPEED_THRESH_0_2       0.2f
#define PA6H_NAV_SPEED_THRESH_0_4       0.4f
#define PA6H_NAV_SPEED_THRESH_0_6       0.6f
#define PA6H_NAV_SPEED_THRESH_0_8       0.8f
#define PA6H_NAV_SPEED_THRESH_1_0       1.0f
#define PA6H_NAV_SPEED_THRESH_1_5       1.5f
#define PA6H_NAV_SPEED_THRESH_2_0       2.0f


int PA6H_jetson_nano_init(const PAH6_config config_data);
void PA6H_jetson_nano_deinit(void);

int PA6H_read(char *buf, const int buf_size);
int PA6H_write(const char *buf, const int buf_size);
