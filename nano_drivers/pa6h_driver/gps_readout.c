/**
 * Executing this program on the Jetson Nano
 * requires superuser privilege.
 * 
 */
#include <stdio.h>
#include <signal.h>

#include "pa6h.h"

sig_atomic_t stop_signal = 0;

void stop_handler(int param) {
    if (param) {
        /* ignore param */
    }
    stop_signal = 1;
}

int main(void) {

    char gps_data_buf[200];
    PAH6_config gps_config;

    gps_config.baud_rate = PA6H_BAUD_115200;
    gps_config.update_rate = 1000;
    gps_config.dgps_mode = WAAS_DGPS;
    gps_config.gll_output = DISABLE_OUTPUT;
    gps_config.rmc_output = DISABLE_OUTPUT;
    gps_config.vtg_output = DISABLE_OUTPUT;
    gps_config.gga_output = ONCE_EVERY_FIX;
    gps_config.gsa_output = DISABLE_OUTPUT;
    gps_config.gsv_output = DISABLE_OUTPUT;
    gps_config.mchn_output = DISABLE_OUTPUT;
    gps_config.nav_speed_threshold = PA6H_NAV_SPEED_THRESH_DISABLE;

    if (PA6H_jetson_nano_init(gps_config) == -1) {
        printf("Initializing serial port failed\n");
        return -1;
    }

    signal(SIGINT, stop_handler);

    while (!stop_signal) {

        PA6H_read(gps_data_buf, sizeof(gps_data_buf));
        printf("\r%s", gps_data_buf);
        fflush(stdout);
    }

    printf("\nClosing serial port and terminating program\n");
    PA6H_jetson_nano_deinit();

    return 0;
}
