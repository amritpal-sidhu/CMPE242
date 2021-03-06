/**
 * Executing this program on the Jetson Nano
 * requires superuser privilege.
 * 
 */
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

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
    PAH6_gps_coordinate coord;

    gps_config.baud_rate = PA6H_BAUD_9600;
    gps_config.update_rate = 1000;
    gps_config.dgps_mode = WAAS_DGPS;
    gps_config.sen_output_rates.gll = DISABLE_OUTPUT;
    gps_config.sen_output_rates.rmc = DISABLE_OUTPUT;
    gps_config.sen_output_rates.vtg = DISABLE_OUTPUT;
    gps_config.sen_output_rates.gga = ONCE_EVERY_FIX;
    gps_config.sen_output_rates.gsa = DISABLE_OUTPUT;
    gps_config.sen_output_rates.gsv = DISABLE_OUTPUT;
    gps_config.sen_output_rates.mchn = DISABLE_OUTPUT;
    gps_config.nav_speed_threshold = PA6H_NAV_SPEED_THRESH_DISABLE;

    if (PA6H_jetson_nano_init(gps_config) == -1) {
        printf("Initializing serial port failed\n");
        return -1;
    }

    signal(SIGINT, stop_handler);

    while (!stop_signal) {

        if (PA6H_read_GP_sentence(gps_data_buf, sizeof(gps_data_buf)) != -1) {
            printf("%s\n", gps_data_buf);
            if (PA6H_parse_coordinate(gps_data_buf, &coord) != -1) {
                printf("\tlat, long = %f, %f\n", coord.latitude, coord.longitude);
            }
        }
        sleep(1);
    }

    printf("\nClosing serial port and terminating program\n");
    PA6H_jetson_nano_deinit();

    return 0;
}
