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

    if (PA6H_jetson_nano_init() == -1) {
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
