#include "pid.h"

// sqrt(2*pi) rounded to 6 sig figs.
static float sqrt_2pi = 2.506628f;

void construct_kernel_central(void) {

    for (unsigned i = 0; i < buffer_size-2; ++i) {
        kernel[i] = (history_buffer[i] - history_buffer[i+2]) / 2;
        // TODO: scale 2 time units as needed.
    }

    // TODO: Determine what to do with these.
    //       Possibly increase history_buffer size,
    //       or decrease kernel size.
    kernel[buffer_size-3] = 0;
    kernel[buffer_size-1] = 0;
}

void construct_kernel_forward(void) {

    for (unsigned i = 0; i < buffer_size-1; ++i) {
        kernel[i] = (history_buffer[i] - history_buffer[i+1]) / 1;
    }

    kernel[buffer_size-1] = 0;
}

void construct_kernel_backward(void) {

    for (unsigned i = 0; i < buffer_size-1; ++i) {
        kernel[i] = (history_buffer[i+1] - history_buffer[i]) / 1;
    }

    kernel[buffer_size-1] = 0;
}

/**
 * @note I used the help of the internet to determine
 *       the end result of the LoG is
 *       (x^2 + y^2 - 2*std^2) * exp(-(x^2 +y^2)/2*std^2) / std^4
 */
void construct_kernel_LoG(float std) {

    const float std_sq = std * std;
    const float std_4 = std_sq * std_sq;

    // using central difference since symmetry is nice.
    // TODO: figure out why symmetry is nice.
    for (unsigned i = 0; i < buffer_size-2; ++i) {
        // not sure if this is correct
        const float x_sq = history_buffer[i] * history_buffer[i];
        const float y_sq = history_buffer[i+2] * history_buffer[i+2];
        
        kernel[i] = (x_sq + y_sq - 2*std_sq) * exp((y_sq - x_sq) / 2*std_sq) / std_4;
    }

    kernel[buffer_size-3] = 0;
    kernel[buffer_size-1] = 0;
}
