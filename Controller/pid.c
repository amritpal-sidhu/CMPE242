#include <math.h>

#include "pid.h"

// sqrt(2*pi) rounded to 6 sig figs.
static const float sqrt_2pi = 2.506628f;

float central_diff(float e0, float e2) {

    return (e0 - e2) / 2; // Multiply by some time unit?
}

float forward_diff(float e0, float e1) {

    return (e0 - e1) / 1;
}

float backward_diff(float e1, float e2){

    return (e1 - e2) / 1;
}

/**
 * @note I used the help of the internet to determine
 *       the end result of the LoG is
 *       (x^2 + y^2 - 2*std^2) * exp(-(x^2 +y^2)/2*std^2) / std^4
 * 
 * Reference: http://fourier.eng.hmc.edu/e161/lectures/gradient/node8.html
 */
float derivative_LoG(float e0, float e2, float std) {

    const float x_sq = e0 * e0;
    const float y_sq = e2 * e2;

    const float std_sq = std * std;
    const float std_4 = std_sq * std_sq;

    return sqrt_2pi * (x_sq + y_sq - 2*std_sq) * exp( -((x_sq + y_sq) / 2*std_sq) ) / std_4;
}
