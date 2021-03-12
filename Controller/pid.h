#pragma once

// TODO: set up interrupt to sample sensor
// TODO: copy queue code from 243 and just use that as the history_buffer
// const unsigned buffer_size = 6;
// float history_buffer[buffer_size] = {0}; // the 0th entry is the current sample

/**
 * @brief  Use the centeral difference to compute
 *         the derivative.
 * @param  e0: most recent sample
 * @param  e2: sample from 2 sample periods ago
 * @retval Value of the derivative.
 */
float derivative_c(float e0, float e2);

/**
 * @brief  Use the centeral difference to compute
 *         the derivative.
 * @param  e0: most recent sample
 * @param  e1: sample from 1 sample period ago
 * @retval Value of the derivative.
 */
float derivative_f(float e0, float e1);

/**
 * @brief  Use the centeral difference to compute
 *         the derivative.
 * @param  e1: sample from 1 sample period ago
 * @param  e2: sample from 2 sample periods ago
 * @retval Value of the derivative.
 */
float derivative_b(float e1, float e2);

/**
 * @brief  Use the centeral difference to compute
 *         the derivative.
 * @param  e0: most recent sample
 * @param  e2: sample from 2 sample periods ago
 * @param  std: standard deviation
 * @retval Value of the derivative.
 */
float derivative_LoG(float e0, float e2, float std);

// TODO: use the above in convolution
