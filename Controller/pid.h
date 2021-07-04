#pragma once

/**
 * @brief  Use the centeral difference to compute
 *         the derivative.
 * @param  e0: most recent sample
 * @param  e2: sample from 2 sample periods ago
 * @retval Value of the derivative.
 */
float central_diff(float e0, float e2);

/**
 * @brief  Use the centeral difference to compute
 *         the derivative.
 * @param  e0: most recent sample
 * @param  e1: sample from 1 sample period ago
 * @retval Value of the derivative.
 */
float forward_diff(float e0, float e1);

/**
 * @brief  Use the centeral difference to compute
 *         the derivative.
 * @param  e1: sample from 1 sample period ago
 * @param  e2: sample from 2 sample periods ago
 * @retval Value of the derivative.
 */
float backward_diff(float e1, float e2);

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
