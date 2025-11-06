/* state_classifier.h - Klasifikasi state dari sensor */

#ifndef STATE_CLASSIFIER_H
#define STATE_CLASSIFIER_H

#include <stdint.h>

/**
 * @brief Klasifikasi state berdasarkan density dan speed
 */
int8_t classify_state(uint8_t density, uint8_t speed);

/**
 * @brief Get nama state untuk debugging / display
 */
const char* get_state_name(int8_t state);

/**
 * @brief Get nama action untuk debugging / display
 */
const char* get_action_name(uint8_t action);

#endif // STATE_CLASSIFIER_H
