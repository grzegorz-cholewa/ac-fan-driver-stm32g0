// Simple logger for STM32 that uses STM's USB implementation to log data through USB CDC

// 1. USB must be initialized outside of this module, before logging anything
// 2. Default log level is LEVEL_INFO
// 3. Logger messages can be formatted the same way like printf arguments
// 4. This header must be included to any file using logger

// Sample use:
// logger_set_level(LEVEL_ERROR); // set log level first
// log_usb(LEVEL_ERROR, "Error code: %d\n\r", err_code);

#include "stm32g0xx_hal.h"

enum log_level_type {LEVEL_NONE = 0, LEVEL_ERROR = 1, LEVEL_INFO = 2, LEVEL_DEBUG = 3};

/**
 * Sets logger level
 * @param level_to_set: level of logging, must be of from 'log_level_type'
 */
void logger_init(UART_HandleTypeDef * huartPointer, int8_t level_to_set);

/**
 * Log stream to using USB CDC
 * @param log_level: logging level
 * @param format: formatted string
 * @return 0 if success, non-zero otherwise
 */
int log_text(int8_t log_level, char * format, ...);
