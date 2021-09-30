// Simple logger for STM32 that uses STM's USB implementation to log data through USB CDC

// 1. USB must be initialized outside of this module, before logging anything
// 2. Default log level is LEVEL_INFO
// 3. Logger messages can be formatted the same way like printf arguments
// 4. This header must be included to any file using logger

// Sample use:
// logger_set_level(LEVEL_ERROR); // set log level first
// log_text(LEVEL_ERROR, "Error code: %d\r\n", err_code);

#include <stdbool.h>
#include <stdint.h>

enum log_level_type {LEVEL_ERROR = 0, LEVEL_INFO = 1, LEVEL_DEBUG = 2};

/**
 * Inits logger.
 * @param transmit_byte_callback: pointer to method for sending a byte
 */
void logger_init(bool (*transmit_byte_callback)(uint8_t *));

/**
 * Sets logger level
 * @param level_to_set: level of logging, as definen by log_level_type
 */
void logger_set_level(int level);

/**
 * Log stream. This is non-blocking.
 * It stores passed data in its own buffer, so the buffer that is passed don't have to exist after making a call.
 * @param log_level
 * @param format: formatted string, similar to what prinf takes
 * @return 0 if success, non-zero otherwise
 */
int logger_log(int log_level, char * format, ...);


/**
 * It must be called by callback from previous byte transmitted.
 */
void logger_transmit_complete(void);
