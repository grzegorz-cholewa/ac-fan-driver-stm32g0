/** 
 * OVERVIEW 
 * Simple logger for handling logs. Supports logging levels.
 * It is not implementing transmission itself, for which callback is stored during init.

 * HOW TO USE
 * logger_init(&logger_transmit_byte); // pass byte transmit function pointer
 * logger_set_level(LEVEL_ERROR);
 * log_text(LEVEL_ERROR, "Error code: %d\r\n", err_code);
 *
 * Default log level is LEVEL_INFO
 * Logger messages can be formatted the same way like printf arguments
 * This header must be included to any file using logger
 * It uses circular buffer as a dependency
**/

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
 * It must be called externally after previous byte is transmitted.
 * Call it in UART transmit complete interrupt if possible.
 */
void logger_transmit_next_byte(void);
