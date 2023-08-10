#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include "main.h"

#ifndef RS485_H_
#define RS485_H_


/// @brief init with UART pointer
/// @param uart_handler_ptr 
void rs485_init(UART_HandleTypeDef * uart_handler_ptr);

/// @brief stores byte to internal buffer
/// @param byte byte to store
/// @return operation success (true) or fail (false)
bool rs485_store_byte(uint8_t * byte);

/// @brief get stored frame from rs485 buffer to user buffer
/// @param dest_array 
/// @param array_size 
void rs485_get_frame(uint8_t * dest_array, uint8_t array_size);

/// @brief transmit byte using HAL
/// @param byte_array array pointer
/// @param array_size size in bytes
void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size);

/// @brief check if buffer empty
/// @return true if buffer empty, false otherwise
bool rs485_is_buffer_empty(void);

/// @brief check if buffer full
/// @return true if buffer full, false otherwise
bool rs485_is_buffer_full(void);

#endif /* RS485_H_ */
