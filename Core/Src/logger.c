#include <logger.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

enum log_level_type level = LEVEL_INFO; // this is a default logging level
UART_HandleTypeDef * huart;

void logger_init(UART_HandleTypeDef * huartPointer, int8_t level_to_set)
{
	huart = huartPointer;
	level = level_to_set;
}

uint8_t get_level()
{
	return (uint8_t)level;
}

int log_text(int8_t log_level, char * format, ...)
{
	if (get_level() < log_level)
	{
		return 0;
	}
	uint8_t max_string_len = 100;
	char string_buffer[max_string_len];

	// create string from format and arguments
	va_list argptr;
	va_start(argptr, format);
	vsnprintf(string_buffer, max_string_len, format, argptr);
	va_end(argptr);

	// send data
	const int max_retries = 3;
	int retries = 0;
	while (retries < max_retries)
	{
		HAL_StatusTypeDef retVal;

		retVal = HAL_UART_Transmit(huart, (uint8_t*)string_buffer, strlen(string_buffer), 1000); // TODO: should be non-blocking
//		retVal = HAL_UART_Transmit_IT(huart, (uint8_t*)string_buffer, strlen(string_buffer));
		if (retVal == HAL_OK)
		{
			return 0;
		}
		else
		{
			retries++;
		}
		if (retries >= max_retries)
		{
			return 1;
		}
	}
	return 0;
}
