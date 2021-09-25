#include <logger.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <buffer.h>

#define buffer_size (1000)
#define max_string_size (100)


UART_HandleTypeDef * huart;
cbuf_handle_t cbuf;
bool transmitting_now = false;
enum log_level_type level; // default logging level
uint8_t buffer[buffer_size];
uint8_t transmit_buffer;

uint8_t get_level();
void transmit_next_byte();


void logger_init(UART_HandleTypeDef * huartPointer)
{
	huart = huartPointer;
	level = LEVEL_INFO; // default level
	cbuf = circular_buf_init(buffer, buffer_size);

	logger_log(LEVEL_INFO, "Logger init complete\r\n");
}


void logger_set_level(uint8_t level_to_set)
{
	if (level_to_set == LEVEL_ERROR)
	{
		logger_log(LEVEL_INFO, "Logging level set to ERROR\r\n");
		level = LEVEL_ERROR;
	}
	else if (level_to_set == LEVEL_INFO)
	{
		level = LEVEL_INFO;
		logger_log(LEVEL_INFO, "Logging level set to INFO\r\n");
	}
	else if (level_to_set == LEVEL_DEBUG)
	{
		logger_log(LEVEL_INFO, "Logging level set to DEBUG\r\n");
		level = LEVEL_DEBUG;
	}
	else
	{
		logger_log(LEVEL_INFO, "Wrong logging level. Setting to default (INFO)\r\n");
	}
}


int logger_log(int8_t log_level, char * format, ...)
{
	if (get_level() < log_level)
	{
		return 0;
	}

	char temp_buffer[max_string_size];

	// create string from format and arguments
	va_list argptr;
	va_start(argptr, format);
	const uint8_t log_level_len = 5;
	switch (log_level)
	{
		case LEVEL_ERROR:
			strcpy(temp_buffer, "ERR: ");
			break;
		case LEVEL_INFO:
			strcpy(temp_buffer, "INF: ");
			break;
		case LEVEL_DEBUG:
			strcpy(temp_buffer, "DBG: ");
			break;
		default:
			;
	}
	vsnprintf(temp_buffer+log_level_len, strlen(format)+1, format, argptr);
	va_end(argptr);

	// put data to buffer
	for (int byte_count = 0; byte_count < strlen(temp_buffer); byte_count++)
	{
		circular_buf_put2(cbuf, temp_buffer[byte_count]);
	}

	transmit_next_byte();

	return 0;
}


void logger_transmit_complete()
{
	transmitting_now = false;
	transmit_next_byte();
}


uint8_t get_level()
{
	return (uint8_t)level;
}


void transmit_next_byte()
{
	// read from buffer if there is something to read
	if (transmitting_now)
	{
		return;
	}
	if(!circular_buf_empty(cbuf))
	{
		circular_buf_get(cbuf, &transmit_buffer);
		HAL_StatusTypeDef retVal = HAL_UART_Transmit_IT(huart, &transmit_buffer, 1);
		if (retVal == HAL_OK)
		{
			transmitting_now = true;
		}
	}
	else
	{
		// empty buffer, nothing to read
		return;
	}
}


