#include <logger.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <buffer.h>

#define buffer_size (1000)
#define max_string_size (100)

/* Global variables */
enum log_level_type level;
cbuf_handle_t cbuf;
bool transmitting_now;
uint8_t buffer[buffer_size];
uint8_t transmit_buffer;
bool (*transmit_byte_method)(uint8_t *);
bool is_initialized = false;

/* Static methods */
static uint8_t get_level();
static void transmit_next_byte();


void logger_init(bool (*transmit_byte_callback)(uint8_t *))
{
	transmit_byte_method = transmit_byte_callback;
	level = LEVEL_INFO; // default level
	transmitting_now = false;
	cbuf = circular_buf_init(buffer, buffer_size);
	is_initialized = true;
	logger_log(LEVEL_INFO, "Logger init complete\r\n");
}


void logger_set_level(int level_to_set)
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
		logger_log(LEVEL_INFO, "Wrong logging level\r\n");
	}
}


int logger_log(int log_level, char * format, ...)
{
	if (!is_initialized)
	{
		return 0;
	}
	
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


void logger_transmit_next_byte()
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
		bool success = transmit_byte_method(&transmit_buffer);
		if (success)
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


