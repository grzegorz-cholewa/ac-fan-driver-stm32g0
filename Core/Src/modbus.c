#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <modbus.h>
#include <crc.h>
#include <config.h>
#include <logger.h>

/* LOCAL DEFINES */
#define MAX_REGISTERS_OFFSET (REGISTERS_NUMBER-1)
#define FUNCTION_READ_MULTIPLE 0x03
#define FUNCTION_WRITE_SINGLE 0x06
#define POSITION_DEVICE_ID 0
#define POSITION_FUNCTION 1

/* STATIC FUNCTION DECLARATIONS */
uint8_t get_high_byte(uint16_t two_byte);
uint8_t get_low_byte(uint16_t two_byte);
uint16_t get_short_little_endian(uint8_t * first_byte_pointer);
uint16_t get_short_big_endian(uint8_t * first_byte_pointer);
void print_buffer(uint8_t * buffer, uint16_t length);

/* GLOBAL VARIABLES */
static int16_t registers[REGISTERS_NUMBER]; // allocates memory for modbus register data


int16_t modbus_get_reg_value(uint16_t offset)
{
	if (offset > MAX_REGISTERS_OFFSET)
	{
		// error handling
	}

	return registers[offset];
}


bool modbus_set_reg_value(uint16_t offset, int16_t value)
{
	if (offset > MAX_REGISTERS_OFFSET)
	{
		return false;
	}

	registers[offset] = value;
	return true;
}


bool modbus_process_frame(uint8_t * request, uint16_t request_size, uint8_t * response, uint16_t * response_size)
{
	logger_log(LEVEL_DEBUG, "Processing Modbus frame\r\n");
	print_buffer(request, request_size);

	// check CRC
	uint16_t crc_calculated = crc16_modbus(request, request_size-2);
	uint16_t crc_received = get_short_little_endian(request+request_size-2);
	if (crc_calculated != crc_received)
	{
		logger_log(LEVEL_ERROR, "ERR: modbus_process_frame, CRC does not match\r\n");
		return false;
	}

	switch (request[POSITION_FUNCTION])
	{
		case FUNCTION_READ_MULTIPLE:
			asm("NOP"); // needed for turn off compiler warning "a label can only be part of a statement and a declaration is not a statement"
			uint16_t first_address_offset = get_short_big_endian(request+2);
			uint16_t registers_number = get_short_big_endian(request+4);
			logger_log(LEVEL_INFO, "Modbus read request received, function 0x%02x, first register offset: %d, number of registers: %d\r\n", FUNCTION_READ_MULTIPLE, first_address_offset, registers_number);


			if ( (first_address_offset >= REGISTERS_NUMBER) || (registers_number > REGISTERS_NUMBER) )
			{
				logger_log(LEVEL_ERROR, "Requested registers not valid\r\n");
				return false;
			}

			*response_size = 3 + 2*registers_number + 2;

			/* Add constant elements */
			*(response + POSITION_DEVICE_ID) = DEVICE_ID;
			*(response + POSITION_FUNCTION) = FUNCTION_READ_MULTIPLE;
			*(response + 2) = 2*registers_number;

			/* Add data registers */
			for (int i = 0; i < registers_number; i++)
			{
				*(response + 3 + 2*i) =  get_high_byte(registers[first_address_offset+i]);
				*(response + 3 + 2*i + 1) =  get_low_byte(registers[first_address_offset+i]);
			}

			/* Add CRC */
			uint16_t crc_value = crc16_modbus(response, (*response_size)-2);
			*(response + 3 + 2*registers_number) = get_low_byte(crc_value);
			*(response + 4 + 2*registers_number) = get_high_byte(crc_value);
			break;
	
		case FUNCTION_WRITE_SINGLE:
			logger_log(LEVEL_INFO, "Modbus write request received, function 0x%02x\r\n", FUNCTION_WRITE_SINGLE);
			print_buffer(request, request_size);
			asm("NOP"); // needed for turn off compiler warning "a label can only be part of a statement and a declaration is not a statement"
			uint16_t register_offset = get_short_big_endian(request+2);
			int16_t value_to_set = get_short_big_endian(request+4);
			logger_log(LEVEL_INFO, "Writing register. Offset %d, value to set: %d\r\n", register_offset, value_to_set);

			if (register_offset >= REGISTERS_NUMBER)
			{
				logger_log(LEVEL_ERROR, "Requested registers not valid\r\n");
				return false;
			}

			registers[register_offset] = value_to_set;
			memcpy(response, request, request_size); // response for that command is echo
			*response_size = request_size;
			break;

		default:
			logger_log(LEVEL_ERROR, "Unsupported Modbus function\r\n");
			return false;
	}

	logger_log(LEVEL_DEBUG, "Modbus response\r\n");
	print_buffer(response, *response_size);

	return true;
}


uint8_t get_high_byte(uint16_t two_byte)
{
	return ((two_byte >> 8) & 0xFF); // MSB
}


uint8_t get_low_byte(uint16_t two_byte)
{
	return (two_byte & 0xFF); // LSB
}


uint16_t get_short_little_endian(uint8_t * first_byte_pointer) // first byte is low byte
{
	return (short) (*(first_byte_pointer+1) << 8 | *(first_byte_pointer));
}


uint16_t get_short_big_endian(uint8_t * first_byte_pointer) // first byte is high byte
{
	return (short) (*first_byte_pointer << 8 | *(first_byte_pointer+1));
}


void print_buffer(uint8_t * buffer, uint16_t length)
{
    char* hexString = (char*)malloc(length * 5 + 3); // 2 characters per byte + termination & endline
    if (!hexString)
    {
        return;
    }

    for (size_t i = 0; i < length; i++)
    {
        sprintf(hexString + i * 5, "0x%02x ", buffer[i]); // Convert byte to 2-character hex representation
    }

    sprintf(hexString + length * 5, "\r\n");

	logger_log(LEVEL_DEBUG, hexString, "\r\n");
	free(hexString);
}
