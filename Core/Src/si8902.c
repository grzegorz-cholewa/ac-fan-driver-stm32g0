/*
 * si8902.c
 *
 *  Created on: 24 Apr 2022
 *      Author: DELL
 */
#include <config.h>
#include <logger.h>


uint8_t getCommandByte(int adc_channel)
{
	// byte 7:6 - not used (set 1), byte 5:4 ADC channel, byte 3 - vref, byte 2 - not used (set 1), byte 1 - mode, byte 0 - gain
	uint8_t defaultCommandByte = 0b11001111; // LSB is byte 0 when referring to datasheet

	// byte 5:4 is setting ADC channel
	uint8_t ain_mask;
	switch (adc_channel)
	{
		case 0:
			ain_mask = 0b00;
			break;
		case 1:
			ain_mask = 0b01;
			break;
		case 2:
			ain_mask = 0b10;
			break;
	}

	return defaultCommandByte | (ain_mask << 4);
}


// take bytes 3:0 from high byte and 6:1 from lower byte to get ADC value
int getAdcValue(uint8_t highByte, uint8_t lowByte)
{
    int value = 0;

    lowByte = lowByte & 0b01111110; // limit to only used values in byte
    highByte = highByte & 0b00001111; // limit to only used values in byte

    value = value | (lowByte >> 1);
    value = value | (highByte << 6);

	return value;
}

