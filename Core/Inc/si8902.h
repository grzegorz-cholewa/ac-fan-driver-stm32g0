/*
 * si8902.h
 *
 *  Created on: 24 Apr 2022
 *      Author: DELL
 */

#ifndef INC_SI8902_H_
#define INC_SI8902_H_

uint8_t getCommandByte(int adc_channel);
int getAdcValue(uint8_t highByte, uint8_t lowByte);

#endif /* INC_SI8902_H_ */
