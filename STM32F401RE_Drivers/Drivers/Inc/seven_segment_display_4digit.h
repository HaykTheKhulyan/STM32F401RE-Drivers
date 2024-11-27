/*
 * seven_segment_display_4digit.h
 *
 *  Created on: Sep 28, 2024
 *      Author: Hayk Khulyan
 */

/*
 * To use the 4 digit seven segment display:
 *
 */

#ifndef SEVEN_SEGMENT_DISPLAY_4DIGIT_H_
#define SEVEN_SEGMENT_DISPLAY_4DIGIT_H_

#include <stdint.h>
#include "stm32f401re.h"

// this config struct will be configured by the user to hold the GPIO ports and
// pins corresponding to each of the segments and control pins on the display
typedef struct
{
	GPIO_RegDef_t *GPIOPort_SegmentA;
	uint8_t GPIOPin_SegmentA;
	GPIO_RegDef_t *GPIOPort_SegmentB;
	uint8_t GPIOPin_SegmentB;
	GPIO_RegDef_t *GPIOPort_SegmentC;
	uint8_t GPIOPin_SegmentC;
	GPIO_RegDef_t *GPIOPort_SegmentD;
	uint8_t GPIOPin_SegmentD;
	GPIO_RegDef_t *GPIOPort_SegmentE;
	uint8_t GPIOPin_SegmentE;
	GPIO_RegDef_t *GPIOPort_SegmentF;
	uint8_t GPIOPin_SegmentF;
	GPIO_RegDef_t *GPIOPort_SegmentG;
	uint8_t GPIOPin_SegmentG;
	GPIO_RegDef_t *GPIOPort_SegmentColon;
	uint8_t GPIOPin_SegmentColon;
	GPIO_RegDef_t *GPIOPort_Digit1;
	uint8_t GPIOPin_Digit1;
	GPIO_RegDef_t *GPIOPort_Digit2;
	uint8_t GPIOPin_Digit2;
	GPIO_RegDef_t *GPIOPort_Digit3;
	uint8_t GPIOPin_Digit3;
	GPIO_RegDef_t *GPIOPort_Digit4;
	uint8_t GPIOPin_Digit4;

} FourDigitDisplay_Config_t;

// initializes static global variables with ODR addresses and pin ports
void init_pins(FourDigitDisplay_Config_t *FourDigitDisplay_Config);

// displays a given value (0-9) at the given digit spot (1-4)
void display_digit(int8_t value, uint8_t digit);
// displays the colon
void display_colon();

// outputs high to all segments (which doesn't allow current to flow to LED)
void clear_digits();
// outputs low to all segments
void clear_segments();

// outputs low to a given digit (allowing current to flow)
void activate_digit(uint8_t digit);
// outputs high to a given segment, delays for a bit, then outputs low
void activate_segment(uint8_t segment);


#endif /* SEVEN_SEGMENT_DISPLAY_4DIGIT_H_ */
