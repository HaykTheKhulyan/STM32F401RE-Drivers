/*
 * seven_segment_display_4digit.c
 *
 *  Created on: Sep 29, 2024
 *      Author: Hayk Khulyan
 */

#include "seven_segment_display_4digit.h"

#define SEGMENT_A      11
#define SEGMENT_B      7
#define SEGMENT_C      4
#define SEGMENT_D      2
#define SEGMENT_E      1
#define SEGMENT_F      10
#define SEGMENT_G      5
#define SEGMENT_CL     3

static FourDigitDisplay_Config_t GPIO_PortsAndPins;


/*******************************************************************************
 * @fn 			- Initialize pins
 *
 * @brief		- Initializes the GPIO pins defined in the config struct
 *
 * @param		- config struct defining which GPIO pins will be used for
 * 				  the display
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void init_pins(FourDigitDisplay_Config_t *FourDigitDisplay_Config) {
	GPIO_PortsAndPins = *FourDigitDisplay_Config;

	// set config values for the GPIO pins (they should all be the same except
	// for the pointer and the pin number
	GPIO_Handle_t GPIO;

	GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIO.GPIO_PinConfig.GPIO_PinOutputType = GPIO_OT_PP;
	GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OS_HIGH;
	GPIO.GPIO_PinConfig.GPIO_PinPUPDRControl = GPIO_PUPD_NONE;

	// initialize a GPIO at the given port and pin with the config settings we
	// just selected

	// digit 1
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_Digit1;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_Digit1;

	GPIO_Init(&GPIO);

	// digit 2
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_Digit2;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_Digit2;

	GPIO_Init(&GPIO);

	// digit 3
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_Digit3;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_Digit3;

	GPIO_Init(&GPIO);

	// digit 4
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_Digit4;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_Digit4;

	GPIO_Init(&GPIO);

	// segment A
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_SegmentA;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_SegmentA;

	GPIO_Init(&GPIO);

	// segment B
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_SegmentB;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_SegmentB;

	GPIO_Init(&GPIO);

	// segment C
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_SegmentC;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_SegmentC;

	GPIO_Init(&GPIO);

	// segment D
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_SegmentD;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_SegmentD;

	GPIO_Init(&GPIO);

	// segment E
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_SegmentE;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_SegmentE;

	GPIO_Init(&GPIO);

	// segment F
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_SegmentF;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_SegmentF;

	GPIO_Init(&GPIO);

	// segment G
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_SegmentG;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_SegmentG;

	GPIO_Init(&GPIO);

	// segment colon
	GPIO.GPIOx_ptr = FourDigitDisplay_Config->GPIOPort_SegmentColon;
	GPIO.GPIO_PinConfig.
	     GPIO_PinNumber = FourDigitDisplay_Config->GPIOPin_SegmentColon;

	GPIO_Init(&GPIO);
}

/*******************************************************************************
 * @fn 			- Display number
 *
 * @brief		- Displays the given 4 digit number
 *
 * @param		- the 4 digit number to be displayed
 *
 * @return		- none
 *
 * @note		- maximum value that can be display is 9999
 * 				- if the number does not contain a value at a certain decimal
 * 			      place, that display digit will be left blank
 *
 ******************************************************************************/
void display_number(int16_t value) {
	if (value > 999) {
		display_digit(value / 1000, DIGIT_1);
	}

	if (value > 99) {
		display_digit((value % 1000) / 100, DIGIT_2);
	}

	if (value > 9) {
		display_digit((value % 100) / 10, DIGIT_3);
	}

	if (value > 0) {
		display_digit(value % 10, DIGIT_4);
	}
}

/*******************************************************************************
 * @fn 			- Display digit
 *
 * @brief		- Displays a digit at a chosen digit position
 *
 * @param		- the digit value (0-9) to be displayed
 * 				- the digit position (1-4) to display the digit at
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void display_digit(int8_t value, uint8_t digit) {
	// set all segment pins to 0
	// set all digit pins to high
	// set the proper digit pin to low
	// set the proper segment pins to 1
	// set all segment pins to 0 (i don't think this is necessary?)
	// set all digit pins to high

	clear_segments();

	clear_digits();

	switch (digit) {
	case DIGIT_1:
		activate_digit(DIGIT_1);
		break;
	case DIGIT_2:
		activate_digit(DIGIT_2);
		break;
	case DIGIT_3:
		activate_digit(DIGIT_3);
		break;
	case DIGIT_4:
		activate_digit(DIGIT_4);
		break;
	}

	switch (value) {
	case 0:
		activate_segment(SEGMENT_A);
		activate_segment(SEGMENT_B);
		activate_segment(SEGMENT_C);
		activate_segment(SEGMENT_D);
		activate_segment(SEGMENT_E);
		activate_segment(SEGMENT_F);
		break;
	case 1:
		activate_segment(SEGMENT_B);
		activate_segment(SEGMENT_C);
		break;
	case 2:
		activate_segment(SEGMENT_A);
		activate_segment(SEGMENT_B);
		activate_segment(SEGMENT_D);
		activate_segment(SEGMENT_E);
		activate_segment(SEGMENT_G);
		break;
	case 3:
		activate_segment(SEGMENT_A);
		activate_segment(SEGMENT_B);
		activate_segment(SEGMENT_C);
		activate_segment(SEGMENT_D);
		activate_segment(SEGMENT_G);
		break;
	case 4:
		activate_segment(SEGMENT_B);
		activate_segment(SEGMENT_C);
		activate_segment(SEGMENT_F);
		activate_segment(SEGMENT_G);
		break;
	case 5:
		activate_segment(SEGMENT_A);
		activate_segment(SEGMENT_C);
		activate_segment(SEGMENT_D);
		activate_segment(SEGMENT_F);
		activate_segment(SEGMENT_G);
		break;
	case 6:
		activate_segment(SEGMENT_A);
		activate_segment(SEGMENT_C);
		activate_segment(SEGMENT_D);
		activate_segment(SEGMENT_E);
		activate_segment(SEGMENT_F);
		activate_segment(SEGMENT_G);
		break;
	case 7:
		activate_segment(SEGMENT_A);
		activate_segment(SEGMENT_B);
		activate_segment(SEGMENT_C);
		break;
	case 8:
		activate_segment(SEGMENT_A);
		activate_segment(SEGMENT_B);
		activate_segment(SEGMENT_C);
		activate_segment(SEGMENT_D);
		activate_segment(SEGMENT_E);
		activate_segment(SEGMENT_F);
		activate_segment(SEGMENT_G);
		break;
	case 9:
		activate_segment(SEGMENT_A);
		activate_segment(SEGMENT_B);
		activate_segment(SEGMENT_C);
		activate_segment(SEGMENT_D);
		activate_segment(SEGMENT_F);
		activate_segment(SEGMENT_G);
		break;
	}

//	clear_segments();

	clear_digits();
}

/*******************************************************************************
 * @fn 			- Display colon
 *
 * @brief		- Displays the colon
 *
 * @param		- none
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void display_colon() {
	clear_segments();

	clear_digits();

	activate_digit(DIGIT_3);
	activate_digit(DIGIT_4);

	activate_segment(SEGMENT_CL);

	clear_digits();
}

/*******************************************************************************
 * @fn 			- Clear segments
 *
 * @brief		- Sets all segments to blank
 *
 * @param		- none
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void clear_segments() {
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentA,
						  GPIO_PortsAndPins.GPIOPin_SegmentA,
						  0);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentB,
						  GPIO_PortsAndPins.GPIOPin_SegmentB,
						  0);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentC,
						  GPIO_PortsAndPins.GPIOPin_SegmentC,
						  0);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentD,
						  GPIO_PortsAndPins.GPIOPin_SegmentD,
						  0);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentE,
						  GPIO_PortsAndPins.GPIOPin_SegmentE,
						  0);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentF,
						  GPIO_PortsAndPins.GPIOPin_SegmentF,
						  0);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentG,
						  GPIO_PortsAndPins.GPIOPin_SegmentG,
						  0);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentColon,
						  GPIO_PortsAndPins.GPIOPin_SegmentColon,
						  0);
}

/*******************************************************************************
 * @fn 			- Clear digits
 *
 * @brief		- De-selects all 4 digits
 *
 * @param		- none
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void clear_digits() {
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_Digit1,
						  GPIO_PortsAndPins.GPIOPin_Digit1,
						  1);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_Digit2,
						  GPIO_PortsAndPins.GPIOPin_Digit2,
						  1);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_Digit3,
						  GPIO_PortsAndPins.GPIOPin_Digit3,
						  1);
	GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_Digit4,
						  GPIO_PortsAndPins.GPIOPin_Digit4,
						  1);
}

/*******************************************************************************
 * @fn 			- Activate segment
 *
 * @brief		- activates a chosen segment on the display
 *
 * @param		- enum for the chosen segment
 *
 * @return		- none
 *
 * @note		- an active segment will only be lit at a digit position that
 * 				  is also active. otherwise, activating a segment has no effect
 *
 ******************************************************************************/
void activate_segment(uint8_t segment) {
//	*GPIO_ODR[segment] |= (1 << GPIO_pin_ports[segment]);
	switch (segment) {
	case SEGMENT_A:
		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentA,
							  GPIO_PortsAndPins.GPIOPin_SegmentA,
							  1);

		// this for-loop adds a small delay, ensuring that the LED remains on
		// for long enough to appear bright otherwise, the latter segments get
		// turned off so quickly that they appear more dim
		for (uint64_t i = 0; i < 100; i++);

		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentA,
							  GPIO_PortsAndPins.GPIOPin_SegmentA,
							  0);

		break;
	case SEGMENT_B:
		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentB,
							  GPIO_PortsAndPins.GPIOPin_SegmentB,
							  1);

		for (uint64_t i = 0; i < 100; i++);

		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentB,
							  GPIO_PortsAndPins.GPIOPin_SegmentB,
							  0);

		break;
	case SEGMENT_C:
		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentC,
							  GPIO_PortsAndPins.GPIOPin_SegmentC,
							  1);

		for (uint64_t i = 0; i < 100; i++);

		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentC,
							  GPIO_PortsAndPins.GPIOPin_SegmentC,
							  0);

		break;
	case SEGMENT_D:
		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentD,
							  GPIO_PortsAndPins.GPIOPin_SegmentD,
							  1);

		for (uint64_t i = 0; i < 100; i++);

		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentD,
							  GPIO_PortsAndPins.GPIOPin_SegmentD,
							  0);

		break;
	case SEGMENT_E:
		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentE,
							  GPIO_PortsAndPins.GPIOPin_SegmentE,
							  1);

		for (uint64_t i = 0; i < 100; i++);

		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentE,
							  GPIO_PortsAndPins.GPIOPin_SegmentE,
							  0);

		break;
	case SEGMENT_F:
		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentF,
							  GPIO_PortsAndPins.GPIOPin_SegmentF,
							  1);

		for (uint64_t i = 0; i < 100; i++);

		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentF,
							  GPIO_PortsAndPins.GPIOPin_SegmentF,
							  0);

		break;
	case SEGMENT_G:
		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentG,
							  GPIO_PortsAndPins.GPIOPin_SegmentG,
							  1);

		for (uint64_t i = 0; i < 100; i++);

		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentG,
							  GPIO_PortsAndPins.GPIOPin_SegmentG,
							  0);

		break;
	case SEGMENT_CL:
		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentColon,
							  GPIO_PortsAndPins.GPIOPin_SegmentColon,
							  1);

		for (uint64_t i = 0; i < 100; i++);

		GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_SegmentColon,
							  GPIO_PortsAndPins.GPIOPin_SegmentColon,
							  0);

		break;
	}
}

/*******************************************************************************
 * @fn 			- Activate digit
 *
 * @brief		- activates the chosen digit
 *
 * @param		- enum for the chosen digit
 *
 * @return		- none
 *
 * @note		- this function enables a digit to be controlled by any active
 * 				  segments. a digit must be activated before the corresponding
 * 				  LED segments at that digit position can be lit
 *
 ******************************************************************************/
void activate_digit(uint8_t digit) {
	switch (digit) {
		case DIGIT_1:
			GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_Digit1,
								  GPIO_PortsAndPins.GPIOPin_Digit1,
								  0);
			break;
		case DIGIT_2:
			GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_Digit2,
								  GPIO_PortsAndPins.GPIOPin_Digit2,
								  0);
			break;
		case DIGIT_3:
			GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_Digit3,
								  GPIO_PortsAndPins.GPIOPin_Digit3,
								  0);
			break;
		case DIGIT_4:
			GPIO_WriteToOutputPin(GPIO_PortsAndPins.GPIOPort_Digit4,
								  GPIO_PortsAndPins.GPIOPin_Digit4,
								  0);
			break;
	}
}


