/*
 * seven_segment_display_4digit.c
 *
 *  Created on: Sep 29, 2024
 *      Author: Hayk Khulyan
 */

#include "seven_segment_display_4digit.h"

//#define A      10
//#define B      6
//#define C      3
//#define D      1
//#define E      0
//#define F      9
//#define G      4
//#define CL     2
//
//#define DIGIT1 11
//#define DIGIT2 8
//#define DIGIT3 7
//#define DIGIT4 5
//
//static uint32_t * GPIO_ODR[12];
//static uint8_t GPIO_pin_ports[12];

// initializes the GPIO_ODR and GPIO_pin_ports variables with the values
void init_pins(FourDigitDisplay_Config_t *FourDigitDisplay_Config) {

}

void display_digit(int8_t value, uint8_t digit) {
	// set all segment pins to 0
	// set all digit pins to high
	// set the proper digit pin to low
	// set the proper segment pins to 1
	// set all segment pins to 0 (i don't think this is necessary?)
	// set all digit pins to high

//	clear_segments();
//
//	clear_digits();
//
//	switch (digit) {
//	case 1:
//		activate_digit(DIGIT1);
//		break;
//	case 2:
//		activate_digit(DIGIT2);
//		break;
//	case 3:
//		activate_digit(DIGIT3);
//		break;
//	case 4:
//		activate_digit(DIGIT4);
//		break;
//	}
//
//	switch (value) {
//	case 0:
//		activate_segment(A);
//		activate_segment(B);
//		activate_segment(C);
//		activate_segment(D);
//		activate_segment(E);
//		activate_segment(F);
//		break;
//	case 1:
//		activate_segment(B);
//		activate_segment(C);
//		break;
//	case 2:
//		activate_segment(A);
//		activate_segment(B);
//		activate_segment(D);
//		activate_segment(E);
//		activate_segment(G);
//		break;
//	case 3:
//		activate_segment(A);
//		activate_segment(B);
//		activate_segment(C);
//		activate_segment(D);
//		activate_segment(G);
//		break;
//	case 4:
//		activate_segment(B);
//		activate_segment(C);
//		activate_segment(F);
//		activate_segment(G);
//		break;
//	case 5:
//		activate_segment(A);
//		activate_segment(C);
//		activate_segment(D);
//		activate_segment(F);
//		activate_segment(G);
//		break;
//	case 6:
//		activate_segment(A);
//		activate_segment(C);
//		activate_segment(D);
//		activate_segment(E);
//		activate_segment(F);
//		activate_segment(G);
//		break;
//	case 7:
//		activate_segment(A);
//		activate_segment(B);
//		activate_segment(C);
//		break;
//	case 8:
//		activate_segment(A);
//		activate_segment(B);
//		activate_segment(C);
//		activate_segment(D);
//		activate_segment(E);
//		activate_segment(F);
//		activate_segment(G);
//		break;
//	case 9:
//		activate_segment(A);
//		activate_segment(B);
//		activate_segment(C);
//		activate_segment(D);
//		activate_segment(F);
//		activate_segment(G);
//		break;
//	}
//
//	clear_digits();
}

void display_colon() {
	clear_segments();

	clear_digits();

//	activate_digit(DIGIT3);
//	activate_digit(DIGIT4);
//
//	activate_segment(CL);

	clear_digits();
}

void clear_segments() {
//	*GPIO_ODR[A]  &= ~(1 << GPIO_pin_ports[A]);
//	*GPIO_ODR[B]  &= ~(1 << GPIO_pin_ports[B]);
//	*GPIO_ODR[C]  &= ~(1 << GPIO_pin_ports[C]);
//	*GPIO_ODR[D]  &= ~(1 << GPIO_pin_ports[D]);
//	*GPIO_ODR[E]  &= ~(1 << GPIO_pin_ports[E]);
//	*GPIO_ODR[F]  &= ~(1 << GPIO_pin_ports[F]);
//	*GPIO_ODR[G]  &= ~(1 << GPIO_pin_ports[G]);
//	*GPIO_ODR[CL] &= ~(1 << GPIO_pin_ports[CL]);
}

void clear_digits() {
//	*GPIO_ODR[DIGIT1] |= (1 << GPIO_pin_ports[DIGIT1]);
//	*GPIO_ODR[DIGIT2] |= (1 << GPIO_pin_ports[DIGIT2]);
//	*GPIO_ODR[DIGIT3] |= (1 << GPIO_pin_ports[DIGIT3]);
//	*GPIO_ODR[DIGIT4] |= (1 << GPIO_pin_ports[DIGIT4]);
}

void activate_segment(uint8_t segment) {
//	*GPIO_ODR[segment] |= (1 << GPIO_pin_ports[segment]);

	// this for-loop adds a small delay, ensuring that the LED remains on for
	// long enough to appear bright
	// otherwise, the latter segments get turned off so
	// quickly that they appear more dim
	for (uint64_t i = 0; i < 100; i++);

//	*GPIO_ODR[segment] &= ~(1 << GPIO_pin_ports[segment]);
}

void activate_digit(uint8_t digit) {
//	*GPIO_ODR[digit] &= ~(1 << GPIO_pin_ports[digit]);
}


