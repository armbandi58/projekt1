/*
 * fuggvenyek.h
 *
 *  Created on: 2020. okt. 10.
 *      Author: borsa
 */

#ifndef FUGGVENYEK_H_
#define FUGGVENYEK_H_

#include "main.h"

void LCD_int(uint32_t number, int lenght);
void LCD_float(float f_number, int lenght);
void ftoa(float n, char* res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char* str, int len);
void uart_newrow(void);

#endif /* FUGGVENYEK_H_ */
