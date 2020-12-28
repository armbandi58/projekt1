/*
 * lcd.h
 *
 *  Created on: 2020. szept. 26.
 *      Author: borsa
 */

#ifndef LCD_H_
#define LCD_H_

#include "main.h"

#define LCD_E 2
#define LCD_Curzor 1
#define LCD_Curzor_blink 0

void LCD_enable(void);
void LCD_init(void);
void LCD_data(uint8_t data);
void LCD_command(uint8_t cmd);
void LCD_string(char *p);
void LCD_goto(uint8_t row, uint8_t col);
void LCD_init_customcurzor(bool on, bool blink);
void LCD1_test(void);

void LCD2_command2(uint8_t cmd);
void LCD2_init_customcurzor2(bool on, bool blink);
void LCD2_data2(uint8_t data);
void LCD2_string2(char *p);
void LCD2_goto2(uint8_t row, uint8_t col);
void LCD2_enable2(void);
void LCD2_test(void);

#endif /* LCD_H_ */
