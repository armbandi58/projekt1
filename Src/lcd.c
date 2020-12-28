/*
 * lcd.c
 *
 *  Created on: 2020. szept. 26.
 *      Author: borsa
 */
#include "lcd.h"
#include "main.h"

void LCD1_test(){
	LCD_string("++++LCD_INIT++++");
	LCD_command(0xC0);
	LCD_string("++++LCD_INIT++++");
	HAL_Delay(1000);
	LCD_command(0x01);
}

void LCD2_test(){
	LCD2_string2("++++LCD_INIT++++");
	LCD2_command2(0xC0);
	LCD2_string2("++++LCD_INIT++++");
	HAL_Delay(1000);
	LCD2_command2(0x01);
}

void LCD_string(char *p){
	while(*p ){
		LCD_data(*p); // lehet igy is LCD_data(*p++)
		p++; //lépünk mindig tovabb
	}
}

void LCD_goto(uint8_t row, uint8_t col){
	switch(row){
		case 1:
			LCD_command(0x80);
			for(uint8_t i = col; i--; i>0){
				LCD_command(0x14);
			}
			break;

		case 2:
			LCD_command(0xC0);
			for(uint8_t i = col; i--; i>0){
				LCD_command(0x14);
			}
			break;

		default:
			break;
	}
}

void LCD_enable(void){
	//LCD_E 0-1-0 atmenetet kell megcsinalnunk
	HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin,RESET);
	//kell bele kesleltetes is , adatlapból kiolvasni miért
	HAL_Delay(5);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin,SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin,RESET);
	HAL_Delay(5);
}

void LCD_command(uint8_t cmd){
	//Rs -> 0-ba kell rakni,
	// R/W(neg) -> 0-ba kell rakni, mert irunk ------------> de ez mar földön van
	//1. felso 4 adatbit
	HAL_GPIO_WritePin(LCD_Rs_GPIO_Port,LCD_Rs_Pin,RESET);

	HAL_GPIO_WritePin(LCD_DATA7_GPIO_Port,LCD_DATA7_Pin, (cmd>>7)&0x01);
	HAL_GPIO_WritePin(LCD_DATA6_GPIO_Port,LCD_DATA6_Pin, (cmd>>6)&0x01);
	HAL_GPIO_WritePin(LCD_DATA5_GPIO_Port,LCD_DATA5_Pin, (cmd>>5)&0x01);
	HAL_GPIO_WritePin(LCD_DATA4_GPIO_Port,LCD_DATA4_Pin, (cmd>>4)&0x01);

	//enable közte, meg utana

	LCD_enable();

	//2. also 4 adatbit

	HAL_GPIO_WritePin(LCD_DATA7_GPIO_Port,LCD_DATA7_Pin, (cmd>>3)&0x01);
	HAL_GPIO_WritePin(LCD_DATA6_GPIO_Port,LCD_DATA6_Pin, (cmd>>2)&0x01);
	HAL_GPIO_WritePin(LCD_DATA5_GPIO_Port,LCD_DATA5_Pin, (cmd>>1)&0x01);
	HAL_GPIO_WritePin(LCD_DATA4_GPIO_Port,LCD_DATA4_Pin, (cmd>>0)&0x01);

	//enable

	LCD_enable();
}

//LCD_data kinézetre ugyan az mint command, csak itt set van  es mast shiftelünk
void LCD_data(uint8_t data){
	//Rs -> 0-ba kell rakni,
	// R/W(neg) -> 0-ba kell rakni, mert irunk ------------> de ez mar földön van
	//1. felso 4 adatbit

	HAL_GPIO_WritePin(LCD_Rs_GPIO_Port,LCD_Rs_Pin,SET);

	HAL_GPIO_WritePin(LCD_DATA7_GPIO_Port,LCD_DATA7_Pin, (data>>7)&0x01);
	HAL_GPIO_WritePin(LCD_DATA6_GPIO_Port,LCD_DATA6_Pin, (data>>6)&0x01);
	HAL_GPIO_WritePin(LCD_DATA5_GPIO_Port,LCD_DATA5_Pin, (data>>5)&0x01);
	HAL_GPIO_WritePin(LCD_DATA4_GPIO_Port,LCD_DATA4_Pin, (data>>4)&0x01);

	//enable közte, meg utana

	LCD_enable();

	//2. also 4 adatbit

	HAL_GPIO_WritePin(LCD_DATA7_GPIO_Port,LCD_DATA7_Pin, (data>>3)&0x01);
	HAL_GPIO_WritePin(LCD_DATA6_GPIO_Port,LCD_DATA6_Pin, (data>>2)&0x01);
	HAL_GPIO_WritePin(LCD_DATA5_GPIO_Port,LCD_DATA5_Pin, (data>>1)&0x01);
	HAL_GPIO_WritePin(LCD_DATA4_GPIO_Port,LCD_DATA4_Pin, (data>>0)&0x01);

	//enable

	LCD_enable();
}

void LCD_init(void){
	//eroszakolos, 4 bites technika
		//4 bites mod - 2 sor - 5*8 pixel - lcd on - cursor on - cursor blink on
	HAL_Delay(15);
	LCD_command(0x20);

	LCD_enable();
	LCD_enable();
	LCD_enable();

	//azert kell 3szor mert nem tudjuk milyen modban van
	LCD_command(0x28);		//mod allitas : 4bit, 2 sor, 5*8 pixel
	LCD_command(0x28);
	LCD_command(0x28);

	LCD_command(0x01);		//lcd clear
	LCD_command(0x02); 		//lcd home

	LCD_command(0x0F);		//lcd - kurzor blinking

}

void LCD_init_customcurzor(bool on, bool blink){
	//eroszakolos, 4 bites technika
		//4 bites mod - 2 sor - 5*8 pixel - lcd on - cursor on - cursor blink on
	HAL_Delay(15);
	LCD_command(0x20);

	LCD_enable();
	LCD_enable();
	LCD_enable();

	//azert kell 3szor mert nem tudjuk milyen modban van
	LCD_command(0x28);		//mod allitas : 4bit, 2 sor, 5*8 pixel
	LCD_command(0x28);
	LCD_command(0x28);

	LCD_command(0x01);		//lcd clear
	LCD_command(0x02); 		//lcd home

	//LCD_command(0x0F);		//lcd - kurzor blinking
	LCD_command(0x08|(1<<LCD_E)|(on<<LCD_Curzor)|(blink<<LCD_Curzor_blink));
}

//				=====================
//						LCD2
//				=====================
void LCD2_init_customcurzor2(bool on, bool blink){
	//eroszakolos, 4 bites technika
		//4 bites mod - 2 sor - 5*8 pixel - lcd on - cursor on - cursor blink on
	HAL_Delay(15);
	LCD2_command2(0x20);

	LCD2_enable2();
	LCD2_enable2();
	LCD2_enable2();

	//azert kell 3szor mert nem tudjuk milyen modban van
	LCD2_command2(0x28);		//mod allitas : 4bit, 2 sor, 5*8 pixel
	LCD2_command2(0x28);
	LCD2_command2(0x28);

	LCD2_command2(0x01);		//lcd clear
	LCD2_command2(0x02); 		//lcd home

	//LCD_command(0x0F);		//lcd - kurzor blinking
	LCD2_command2(0x08|(1<<LCD_E)|(on<<LCD_Curzor)|(blink<<LCD_Curzor_blink));
}

void LCD2_command2(uint8_t cmd){
	//Rs -> 0-ba kell rakni,
	// R/W(neg) -> 0-ba kell rakni, mert irunk ------------> de ez mar földön van
	//1. felso 4 adatbit
	HAL_GPIO_WritePin(LCD2_Rs_GPIO_Port,LCD2_Rs_Pin,RESET);

	HAL_GPIO_WritePin(LCD2_DATA7_GPIO_Port,LCD2_DATA7_Pin, (cmd>>7)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA6_GPIO_Port,LCD2_DATA6_Pin, (cmd>>6)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA5_GPIO_Port,LCD2_DATA5_Pin, (cmd>>5)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA4_GPIO_Port,LCD2_DATA4_Pin, (cmd>>4)&0x01);

	//enable közte, meg utana

	LCD2_enable2();

	//2. also 4 adatbit

	HAL_GPIO_WritePin(LCD2_DATA7_GPIO_Port,LCD2_DATA7_Pin, (cmd>>3)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA6_GPIO_Port,LCD2_DATA6_Pin, (cmd>>2)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA5_GPIO_Port,LCD2_DATA5_Pin, (cmd>>1)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA4_GPIO_Port,LCD2_DATA4_Pin, (cmd>>0)&0x01);

	//enable

	LCD2_enable2();
}

void LCD2_data2(uint8_t data){
	//Rs -> 0-ba kell rakni,
	// R/W(neg) -> 0-ba kell rakni, mert irunk ------------> de ez mar földön van
	//1. felso 4 adatbit

	HAL_GPIO_WritePin(LCD2_Rs_GPIO_Port,LCD2_Rs_Pin,SET);

	HAL_GPIO_WritePin(LCD2_DATA7_GPIO_Port,LCD2_DATA7_Pin, (data>>7)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA6_GPIO_Port,LCD2_DATA6_Pin, (data>>6)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA5_GPIO_Port,LCD2_DATA5_Pin, (data>>5)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA4_GPIO_Port,LCD2_DATA4_Pin, (data>>4)&0x01);

	//enable közte, meg utana

	LCD2_enable2();

	//2. also 4 adatbit

	HAL_GPIO_WritePin(LCD2_DATA7_GPIO_Port,LCD2_DATA7_Pin, (data>>3)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA6_GPIO_Port,LCD2_DATA6_Pin, (data>>2)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA5_GPIO_Port,LCD2_DATA5_Pin, (data>>1)&0x01);
	HAL_GPIO_WritePin(LCD2_DATA4_GPIO_Port,LCD2_DATA4_Pin, (data>>0)&0x01);

	//enable

	LCD2_enable2();
}

void LCD2_string2(char *p){
	while(*p ){
		LCD2_data2(*p); // lehet igy is LCD_data(*p++)
		p++; //lépünk mindig tovabb
	}
}

void LCD2_goto2(uint8_t row, uint8_t col){
	switch(row){
		case 1:
			LCD2_command2(0x80);
			for(uint8_t i = col; i--; i>0){
				LCD2_command2(0x14);
			}
			break;

		case 2:
			LCD2_command2(0xC0);
			for(uint8_t i = col; i--; i>0){
				LCD2_command2(0x14);
			}
			break;

		default:
			break;
	}
}

void LCD2_enable2(){
	//LCD_E 0-1-0 atmenetet kell megcsinalnunk
	HAL_GPIO_WritePin(LCD2_E_GPIO_Port,LCD2_E_Pin,RESET);
	//kell bele kesleltetes is , adatlapból kiolvasni miért
	HAL_Delay(5);
	HAL_GPIO_WritePin(LCD2_E_GPIO_Port,LCD2_E_Pin,SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(LCD2_E_GPIO_Port,LCD2_E_Pin,RESET);
	HAL_Delay(5);
}
