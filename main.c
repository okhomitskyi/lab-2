
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "stm32f4xx_exti.h"

static __IO uint32_t usTicks;

#define LCD_PORT GPIOE
#define LCD_PIN_RS GPIO_Pin_7
#define LCD_PIN_RW GPIO_Pin_10
#define LCD_PIN_EN GPIO_Pin_11
#define LCD_PIN_D7 GPIO_Pin_15
#define LCD_PIN_D6 GPIO_Pin_14
#define LCD_PIN_D5 GPIO_Pin_13
#define LCD_PIN_D4 GPIO_Pin_12
#define LCD_PIN_MASK ((LCD_PIN_RS | LCD_PIN_EN | LCD_PIN_D7 | LCD_PIN_D6 | LCD_PIN_D5 | LCD_PIN_D4))
#define LCD_PIN_OFFSET 12

const char *messages[5];

const unsigned char lcdAddLut[2] = {0x80, 0xC0};
unsigned char address, currentLine;

void lcdPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LCD_PIN_RS | LCD_PIN_RW | LCD_PIN_EN | LCD_PIN_D7 | LCD_PIN_D6 | LCD_PIN_D5 | LCD_PIN_D4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(LCD_PORT, LCD_PIN_RW);
}

void lcdWriteByte(uint8_t data)
{
	GPIO_SetBits(LCD_PORT, ((data & 0x0F)) << LCD_PIN_OFFSET);
	GPIO_SetBits(LCD_PORT, LCD_PIN_EN);
	lcd_delay_func(100);
	GPIO_ResetBits(LCD_PORT, LCD_PIN_EN);
	GPIO_ResetBits(LCD_PORT, 0x0F << LCD_PIN_OFFSET);

	GPIO_SetBits(LCD_PORT, (((data >> 4) & 0x0F)) << LCD_PIN_OFFSET);
	GPIO_SetBits(LCD_PORT, LCD_PIN_EN);
	lcd_delay_func(100);
	GPIO_ResetBits(LCD_PORT, LCD_PIN_EN);
	GPIO_ResetBits(LCD_PORT, 0x0F << LCD_PIN_OFFSET);
}

void lcdInit(void)
{
	lcdWriteByte(0x20);
	lcd_delay_func(100);
	lcdWriteByte(0x28);
	lcd_delay_func(100);
	lcdWriteByte(0x28);
	lcd_delay_func(100);
	lcdWriteByte(0x0f);
	lcd_delay_func(100);
	lcdWriteByte(0x01);
	lcd_delay_func(100);
	lcdWriteByte(0x06);
	lcd_delay_func(100);
}

void lcd_delay_func(unsigned int p)
{
	unsigned long i;
	for (i = 0; i < (p * 10); i++)
	{
		__nop();
	}
}

void lcdSetLine(char line)
{
	GPIO_ResetBits(LCD_PORT, LCD_PIN_RS);
	address = lcdAddLut[line - 1];
	lcdWriteByte(address);
	GPIO_SetBits(LCD_PORT, LCD_PIN_RS);
	address = 0;
	currentLine = line;
}

void lcdClear(void)
{
	GPIO_ResetBits(LCD_PORT, LCD_PIN_RS);
	lcdWriteByte(0x01);
	lcd_delay_func(100);
	GPIO_SetBits(LCD_PORT, LCD_PIN_RS);
	lcdSetLine(1);
}

void lcdShowChar(char c)
{
	GPIO_SetBits(LCD_PORT, LCD_PIN_RS);
	lcdWriteByte(c);
	address++;
	switch (address)
	{
	case 20:
		lcdSetLine(2);
		break;
	case 40:
		lcdSetLine(3);
		break;
	case 60:
		lcdSetLine(4);
		break;
	case 80:
		lcdSetLine(1);
		break;
	}
}

void lcdShowString(char *s)
{
	while (*s != 0)
		lcdShowChar(*s++);
}

void userButtonInit(void)
{
	GPIO_InitTypeDef GPIO_Init_Button;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_Init_Button.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init_Button.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init_Button.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init_Button.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_Button.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_Init_Button);
}

void SysTick_Handler()
{
	if (usTicks != 0)
	{
		usTicks--;
	}
}
void DelayInit()
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
}

void DelayMs(uint32_t ms)
{
	usTicks = ms;
	while (usTicks)
		;
}

int generate_random(int lower, int upper)
{

	int num = (rand() % (upper - lower + 1)) + lower;
}

int main(void)
{
	unsigned int inputValue;
	DelayInit();
	userButtonInit();
	lcdPinInit();
	lcdInit();

	messages[0] = "Message1";
	messages[1] = "Message2";
	messages[2] = "Message3";
	messages[3] = "Message4";
	messages[4] = "Message5";

	srand(time(0)); // generate seed timestamp

	while (1)
	{
		inputValue = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		if (inputValue == 1)
		{
			int randomNumber = generate_random(0, 4);
			const char *randomMessage = messages[randomNumber];
			// write to LCD random string
			lcdShowString(randomMessage);
			inputValue = 0;
			DelayMs(1000);
			lcdClear();
		}
	}
}
