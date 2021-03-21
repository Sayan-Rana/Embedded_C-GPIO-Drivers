/*
 * lcd.c
 *
 *  Created on: 08-Mar-2021
 *      Author: Sayan Rana
 */

#include "lcd.h"

// Some helper function prototype
static void write_4_bits(uint8_t value);
static void lcd_enable(void);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);


void lcd_send_command(uint8_t cmd)
{
	/* RS = 0, for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* RW = 0, for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	/* Send higher nibble first */
	write_4_bits(cmd >> 4);

	/* Then send lower nibble */
	write_4_bits(cmd & 0x0F);
}


/*
 * This function sends a character to the LCD.
 * Here we used a 4 bit parallel data transmission.
 * First higher nibble of the data will be sent on the data line D4,D5,D6,D7
 * Then lower nibble of the data will be sent on the data line D4,D5,D6,D7
 */
void lcd_print_char(uint8_t data)
{
	/* RS = 1, for LCD user data */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	/* RW = 0, for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	/* Send higher nibble */
	write_4_bits(data >> 4);

	/* Send lower nibble */
	write_4_bits(data & 0x0F);
}



void lcd_print_string(char *message)
{
	do
	{
		lcd_print_char((uint8_t)*message++);
	}
	while(*message != '\0');
}



void lcd_init(void)
{
	// 1. Configure GPIO pins which are used for LCD connection
	GPIO_Handle_t lcd_signal;

	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	lcd_signal.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	lcd_signal.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	// Initially set all the GPIO pins to logic zero
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	// 2. LCD initialization
	mdelay(40);

	// RS = 0 for LCD command
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	// RW = 0 for writing to lcd
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(0x3);
	mdelay(5);
	write_4_bits(0x3);
	udelay(150);
	write_4_bits(0x3);
	write_4_bits(0x2);

	// Function set command
	lcd_send_command(LCD_CMD_4DL_2N_5x8F);

	// Display off command(display control command)
	lcd_send_command(LCD_CMD_DON_CURON);

	// Display clear command
	lcd_display_clear();

	// Entry mode set command
	lcd_send_command(LCD_CMD_INCADD);
}


/* Write 4 bits of data/command on to D4, D5, D6, D7 lines */
static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1));

	lcd_enable();
}


void lcd_display_clear(void)
{
	lcd_send_command(LCD_CMD_DIS_CLEAR);
	mdelay(2);
}



/* Cursor returns to home position */
void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	mdelay(2);
}



/*
 * Set LCD a specific location given by a row and column information
 * Row number (1 to 2)
 * Column number (1 to 16) assuming a 16x2 character display
 */
void lcd_set_cursor(uint8_t row, uint8_t column)
{
	column--;
	switch (row)
	{
		case 1:
			/* Set the cursor to the first row address and add index */
			lcd_send_command(column |= 0x80);
			break;
		case 2:
			/* Set the cursor to the second row address and add index */
			lcd_send_command(column |= 0xC0);
			break;
		default:
			break;
	}
}


static void lcd_enable(void)
{
	// First pull the pin high
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);

	// Wait for 10usec
	udelay(10);
	// Then pull the pin low
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);

	// Again wait for 10usec
	udelay(100);
}



static void mdelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < (cnt * 1000); i++);
}


static void udelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < (cnt * 1); i++);
}

























