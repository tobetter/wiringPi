/*
 * i2c_lcd.c:
 *   Standard "i2c_lcd" program in wiringPi.
 *   Supports 16x2 and 20x4 screens.
 *   reference: lcd_i2c.py (https://bitbucket.org/MattHawkinsUK/rpispy-misc/raw/master/python/lcd_i2c.py)
 *
 * Copyright (C) 2023 Steve Jeong
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Define some device parameters
#define I2C_ADDR  0x3f // I2C device address
#define LCD_WIDTH  20

// Define some device constants
#define LCD_CHR  1 // Mode - Sending data
#define LCD_CMD  0 // Mode - Sending command

#define LINE1  0x80 // 1st line
#define LINE2  0xC0 // 2nd line
#define LINE3  0x94 // 3rd line
#define LINE4  0xD4 // 4th line

#define LCD_BACKLIGHT   0x08  // On
// LCD_BACKLIGHT = 0x00  # Off

#define ENABLE  0b00000100 // Enable bit

void lcd_init(void);
void lcd_byte(int bits, int mode);
void lcd_toggle_enable(int bits);
void display_string(const char *string, int line);

int fd;
char *bus; /* /dev/i2c-0 ~ /dev/i2c-9 */
int address;

int main(int argc, char *argv[]) {

  char device[16] = "/dev/i2c-";

  bus = "0";
  address = 0x3f;

  if (wiringPiSetup () == -1)
    exit (1);

  if (argc > 1)
    address = strtoul(argv[1], NULL, 16);

  if (argc > 2) {
    bus = argv[1];
    address = strtoul(argv[2], NULL, 16);
  }

  strncat(device, bus, 1);
  fd = wiringPiI2CSetupInterface(device, address);

  lcd_init(); // setup LCD

  while (1) {
    display_string("Hard Kernel", LINE1);
    display_string("Hello ODROID", LINE2);
  }

  return 0;

}

void display_string(const char *string, int line) {
  // go to location on LCD
  lcd_byte(line, LCD_CMD);

  if (strlen(string) > LCD_WIDTH) {
    printf("message is too long!\n");
    return;
  }

  while (*string) lcd_byte(*(string++), LCD_CHR);
}

void lcd_byte(int bits, int mode) {

  //Send byte to data pins
  // bits = the data
  // mode = 1 for data, 0 for command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xf0) | LCD_BACKLIGHT ;
  bits_low = mode | ((bits << 4) & 0xf0) | LCD_BACKLIGHT ;

  // High bits
  wiringPiI2CReadReg8(fd, bits_high);
  lcd_toggle_enable(bits_high);

  // Low bits
  wiringPiI2CReadReg8(fd, bits_low);
  lcd_toggle_enable(bits_low);
}

void lcd_toggle_enable(int bits)   {
  // Toggle enable pin on LCD display
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits | ENABLE));
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits & ~ENABLE));
  delayMicroseconds(500);
}

void lcd_init()   {
  // Initialise display
  lcd_byte(0x33, LCD_CMD); // Initialise
  lcd_byte(0x32, LCD_CMD); // Initialise
  lcd_byte(0x06, LCD_CMD); // Cursor move direction
  lcd_byte(0x0C, LCD_CMD); // 0x0F On, Blink Off
  lcd_byte(0x28, LCD_CMD); // Data length, number of lines, font size
  lcd_byte(0x01, LCD_CMD); // Clear display
  delayMicroseconds(1000);
}
