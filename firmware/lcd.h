#include <LiquidCrystal_I2C.h>
#include <Wire.h>

const int nRows = 2;
const int nCols = 16;
#define LCD_ADDR 0x27


byte line0[8] = {  // Bottom line
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111
};

byte line1[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b00000
};

byte line2[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b00000,
  0b00000
};

byte line3[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b00000,
  0b00000,
  0b00000
};

byte line4[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte line5[8] = {
  0b00000,
  0b00000,
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte line6[8] = {
  0b00000,
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte line7[8] = {  // Top line
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};


LiquidCrystal_I2C lcd = LiquidCrystal_I2C(LCD_ADDR, nCols, nRows);

void lcd_set_up() {
  Wire.setClock(400000);
  // Initialize the LCD
  lcd.init();
  lcd.backlight();

  lcd.createChar(0, line0);
  lcd.createChar(1, line1);
  lcd.createChar(2, line2);
  lcd.createChar(3, line3);
  lcd.createChar(4, line4);
  lcd.createChar(5, line5);
  lcd.createChar(6, line6);
  lcd.createChar(7, line7);
}
