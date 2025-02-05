
// ============================================================================
//
// oled.cpp - A simple OLED library
//
// ============================================================================

#include <stdint.h>
#include <Arduino.h>
#include "i2c.h"
#include "oled.h"
#include "font.h"

extern I2C i2c;

OLED::OLED() {
}

// Public Methods

void OLED::begin() {
  i2c.write(OLED_ADDR, OLED_COMMAND, oled_init, sizeof(oled_init));
  wait(300);
  clrScreen();
}

void OLED::end() {
}

// delay
void OLED::wait(uint16_t x) {
  for (int16_t i=0; i<x; i++) {
    asm("");
  }
}

// send data
void OLED::senddata(uint8_t data) {
  i2c.write(OLED_ADDR, OLED_DATA, data);
}

// send zeros
void OLED::sendzeros(uint8_t nbytes) {
  i2c.writezeros(OLED_ADDR, OLED_DATA, nbytes);
}

// send ones
void OLED::sendones(uint8_t nbytes) {
  i2c.writeones(OLED_ADDR, OLED_DATA, nbytes);
}

// turn off the display
void OLED::noDisplay() {
  i2c.write(OLED_ADDR, OLED_COMMAND, OLED_OFF);
}

// turn on the display
void OLED::onDisplay() {
  i2c.write(OLED_ADDR, OLED_COMMAND, OLED_ON);
}

// set page
void OLED::setPage(uint8_t x, uint8_t y) {
  uint8_t data_arr[] = {
  (OLED_PAGE | y),
  (0x10 | ((x & 0xf0) >> 4)),
  (x & 0x0f)};
  i2c.write(OLED_ADDR, OLED_COMMAND, data_arr, 3);
}

// set cursor column and row
void OLED::setCursor(uint8_t col, uint8_t row) {
  m_row = row;
  m_col = col;
  oledX = col*FONT_W;
  oledY = ((row*FONT_H) & 0x06) | 0x01;
  setPage(oledX, oledY);
}

// set cursor to XY with no scaling
void OLED::setXY(uint8_t col, uint8_t row) {
  oledX = col;
  oledY &= 0x06;
  setPage(oledX, oledY);
}

// show the stepsize cursor
void OLED::showCursor() {
  #define ROW 4
  setPage(0, ROW);
  sendzeros(OLED_MAXCOL);
  setPage(oledX+2, ROW);
  i2c.writecursor(OLED_ADDR, OLED_DATA);
}

// set cursor to home
void OLED::home() {
  setCursor(0,0);
}

// clear to end of line
void OLED::clr2eol() {
  sendzeros(OLED_MAXCOL - oledX);
  setXY(oledX, oledY+1);
  sendzeros(OLED_MAXCOL - oledX);
}

// clear a line
void OLED::clrLine(uint8_t row) {
  setCursor(0, row);
  sendzeros(OLED_MAXCOL);
  setXY(0, oledY);
  sendzeros(OLED_MAXCOL);
  setCursor(0, row);
}

// clear the screen
void OLED::clrScreen() {
  setCursor(0,0);
  sendzeros(OLED_MAXCOL);
  for (uint8_t p=1; p<8; p++) {
    setPage(oledX, oledY+p);
    sendzeros(OLED_MAXCOL);
  }
  setCursor(0,0);
}

// lookup the char in the font table and stretch it
void OLED::lookup(uint8_t ch) {
  uint8_t mk1;
  uint8_t mk2;
  uint8_t dat;
  uint8_t dax;
  for (uint8_t i=0; i<FONT_W; i++) {
    // read the font data
    dat = pgm_read_byte(&(font[((ch-32)*FONT_W)+i]));
    mk1 = 0x01;
    mk2 = 0x03;
    dax = 0;
    // stretch the data vertically
    for (uint8_t j=0; j<4; j++) {
      if (dat & mk1) dax |= mk2;
      mk1 = mk1 <<1;
      mk2 = mk2 <<2;
    }
    fx0[i]= dax;
    mk1 = 0x10;
    mk2 = 0x03;
    dax = 0;
    // stretch the data vertically
    for (uint8_t j=0; j<4; j++) {
      if (dat & mk1) dax |= mk2;
      mk1 = mk1 <<1;
      mk2 = mk2 <<2;
    }
    fx1[i]= dax;
  }
}

// print a char
void OLED::putch(uint8_t ch) {
  uint8_t i;
  if ((ch == '\n') || (oledX > (128 - FONT_W))) return;
  if (ch < 32 || ch > 137) ch = 32;
  lookup(ch);
  for (i=0; i<FONT_W; i++) senddata(fx1[i]);
  setXY(oledX, oledY);
  for (i=0; i<FONT_W; i++) senddata(fx0[i]);
  m_col++;
  setCursor(m_col, m_row);
}

// print a string
void OLED::putstr(char *str) {
  while(*str) putch(*str++);
  clr2eol();
}

// print a line
void OLED::printline(uint8_t row, char *str) {
  setCursor(0,row);
  putstr(str);
}

// print an 8-bit integer value
void OLED::print8(uint8_t val) {
  char tmp[4] = "  0";
  // convert to string
  for (uint8_t i=2; val; i--) {
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  // left justify
  while (tmp[0] == ' ') {
    tmp[0] = tmp[1];
    tmp[1] = tmp[2];
    tmp[2] = tmp[3];
  }
  putstr(tmp);
}

// print an 16-bit integer value
void OLED::print16(uint16_t val) {
  char tmp[6] = "    0";
  // convert to string
  for (uint8_t i=4; val; i--) {
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  // left justify
  while (tmp[0] == ' ') {
    tmp[0] = tmp[1];
    tmp[1] = tmp[2];
    tmp[2] = tmp[3];
    tmp[3] = tmp[4];
    tmp[4] = tmp[5];
  }
  putstr(tmp);
}

// print a 32-bit integer value
void OLED::print32(uint32_t val) {
  char tmp[15] = "               ";
  // convert to string
  for (uint8_t i=9; val; i--) {
    if ((i==6) || (i==2)) {
      tmp[i] = ',';
      i--;
    }
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  setCursor(0,1);
  putstr(tmp);
}

// print a frequency value
void OLED::print_freq(uint64_t val) {
  char tmp[9] = "        ";
  val /= 100;
  int i = 7;
  // convert to string
  for (; val; i--) {
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  putstr(tmp);
}
