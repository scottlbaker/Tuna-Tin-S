
// ============================================================================
//
// oled.cpp - A simple OLED library
//
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include "oled.h"
#include "font.h"

OLED::OLED() {
}

// Public Methods

// initialization
void OLED::begin() {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(oled_init, sizeof(oled_init));
  Wire.endTransmission();
  delayMicroseconds(100);
  clrScreen();
}

void OLED::end() {
}

// send a command
void OLED::sendcmd(uint8_t cmd) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(OLED_COMMAND);
  Wire.write(cmd);
  Wire.endTransmission();
}

// send data
void OLED::senddata(uint8_t data) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(OLED_DATA);
  Wire.write(data);
  Wire.endTransmission();
}

// turn off the display
void OLED::noDisplay() { sendcmd(OLED_OFF); }

// turn on the display
void OLED::onDisplay() { sendcmd(OLED_ON); }

// set page
void OLED::setPage(uint8_t X, uint8_t Y) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(OLED_COMMAND);
  Wire.write(OLED_PAGE | Y);
  Wire.write(0x10 | ((X & 0xf0) >> 4));
  Wire.write(X & 0x0f);
  Wire.endTransmission();
}

// set cursor column and row
void OLED::setCursor(uint8_t col, uint8_t row) {
  oledX = col*FONT_W; oledY = row*FONT_H;
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(OLED_COMMAND);
  Wire.write(OLED_PAGE | (oledY & 0x07));
  uint8_t _oledX = oledX;
  Wire.write(0x10 | ((_oledX & 0xf0) >> 4));
  Wire.write(_oledX & 0x0f);
  Wire.endTransmission();
}

// clear to end of line
void OLED::clr2eol() {
  for (uint8_t x=oledX; x<OLED_MAXCOL; x++) senddata(0);
  for (uint8_t p=1; p<4; p++) {
    setPage(oledX, oledY+p);
    for (uint8_t x=oledX; x<OLED_MAXCOL; x++) senddata(0);
  }
}

// clear a line
void OLED::clrLine(uint8_t row) {
  if (row) row = 4;
  setCursor(0,row);
  for (uint8_t x=0; x<OLED_MAXCOL; x++) senddata(0);
  for (uint8_t p=1; p<4; p++) {
    setPage(oledX, oledY+p);
    for (uint8_t x=0; x<OLED_MAXCOL; x++) senddata(0);
  }
  setCursor(0,row);
}

// clear the screen
void OLED::clrScreen() {
  setCursor(0,0);
  for (uint8_t x=0; x<OLED_MAXCOL; x++) senddata(0);
  for (uint8_t p=1; p<8; p++) {
    setPage(oledX, oledY+p);
    for (uint8_t x=0; x<OLED_MAXCOL; x++) senddata(0);
  }
  setCursor(0,0);
}

// print a char
void OLED::putch(uint8_t ch) {
  uint8_t i, j;
  uint8_t fx[8] = {0,0,0,0,0,0,0,0};
  uint8_t mk = 0x01;
  uint8_t dat;
  if ((ch == '\n') || (oledX > (OLED_MAXCOL - FONT_W))) return;
  if (ch < 32 || ch > 137) ch = 32;
  // lookup the character and stretch it
  for (i=0; i<FONT_W; i++) {
    fx[i] = pgm_read_byte(&(font[((ch-32)*FONT_W)+i]));
  }
  for (j=0; j<4; j++) {
    for (i=0; i<FONT_W; i++) {
      dat = 0;
      if (fx[i] & mk) dat |= 0x0f;
      if (fx[i] & (mk<<1)) dat |= 0xf0;
      senddata(dat);
    }
    if (j<3) {
      mk = mk<<2;
      setPage(oledX, oledY+j+1);
    }
  }
  m_col++;
  setCursor(m_col,m_row);
}

// print a string
void OLED::putstr(char *str) {
  while(*str) putch(*str++);
  clr2eol();
}

// print a line
void OLED::printLine(uint8_t row, char *str) {
  setCursor(0,row);
  putstr(str);
}

// print a 32-bit frequency value
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

