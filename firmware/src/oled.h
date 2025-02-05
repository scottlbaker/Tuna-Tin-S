
// ============================================================================
//
// oled.h - A simple OLED library
//
// ============================================================================

#ifndef OLED_H_
#define OLED_H_

#define OLED_ADDR     0x3C
#define OLED_COMMAND  0x00
#define OLED_DATA     0x40
#define OLED_PAGE     0xB0
#define OLED_OFF      0xAE
#define OLED_ON       0xAF
#define OLED_MAXCOL   128

class OLED {

public:

  // functions
  OLED();
  void begin();
  void end();
  void sendcmd(uint8_t cmd);
  void senddata(uint8_t data);
  void noDisplay();
  void onDisplay();
  void setPage(uint8_t x, uint8_t y);
  void setCursor(uint8_t col, uint8_t row);
  void clr2eol();
  void clrLine(uint8_t row);
  void clrScreen();
  void putch(uint8_t ch);
  void putstr(char *str);
  void printLine(uint8_t row, char *str);
  void print32(uint32_t val);

  // variables
  uint8_t oledX;
  uint8_t oledY;
  uint8_t m_row;
  uint8_t m_col;
  uint8_t maddr = 1;
  uint8_t myrow = 0;
  uint8_t mycol = 0;
  uint8_t fx1[10] = {0,0,0,0,0,0,0,0,0,0};
  uint8_t fx0[10] = {0,0,0,0,0,0,0,0,0,0};

  // SSD1306 initialization commands
  const uint8_t oled_init [25] = {
    0xD5, 0x80,   // set display clock divide ratio
    0xA8, 0x3F,   // Set multiplex ratio to 1:64
    0xD3, 0x00,   // set display offset = 0
    0x40,         // set display start line address
    0x8D, 0x14,   // set charge pump, internal VCC
    0x20, 0x02,   // set page mode memory addressing
    0xA4,         // output RAM to display
    0xA1,         // set segment re-map
    0xC8,         // set COM output scan direction
    0xDA, 0x12,   // Set com pins hardware configuration
    0x81, 0x80,   // set contrast control register
    0xDB, 0x40,   // set vcomh
    0xD9, 0xF1,   // 0xF1=brighter
    0xB0,         // set page address (0-7)
    0xA6,         // set display mode to normal
    0xAF          // display ON
  };

};

#endif
