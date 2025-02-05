
// ============================================================================
//
// tuna.ino  ::  Tuna-Tin CW radio control program
//
// (c) Scott Baker KJ7NLA
//
// Libraries
// ---------
// i2c.h        - a simple I2C lib
// ee.h         - a simple EEPROM lib
// oled.h       - an OLED display lib
// font.h       - a font that I designed
// si5351.h     - by Milldrum and Myers
//
// Arduino IDE settings
// --------------------
// board: Arduino UNO
// bootloader: no bootloader
// programmer: AVRISP mkII
//
// Acknowledgement
// ---------------
// The original Tuna Tin "S" transmitter was first desribed in the
// December 2020 issue of QST magazine and more information about
// this project can be found here:
// sites.google.com/site/rcarcs/diy-radio/tuna-tin-s-qrp-transmitter
//
// Description
// -----------
// This firmware is compatible with the original Tuna Tin "S"
// transmitter but it a complete re-write with additional features
// including an iambic keyer.
//
// Hardware Requirements
// ---------------------
// The following modifications to the original Tuna Tin "S" hardware
// are required to run this firmware:
// a) The Arduino KEYOUT pin must be connected the improved
// keying circuit. Refer to this post for more info:
// groups.io/g/DIYRadio/topic/improved_keying_circuit_for/110074812
// b) The Arduino DIT and DAH pins must be connected to a key or paddle
// input connector
// c) The Arduino SW2 pin must be connected to a pushbutton to support
// the new menu system of this firmware.
// d) The Arduino SIDETONE pin must be connected to a low pass filter
// followed by an amplified speaker.
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// ============================================================================

#define VERSION   "TunaTin 3.0x"        // firmware version
#define DATE      "Jan 19 2025"         // firmware date
#define AUTHOR    "KJ7NLA"              // firmware author

// Arduino Pins
#define RXD       0      // PD0   UART RX            (pin 30)
#define TXD       1      // PD1   UART TX            (pin 31)
#define ROTA      2      // PD2   encoder A          (pin 32)
#define SW1       3      // PD3   UI switch 1        (pin  1)
#define ROTB      4      // PD4   encoder B          (pin  2)
#define SW2       5      // PD5   UI switch 2        (pin  9)
#define DIT       6      // PD6   paddle input       (pin 10)
#define DAH       7      // PD7   paddle input       (pin 11)
#define KEYOUT    8      // PB0   key out            (pin 12)
#define SIDETONE  9      // PB1   OC1A PWM audio     (pin 13)
#define TXLED    13      // PB5   Tx LED             (pin 17)
#define VBATT    20      // ADC6  battery voltage    (pin 19)

#include "i2c.h"
#include "ee.h"
#include "oled.h"
#include "si5351.h"
#include "lookup.h"
#include "font.h"

// generic
#define OFF      0
#define ON       1
#define NO       0
#define YES      1
#define FALSE    0
#define TRUE     1
#define FACTORY  0    // factory reset
#define SOFT     1    // soft reset
#define SERIAL   1    // print to UART
#define LOCAL    2    // print to OLED
#define BOTH     3    // print to UART and OLED

// string prototype defs
char getc();
char gnac();
void getsemi();
char gcal(char ch);
uint8_t len(char *str);
void send(char *str);
uint8_t cmpstr(char *dst, char *src);
void catc(char *dst, char c);
void cpystr(char *dst, char *src);
void catstr(char *dst, char *src);
void uppercase(char *str);
uint8_t alpha(char ch);
uint8_t numeric(char ch);
uint32_t fs2int(char *str);

// more prototype defs
void show_version(uint8_t x);
void show_help();
void show_cal();
void show_info();
void show_debug();
void wait_ms(uint16_t dly);
void wait_us(uint16_t dly);
void blinkLED();
void error_blink();
void stepsize_cursor();
inline void CAT_VFO();
void check_CAT();
inline void CAT_cmd();
void save_eeprom();
void init_VFO();
void init_wpm();
void init_freq();
void init_timer0();
void init_timer1();
void start_timer1();
void stop_timer1();
void init_timer2();
void start_timer2();
void stop_timer2();
void init_pins();
void init_i2c();
void init_uart();
void init_oled();
void init_check();
void init_adc();

void minsky();
void read_adc();
void set_oled_timeout();
void set_tx_status(uint8_t tx);
void freq2band(uint32_t freq);
void update_display();
void check_timeout();
void check_UI();
void check_menu();
void exit_menu();
void menuAction(uint8_t id);
void paramAction(uint8_t id, uint8_t* ptr, const char* sap[], uint8_t min, uint8_t max);
void show_label(uint8_t id);
void show_value (uint8_t id, uint8_t val, const char* sap[]);
void update_vfo();
void reset_xtimer();
void do_reset(uint8_t soft);
void run_calibrate();

char lookup_cw(uint8_t addr);
void print_cw();
void maddr_cmd(uint8_t cmd);
void read_paddles();
void iambic_keyer();
void straight_key();

// eeprom addresses
#define DATA_ADDR    10      // calibration data
#define FREQ_ADDR    20      // frequency

// Si5351 xtal frequency (25 MHz)
#define SI5351_REF  25000000UL

// calibration data
#define CAL_DATA_MAX  100000ULL
#define CAL_DATA_INIT  64000ULL
uint32_t cal_data = CAL_DATA_INIT;

// class instantiation
Si5351  si5351;
I2C     i2c;
EE      eeprom;
OLED    oled;

// delay times (ms)
#define DEBOUNCE          50
#define LED_BLINK        100
#define ONE_SECOND      1000
#define TWO_SECONDS     2000
#define THREE_SECONDS   3000
#define TEN_SECONDS    10000
#define HALF_MINUTE    30000
#define ONE_MINUTE     60000
#define TWO_MINUTES    120000
#define FIVE_MINUTES   600000
#define HALF_HOUR     3600000

// user interface macros
#define NBP  0  // no-button-pushed
#define BSC  1  // button-single-click
#define BPL  2  // button-push-long
#define DLP  3  // double-long-press
#define SLP  4  // super-long-press

#define SUPERPRESS  3500
#define XLPRESS     1200
#define LONGPRESS   500

#define SW1_PRESSED  !digitalRead(SW1)
#define SW2_PRESSED  !digitalRead(SW2)
#define ANY_PRESSED  SW1_PRESSED | SW2_PRESSED

#define INITWPM   25         // initial keyer speed
#define INITVOL    4         // initial volume
#define INITVFO 14074000ULL  // initial vfo frequency

// keyer mode definitions
#define STRAIGHT   0         // straight key
#define IAMBICA    1         // iambic A
#define IAMBICB    2         // iambic B
#define ULTIMATIC  3         // ultimatic

// step sizes
#define STEP_0    0
#define STEP_1    1
#define STEP_10   2
#define STEP_100  3
#define STEP_1K   4
#define STEP_10K  5
#define STEP_100K 6
#define STEP_1M   7

uint32_t stepsizes[] = { 0, 1, 10, 100, 1000, 10000, 100000, 1000000 };

// menu states
#define NOT_IN_MENU   0
#define SELECT_MENU   1
#define SELECT_VALUE  2

#define VOLUME      0
#define KEYERWPM    1
#define RADIOBAND   2
#define CWTONE      3
#define DXBLANK     4
#define CALIBRATE   5
#define SAVE2EE     6
#define KEYERMODE   7
#define KEYSWAP     8
#define SWVER       9

char menulabel[][16] = {
  "Volume",
  "Keyer WPM",
  "HF Band",
  "CW Tone",
  "OLED Timeout",
  "Calibrate",
  "EEPROM Save",
  "Keyer Mode",
  "Key Swap",
  "Version",
};

#define FIRSTMENU  VOLUME
#define LASTMENU   SWVER

#define TONE700    2        // 700 Hz sidetone
#define TONE600    1        // 600 Hz sidetone

uint16_t ct[] = {600, 700};

// band assignments
#define UNKNOWN   0
#define BAND_06M  1
#define BAND_10M  2
#define BAND_12M  3
#define BAND_15M  4
#define BAND_17M  5
#define BAND_20M  6
#define BAND_30M  7
#define BAND_40M  8
#define BAND_60M  9
#define BAND_80M 10

const char* band_label[]   = {
  "???",
  "06M",
  "10M",
  "12M",
  "15M",
  "17M",
  "20M",
  "30M",
  "40M",
  "60M",
  "80M"
};

// menu variables
uint8_t volume     = INITVOL;    // audio volume
uint8_t keyerwpm   = INITWPM;    // keyer speed
uint8_t keyermode  = ULTIMATIC;  // keyer mode
uint8_t keyswap    = OFF;        // key swap
uint8_t radioband  = BAND_20M;   // radio band
uint8_t prevband   = BAND_20M;   // previous radio band
uint8_t cwtone     = TONE600;    // CW tone select
uint8_t dxblank    = ON;         // display blanking
uint8_t calibrate  = OFF;        // calibrate mode
uint8_t save2ee    = OFF;        // save to eeprom
int8_t  stepsize   = STEP_1K;    // freq tuning step size

// keyer globals
uint16_t dittime;        // dit time
uint16_t dahtime;        // dah time
uint16_t lettergap1;     // letter space for decode
uint16_t lettergap2;     // letter space for send
uint16_t wordgap1;       // word space for decode
uint16_t wordgap2;       // word space for send

// other globals
uint8_t menumode   = NOT_IN_MENU;
uint8_t event      = NBP;
uint8_t enc_locked = NO;
int8_t  menu       = VOLUME;
uint8_t maddr      = 1;
uint8_t recordMsg  = OFF;
uint8_t DEBUG      = FALSE;
uint8_t tx_status  = OFF;

const char* cwtone_label[] = { "600", "700" };
const char* dxbk_label[]   = { "OFF", "5 Minutes", "30 Minutes"};
const char* keyer_label[]  = { "OFF", "Iambic A", "Iambic B", "Ultimatic"};
const char* onoff_label[]  = { "OFF", "ON" };

// for CW messages
#define MAXLEN  50
char tmpstr[MAXLEN];

// for display blank/timeout
uint8_t  display = ON;
uint32_t xtimer;

// vfo frequency
uint32_t vfofreq;
uint32_t catfreq;

// per-band stored frequencies
int32_t bandfreq[] = {
  3573000,  5357000,  7074000,  10136000, 14074000,
  18100000, 21074000, 24915000, 28074000
};

// millisecond time
volatile uint32_t msTimer = 0;

// timer 0 interrupt service routine
ISR(TIMER0_COMPA_vect) {
  msTimer++;
}

#define COSINIT   250
volatile int16_t msin = 0;
volatile int16_t mcos = COSINIT;

// timer2 interrupt handler
ISR(TIMER2_COMPA_vect) {
  if (tx_status) {
    minsky();
    OCR1AL = (msin >> ((10-volume) >> 1)) + 128;
  } else {
    OCR1AL = 128;
    msin = 0;
    mcos = COSINIT;
  }
}

// rotary encoder variables
volatile int8_t  enc_val  = 0;
volatile uint8_t enc_state;
volatile uint8_t enc_a;
volatile uint8_t enc_b;

// rotary encoder interrupt handler
ISR(PCINT2_vect) {
  enc_a = digitalRead(ROTA);
  enc_b = digitalRead(ROTB);
  enc_state = (enc_state << 4) | (enc_b << 1) | enc_a;
  switch (enc_state) {
    case 0x23:  enc_val++; break;
    case 0x32:  enc_val--; break;
    default: break;
  }
  if (enc_locked || !display) enc_val = 0;
  reset_xtimer();
}

// read char from the serial port
char getc() {
  while (!Serial.available());
  return(Serial.read());
}

// get next alpha char from serial buffer
char gnac() {
  char ch;
  uint16_t tc = 0;  // for timeout
  while (TRUE) {
    if (Serial.available()) {
      ch = Serial.read();
      if (alpha(ch)) return(ch);
    } else {
      if (tc++ > 1024) return('z');
    }
  }
}

// wait for semicolon from serial buffer
void getsemi() {
  char ch;
  while (TRUE) {
    if (Serial.available()) {
      ch = Serial.read();
      if (ch == ';') return;
    }
  }
}

// get cal control char from serial buffer
char gcal(char ch) {
  char new_ch;
  uint16_t tc = 0;  // timeout counter
  while (TRUE) {
    if (Serial.available()) {
      new_ch = Serial.read();
      if ((new_ch=='+')||(new_ch=='-')||
          (new_ch=='/')||(new_ch=='\\')||
          (new_ch=='=')||(new_ch=='.')){
        return(new_ch);
      }
    } else {
      if (tc++ > 1024) return(ch);
    }
  }
}

// return the length of string
uint8_t len(char *str) {
  uint8_t i=0;
  while (str[i++]);
  return i-1;
}

// send a command
void send(char *str) {
  getsemi(); // get semicolon
  Serial.print(str);
}

// compare command
uint8_t cmpstr(char *x, char *y) {
  if ((x[0] == y[0]) && (x[1] == y[1])) return(1);
  else return(0);
}

// concatenate a character to a string
void catc(char *dst, char ch) {
  uint8_t strlen= len(dst);
  dst[strlen++] = ch;
  dst[strlen] = '\0';
}

// copy a string
void cpystr(char *dst, char *src) {
  uint8_t i=0;
  while (src[i]) dst[i++] = src[i];
  dst[i] = '\0';
}

// concatenate a string
void catstr(char *dst, char *src) {
  uint8_t i=0;
  uint8_t strlen = len(dst);
  while (src[i]) dst[strlen++] = src[i++];
  dst[strlen] = '\0';
}

// convert command to upper case
void uppercase(char *str) {
  if ((str[0] >= 'a') && (str[0] <= 'z')) str[0] -= 32;
  if ((str[1] >= 'a') && (str[1] <= 'z')) str[1] -= 32;
}

// check if char is alpha
uint8_t alpha(char ch) {
  if ((ch >= 'A') && (ch <= 'z')) return(1);
  return(0);
}

// check if char is numeric
uint8_t numeric(char ch) {
  if ((ch >= '0') && (ch <= '9')) return(1);
  return(0);
}

// convert frequency string to integer
uint32_t fs2int(char *str) {
  uint32_t acc = 0;
  uint32_t pwr10 = 1;
  uint8_t digit;
  for (uint8_t i=10; i>2; i--) {
    digit = (uint8_t)str[i] - 48;
    acc += digit * pwr10;
    pwr10 = ((pwr10<<3)+(pwr10<<1));
  }
  return(acc);
}

// print the firmware version
void show_version(uint8_t x) {
  if ((x == SERIAL) || (x == BOTH)) {
    // print to serial port
    Serial.print("  ");
    Serial.print(VERSION);
    Serial.print("\r\n  ");
    Serial.print(DATE);
    Serial.print("\r\n  ");
    Serial.print(AUTHOR);
    Serial.print("\r\n\n");
  }
  if ((x == LOCAL) || (x == BOTH)) {
    // print to OLED
    oled.clrScreen();
    oled.printline(0, VERSION);
    oled.printline(1, DATE);
    oled.printline(2, AUTHOR);
    wait_ms(TWO_SECONDS);
    oled.clrScreen();
  }
}

#define HELP_MSG "\r\n\
  IF  G -  radio status\r\n\
  ID  G -  radio ID\r\n\
  FA  G S  frequency\r\n\
  AI  G S  auto-information\r\n\
  MD  G S  radio mode\r\n\
  PS  G S  power-on status\r\n\
  XT  G S  XIT status\r\n\
  TX  - S  transmit\r\n\
  RX  - S  receive\r\n\n\
  HE => print help\r\n\
  HH => print help\r\n\
  DD => debug on/off\r\n\
  II => print info\r\n\
  FR => factory reset\r\n\
  SR => soft reset\r\n\
  CM => calibration mode\r\n\n"

// print help message
void show_help() {
  Serial.print(HELP_MSG);
}

// print calibration data
void show_cal() {
  Serial.print("  cal_data = ");
  Serial.print(cal_data);
  Serial.print("\r\n\n");
}

// print info to serial port
void show_info() {
  show_version(SERIAL);
  // print band
  Serial.print("  band = ");
  Serial.println(band_label[radioband]);
  // print frequency
  Serial.print("  freq = ");
  Serial.print(vfofreq);
  Serial.print("\r\n");
  show_cal();
}

// show debug status
void show_debug() {
  DEBUG = ! DEBUG;
  Serial.print("DEBUG=");
  Serial.print(DEBUG);
  Serial.println("");
}

// millisecond delay
void wait_ms(uint16_t dly) {
  uint32_t startTime = msTimer;
  while((msTimer - startTime) < dly) {
    wait_us(10);
  }
}

// microsecond delay
void wait_us(uint16_t x) {
  uint16_t t = ((x * 3) + (x>>1));
  for (int16_t i=0; i<t; i++) {
    asm("");
  }
}

// blink the LED
void blinkLED() {
  digitalWrite(TXLED,ON);
  wait_ms(LED_BLINK);
  digitalWrite(TXLED,OFF);
}

// multiple LED blinks
void error_blink() {
  for (uint8_t i=0; i<2; i++) {
    digitalWrite(TXLED,ON);
    wait_ms(LED_BLINK);
    digitalWrite(TXLED,OFF);
    wait_ms(LED_BLINK);
  }
}

// set the location of the stepsize cursor
void stepsize_cursor() {
  switch (stepsize) {
    case STEP_1:    oled.setCursor(9,1); break;
    case STEP_10:   oled.setCursor(8,1); break;
    case STEP_100:  oled.setCursor(7,1); break;
    case STEP_1K:   oled.setCursor(5,1); break;
    case STEP_10K:  oled.setCursor(4,1); break;
    case STEP_100K: oled.setCursor(3,1); break;
    case STEP_1M:   oled.setCursor(1,1); break;
    default:        oled.setCursor(9,1); break;
  }
  oled.showCursor();
}

// print (11-bit) VFO frequency
inline void CAT_VFO() {
  if      (vfofreq >= 10000000) Serial.print("000");
  else if (vfofreq >=  1000000) Serial.print("0000");
  else                          Serial.print("00000");
  Serial.print(vfofreq);
}

// ==============================================================
// The following Kenwood TS-2000 CAT commands are implemented
//
// command get/set  name              operation
// ------- -------  ----------------  -----------------------
// IF        G -    radio status      returns frequency and other status
// ID        G -    radio ID          returns 019 = Kenwood TS-2000
// FA        G S    frequency         gets or sets the ADX frequency
// AI        G S    auto-information  returns 0   = OFF
// MD        G S    radio mode        returns 3   = CW
// PS        G S    power-on status   returns 1   = ON
// XT        G S    XIT status        returns 0   = OFF
// TX        - S    transmit          returns 0 and set TX LED
// RX        - S    receive           returns 0 and clears TX LED
//
// The following ADX-specific CAT commands are implemented
//
//  HE => print help
//  HH => print help
//  II => print info
//  DD => turn on/off debug
//  FR => factory reset
//  SR => soft reset
//  CM => calibration mode
// ==============================================================

// check for CAT control
void check_CAT() {
  if (Serial.available()) CAT_cmd();
  if (vfofreq != catfreq) {
    update_display();
  }
}

void CAT_cmd() {
  char cmd[3] = "zz";
  char param[20] = "";

  // get next alpha char from serial buffer
  char ch = gnac();
  if (ch == 'z') return;  // non-alpha char

  // get the command
  cmd[0] = ch;
  cmd[1] = getc();
  uppercase(cmd);

  // ===========================
  //  TS-2000 CAT commands
  // ===========================

  //====================================
  //  IF           // (command)       2
  //  00014074000  // P1 (VF0)       11
  //  0000         // P2 (step size)  4
  //  +00000       // P3 (rit)        6
  //  00000        // P4->P7          5
  //  0/1          // P8 (Tx/Rx)      1
  //  20000000     // P9->P15         8
  //                       TOTAL  =  37
  //====================================

  // get frequency and other status
  if (cmpstr(cmd, "IF")) {
    send("IF");
    CAT_VFO();
    Serial.print("00000+000000000");
    if (tx_status) Serial.print('1');
    else Serial.print('0');
    Serial.print("20000000;");
  }

  // get radio ID
  else if (cmpstr(cmd, "ID")) send("ID019;");

  // get or set frequency
  else if (cmpstr(cmd, "FA")) {
    ch = getc();
    if (numeric(ch)) {
      // set frequency
      catc(param, ch);
      for (uint8_t i=0; i<10; i++) {
        catc(param, getc());
      }
      catfreq = fs2int(param);
      getsemi(); // get semicolon
    } else {
      // get frequency
      Serial.print("FA");
      CAT_VFO();
      Serial.print(";");
    }
  }

  // get or set the radio mode
  else if (cmpstr(cmd, "MD")) {
    ch = getc();
    if (numeric(ch)) {
      // set radio mode
      // does nothing .. always 3
      getsemi();
    } else {
      // get status
      Serial.print("MD3;");
    }
  }

  // get or set auto-information status
  else if (cmpstr(cmd, "AI")) {
    ch = getc();
    if (numeric(ch)) {
      // set auto-information status
      // does nothing .. always 0
      getsemi();
    } else {
      // get status
      Serial.print("AI0;");
    }
  }

  // get or set the power (ON/OFF) status
  else if (cmpstr(cmd, "PS")) {
    ch = getc();
    if (numeric(ch)) {
      // set power (ON/OFF) status
      // does nothing .. always 1
      getsemi();
    } else {
      // get power (ON/OFF) status
      Serial.print("PS1;");
    }
  }

  // get or set the XIT (ON/OFF) status
  else if (cmpstr(cmd, "XT")) {
    ch = getc();
    if (numeric(ch)) {
      // set XIT (ON/OFF) status
      // does nothing .. always OFF
      getsemi();
    } else {
      // get XIT (ON/OFF) status
      Serial.print("XT0;");
    }
  }

  // CAT transmit
  else if (cmpstr(cmd, "TX")) {
    getsemi(); // get semicolon
    set_tx_status(ON);
  }

  // CAT receive
  else if (cmpstr(cmd, "RX")) {
    getsemi(); // get semicolon
    set_tx_status(OFF);
  }

  // ===========================
  //  ADX-specific CAT commands
  // ===========================

  // print help
  if (cmpstr(cmd, "HE")) {
    show_help();
  }

  // print help
  else if (cmpstr(cmd, "HH")) {
    show_help();
  }

  // toggle debug on/off
  else if (cmpstr(cmd, "DD")) {
    show_debug();
  }

  // print info
  else if (cmpstr(cmd, "II")) {
    show_info();
  }

  // factory reset
  else if (cmpstr(cmd, "FR")) {
    do_reset(FACTORY);
    update_display();
  }

  // soft reset
  else if (cmpstr(cmd, "SR")) {
    do_reset(SOFT);
    update_display();
  }

  // calibrate mode
  else if (cmpstr(cmd, "CM")) {
    run_calibrate();
  }

}

// write config data to the eeprom
void save_eeprom() {
  Serial.print("  Saving to EEPROM\r\n");
  eeprom.put32(DATA_ADDR, cal_data);
  eeprom.put32(FREQ_ADDR, vfofreq);
}

// initialize the Si5351 VFO clocks
void init_VFO() {
  si5351.init();
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.output_enable(SI5351_CLK0, OFF);   // Tx off
  si5351.output_enable(SI5351_CLK1, OFF);   // Rx off
  si5351.output_enable(SI5351_CLK2, OFF);   // Cal off
  // clock drive strength
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
}

#define DITCONST  1200   // dit time constant

// initialize the keyer speed
void init_wpm() {
  dittime    = DITCONST/keyerwpm;
  dahtime    = (DITCONST * 3)/keyerwpm;
  lettergap1 = (DITCONST * 2.5)/keyerwpm;
  lettergap2 = (DITCONST * 3)/keyerwpm;
  wordgap1   = (DITCONST * 3)/keyerwpm;
  wordgap2   = (DITCONST * 7)/keyerwpm;
}

// initialize the Si5351 frequency
void init_freq() {
  si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
  si5351.set_freq(vfofreq*100, SI5351_CLK0);
}

#define T0CTC      0x02   // CTC mode
#define T064PRE    0x03   // prescale by 64
#define T0ON       0x02   // interrupt on

// initialize timer 0
void init_timer0() {
  TCCR0A = T0CTC;         // count mode
  OCR0A  = 249;           // 1 ms count value
  TCCR0B = T064PRE;       // set prescaler
  TIMSK0 = T0ON;          // start timer 0
}

#define T1ON       0x82   // OC1A PWM on
#define T1OFF      0x00   // PWM off
#define T1PRE      0x19   // prescale by  1
#define T1ICR      0xff   // input capture register

// initialize timer1
void init_timer1() {
  TCCR1A = T1OFF;       // PWM off
  TCCR1B = T1PRE;       // set prescaler
  ICR1H  = 0x00;        // top high
  ICR1L  = T1ICR;       // top low
  OCR1AH = 0x00;        // OC1A PWM init
  OCR1AL = 0x80;        // to 50% duty
  OCR1BH = 0x00;        // OC1B is
  OCR1BL = 0x00;        // not used
  TCCR1A = T1ON;        // PWM on
}

// timer 1 start
void start_timer1() {
  TCCR1A = T1ON;
}

// timer 1 stop
void stop_timer1() {
  TCCR1A = T1OFF;
}

#define T2ON       0x02   // interrupt on
#define T2OFF      0x00   // interrupt off
#define T2NOCLK    0x00   // no clock
#define T2CTC      0x02   // count mode
#define T2PRE      0x07   // prescale by 1024

// timer 2 init           // 15 kHz
void init_timer2() {
  TIMSK2 = T2OFF;         // interrupt off
  TCCR2B = T2NOCLK;       // no clock
  OCR2A  = 100;           // output compare
  TCCR2A = T2CTC;         // count mode
  TCCR2B = T2PRE;         // set prescaler
  TIMSK2 = T2ON;          // interrupt on
}

// timer 2 start
void start_timer2() {
  TIMSK2 = T2ON;
}

// timer 2 stop
void stop_timer2() {
  TIMSK2 = T2OFF;
}

// initialize pins
void init_pins() {
  pinMode(KEYOUT,   OUTPUT);
  pinMode(SIDETONE, OUTPUT);
  pinMode(TXLED,    OUTPUT);
  pinMode(TXD,      OUTPUT);
  pinMode(VBATT, INPUT);
  pinMode(ROTA,  INPUT_PULLUP);
  pinMode(ROTB,  INPUT_PULLUP);
  pinMode(DIT,   INPUT_PULLUP);
  pinMode(DAH,   INPUT_PULLUP);
  pinMode(SW1,   INPUT_PULLUP);
  pinMode(SW2,   INPUT_PULLUP);
  pinMode(RXD,   INPUT_PULLUP);
}

// initialize the I2C bus
void init_i2c() {
  i2c.begin();
}

// initialize the serial port
void init_uart() {
  #define BAUDRATE  115200
  Serial.begin(BAUDRATE);
}

// initialize the OLED
void init_oled() {
  oled.begin();
  display = ON;
  oled.onDisplay();
  set_oled_timeout();
}

// check for factory reset during setup
void init_check() {
  if (ANY_PRESSED) do_reset(FACTORY);
  else do_reset(SOFT);
  show_version(BOTH);
}

// rotary encoder init
void init_encoder() {
  // interrupt-enable for ROTA, ROTB pin changes
  PCMSK2 = (1 << PCINT20) | (1 << PCINT18);
  PCICR  = (1 << PCIE2);
  enc_a = digitalRead(ROTA);
  enc_b = digitalRead(ROTB);
  enc_state = (enc_b << 1) | enc_a;
  interrupts();
}

// initialize the ADC
void init_adc() {
  analogReference(INTERNAL);
}

// Minsky sin/cos calculations
void minsky() {
  // delta = f_tone * 2 * pi / fs
  // delta = f_tone * 628 / F_SAMP_TX;
  uint8_t delta;
  switch (cwtone) {
    case 1:
      delta = 36;
      break;
    default:
      delta = 31;
  }
  msin += (delta * mcos) >> 7;
  mcos -= (delta * msin) >> 7;
}

float fpv = 0.0;

// measure battery voltage
void read_adc() {
  uint16_t val;
  val = analogRead(VBATT);
  fpv = ((float)val * 14.1) / 1024.0;
}

// oled timeout
uint32_t oled_timeout;

// set oled timeout
void set_oled_timeout() {
  switch (dxblank) {
    case 0:
      oled_timeout = OFF;
      break;
    case 1:
      oled_timeout = FIVE_MINUTES;
      break;
    case 2:
      oled_timeout = HALF_HOUR;
      break;
    default:
      break;
  }
}

// set the Rx/Tx status
void set_tx_status(uint8_t tx) {
  if (tx) {
    tx_status = ON;
    digitalWrite(TXLED,  ON);
    digitalWrite(KEYOUT, ON);
    si5351.output_enable(SI5351_CLK0, ON);   // Tx on
//  si5351.output_enable(SI5351_CLK1, OFF);  // Rx off
  } else {
    tx_status = OFF;
    digitalWrite(TXLED, OFF);
    digitalWrite(KEYOUT,OFF);
    si5351.output_enable(SI5351_CLK0, OFF);  // Tx off
//  si5351.output_enable(SI5351_CLK1, ON);   // Rx on
  }
}

// frequency to band
void freq2band(uint32_t freq) {
  if ((freq > 50000000) && (freq < 54000000)) {
    radioband = BAND_06M;
  } else if ((freq > 28000000) && (freq < 29700000)) {
    radioband = BAND_10M;
  } else if ((freq > 24890000) && (freq < 24990000)) {
    radioband = BAND_12M;
  } else if ((freq > 21000000) && (freq < 21450000)) {
    radioband = BAND_15M;
  } else if ((freq > 18070000) && (freq < 18170000)) {
    radioband = BAND_17M;
  } else if ((freq > 14000000) && (freq < 14350000)) {
    radioband = BAND_20M;
  } else if ((freq > 10100000) && (freq < 10150000)) {
    radioband = BAND_30M;
  } else if ((freq >  7000000) && (freq <  7300000)) {
    radioband = BAND_40M;
  } else if ((freq >  5300000) && (freq <  5500000)) {
    radioband = BAND_60M;
  } else if ((freq >  3500000) && (freq <  4000000)) {
    radioband = BAND_80M;
  } else {
    radioband = UNKNOWN;
  }
}

// update display with band and vfo frequency
void update_display() {
  freq2band(vfofreq);
  si5351.set_freq(vfofreq*100, SI5351_CLK1);
  oled.printline(0, band_label[radioband]);
  if (keyermode) {
    oled.setCursor(9,0);
    switch (keyermode) {
      case IAMBICA:
        oled.putstr("IA");
        break;
      case IAMBICB:
        oled.putstr("IB");
        break;
      case ULTIMATIC:
        oled.putstr("UM");
        break;
      default:
        break;
    }
    oled.setCursor(12,0);
    oled.print8(keyerwpm);
  }
  oled.print32(vfofreq);
  stepsize_cursor();
}

// check for display timeout
void check_timeout() {
  if (!dxblank) return;  // skip if disabled
  if ((display == ON) && ((msTimer - xtimer) > oled_timeout)) {
    display = OFF;
    oled.noDisplay();
  }
}

// check the user interface
void check_UI() {
  uint8_t event = NBP;
  uint32_t t0 = msTimer;
  // check for wakeup
  if (display == OFF) {
    // check for wake-up
    if (ANY_PRESSED) {
      reset_xtimer();
      while (ANY_PRESSED) wait_ms(DEBOUNCE);
      return;
    }
  }
  // check for key/paddle input
  if (keyermode) iambic_keyer();
  else straight_key();
  // check for pushbutton input
  if (SW1_PRESSED) {
    reset_xtimer();
    // SW1 button click controls the menu system
    // SW1 button has no long-press function
    switch (menumode) {
      case NOT_IN_MENU:
        // enter menu selection mode
        menumode = SELECT_MENU;
        oled.clrScreen();
        enc_locked = NO;
        break;
      case SELECT_MENU:
        // enter value selection mode
        menumode = SELECT_VALUE;
        break;
      case SELECT_VALUE:
        // enter menu selection mode
        menumode = SELECT_MENU;
        break;
      default:
        menumode = NOT_IN_MENU;
        break;
    }
    menuAction(menu);
    // wait for SW1 released
    while (SW1_PRESSED) wait_ms(DEBOUNCE);
  } else if (SW2_PRESSED) {
    reset_xtimer();
    // SW2 button click when in menu mode will exit the menu
    // SW2 button click when not in menu mode will update the step size
    // SW2 button long press changes bands with encoder
    event = BSC;
    if (menumode) {
      exit_menu();
      // wait for SW2 released
      while (SW2_PRESSED) wait_ms(DEBOUNCE);
    } else {
      while (SW2_PRESSED) {
        // check for long press
        if ((msTimer - t0) > LONGPRESS) { event = BPL; break; }
        wait_ms(DEBOUNCE);
      }
      if (event == BPL) {
        // during an SW2 long press
        // use the encoder to update the radio band
        oled.clrScreen();
        menumode = SELECT_VALUE;
        menuAction(RADIOBAND);
        while (SW2_PRESSED) {
          if (enc_val) menuAction(RADIOBAND);
          wait_ms(DEBOUNCE);
        }
      } else {
        // SW2 click updates the step size
        stepsize--;
        if (stepsize < STEP_1)  stepsize = STEP_1M;
        stepsize_cursor();
        wait_ms(100);  // more debounce
      }
      exit_menu();
    }
  } else {
    // no buttons are pressed
    // use the encoder to update the VFO
    if (enc_val && !menumode) update_vfo();
    check_timeout();   // check for display timeout
  }
}

// check the menu state
void check_menu() {
  switch (menumode) {
    case NOT_IN_MENU:
      if (save2ee) {
        save_eeprom();
        save2ee = OFF;
        calibrate = OFF;
      }
      if (calibrate) {
        run_calibrate();
        calibrate = OFF;
      }
      break;
    case SELECT_MENU:
      if (enc_val) {
        menu += enc_val;
        // check menu for lower and upper limits
        // and wrap around at limits
        if (menu < FIRSTMENU) menu = LASTMENU;
        else if (menu > LASTMENU) menu = FIRSTMENU;
        menuAction(menu);
      }
      break;
    case SELECT_VALUE:
      if (enc_val) menuAction(menu);
      break;
    default:
      break;
  }
  enc_val = 0;
}

// exit menu and update display
void exit_menu() {
  menumode = NOT_IN_MENU;
  enc_val = 0;
  update_display();
}

// menu actions
void menuAction(uint8_t id) {

  switch (id) {
    //   id                          variable    label         min max
    //   ---------                   --------    -------       --- ---
    case VOLUME:     paramAction(id, &volume,    NULL,         0,   6); break;
    case KEYERWPM:   paramAction(id, &keyerwpm,  NULL,         10, 60); break;
    case RADIOBAND:  paramAction(id, &radioband, band_label,   0,  10); break;
    case CWTONE:     paramAction(id, &cwtone,    cwtone_label, 0,   1); break;
    case DXBLANK:    paramAction(id, &dxblank,   dxbk_label,   0,   2); break;
    case CALIBRATE:  paramAction(id, &calibrate, onoff_label,  0,   1); break;
    case SAVE2EE:    paramAction(id, &save2ee,   onoff_label,  0,   1); break;
    case KEYERMODE:  paramAction(id, &keyermode, keyer_label,  0,   3); break;
    case KEYSWAP:    paramAction(id, &keyswap,   onoff_label,  0,   1); break;
    case SWVER:      paramAction(id, NULL,       NULL,         0,   1); break;

    default: break;
  }
}

// parameters actions
void paramAction(uint8_t id, uint8_t* ptr, const char* sap[], uint8_t min, uint8_t max) {
  uint8_t value = *ptr;
  int16_t newvalue;
  switch (menumode) {
    case SELECT_MENU:
      show_label(id);              // show menu label
      show_value(id, value, sap);  // show menu value
      break;
    case SELECT_VALUE:
      show_label(id);              // show menu label
      // read encoder and update value
      newvalue = value + enc_val;
      // check min and max value limits
      if (newvalue < min) value = min;
      else if (newvalue > max) value = max;
      else value = newvalue;
      enc_val = 0;
      *ptr = value;
      show_value(id, value, sap);
      // parameter-specific actions
      switch (id) {
        case RADIOBAND:
          if (radioband != prevband) {
            prevband = radioband;
            vfofreq = bandfreq[radioband];
            catfreq = vfofreq;
          }
          break;
        case KEYERWPM:
          init_wpm();
          break;
        case DXBLANK:
          set_oled_timeout();
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

// print a menu label
void show_label(uint8_t id) {
  char tmp[16];
  cpystr(tmp, menulabel[id]);
  if ((menumode == SELECT_VALUE) && (menu != SWVER)) {
    catstr(tmp, " >");
  }
  oled.printline(0, tmp);
}

// print a menu value field
void show_value (uint8_t id, uint8_t val, const char* sap[]) {
  switch (id) {
    case SWVER:
      oled.printline(1, VERSION);
      break;
    default:
      if (sap == NULL) {
        oled.setCursor(0,1);
        oled.print8(val);
      } else {
        oled.printline(1, (char *)sap[val]);
      }
  }
}

// update the displayed frequency
void update_vfo() {
  int32_t stepval = stepsizes[stepsize];
  vfofreq += enc_val * stepval;
  catfreq = vfofreq;
  enc_val = 0;
  update_display();
}

// reset display timeout
void reset_xtimer() {
  xtimer = msTimer;
  if (display == OFF) {
    display = ON;
    oled.onDisplay();
  }
}

// reset (CAT command)
void do_reset(uint8_t soft) {
  oled.clrScreen();
  // check for unitialized eeprom
  Serial.print("  Reading EEPROM\r\n");
  vfofreq  = eeprom.get32(FREQ_ADDR);
  cal_data = eeprom.get32(DATA_ADDR);
  freq2band(vfofreq);
  if ((radioband == UNKNOWN) || (cal_data > CAL_DATA_MAX)) {
    soft = 0;
  }
  if (soft) {
    // soft reset
    Serial.print("  Soft Reset\r\n");
  } else {
    // factory reset
    oled.putstr("FACTORY RESET");
    Serial.print("  Factory Reset\r\n");
    wait_ms(ONE_SECOND);
    cal_data = CAL_DATA_INIT;
    vfofreq  = INITVFO;
    freq2band(vfofreq);
    save_eeprom();
  }
  catfreq  = vfofreq;
  si5351.set_freq(vfofreq*100, SI5351_CLK1);
  show_cal();
  set_tx_status(OFF);
}

#define CAL_FREQ  100000000ULL

#define CAL_MSG  "\r\n\
  Adjust cal freq to 1 MHz\r\n\
  press + to increase cal freq\r\n\
  press - to decrease cal freq\r\n\
  press = to stop\r\n\
  press . to save and exit\r\n\
  press / to exit without saving\r\n\n"

// calibrate the VFO (CAT command)
void run_calibrate() {
  char ch;
  uint8_t up = FALSE;
  uint8_t dn = FALSE;
  uint8_t xx = 0;
  uint8_t done = NO;
  uint8_t save = YES;
  reset_xtimer();
  // print to serial port
  Serial.print(CAL_MSG);
  // print to OLED
  oled.clrScreen();
  oled.putstr("CALIBRATION MODE");
  // update the VFO
  si5351.output_enable(SI5351_CLK0, OFF);  // Tx off
  si5351.output_enable(SI5351_CLK1, OFF);  // Rx off
  si5351.set_freq(CAL_FREQ, SI5351_CLK2);
  si5351.set_clock_pwr(SI5351_CLK2, ON);
  si5351.output_enable(SI5351_CLK2, ON);
  ch = getc();
  while (!done) {
    switch (ch) {
      case '+':      // increment
        up = TRUE;
        dn = FALSE;
        break;
      case '-':      // decrement
        up = FALSE;
        dn = TRUE;
        break;
      case '=':      // stop
        up = FALSE;
        dn = FALSE;
        break;
      case '/':      // exit without saving
      case '\\':
        save = NO;
      case '.':      // exit and save
        done = TRUE;
        up = FALSE;
        dn = FALSE;
        break;
      default:
        break;
    }
    if (up||dn) {
      if (up) cal_data = cal_data - 10;
      if (dn) cal_data = cal_data + 10;
      si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
      wait_us(100);
      si5351.set_freq(CAL_FREQ, SI5351_CLK2);
      if (xx == 0) Serial.print(ch);
      if (xx++ == 100) xx = 0;
    }
    ch = gcal(ch);
  }
  si5351.output_enable(SI5351_CLK2, OFF);
  si5351.set_clock_pwr(SI5351_CLK2, OFF);
  // print to serial port
  Serial.print("\r\n\  Exiting Calibration Mode\r\n");
  show_cal();
  // print to OLED
  oled.printline(0,"CAL COMPLETE");
  if (save) {
    Serial.print("  Saving to EEPROM\r\n");
    eeprom.put32(DATA_ADDR, cal_data);
  }
  wait_ms(TWO_SECONDS);
  update_display();
}

// table lookup for CW decoder
char lookup_cw(uint8_t addr) {
  char ch = '*';
  if (addr < 128) ch = pgm_read_byte(m2a + addr);
  return ch;
}

uint8_t cwcol = 0;
uint8_t cwrow = 2;

// convert morse to ascii and print
void print_cw() {
  oled.setCursor(cwcol,cwrow);
  char ch = lookup_cw(maddr);
  switch (maddr) {
    case 0xc5:
      // oled.putstr("<BK>");
      break;
    case 0x45:
      // oled.putstr("<SK>");
      break;
    default:
      // clear screen if 8-dit code is received
      if (!maddr) {
        oled.clrLine(2);
        oled.clrLine(3);
        cwcol = 0;
        cwrow = 2;
      } else {
        oled.putch(ch);
        cwcol++;
        if (cwcol > 15) {
          cwcol = 0;
          if (cwrow == 2) {
            cwrow = 3;
            oled.clrLine(3);
          } else {
            cwrow = 2;
            oled.clrLine(2);
          }
        }
      }
      if (recordMsg) catc(tmpstr, ch);
      break;
  }
}

// update the morse code table address
void maddr_cmd(uint8_t cmd) {
  if (cmd == 2) {
    // print the translated ascii
    // and reset the table address
    print_cw();
    maddr = 1;
  }
  else {
    // send dit/dah
    set_tx_status(ON);
    // update the table address
    maddr = maddr<<1;
    maddr |= cmd;
  }
}

// An Iambic (mode A/B) morse code keyer
// Copyright 2021 Scott Baker KJ7NLA

// keyerinfo bit definitions
#define DIT_REG     0x01     // dit pressed
#define DAH_REG     0x02     // dah pressed
#define KEY_REG     0x03     // dit or dah pressed
#define WAS_DIT     0x04     // last key was dit
#define SQUEEZE     0x08     // both dit and dah pressed

// keyerstate machine states
#define KEY_IDLE    0
#define CHK_KEY     1
#define KEY_WAIT    2
#define IDD_WAIT    3
#define LTR_GAP     4
#define WORD_GAP    5

// more key info
#define GOTDIT  (keyerinfo & DIT_REG)
#define GOTDAH  (keyerinfo & DAH_REG)
#define GOTKEY  (keyerinfo & KEY_REG)
#define NOKEY   !GOTKEY
#define GOTBOTH  GOTKEY == KEY_REG

uint8_t  keyerstate = KEY_IDLE;
uint8_t  keyerinfo  = 0;

// read and debounce paddles
void read_paddles() {
  uint8_t ditv1 = !digitalRead(DIT);
  uint8_t dahv1 = !digitalRead(DAH);
  uint8_t ditv2 = !digitalRead(DIT);
  uint8_t dahv2 = !digitalRead(DAH);
  if (ditv1 && ditv2) {
    if (keyswap) keyerinfo |= DAH_REG;
    else keyerinfo |= DIT_REG;
  }
  if (dahv1 && dahv2) {
    if (keyswap) keyerinfo |= DIT_REG;
    else keyerinfo |= DAH_REG;
  }
  if (GOTBOTH) keyerinfo |= SQUEEZE;
  else keyerinfo &= ~SQUEEZE;
  if (GOTKEY) reset_xtimer();
}

// iambic keyer state machine
void iambic_keyer() {
  static uint32_t ktimer;
  uint8_t send_dit = NO;
  uint8_t send_dah = NO;
  switch (keyerstate) {
    case KEY_IDLE:
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
      } else {
        keyerinfo = 0;
      }
      break;
    case CHK_KEY:
      if (keyerinfo & SQUEEZE) {
        if (keyermode == ULTIMATIC) {
          if (keyerinfo & WAS_DIT) send_dah = YES;
          else send_dit = YES;
        } else {
          // Iambic A or B
          if (keyerinfo & WAS_DIT) {
            keyerinfo &= ~WAS_DIT;
            send_dah = YES;
          } else {
            keyerinfo |= WAS_DIT;
            send_dit = YES;
          }
        }
      } else {
        if (GOTDIT) {
          keyerinfo |= WAS_DIT;
          send_dit = YES;
        }
        if (GOTDAH) {
          keyerinfo &= ~WAS_DIT;
          send_dah = YES;
        }
      }
      if (send_dit) {
        ktimer = msTimer + dittime;
        maddr_cmd(0);
        keyerstate = KEY_WAIT;
      }
      else if (send_dah) {
        ktimer = msTimer + dahtime;
        maddr_cmd(1);
        keyerstate = KEY_WAIT;
      }
      else keyerstate = KEY_IDLE;
      break;
    case KEY_WAIT:
      // wait dit/dah duration
      if (msTimer > ktimer) {
        // done sending dit/dah
        set_tx_status(OFF);
        // inter-symbol time is 1 dit
        ktimer = msTimer + dittime;
        keyerstate = IDD_WAIT;
      }
      break;
    case IDD_WAIT:
      // wait time between dit/dah
      if (msTimer > ktimer) {
        // wait done
        keyerinfo &= ~KEY_REG;
        if ((keyermode == IAMBICA) || (keyermode == ULTIMATIC)) {
          // Iambic A or Ultimatic
          // check for letter space
          ktimer = msTimer + lettergap1;
          keyerstate = LTR_GAP;
        } else {
          // Iambic B
          read_paddles();
          if (NOKEY && (keyerinfo & SQUEEZE)) {
            // send opposite of last paddle sent
            if (keyerinfo & WAS_DIT) {
              // send a dah
              ktimer = msTimer + dahtime;
              maddr_cmd(1);
            }
            else {
              // send a dit
              ktimer = msTimer + dittime;
              maddr_cmd(0);
            }
            keyerinfo = 0;
            keyerstate = KEY_WAIT;
          } else {
            // check for letter space
            ktimer = msTimer + lettergap1;
            keyerstate = LTR_GAP;
          }
        }
      }
      break;
    case LTR_GAP:
      if (msTimer > ktimer) {
        // letter space found so print char
        maddr_cmd(2);
        // check for word space
        ktimer = msTimer + wordgap1;
        keyerstate = WORD_GAP;
      }
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
      } else {
        keyerinfo = 0;
      }
      break;
    case WORD_GAP:
      if (msTimer > ktimer) {
        // word gap found so print a space
        maddr = 1;
        print_cw();
        keyerstate = KEY_IDLE;
      }
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
      } else {
        keyerinfo = 0;
      }
      break;
    default:
      break;
  }
}

// handle straight key mode
void straight_key() {
  keyerinfo = 0;
  read_paddles();
  if (GOTDIT) {
    reset_xtimer();
    set_tx_status(ON);
  } else {
    set_tx_status(OFF);
  }
}

// main code starts here
int main() {

  // setup
  init();
  init_pins();
  init_timer0();
  init_timer1();
  init_timer2();
  init_uart();
  init_adc();
  init_i2c();
  init_VFO();
  init_encoder();
  init_check();
  init_oled();
  init_wpm();
  init_freq();
  update_display();

  // main loop
  while (TRUE) {
    check_CAT();      // check CAT interface
    check_UI();       // check UI pushbutton
    check_menu();     // check for menu ops
  }
  return 0;
}

