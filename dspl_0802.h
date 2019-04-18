#ifndef _DSPL0802_H
#define _DSPL0802_H
#include "config.h"
#include <LiquidCrystal.h>

#ifdef DISPLAY_0802

// The LCD 0802 parallel interface
const uint8_t  LCD_RS_PIN     = 13;
const uint8_t  LCD_E_PIN      = 12;
const uint8_t  LCD_DB4_PIN    = 5;
const uint8_t  LCD_DB5_PIN    = 6;
const uint8_t  LCD_DB6_PIN    = 7;
const uint8_t  LCD_DB7_PIN    = 8;

//------------------------------------------ class lcd DSPLay for soldering IRON -----------------------------
class DSPL : protected LiquidCrystal {
  public:
    DSPL(void) : LiquidCrystal(LCD_RS_PIN, LCD_E_PIN, LCD_DB4_PIN, LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN) { }
    void init(void);
    void clear(void) { LiquidCrystal::clear(); }
    void tip(uint8_t  index, bool top);        	// Show the current tip (on top line)
    void tSet(uint16_t t, bool celsuis);        // Show the temperature set
    void tCurr(uint16_t t);                     // Show The current temperature
    void pSet(uint8_t  p);                      // Show the power set
    void timeToOff(uint8_t  sec);               // Show the time to automatic off the IRON
    void msgReady(void);                        // Show 'Ready' message
    void msgWorking(void);                      // Show 'Working' message
    void msgOn(void);                           // Show 'On' message
    void msgOff(void);                          // Show 'Off' message
    void msgCold(void);                         // Show 'Cold' message
    void msgFail(void);                         // Show 'Fail' message
    void msgTune(void);                         // Show 'Tune' message
    void msgCelsius(void);                      // Show 'Celsius' message
    void msgFarneheit(void);                    // Show 'Fahren.' message
    void msgDefault();                          // Show 'default' message (load default configuration)
    void msgCancel(void);                       // Show 'cancel' message
    void msgApply(void);                        // Show 'save' message
    void msgSelectTip(void);                    // Show 'tip:' message
    void setupMode(uint8_t  mode, uint8_t  p = 0);      // Show the configuration mode [0 - 3]
    void percent(uint8_t  Power);               // Show the percentage
    void calibrated(bool calibrated);           // Show '*' if the tip was not calibrated
  private:
    bool full_second_line;                      // Whether the second line is full with the message
};

#endif
#endif
