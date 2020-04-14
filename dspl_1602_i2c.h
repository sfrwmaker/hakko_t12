#ifndef _DSPL1602_I2C_H
#define _DSPL1602_I2C_H
#include "config.h"
#include <LiquidCrystal_I2C.h>

#ifdef DISPLAY_1602_I2C

// The LCD 1602 I2C interface
const uint8_t  LCD_I2C_ADDR   = 0x27;

//------------------------------------------ class lcd DSPLay for soldering IRON -----------------------------
class DSPL : protected LiquidCrystal_I2C {
    public:
        DSPL(void) : LiquidCrystal_I2C(0x27, 16, 2) { }
        void    init(void);
        void    clear(void)                         { LiquidCrystal_I2C::clear(); }
        void    tip(const char *tip_name, bool top);// Show the current tip (on top line)
        void    tSet(uint16_t t, bool celsuis);     // Show the temperature set
        void    tCurr(uint16_t t);                  // Show The current temperature
        void    pSet(uint8_t  p);                   // Show the power set
	    void    tRef(uint8_t ref);					// Show the reference temperature index
        void    timeToOff(uint8_t  sec);            // Show the time to automatic off the IRON
        void    msgReady(void);                     // Show 'Ready' message
        void    msgWorking(void);                   // Show 'Working' message
        void    msgOn(void);                        // Show 'On' message
        void    msgOff(void);                       // Show 'Off' message
        void    msgStandby(void);                   // Show 'Standby' message
        void    msgCold(void);                      // Show 'Cold' message
        void    msgFail(void);                      // Show 'Fail' message
        void    msgTune(void);                      // Show 'Tune' message
        void    msgCelsius(void);                   // Show 'Celsius' message
        void    msgFarneheit(void);                 // Show 'Fahren.' message
        void    msgDefault();                       // Show 'default' message (load default configuration)
        void    msgCancel(void);                    // Show 'cancel' message
        void    msgApply(void);                     // Show 'save' message
        void    msgSelectTip(void);                 // Show 'tip:' message
        void DSPL::msgActivateTip(void);            // Show 'activate tip' message
        void    setupMode(uint8_t  mode, uint16_t  p = 0); // Show the configuration mode [0 - 11]
        void    percent(uint8_t  Power);            // Show the percentage
        void    calibrated(bool calibrated);        // Show '*' if the tip was not calibrated
    private:
        bool full_second_line;                      // Whether the second line is full with the message
};

#endif
#endif
