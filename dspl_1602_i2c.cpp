#include "config.h"
#include "dspl_1602_i2c.h"

#ifdef DISPLAY_1602_I2C

void DSPL::init(void) {
    LiquidCrystal_I2C::begin();
    LiquidCrystal_I2C::clear();
    full_second_line = false;
}

void DSPL::tip(const char *tip_name, bool top) {
    uint8_t  y = 1; if (top) y = 0;
    LiquidCrystal_I2C::setCursor(0, y);
    LiquidCrystal_I2C::print(tip_name);
        for (uint8_t i = strlen(tip_name); i < 16; ++i)
        LiquidCrystal_I2C::print(' ');
}

void DSPL::msgSelectTip(void) {
    LiquidCrystal_I2C::setCursor(0, 0);
    LiquidCrystal_I2C::print(F("iron tip        "));
}

void DSPL::msgSelectTip(void) {
    LiquidCrystal_I2C::setCursor(0, 0);
    LiquidCrystal_I2C::print(F("activate tip    "));
}

void DSPL::tSet(uint16_t t, bool celsius) {
    char buff[5];
    char units = 'C';
    if (!celsius) units = 'F';
    LiquidCrystal_I2C::setCursor(0, 0);
    sprintf(buff, "%3d%c", t, units);
    LiquidCrystal_I2C::print(buff);
}

void DSPL::tCurr(uint16_t t) {
    char buff[4];
    LiquidCrystal_I2C::setCursor(0, 1);
    if (t < 1000) {
        sprintf(buff, "%3d", t);
    } else {
        LiquidCrystal_I2C::print(F("xxx"));
        return;
    }
    LiquidCrystal_I2C::print(buff);
    if (full_second_line) {
        LiquidCrystal_I2C::print(F("            "));
        full_second_line = false;
    }
}

void DSPL::pSet(uint8_t  p) {
    char buff[6];
    sprintf(buff, "P:%3d", p);
    LiquidCrystal_I2C::setCursor(0, 0);
    LiquidCrystal_I2C::print(buff);
}

void DSPL::tRef(uint8_t ref) {
    char buff[9];
    LiquidCrystal::setCursor(0, 0);
    sprintf(buff, "Ref. #%1d", ref+1);
    LiquidCrystal::print(buff);
}

void DSPL::timeToOff(uint8_t  sec) {
    char buff[5];
    sprintf(buff, " %3d", sec);
    LiquidCrystal_I2C::setCursor(4, 0);
    LiquidCrystal_I2C::print(buff);
    LiquidCrystal_I2C::print(F("        "));
}

void DSPL::msgReady(void) {
    LiquidCrystal_I2C::setCursor(4, 0);
    LiquidCrystal_I2C::print(F("       ready"));
}

void DSPL::msgWorking(void) {
    LiquidCrystal_I2C::setCursor(4, 0);
    LiquidCrystal_I2C::print(F("     working"));
}

void DSPL::msgOn(void) {
    LiquidCrystal_I2C::setCursor(4, 0);
    LiquidCrystal_I2C::print(F("         ON"));
}

void DSPL::msgOff(void) {
    LiquidCrystal_I2C::setCursor(4, 0);
    LiquidCrystal_I2C::print(F("        OFF"));
}

void DSPL::msgStandby(void) {
    LiquidCrystal_I2C::setCursor(4, 0);
    LiquidCrystal_I2C::print(F("    stabdby"));
}

void DSPL::msgCold(void) {
    LiquidCrystal_I2C::setCursor(0, 1);
    LiquidCrystal_I2C::print(F("       cold"));
    full_second_line = true;
}

void DSPL::msgFail(void) {
    LiquidCrystal_I2C::setCursor(0, 1);
    LiquidCrystal_I2C::print(F("   Failed  "));
}

void DSPL::msgTune(void) {
    LiquidCrystal_I2C::setCursor(0, 0);
    LiquidCrystal_I2C::print(F("Tune"));
}

void DSPL::msgCelsius(void) {
    LiquidCrystal_I2C::setCursor(0, 1);
    LiquidCrystal_I2C::print(F("Celsius         "));
}

void DSPL::msgFarneheit(void) {
    LiquidCrystal_I2C::setCursor(0, 1);
    LiquidCrystal_I2C::print(F("Fahrenheit      "));
}

void DSPL::msgDefault() {
    LiquidCrystal_I2C::setCursor(0, 1);
    LiquidCrystal_I2C::print(F(" default        "));
}

void DSPL::setupMode(uint8_t mode, bool tune, uint16_t p) {
    char buff[5];
    LiquidCrystal::clear();
    if (!tune) {
        LiquidCrystal::print(F("setup"));
        LiquidCrystal::setCursor(1,1);
    }
    switch (mode) {
        case 0:                                 // C/F. In-line editing
            LiquidCrystal::print(F("units"));
            LiquidCrystal::setCursor(7, 1);
            if (p)
                LiquidCrystal::print("C");
            else
                LiquidCrystal::print("F");
            break;
        case 1:                                 // Buzzer
            LiquidCrystal::print(F("buzzer"));
            if (tune) {
                LiquidCrystal::setCursor(1,1);
                if (p)
                    LiquidCrystal::print(F("ON"));
                else
                    LiquidCrystal::print(F("OFF"));
            }
            break;
        case 2:                                 // Switch type
            LiquidCrystal::print(F("switch"));
            if (tune) {
                LiquidCrystal::setCursor(1,1);
                if (p)
                    LiquidCrystal::print(F("REED"));
                else
                    LiquidCrystal::print(F("TILT"));
            }
            break;
        case 3:                                 // ambient temperatyre sensor
            LiquidCrystal::print(F("ambient"));
            if (tune) {
                LiquidCrystal::setCursor(1,1);
                if (p)
                    LiquidCrystal::print(F("ON"));
                else
                    LiquidCrystal::print(F("OFF"));
            }
            break;
        case 4:                                 // standby temperature
            LiquidCrystal::print(F("stby Temp"));
            if (tune) {
                LiquidCrystal::setCursor(1,1);
                if (p > 0) {
                    sprintf(buff, "%3d", p);
                    LiquidCrystal::print(buff);
                } else {
                    LiquidCrystal::print(" NO");
                }   
            }
            break;
        case 5:                                 // Standby Time
            LiquidCrystal::print(F("stby time"));
            if (tune) {
                LiquidCrystal::setCursor(1,1);
                sprintf(buff, "%3ds", p);
                LiquidCrystal::print(buff);
            }
            break;
        case 6:                                 // off-timeout
            LiquidCrystal::print(F("off"));
            if (tune) {
                LiquidCrystal::setCursor(1,1);
                if (p > 0) {
                    sprintf(buff, "%2dm", p);
                    LiquidCrystal::print(buff);
                } else {
                    LiquidCrystal::print(" NO");
                }   
            }
            break;
        case 7:                                 // Tip calibrage
            LiquidCrystal::print(F("tip config."));
            break;
        case 8:                                 // Activate tip
            LiquidCrystal::print(F("activate tip"));
            break;
        case 9:                                 // Tune controller
            LiquidCrystal::print(F("tune"));
            break;
        case 10:                                // save
            LiquidCrystal::print(F("apply"));
            break;
        case 11:                                // cancel
            LiquidCrystal::print(F("cancel"));
        default:
            break;
    }
}

void DSPL::percent(uint8_t  Power) {
    char buff[6];
    sprintf(buff, " %3d%c", Power, '%');
    LiquidCrystal_I2C::setCursor(3, 1);
    LiquidCrystal_I2C::print(buff);
}

void DSPL::calibrated(bool calibrated) {
    char buff[5];
    for (uint8_t  i = 0; i < 4; ++i) buff[i] = ' ';
    if (!calibrated) buff[3] = '*';
    buff[4] = '\0';
    LiquidCrystal_I2C::setCursor(4, 1);
    LiquidCrystal_I2C::print(buff);
}

#endif
