#include "config.h"
#include "dspl_0802.h"
#include "iron_tips.h"

#ifdef DISPLAY_0802

void DSPL::init(void) {
    LiquidCrystal::begin(8, 2);
    LiquidCrystal::clear();
    full_second_line = false;
}

void DSPL::tip(const char *tip_name, bool top) {
    uint8_t  y = 1; if (top) y = 0;
    LiquidCrystal::setCursor(0, y);
    LiquidCrystal::print(tip_name);
    for (uint8_t i = strlen(tip_name); i < 8; ++i)
        LiquidCrystal::print(' ');
}

void DSPL::msgSelectTip(void) {
    LiquidCrystal::setCursor(0, 0);
    LiquidCrystal::print(F("iron tip"));
}

void DSPL::msgActivateTip(void) {
    LiquidCrystal::setCursor(0, 0);
    LiquidCrystal::print(F("act. tip"));
}

void DSPL::tSet(uint16_t t, bool celsius) {
    char buff[5];
    char units = 'C';
    if (!celsius) units = 'F';
    LiquidCrystal::setCursor(0, 0);
    sprintf(buff, "%3d%c", t, units);
    LiquidCrystal::print(buff);
}

void DSPL::tCurr(uint16_t t) {
    char buff[4];
    LiquidCrystal::setCursor(0, 1);
    if (t < 1000) {
        sprintf(buff, "%3d", t);
    } else {
        LiquidCrystal::print(F("xxx"));
        return;
    }
    LiquidCrystal::print(buff);
    if (full_second_line) {
        LiquidCrystal::print(F("    "));
        full_second_line = false;
    }
}

void DSPL::pSet(uint8_t  p) {
    char buff[6];
    sprintf(buff, "P:%3d", p);
    LiquidCrystal::setCursor(0, 0);
    LiquidCrystal::print(buff);
}

void DSPL::tRef(uint8_t ref) {
    char buff[9];
    LiquidCrystal::setCursor(0, 0);
    sprintf(buff, "Ref. #%1d ", ref+1);
    LiquidCrystal::print(buff);
}

void DSPL::timeToOff(uint8_t  sec) {
    char buff[5];
    sprintf(buff, " %3d", sec);
    LiquidCrystal::setCursor(4, 0);
    LiquidCrystal::print(buff);
}

void DSPL::msgReady(void) {
    LiquidCrystal::setCursor(4, 0);
    LiquidCrystal::print(F(" rdy"));
}

void DSPL::msgWorking(void) {
    LiquidCrystal::setCursor(4, 0);
    LiquidCrystal::print(F(" wrk"));
}

void DSPL::msgOn(void) {
    LiquidCrystal::setCursor(4, 0);
    LiquidCrystal::print(F("  ON"));
}

void DSPL::msgOff(void) {
    LiquidCrystal::setCursor(4, 0);
    LiquidCrystal::print(F(" OFF"));
}

void DSPL::msgStandby(void) {
    LiquidCrystal::setCursor(4, 0);
    LiquidCrystal::print(F(" stb"));
}

void DSPL::msgCold(void) {
    LiquidCrystal::setCursor(0, 1);
    LiquidCrystal::print(F("  cold  "));
    full_second_line = true;
}

void DSPL::msgFail(void) {
    LiquidCrystal::setCursor(0, 1);
    LiquidCrystal::print(F(" Failed "));
}

void DSPL::msgTune(void) {
    LiquidCrystal::setCursor(0, 0);
    LiquidCrystal::print(F("Tune"));
}

void DSPL::msgCelsius(void) {
    LiquidCrystal::setCursor(0, 1);
    LiquidCrystal::print(F("Celsius "));
}

void DSPL::msgFarneheit(void) {
    LiquidCrystal::setCursor(0, 1);
    LiquidCrystal::print(F("Fahren. "));
}

void DSPL::msgDefault() {
    LiquidCrystal::setCursor(0, 1);
    LiquidCrystal::print(F(" default"));
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
            LiquidCrystal::print(F("stby T."));
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
            LiquidCrystal::print(F("stby tm"));
            if (tune) {
                LiquidCrystal::setCursor(1,1);
                sprintf(buff, "%3ds", p);
                LiquidCrystal::print(buff);
            }
            break;
        case 6:                                 // off-timeout
            LiquidCrystal::print(F("off tm"));
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
            LiquidCrystal::print(F("tip cfg"));
            break;
        case 8:                                 // Activate tip
            LiquidCrystal::print(F("activ."));
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
    LiquidCrystal::setCursor(3, 1);
    LiquidCrystal::print(buff);
}

void DSPL::mark(char sym, bool on) {
    char buff[5];
    for (uint8_t  i = 0; i < 4; ++i) buff[i] = ' ';
    if (on) buff[3] = sym;
    buff[4] = '\0';
    LiquidCrystal::setCursor(4, 1);
    LiquidCrystal::print(buff);
}

#endif
