#include "config.h"
#include "dspl_1602_i2c.h"

#ifdef DISPLAY_1602_I2C

void DSPL::init(void) {
  LiquidCrystal_I2C::begin();
  LiquidCrystal_I2C::clear();
  full_second_line = false;
}

void DSPL::tip(uint8_t index, bool top) {
  if (index >= max_tips) {
    LiquidCrystal_I2C::setCursor(0, 1);
    LiquidCrystal_I2C::print(F("      ????      "));
    return;
  }
  char buff[5];
  uint8_t  i = 0;
  const char *p = tip_name[index];
  for ( ; i < 4; ++i)
    if (!(buff[i] = *p++))
      break;
  for ( ; i < 4; ++i) buff[i] = ' ';
  buff[4] = '\0';
  uint8_t  y = 1; if (top) y = 0;
  LiquidCrystal_I2C::setCursor(0, y);
  LiquidCrystal_I2C::print(buff);
}

void DSPL::msgSelectTip(void) {
  LiquidCrystal_I2C::setCursor(0, 0);
  LiquidCrystal_I2C::print(F("iron tip        "));
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

void DSPL::msgCancel(void) {
  LiquidCrystal_I2C::setCursor(0, 1);
  LiquidCrystal_I2C::print(F(" cancel         "));
}

void DSPL::msgApply(void) {
  LiquidCrystal_I2C::setCursor(0, 1);
  LiquidCrystal_I2C::print(F(" save           "));
}

void DSPL::setupMode(uint8_t  mode, uint8_t  p) {
  char buff[5];
  LiquidCrystal_I2C::clear();
  LiquidCrystal_I2C::print(F("setup"));
  LiquidCrystal_I2C::setCursor(1,1);
  switch (mode) {
    case 0:
    LiquidCrystal_I2C::print(F("off:"));
    if (p > 0) {
      sprintf(buff, "%2dm", p);
      LiquidCrystal_I2C::print(buff);
    } else {
      LiquidCrystal_I2C::print(" NO");
    }
    break;
    case 1:
      LiquidCrystal_I2C::print(F("units"));
      LiquidCrystal_I2C::setCursor(7, 1);
      if (p)
        LiquidCrystal_I2C::print("C");
      else
        LiquidCrystal_I2C::print("F");
      break;
    case 2:
      LiquidCrystal_I2C::print(F("tip cfg"));
    case 3:
      LiquidCrystal_I2C::print(F("tune"));
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
