#include "config.h"
#include "dspl_0802.h"

#ifdef DISPLAY_0802

void DSPL::init(void) {
  LiquidCrystal::begin(8, 2);
  LiquidCrystal::clear();
  full_second_line = false;
}

void DSPL::tip(uint8_t index, bool top) {
  if (index >= max_tips) {
    LiquidCrystal::setCursor(0, 1);
    LiquidCrystal::print(F("  ????  "));
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
  LiquidCrystal::setCursor(0, y);
  LiquidCrystal::print(buff);
}

void DSPL::msgSelectTip(void) {
  LiquidCrystal::setCursor(0, 0);
  LiquidCrystal::print(F("iron tip"));
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

void DSPL::msgCancel(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" cancel "));
}

void DSPL::msgApply(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" save   "));
}

void DSPL::setupMode(uint8_t  mode, uint8_t  p) {
  char buff[5];
  LiquidCrystal::clear();
  LiquidCrystal::print(F("setup"));
  LiquidCrystal::setCursor(1,1);
  switch (mode) {
    case 0:
    LiquidCrystal::print(F("off:"));
    if (p > 0) {
      sprintf(buff, "%2dm", p);
      LiquidCrystal::print(buff);
    } else {
      LiquidCrystal::print(" NO");
    }
    break;
    case 1:
      LiquidCrystal::print(F("units"));
      LiquidCrystal::setCursor(7, 1);
      if (p)
        LiquidCrystal::print("C");
      else
        LiquidCrystal::print("F");
      break;
    case 2:
      LiquidCrystal::print(F("tip cfg"));
    case 3:
      LiquidCrystal::print(F("tune"));
      break;
  }
}

void DSPL::percent(uint8_t  Power) {
  char buff[6];
  sprintf(buff, " %3d%c", Power, '%');
  LiquidCrystal::setCursor(3, 1);
  LiquidCrystal::print(buff);
}

void DSPL::calibrated(bool calibrated) {
  char buff[5];
  for (uint8_t  i = 0; i < 4; ++i) buff[i] = ' ';
  if (!calibrated) buff[3] = '*';
  buff[4] = '\0';
  LiquidCrystal::setCursor(4, 1);
  LiquidCrystal::print(buff);
}

#endif
