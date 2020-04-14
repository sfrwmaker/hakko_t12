#include "encoder.h"

//------------------------------------------ class BUTTON ------------------------------------------------------
BUTTON::BUTTON(uint8_t b_pin, uint16_t to = 3000) {
    this->b_pin = b_pin; 
    setTimeout(to);
}

/*
 * The Encoder button current status
 * 0    - not pressed
 * 1    - short press
 * 2    - long press
 */
uint8_t BUTTON::buttonCheck(void) {
    if (millis() >= b_check) {                                  // It is time to check the button status
        b_check = millis() + b_check_period;
        uint8_t s = 0;
        if (!digitalRead(b_pin))                               // if port state is low, the button pressed
            s = trigger_on << 1;
        if (b_on) {
            if (avg.average(s) < trigger_off)
                b_on = false;
        } else {
            if (avg.average(s) > trigger_on)
                b_on = true;
        }

        if (b_on) {                                             // Button status is 'pressed'
            uint32_t n = millis() - bpt;
            if ((bpt == 0) || (n > over_press)) {
                bpt = millis();
            } else if (n > long_press) {                        // Long press
                if (i_b_rel) {
                    return 0;
                } else{
                    if (tick_period)
                        return buttonTick();
                    i_b_rel = true;
                    return 2;
                }
            }
        } else {                                                // Button status is 'not pressed'
            if (bpt == 0 || i_b_rel) {
                bpt = 0;
                i_b_rel = false;
                return 0;
            }
            uint32_t e = millis() - bpt;
            bpt = 0;                                            // Ready for next press
            if (e < over_press) {                               // Long press already managed
                return 1;
            }
        }
    }
    return 0;
}

void BUTTON::setTimeout(uint16_t to) {
    over_press = constrain(to, def_over_press, 5000);
    tick_period = 0;
}

bool BUTTON::setTick(uint16_t to) {
    if (to > 0) {                                               // Setup tick period
        tick_period = constrain(to, 100, 300);
        over_press  = 16000;
        return true;
    } else {                                                    // disable tick
        tick_period = 0;
        over_press  = def_over_press;
    }
    return false;
}

uint8_t BUTTON::buttonTick(void) {
    uint32_t now_t = millis();
    if (now_t - tick_time > tick_period) {
        tick_time = now_t;
        return (bpt != 0);
    }
    return 0;
}

//------------------------------------------ class ENCODER ------------------------------------------------------
RENC::RENC(uint8_t main_pin, uint8_t slave_pin, uint8_t button_pin, int16_t init_pos) : BUTTON(button_pin) {
    pt = 0; m_pin = main_pin; s_pin = slave_pin; pos = init_pos;
    min_pos = -32767; max_pos = 32766; ch_b = false; increment = 1;
    changed = 0;
    is_looped = false;
}

void RENC::init(void) {
    BUTTON::init();
    pinMode(m_pin, INPUT_PULLUP);
    pinMode(s_pin, INPUT_PULLUP);
}

bool RENC::write(int16_t init_pos) {
    if ((init_pos >= min_pos) && (init_pos <= max_pos)) {
        pos = init_pos;
        return true;
    }
    return false;
}

void RENC::reset(int16_t init_pos, int16_t low, int16_t upp, uint8_t inc, uint8_t fast_inc, bool looped) {
    min_pos = low; max_pos = upp;
    if (!write(init_pos)) init_pos = min_pos;
    increment = fast_increment = inc;
    if (fast_inc > increment) fast_increment = fast_inc;
    is_looped = looped;
}

void RENC::encoderIntr(void) {                                  // Interrupt function, called when the channel A of encoder changed
    bool rUp = digitalRead(m_pin);
    unsigned long now_t = millis();
    if (!rUp) {                                                 // The channel A has been "pressed"
        if ((pt == 0) || (now_t - pt > over_press)) {
            pt = now_t;
            ch_b = digitalRead(s_pin);
        }
    } else {
        if (pt > 0) {
            uint8_t inc = increment;
            if ((now_t - pt) < over_press) {
                if ((now_t - changed) < fast_timeout) inc = fast_increment;
                changed = now_t;
                if (ch_b) pos -= inc; else pos += inc;
                if (pos > max_pos) { 
                    if (is_looped)
                        pos = min_pos;
                    else 
                        pos = max_pos;
                }
                if (pos < min_pos) {
                    if (is_looped)
                        pos = max_pos;
                    else
                        pos = min_pos;
                }
            }
            pt = 0; 
        }
    }
}
