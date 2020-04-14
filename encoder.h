#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "stat.h"

//------------------------------------------ class BUTTON ------------------------------------------------------
class BUTTON {
    public:
        BUTTON(uint8_t b_pin, uint16_t timeout_ms = 3000);
        uint8_t     buttonCheck(void);
        void        setTimeout(uint16_t to = 3000);
        bool        setTick(uint16_t to);
        void        init(void)                              { pinMode(b_pin, INPUT_PULLUP); }
    private:
        uint8_t     buttonTick(void);
        EMP_AVERAGE         avg;                            // Do average the button readings to mainain the button status
        uint16_t            over_press;                     // Maximum time in ms the button can be pressed
        volatile bool       i_b_rel         = false;        // Ignore button release event
        bool                b_on            = false;        // The button current position: true - pressed
        uint32_t            bpt             = 0;            // Time in ms when the button was pressed (press time)
        uint32_t            b_check         = 0;            // Time in ms when the button should be checked
        uint32_t            tick_time       = 0;            // Time in ms when the last 'tick' was generated
        uint8_t             b_pin           = 0;            // The PIN number of the button
        uint16_t            tick_period     = 0;            // Repeat 'tick' period
        const uint8_t       trigger_on      = 100;          // avg limit to change button status to on
        const uint8_t       trigger_off     = 50;           // avg limit to change button status to off
        const uint8_t       avg_length      = 4;            // avg length
        const uint8_t       b_check_period  = 20;           // The button check period, ms
        const uint16_t      long_press      = 1500;         // If the button was pressed more that this timeout, we assume the long button press
        const uint16_t      def_over_press  = 2500;         // Default value for button overpress timeout (ms)
};

//------------------------------------------ class ENCODER ------------------------------------------------------
class RENC : public BUTTON {
    public:
        RENC(uint8_t main_pin, uint8_t slave_pin, uint8_t button_pin, int16_t init_pos = 0);
        void        init(void);
        void        set_increment(uint8_t inc)      { increment = inc; }
        uint8_t     get_increment(void)             { return increment; }
        int16_t     read(void)                      { return pos; }
        void        reset(int16_t init_pos, int16_t low, int16_t upp, uint8_t inc = 1, uint8_t fast_inc = 0, bool looped = false);
        bool        write(int16_t initPos);
        void        encoderIntr(void);
    private:
        int32_t     min_pos, max_pos;
        uint8_t     m_pin, s_pin;                   // The pin numbers connected to the main channel and to the socondary channel
        bool        is_looped;                      // Whether the encoder is looped
        uint8_t     increment;                      // The value to add or subtract for each encoder tick
        uint8_t     fast_increment;                 // The value to change encoder when in runs quickly
        volatile uint32_t   pt;                     // Time in ms when the encoder was rotaded
        volatile uint32_t   changed;                // Time in ms when the value was changed
        volatile bool       ch_b;
        volatile int16_t    pos;                    // Encoder current position
        const uint16_t      fast_timeout    = 300;  // Time in ms to change encoder quickly
        const uint16_t      over_press      = 1000;
};

#endif
