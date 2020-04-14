#ifndef _STAT_H_
#define _STAT_H_
#include <Arduino.h>

// Exponential average
class EMP_AVERAGE {
    public:
        EMP_AVERAGE(uint8_t h_length = 8)               { emp_k = h_length; emp_data = 0; }
        void            length(uint8_t h_length)        { emp_k = h_length; emp_data = 0; }
        void            reset(void)                     { emp_data = 0; }
        int32_t         average(int32_t value);
        void            update(int32_t value);
        int32_t         read(void);
    private:
        volatile    uint8_t     emp_k       = 8;
        volatile    uint32_t    emp_data    = 0;
};

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 16
class HISTORY {
    public:
        HISTORY(void)                           { len = 0; }
        void     init(void)                     { len = 0; }
        uint16_t read(void);
        uint16_t top(void)                      { return queue[0]; }
        void     update(uint16_t item);         // Put new entry to the history
        uint16_t average(void);                 // calculate the average value
        float    dispersion(void);              // calculate the math dispersion
    private:
        volatile uint16_t queue[H_LENGTH];
        volatile uint8_t len;                   // The number of elements in the queue
        volatile uint8_t index;                 // The current element position, use ring buffer
};

#endif
