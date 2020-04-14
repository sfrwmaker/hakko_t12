#include "stat.h"

int32_t EMP_AVERAGE::average(int32_t value) {
    uint8_t round_v = emp_k >> 1;
    update(value);
    return (emp_data + round_v) / emp_k;
}

void EMP_AVERAGE::update(int32_t value) {
    uint8_t round_v = emp_k >> 1;
    emp_data += value - (emp_data + round_v) / emp_k;
}

int32_t EMP_AVERAGE::read(void) {
    uint8_t round_v = emp_k >> 1;
    return (emp_data + round_v) / emp_k;
}

//------------------------------------------ class HISTORY ----------------------------------------------------
void HISTORY::update(uint16_t item) {
    if (len < H_LENGTH) {
        queue[len++] = item;
    } else {
        queue[index ] = item;
        if (++index >= H_LENGTH) index = 0;     // Use ring buffer
    }
}

uint16_t HISTORY::read(void) {
    uint8_t i = H_LENGTH - 1;
    if (index)
        i = index - 1;
    return queue[i];
}

uint16_t HISTORY::average(void) {
    uint32_t sum = 0;
    if (len == 0) return 0;
    if (len == 1) return queue[0];
    for (uint8_t i = 0; i < len; ++i) sum += queue[i];
    sum += len >> 1;                            // round the average
    sum /= len;
    return uint16_t(sum);
}

float HISTORY::dispersion(void) {
    if (len < 3) return 1000;
    uint32_t sum = 0;
    uint32_t avg = average();
    for (uint8_t i = 0; i < len; ++i) {
        int32_t q = queue[i];
        q -= avg;
        q *= q;
        sum += q;
    }
    sum += len << 1;
    float d = (float)sum / (float)len;
    return d;
}
