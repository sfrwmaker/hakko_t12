#include "vars.h"

const uint16_t default_temperature = 240;       // Default value of IRON temperature
const uint16_t temp_max  = 950;                 // Maximum possible temperature in internal units
const uint16_t temp_minC = 180;                 // Minimum calibration temperature in degrees of Celsius
const uint16_t temp_maxC = 450;                 // Maximum calibration temperature in degrees of Celsius
const uint16_t temp_minF = (temp_minC *9 + 32*5 + 2)/5;
const uint16_t temp_maxF = (temp_maxC *9 + 32*5 + 2)/5;
const uint16_t temp_tip[3] = {200, 300, 400};   // Reference temperatures for tip calibration (Celsius)
const uint16_t ambient_tempC = 25;              // Default ambient temperature in Celsius
const uint16_t ambient_tempF = (ambient_tempC *9 + 32*5 + 2)/5;
