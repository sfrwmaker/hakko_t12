#include <EEPROM.h>
#include "cfg.h"
#include "vars.h"
#include "config.h"

//------------------------------------------ Configuration data ------------------------------------------------
CONFIG::CONFIG(uint8_t max_custom_tips) {
    uint8_t rs = sizeof(struct cfg) + 5;        // The total config record size
    // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
    for (record_size = 8; record_size < rs; record_size <<= 1);
    uint16_t custom_tips_area = sizeof(TIP) * MAX_CUSTOM_TIPS;
    uint8_t  rest = custom_tips_area % record_size;
    start_addr = custom_tips_area;              // Address in EEPROM when configuration records are saved
    if (rest) start_addr += record_size - rest;
    can_write       = false;
    r_addr = w_addr = start_addr;
    e_length        = 0;
    next_rec_ID     = 0;
}

// Read the records until the last one, point w_addr (write address) after the last record
void CONFIG::init(void) {
    e_length = EEPROM.length();
    uint32_t rec_ID;
    uint32_t min_rec_ID     = 0xffffffff;
    uint16_t min_rec_addr   = 0;
    uint32_t max_rec_ID     = 0;
    uint16_t max_rec_addr   = 0;
    uint8_t records         = 0;

  next_rec_ID = 0;

    // read all the records in the EEPROM, find min and max record ID
    for (uint16_t addr = start_addr; addr < e_length; addr += record_size) {
        if (readRecord(addr, rec_ID)) {
            ++records;
            if (min_rec_ID > rec_ID) {
                min_rec_ID = rec_ID;
                min_rec_addr = addr;
            }
            if (max_rec_ID < rec_ID) {
                max_rec_ID = rec_ID;
                max_rec_addr = addr;
            }
        } else {
            break;
        }
    }

    if (records == 0) {
        w_addr = r_addr = start_addr;
        can_write = true;
        return;
    }
    r_addr = max_rec_addr;
    if (records < ((e_length - start_addr)/ record_size)) {   // The EEPROM is not full
        w_addr = r_addr + record_size;
        if (w_addr > e_length) w_addr = start_addr;
    } else {
        w_addr = min_rec_addr;
    }
    can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
    memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
    memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
    updateConfig(Cfg);
    return save();                              // Save new data into the EEPROM
}

bool CONFIG::save(void) {
    if (!can_write) return can_write;
    if (next_rec_ID == 0) next_rec_ID = 1;

    uint16_t start_write = w_addr;
    uint32_t nxt = next_rec_ID;
    uint8_t summ = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.write(start_write++, nxt & 0xff);
        summ <<=2; summ += nxt;
        nxt >>= 8;
    }
    uint8_t* p = (uint8_t *)&Config;
    for (uint8_t i = 0; i < sizeof(struct cfg); ++i) {
        summ <<= 2; summ += p[i];
        EEPROM.write(start_write++, p[i]);
    }
    summ ++;                                    // To avoid empty records
    EEPROM.write(w_addr+record_size-1, summ);

    r_addr = w_addr;
    w_addr += record_size;
    if (w_addr > EEPROM.length()) w_addr = 0;
    next_rec_ID ++;                             // Get ready to write next record
    return true;
}

bool CONFIG::load(void) {
    bool is_valid = readRecord(r_addr, next_rec_ID);
    next_rec_ID ++;
    return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &rec_ID) {
    uint8_t buff[record_size];

    for (uint8_t i = 0; i < record_size; ++i) 
        buff[i] = EEPROM.read(addr+i);
  
    uint8_t summ = 0;
    for (uint8_t i = 0; i < sizeof(struct cfg) + 4; ++i) {
        summ <<= 2; summ += buff[i];
    }
    summ ++;                                    // To avoid empty fields
    if (summ == buff[record_size-1]) {          // Checksumm is correct
        uint32_t ts = 0;
        for (int8_t i = 3; i >= 0; --i) {
            ts <<= 8;
            ts |= buff[uint8_t(i)];
        }
        rec_ID = ts;
        memcpy(&Config, &buff[4], sizeof(struct cfg));
        return true;
    }
    return false;
}

//------------------------------------------ class IRON CONFIG -------------------------------------------------
void IRON_CFG::init(void) {
    CONFIG::init();
    if (!CONFIG::load()) {                      // If failed to load the data from EEPROM, initialize the config data with the default values
        setDefaults();
    }
    removeTipDuplication();
    tip_index = selectTip(Config.tip);
}

void IRON_CFG::setLowPower(uint16_t low_temp, uint8_t low_to, bool reed) {
    Config.low_temp = low_temp;
    Config.low_to   = low_to;
    Config.bit_mask &= ~CFG_SWITCH;
    if (reed)
        Config.bit_mask |= CFG_SWITCH;
}

// Translate the human readable temperature into internal value
uint16_t IRON_CFG::humanToTemp(uint16_t t, int16_t ambient) {
    if (tip_mask & CFG_THERM == 0) ambient = ambient_tempC;
    if (isCelsius()) {
        t = constrain(t, temp_minC, temp_maxC);
    } else {
        t = constrain(t, temp_minF, temp_maxF);
        t = map(t, temp_minF, temp_maxF, temp_minC, temp_maxC);
    }

    uint16_t left   = 0;
    uint16_t right  = temp_max;
    uint16_t temp = map(t, temp_tip[0], temp_tip[2], t_tip[0], t_tip[2]);

    if (temp > (left+right)/ 2) {
        temp -= (right-left) / 4;
    } else {
        temp += (right-left) / 4;
    }

    for (uint8_t i = 0; i < 20; ++i) {
        uint16_t tempH = tempToHuman(temp, ambient);
        if (tempH == t) {
            return temp;
        }
        uint16_t new_temp;
        if (tempH < t) {
            left = temp;
             new_temp = (left+right)/2;
            if (new_temp == temp)
                new_temp = temp + 1;
        } else {
            right = temp;
            new_temp = (left+right)/2;
            if (new_temp == temp)
                new_temp = temp - 1;
        }
        temp = new_temp;
    }
    return temp;
}

// Thanslate temperature from internal units to the human readable value (Celsius or Fahrenheit)
uint16_t IRON_CFG::tempToHuman(uint16_t temp, int16_t ambient) {
    if (tip_mask & CFG_THERM == 0) ambient = ambient_tempC;
    // The temperature difference between current ambient temperature and ambient temperature during tip calibration
    int d = ambient - tip_ambient;
    uint16_t tempH = 0;
    if (temp < t_tip[0]) {
        tempH = map(temp, 0, t_tip[0], ambient, temp_tip[0]);
    } else if (temp >= t_tip[1]) {
        tempH = map(temp, t_tip[1], t_tip[2], temp_tip[1]+d, temp_tip[2]+d);
    } else {
        tempH = map(temp, t_tip[0], t_tip[1], temp_tip[0]+d, temp_tip[1]+d);
    }
    if ((Config.bit_mask & CFG_CELSIUS) == 0)
        tempH = map(tempH, temp_minC, temp_maxC, temp_minF, temp_maxF);
    return tempH;
}

uint8_t IRON_CFG::selectTip(uint8_t index) {
    if (index >= MAX_CUSTOM_TIPS)               // Keep current TIP
        return tip_index;
    TIP tmp;                                    // Load the TIP configuration here
    if (loadTipData(&tmp, index) && validateTip(tmp)) { // If TIP calibration is valid, load it
        tip_mask        = tmp.mask;
        tip_ambient     = tmp.ambient;
        if (tip_mask & CFG_THERM == 0)
            tip_ambient     = ambient_tempC;
        unpackTipCalibration(t_tip, tmp.calibration_data);
        for (uint8_t i = 0; i < tip_name_sz; ++i) {
            tip_name[i] = tmp.tip_name[i];
        }
        tip_name[tip_name_sz] = '\0';
    } else {                                    // The selected tip is not calibrated
        tip_mask        = TIP_ACTIVE;
        tip_ambient     = ambient_tempC;
        t_tip[0]        = def_tip[0];
        t_tip[1]        = def_tip[1];
        t_tip[2]        = def_tip[2];
        TIPS::name(tip_name, index);
    }
    tip_index  = index;
    return index;
}

bool IRON_CFG::savePresetTempHuman(uint16_t temp) {
    if (Config.temp == temp && Config.tip  == tip_index) return false;
    Config.temp = temp;
    // When the tip changed, tip_index updated, but congiguration data remained the same
    Config.tip  = tip_index;
    return CONFIG::save();
}

bool IRON_CFG::savePresetTemp(uint16_t temp) {
    // temp - in internal units should be converted into Celsius or Fahrenheit
    uint16_t tempH;
    if (temp >= t_tip[1]) {                     // Approximate the temperature between t_mid and t_max 
        tempH = map(temp, t_tip[1], t_tip[2], temp_tip[1], temp_tip[2]);
    } else {
        tempH = map(temp, t_tip[0], t_tip[1], temp_tip[0], temp_tip[1]);
    }
    if (!Config.bit_mask & CFG_CELSIUS)
        tempH = map(temp, temp_minC, temp_maxC, temp_minF, temp_maxF);
    if (tempH == Config.temp || Config.tip == tip_index)
        return true;
    return savePresetTempHuman(tempH);
}

void IRON_CFG::applyCalibration(uint16_t tip[3]) {
    for (uint8_t i = 0; i < 3; ++i)
        t_tip[i] = tip[i];
}

void IRON_CFG::saveConfig(uint8_t off, bool cels, bool buzzer, bool ambient) {
    if (off > 30) off = 0;
    Config.off_timeout = off;
    bool cfg_celsius = Config.bit_mask & CFG_CELSIUS;
    Config.bit_mask = 0;
    if (cfg_celsius != cels) {                  // Need to translate preset temperature
        if (cels) {                             // Convert the preset temperature from Fahrenheit to Celsius
            Config.temp = map(Config.temp, temp_minF, temp_maxF, temp_minC, temp_maxC);
            Config.bit_mask |= CFG_CELSIUS;
        } else {                                // Convert the preset temperature from Celsius to Fahrenheit
            Config.temp = map(Config.temp, temp_minC, temp_maxC, temp_minF, temp_maxF);
        }
    }
    if (buzzer)  Config.bit_mask |= CFG_BUZZER;
    if (ambient) Config.bit_mask |= CFG_THERM;
    CONFIG::save();                             // Save new data into the EEPROM
}

void IRON_CFG::getCalibrationData(uint16_t tip[3]) {
    tip[0] = t_tip[0];
    tip[1] = t_tip[1];
    tip[2] = t_tip[2];
}

void IRON_CFG::saveCalibrationData(uint16_t tip[3], int8_t ambient) {
    if (tip[3] > temp_max) tip[3] = temp_max;
    uint32_t cd = tip[2] & 0x3FF; cd <<= 10;    // Pack tip calibration data in one 32-bit word: 10-bits per value
    cd |= tip[1] & 0x3FF; cd <<= 10;
    cd |= tip[0];
    TIP tip_data;
    for (uint8_t i = 0; i < tip_name_sz; ++i)   // Make sure to write tip name also
        tip_data.tip_name[i] = tip_name[i];
    tip_data.calibration_data   = cd;
    tip_data.mask               = TIP_ACTIVE | TIP_CALIBRATED;
    tip_data.ambient            = ambient;
    checkTipCRC(tip_data, true);
    uint16_t addr = tip_index * sizeof(TIP);
    uint8_t *p = (uint8_t *)&tip_data;
    for (uint8_t i = 0; i < sizeof(TIP); ++i)   // Save TIP configuration data to EEPROM 
        EEPROM.write(addr+i, *p++);
    selectTip(tip_index);                       // Reload the configuration
}

/*
 * look for next customized tip
 * Here index is the tip number in EEPROM tip configuration area
 * 
 */
uint8_t IRON_CFG::nextTip(uint8_t index, bool forward) {
    uint8_t next = index;
    TIP tip_data;
    while (true) {
        if (forward) {
            next ++;
            if (next >= MAX_CUSTOM_TIPS)
                next = 0;
        } else {
            if (next > 0)
                next --;
            else
                next = MAX_CUSTOM_TIPS - 1;
        }
        if (next == index)                      // All the list checked
            break;
        if (loadTipData(&tip_data, next) && tip_data.mask & TIP_ACTIVE) {
            break;
        }
    }
    return next;
}

bool IRON_CFG::isTipActive(uint8_t global_index) {
    char tip_n[tip_name_sz];                    // Put here the tip name
    TIPS::name(tip_n, global_index);
    int8_t indx = calibratedTipIndex(tip_n);
    if (indx >= 0) {                            // tip calibration data found in the EEPROM
        TIP tip_data;
        if (loadTipData(&tip_data, indx)) {
            return tip_data.mask & TIP_ACTIVE;
        }
    }
    return false;
}

bool IRON_CFG::toggleTipActivation(uint8_t global_index) {
    char tip_n[tip_name_sz];                    // Put here the tip name
    TIPS::name(tip_n, global_index);
    int8_t indx = calibratedTipIndex(tip_n);
    TIP tip_data;
    bool init_tip_data = true;                  // Flag: initialize tip structure for new tip
    if (indx >= 0) {                            // tip calibration data found in EEPROM
        if (loadTipData(&tip_data, indx)) {     // The tip calibration data is valid
            tip_data.mask   ^= TIP_ACTIVE;      // Toggle tip activation flag
            init_tip_data   = false;
        }        
    } else {
        indx = emptyTipSlot();
        if (indx < 0) return false;             // Failed to save yet another tip calibration data
    }
    if (init_tip_data) {
        TIPS::name(tip_data.tip_name, global_index);
        uint32_t cd = def_tip[2] & 0x3FF; cd <<= 10; // Pack tip calibration data in one 32-bit word: 10-bits per value
        cd |= def_tip[1] & 0x3FF; cd <<= 10;
        cd |= def_tip[0];
        tip_data.calibration_data = cd;
        tip_data.mask       = TIP_ACTIVE;
        tip_data.ambient    = ambient_tempC;
    }
    checkTipCRC(tip_data, true);
    uint8_t *p = (uint8_t *)&tip_data;
    uint16_t addr = indx * sizeof(TIP);
    for (uint8_t i = 0; i < sizeof(TIP); ++i)   // Save TIP configuration data to EEPROM 
        EEPROM.write(addr+i, *p++);
    return tip_data.mask & TIP_ACTIVE;
}

bool IRON_CFG::checkTipCRC(TIP& tip_data, bool write) {
    uint8_t *buff = (uint8_t *)&tip_data;
    uint8_t summ = 17;                          // Do not allow all-zero data
    for (uint8_t i = 0; i < sizeof(TIP)-1; ++i) {
        summ <<= 2; summ += buff[i];
    }
    if (write) {
        tip_data.crc = summ;
    }
    return (tip_data.crc == summ);
}
    
bool IRON_CFG::loadTipData(TIP* tip_data, uint8_t index) {
    uint16_t addr = index * sizeof(TIP);        // Start address of TIP configuration in EEPROM
    uint8_t *p = (uint8_t *)tip_data;
    for (uint8_t i = 0; i < sizeof(TIP); ++i)   // Read TIP configuration data from EEPROM 
        *p++ = EEPROM.read(addr+i);
    return checkTipCRC(*tip_data);
}

bool IRON_CFG::validateTip(TIP& tip_data) {
    uint32_t cd = tip_data.calibration_data;
    if (cd == 0) return false;
    uint16_t tip[3];                            // Tip calibration data, unpacked
    unpackTipCalibration(tip, cd);
    bool valid = ((tip[0] < tip[1]) && (tip[1] < tip[2]) && (tip[2] <= temp_max));
    uint16_t delta = temp_tip[1] - temp_tip[0];
    delta <<= 1; delta /= 3;                    // 2/3 delta
    if ((tip[0] + delta) > tip[1]) valid = false;
    delta = temp_tip[2] - temp_tip[1];
    delta <<= 1; delta /= 3;
    if ((tip[1] + delta) > tip[2]) valid = false;
    return valid;
}

void IRON_CFG::setDefaults(void) {
    Config.temp         = default_temperature;
    Config.tip          = 0;
    Config.low_temp     = 0;
    Config.low_to       = 0;
    Config.off_timeout  = 0;                    // Default automatic switch-off timeout (disabled)
    Config.bit_mask     = 0b11;                 // Celsius, buzzer is enabled
}

void IRON_CFG::unpackTipCalibration(uint16_t tip[3], uint32_t cd) {
    tip[0] = cd & 0x3FF; cd >>= 10;             // 10 bits per calibration parameter, because the ADC readings are 10 bits
    tip[1] = cd & 0x3FF; cd >>= 10;
    tip[2] = cd & 0x3FF;
}

int8_t IRON_CFG::calibratedTipIndex(const char name[tip_name_sz]) {
    for (uint8_t i = 0; i < MAX_CUSTOM_TIPS; ++i) {
        TIP tmp;
        if (loadTipData(&tmp, i)) {
            if (strncmp(name, tmp.tip_name, tip_name_sz) == 0) {
                return i;
            }
        }
    }
    return -1;
}

int8_t IRON_CFG::emptyTipSlot(void) {
    for (uint8_t i = 0; i < MAX_CUSTOM_TIPS; ++i) {
        TIP tmp;
        if (loadTipData(&tmp, i)) {
            if (!(tmp.mask & TIP_ACTIVE))
                return i;
        } else {
            return i;
        }
    }
    return -1;
}

void IRON_CFG::removeTipDuplication(void) {
    uint8_t global_index[MAX_CUSTOM_TIPS];
    uint8_t tips = TIPS::tipsLoaded();
    for (uint8_t i = 0; i < MAX_CUSTOM_TIPS; ++i)
        global_index[i] = tips;

    for (uint8_t i = 0; i < MAX_CUSTOM_TIPS; ++i) {
        TIP tmp;
        if (loadTipData(&tmp, i)) {
            uint8_t g_index = TIPS::index(tmp.tip_name);
            for (uint8_t j = 0; j < i; ++j) {
                if (global_index[j] == g_index) { // Duplicated records found!
                    uint16_t addr = i * sizeof(TIP);
                    for (uint8_t c = 0; c < sizeof(TIP); ++c)
                        EEPROM.write(addr+c, 0xff);
                } else {
                    global_index[i] = g_index;
                }
            }
        }
    }
}
