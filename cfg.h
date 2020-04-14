#ifndef _CFG_H
#define _CFG_H
#include <Arduino.h>
#include "iron_tips.h"
#include "vars.h"
/*
 * First 504 bytes of EEPROM are allocated to store the calibration tip data (see iron_tips.h)
 * The EEPROM space starting from 512 is used to store the controller configuration data
 * in the following format. Each record requires 13 bytes, and it will rounded to 16 bytes.
 * 
 * uint32_t ID                                  The ID of the record. Each time incremented by 1
 * struct cfg                                   config data, 8 bytes
 * uint8_t CRC                                  the checksum
 */
struct cfg {
    uint16_t    temp;                           // The preset temperature of the IRON (Celsius or Fahrenheit)
    uint8_t     tip;                            // Current tip index [0 - MAX_CUSTOM_TIPS]
    uint16_t    low_temp;                       // The low power temperature (C) or 0 if the tilt sensor is disabled
    uint8_t     low_to;                         // The low power timeout (seconds)
    uint8_t     off_timeout;                    // The Automatic switch-off timeout in minutes [0 - 30]
    uint8_t     bit_mask;                       // See description below
};

/*
 * The configuration bit map:
 * 0: CFG_CELSIUS       - The temperature units: Celsius (1) or Fahrenheit (0)
 * 1: CFG_BUZZER        - Is the Buzzer Enabled (1)
 * 2: CFG_SWITCH        - The hardware switch type: tilt/vibro/mercury (0) or reed (1)
 * 3: CFG_THERM         - The ambient thermistor used (1) or disabled (0)
 */
typedef enum { CFG_CELSIUS = 1, CFG_BUZZER = 2, CFG_SWITCH = 4, CFG_THERM = 8 } CFG_BIT_MASK;

//------------------------------------------ Configuration data ------------------------------------------------
class CONFIG {
    public:
        CONFIG(uint8_t max_custom_tips);
        void    init();
        bool    load(void);
        void    getConfig(struct cfg &Cfg);     // Copy config structure from this class
        void    updateConfig(struct cfg &Cfg);  // Copy updated config into this class
        bool    save(void);                     // Save current config copy to the EEPROM
        bool    saveConfig(struct cfg &Cfg);    // write updated config into the EEPROM
    protected:
        struct      cfg Config;
    private:
        bool        readRecord(uint16_t addr, uint32_t &rec_ID);
        bool        can_write;                  // The flag indicates that data can be saved
        uint16_t    start_addr = 0;             // The first address in EEPROM to start configuration record
        uint16_t    r_addr;                     // Address of thecorrect record in EEPROM to be read
        uint16_t    w_addr;                     // Address in the EEPROM to start write new record
        uint16_t    e_length;                   // Length of the EEPROM, depends on arduino model
        uint32_t    next_rec_ID;                // next record ID
        uint8_t     record_size;                // The size of one record in bytes
};

//------------------------------------------ class IRON CONFIG -------------------------------------------------
class IRON_CFG : public CONFIG, public TIPS {
    public:
        IRON_CFG(uint8_t max_custom_tips) : CONFIG(max_custom_tips) { }
        void        init(void);
        bool        isCelsius(void)             { return Config.bit_mask & CFG_CELSIUS; }
        bool        isBuzzer(void)              { return Config.bit_mask & CFG_BUZZER;  }
        uint16_t    tempPresetHuman(void)       { return Config.temp;                   }
        uint8_t     tipIndex(void)              { return tip_index;                     }
        bool        isCalibrated(void)          { return tip_mask & TIP_CALIBRATED;     }
        uint8_t     getOffTimeout(void)         { return Config.off_timeout;            }
        uint8_t     getLowTemp(void)            { return Config.low_temp;               }
        uint8_t     getLowTimeout(void)         { return Config.low_to;                 }
        char        *tipName(void)              { return tip_name;                      }
        uint16_t    lowTemp(void)               { return Config.low_temp;               }
        uint8_t     lowTimeout(void)            { return Config.low_to;                 }
        bool        isReedType(void)            { return Config.bit_mask & CFG_SWITCH;  }
        bool        isAmbientSensor(void)       { return Config.bit_mask & CFG_THERM;   }
        void        setLowPower(uint16_t low_temp, uint8_t low_to, bool reed);
        uint16_t    humanToTemp(uint16_t temp, int16_t ambient); // Translate the human readable temperature into internal value
        uint16_t    tempToHuman(uint16_t temp, int16_t ambient); // Translate temperature from internal units to the human readable value (Celsius or Fahrenheit)
        uint8_t     selectTip(uint8_t index);   // Select new tip, return selected tip index
        bool        savePresetTempHuman(uint16_t temp); // Save preset temperature in the human readable units
        bool        savePresetTemp(uint16_t temp); // Save preset temperature in the internal units (convert it to the human readable units)
        void        applyCalibration(uint16_t tip[3]);  // Apply calibration data, used during calibration procedure
        void        saveConfig(uint8_t off, bool cels, bool buzzer, bool ambient); // Save global configuration parameters
        void        getCalibrationData(uint16_t tip[3]);
        void        saveCalibrationData(uint16_t tip[3], int8_t ambient);
        uint8_t     nextTip(uint8_t index, bool forward = true);    // look for next customized tip
        bool        isTipActive(uint8_t global_index);
        bool        toggleTipActivation(uint8_t global_index);
    private:
        bool        checkTipCRC(TIP& tip_data, bool write = false);
        bool        loadTipData(TIP* tip_data, uint8_t index);
        bool        validateTip(TIP& tip_data); // Validate the IRON tip calibration
        void        setDefaults(void);          // Set default parameter values if failed to load data from EEPROM
        void        unpackTipCalibration(uint16_t tip[3], uint32_t cd);
        int8_t      calibratedTipIndex(const char name[tip_name_sz]);   // Tip idex in EEPROM area or -1 if not found
        int8_t      emptyTipSlot(void);         // Unused Tip slot idex in EEPROM area or -1 if not found
        void        removeTipDuplication(void);
        char        tip_name[tip_name_sz+1];
        uint16_t    t_tip[3]    = {0};          // The current tip calibration
        uint8_t     tip_index   = 0;            // The current tip index (EEPROM customized  array)
        uint8_t     tip_mask    = 0;            // The current tip status (see TIP_STATUS in iron_tips.h)
        int8_t      tip_ambient = ambient_tempC;// The ambient temperature when tip was calibrated
        const uint16_t def_tip[3] = {279, 501, 700};// Default values of internal sensor readings at reference temperatures
};

#endif
