#ifndef _IRON_TIPS_H_
#define _IRON_TIPS_H_

#include <Arduino.h>

// The length of the tip name
#define	 	tip_name_sz		(5)

/*
 * The custip TIP structure, 12 bytes
 * The lower EEPROM area reserved for storing upto MAX_CUSTOM_TIPS configuration data
 * When some tip is activated, new structure is allocated for it in the lower EEPROM Area
 * The Active tips are stored concequently in this area, tip index is the number of tip
 * in the custom area.
 * Whe tip disabled, the tip configuration data is kept in the EEPROM area till space 
 * required for another tip data.
 */
typedef struct s_tip TIP;
struct s_tip {
    char        tip_name[tip_name_sz];                          // The TIP name
    uint32_t    calibration_data;                               // The temperature calibration data for soldering tips. (3 reference points: 200, 300, 400 Centigrades)
    uint8_t     mask;                                           // The bit mask: TIP_ACTIVE + TIP_CALIBRATED
    int8_t      ambient;                                        // The ambient temperature in Celsius when the tip being calibrated
    uint8_t     crc;                                            // CRC checksum
};

typedef enum tip_status { TIP_ACTIVE = 1, TIP_CALIBRATED = 2 } TIP_STATUS;

class TIPS {
	public:
		TIPS()										            { }
		uint16_t		tipsLoaded(void);
		bool            name(char tip_n[tip_name_sz], uint8_t index);
        int8_t          index(const char name[tip_name_sz]);    // Tip idex in global list or -1 if not found
};


#endif
