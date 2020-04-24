 /*
 * Soldering IRON controller for hakko t12 tips built on atmega328 microcontroller running 16 MHz
 * The controller is using interrupts from the Timer1 to generate high-frequence PWM signal on port D10
 * to silently heat the IRON and periodically check the IRON temperature on overflow interrupts
 * Timer1 runs with prescale 1 through 0 to 255 and back and its frequency is 31250 Hz.
 * The owerflow interrupt running as folows:
 * First, the current through the IRON is checked
 * then the power, supplien to the IRON interrupted and the controller waits for 32 timer interrupts (about 1 ms)
 * then the IRON temperature is checked and the power to the IRON restored
 * then the controller waits for check_period Timer1 interrupts to restart the all procedure over again
 */

// Edit the configuration file to select appropriate display type
#include <EEPROM.h>
#include "config.h"
#include "encoder.h"
#include "cfg.h"
#include "iron_tips.h"
#include "vars.h"

// Rotary encoder interface
const uint8_t R_MAIN_PIN = 2;					// Rotary Encoder main pin (right)
const uint8_t R_SECD_PIN = 4;                   // Rotary Encoder second pin (left)
const uint8_t R_BUTN_PIN = 3;                   // Rotary Encoder push button pin

const uint8_t probePIN  = A0;                   // Thermometer pin from soldering IRON
const uint8_t checkPIN  = A1;                   // Iron current check pin
const uint8_t termisPIN = A2;                   // The thermistor pin to check ambient temperature
const uint8_t tiltswPIN = A3;                   // The tilt/reed/vibro switch pin 
const uint8_t buzzerPIN = 11;                   // The simple buzzer to make a noise
const uint8_t heaterBIT = 0b00000100;           // The Heater pin, D10, is 2-nd bit on the PORTB register

// The variables for Timer1 operations
volatile uint16_t  tmr1_count;                  // The count to calculate the temperature and the current check periods
volatile bool      iron_off;                    // Whether the IRON is switched off to check the temperature
const uint32_t     temp_check_period = 20;      // The IRON temperature check period, ms

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
	public:
		BUZZER(uint8_t buzzerP)                 { buzzer_pin = buzzerP; }
        void    activate(bool on)               { this->on = on; }
		void    init(void)						{ pinMode(buzzer_pin, OUTPUT); noTone(buzzer_pin); }
		void    shortBeep(void)					{ if (on) tone(buzzer_pin, 3520, 160); }
		void    lowBeep(void)    				{ if (on) tone(buzzer_pin,  880, 160); }
		void    doubleBeep(void) 				{ if (on) { tone(buzzer_pin, 3520, 160); delay(300); tone(buzzer_pin, 3520, 160); } }
		void    failedBeep(void) 			    { if (on) { tone(buzzer_pin, 3520, 160); delay(170);
													tone(buzzer_pin, 880, 250); delay(260);
													tone(buzzer_pin, 3520, 160); }
												}
	private:
		uint8_t buzzer_pin;
        bool    on = true;
};

//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algorithm 
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formula is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 *  
 *  PID coefficients history:
 *  10/14/2017  [768, 32, 328]
 */
class PID {
	public:
		PID(void) {
			Kp = 2009;
			Ki =   16;
			Kd = 2048;
		}
		void	resetPID(int temp = -1);      	// reset PID algorithm history parameters
		// Calculate the power to be applied
		int32_t	reqPower(int temp_set, int temp_curr);
		int16_t	changePID(uint8_t p, int k);  	// set or get (if parameter < 0) PID parameter
	private:
		void  	debugPID(int t_set, int t_curr, int32_t kp, int32_t ki, int32_t kd, int32_t delta_p);
		int16_t	temp_h0, temp_h1;               // previously measured temperature
		bool  	pid_iterate;                    // Whether the iterative process is used
		int32_t i_summ;                        	// Ki summary multiplied by denominator
		int32_t power;                         	// The power iterative multiplied by denominator
		int32_t Kp, Ki, Kd;                    	// The PID algorithm coefficients multiplied by denominator
		const uint8_t denominator_p = 11;      	// The common coefficient denominator power of 2 (11 means divide by 2048)
};

void PID::resetPID(int temp) {
	temp_h0 = 0;
	power  = 0;
	i_summ = 0;
	pid_iterate = false;
	if ((temp > 0) && (temp < 1000))
		temp_h1 = temp;
	else
		temp_h1 = 0;
}

int16_t PID::changePID(uint8_t p, int k) {
	switch(p) {
		case 1:
			if (k >= 0) Kp = k;
			return Kp;
		case 2:
			if (k >= 0) Ki = k;
			return Ki;
		case 3:
			if (k >= 0) Kd = k;
			return Kd;
		default:
			break;
	}
	return 0;
}

int32_t PID::reqPower(int temp_set, int temp_curr) {
	if (temp_h0 == 0) {
		// When the temperature is near the preset one, reset the PID and prepare iterative formula                        
		if ((temp_set - temp_curr) < 30) {
			if (!pid_iterate) {
				pid_iterate = true;
				power = 0;
				i_summ = 0;
			}
		}
		i_summ += temp_set - temp_curr;       	// first, use the direct formula, not the iterate process
		power = Kp*(temp_set - temp_curr) + Ki*i_summ;
		// If the temperature is near, prepare the PID iteration process
	} else {
		int32_t kp = Kp * (temp_h1 - temp_curr);
		int32_t ki = Ki * (temp_set - temp_curr);
		int32_t kd = Kd * (temp_h0 + temp_curr - 2*temp_h1);
		int32_t delta_p = kp + ki + kd;
		power += delta_p;                     	// power kept multiplied by denominator!
	}
	if (pid_iterate) temp_h0 = temp_h1;
	temp_h1 = temp_curr;
	int32_t pwr = power + (1 << (denominator_p-1));  // prepare the power to delete by denominator, round the result
	pwr >>= denominator_p;                    	// delete by the denominator
	return pwr;
}

//------------------------- class FastPWM operations using Timer1 on pin D10 at 31250 Hz ----------------------
class FastPWM {
	public:
		FastPWM()                               { }
		void init(void);
		void duty(uint8_t d)                    { OCR1B = d; }
		void off(void)                        	{ OCR1B = 0; PORTB &= ~heaterBIT; }
};

void FastPWM::init(void) {
	pinMode(10, OUTPUT);                        // Use D10 pin for heating the IRON
	PORTB &= ~heaterBIT;                        // Switch-off the power
	tmr1_count = 0;
	iron_off = false;                           // The volatile global variable
	noInterrupts();
	TCNT1   = 0;
	TCCR1B  = _BV(WGM13);                       // Set mode as phase and frequency correct pwm, stop the timer
	TCCR1A  = 0;
	ICR1    = 256;
	TCCR1B  = _BV(WGM13) | _BV(CS10);           // Top value = ICR1, prescale = 1; 31250 Hz
	TCCR1A |= _BV(COM1B1);                      // XOR D10 on OC1B, detached from D09
	OCR1B   = 0;                                // Switch-off the signal on pin D10;
	TIMSK1  = _BV(TOIE1);                     	// Enable overflow interrupts @31250 Hz
	interrupts();
}

//------------------------------------------ class soldering IRON ---------------------------------------------
class IRON : protected PID {
	public:
		IRON(uint8_t sensor_pin, uint8_t check_pin, uint8_t ambient_pin, uint8_t tilt_pin) {
			sPIN = sensor_pin;
			cPIN = check_pin;
            aPIN = ambient_pin;
            tPIN = tilt_pin;
			h_counter = h_max_counter;
		}
        typedef enum { POWER_OFF, POWER_ON, POWER_FIXED, POWER_COOLING } PowerMode;
		void     	init(void);
		void     	switchPower(bool On);
        bool        isOn(void);
		uint16_t 	presetTemp(void)            { return temp_set;              }
		uint16_t 	currTemp(void)              { return h_temp.read();         }
		uint16_t 	tempAverage(void)           { return h_temp.average();      }
		uint16_t 	tempDispersion(void)        { return h_temp.dispersion();   }
		uint16_t 	powerDispersion(void)       { return h_power.dispersion();  }
		uint8_t		getMaxFixedPower(void)      { return max_fixed_power;       }
		int16_t		changePID(uint8_t p, int k)	{ return PID::changePID(p, k);  }
		bool     	checkIron(void);           	// Check the IRON, return true if the iron is not connected
		void     	keepTemp(void);            	// Keep the IRON temperature, called by Timer1 interrupt
		uint8_t     appliedPower(void);         // Power applied to the IRON in percents
		void     	setTemp(uint16_t t);        // Set the temperature to be kept (internal units)
        void        lowPowerMode(uint16_t t);   // Activate low power mode (setup temp. or 0 to return to standard mode)
		uint8_t     getAvgPower(void);          // Average applied power
		void     	fixPower(uint8_t Power);    // Set the specified power to the the soldering IRON
		void     	initTempHistory(void)       { h_counter = h_max_counter; h_temp.init(); mode = POWER_OFF; }
        bool        isCold(void)                { return (mode == POWER_OFF); }
        int16_t     ambientTemp(void);
        void        adjust(uint16_t t);
        bool        isIronTiltSwitch(bool reed);
        void        checkSWStatus(void);
  private:
		FastPWM  	fastPWM;                    // Power the iron using fast PWM through D10 pin using Timer1
		uint8_t     sPIN, cPIN, aPIN, tPIN;     // The sensor PIN, current check PIN, ambient temperature PIN, tilt switch PIN
        uint16_t    temp_set;                   // The temperature that should be kept
        uint16_t    temp_low;                   // Low power mode temperature
		uint8_t     fix_power = 0;              // Fixed power value of the IRON (or zero if off)
		uint32_t 	check_iron_ms = 0;          // The time in ms when check the IRON next time
        uint32_t    check_tilt_ms = 0;          // The time in ms when check the tilt switch next time
		bool     	disconnected;               // Whether no current through the IRON (the iron disconnected)
		int      	h_counter;                  // Put the temperature and power to the history, when the counter become 0 
        uint8_t     applied_power = 0;          // The applied power to the IRON, used in checkIron() 
		volatile    PowerMode mode = POWER_OFF; // Working mode of the IRON
		volatile 	bool chill;                 // Whether the IRON should be cooled (preset temp is lower than current)
		HISTORY     h_power;                    // The history data of power applied values
		HISTORY     h_temp;                     // The history data of the temperature
		EMP_AVERAGE current;                    // The average value for the current through the IRON
        EMP_AVERAGE tilt;                       // The average value of tilt port
        volatile    EMP_AVERAGE amb_int;        // The internal reading of ambient temperature                        
        bool        tilt_toggle = false;        // The tilt switch changed state
		const uint8_t     max_power       = 210; // maximum power to the IRON
		const uint8_t     max_fixed_power = 120; // Maximum power in fixed power mode
		const uint16_t min_curr        = 10;    // The minimum current value to check the IRON is connected
		const uint32_t check_period    = 503;   // Check the iron period in ms
		const uint16_t h_max_counter   = 500 / temp_check_period;     // Put the history data twice a second
		const uint8_t  emp_k = 2;           	// The exponential average coefficient
        const uint8_t  emp_tilt = 3;            // The exponential average coefficient for tilt pin
        const uint16_t iron_cold = 25;          // The internal temperature when the IRON is cold
};

void IRON::init(void) {
	pinMode(sPIN, INPUT);
    pinMode(aPIN, INPUT);
    pinMode(tPIN, INPUT);
	fastPWM.init();                           	// Initialization for 31.5 kHz PWM on D10 pin
	mode            = POWER_OFF;
	fix_power       = 0;
    applied_power   = 0;
	disconnected    = true;                     // Do not read the ambient temperature when the IRON disconnected
	check_iron_ms   = 0;
	resetPID();
	h_counter = h_max_counter;
	h_power.init();
	h_temp.init();
	current.length(emp_k);
    tilt.length(emp_tilt);
    amb_int.length(4);
}

void IRON::setTemp(uint16_t t) {
    if (mode == POWER_ON) resetPID();
    temp_set = t;
    uint16_t ta = h_temp.average();
    chill = (ta > t + 5);                       // The IRON must be cooled
    temp_low = 0;                               // disable low power mode
}

void IRON::lowPowerMode(uint16_t t) {
    if ((mode == POWER_ON && t < temp_set) || t == 0)
        temp_low = t;                           // Activate low power mode
}

uint8_t IRON::getAvgPower(void) {
    uint16_t p = h_power.average();
    return p & 0xff;  
}

uint8_t IRON::appliedPower(void) {
    uint8_t p = getAvgPower(); 
    return map(p, 0, max_power, 0, 100);  
}

void IRON::switchPower(bool On) {
	if (!On) {
		fastPWM.off();
		fix_power = 0;
        if (mode != POWER_OFF)
            mode = POWER_COOLING;
		return;
	}
	resetPID(analogRead(sPIN));
	h_power.init();
    mode = POWER_ON;
}

bool IRON::isOn(void) {
    return (mode == POWER_ON || mode == POWER_FIXED);
}

bool IRON::checkIron(void) {
	if (millis() < check_iron_ms)
		return disconnected;

	check_iron_ms = millis() + check_period;
	uint16_t curr = 0;
	if (applied_power == 0) {                   // The IRON is switched-off
		fastPWM.duty(127);                      // Quarter of maximum power
		for (uint8_t i = 0; i < 5; ++i) {       // Make sure we check the current in active phase of PWM signal
			delayMicroseconds(31);
			uint16_t c = analogRead(cPIN);      // Check the current through the IRON
			if (c > curr) curr = c;             // The maximum value
		}
		fastPWM.off();
		if (curr > min_curr * 2)                // Do not put big values in to the history 
			curr = min_curr * 2;                // This is enough to ensure the IRON is connected
		curr = current.average(curr);           // Calculate exponential average value of the current
	} else {
		curr = analogRead(cPIN);
	}
	disconnected = (curr < min_curr);

	if (mode == POWER_OFF || mode == POWER_COOLING) { // If the soldering IRON is set to be switched off
		fastPWM.off();                          // Surely power off the IRON
	}
	return disconnected;
}

// This routine is used to keep the IRON temperature near required value and is activated by the Timer1
void IRON::keepTemp(void) {
    uint16_t  ambient = analogRead(aPIN);       // Update ambient temperature
    amb_int.update(ambient);
    uint16_t t = analogRead(sPIN);              // Read the IRON temperature
    volatile uint16_t t_set = temp_set;         // The preset temperature depends on usual/low power mode
    if (temp_low) t_set = temp_low;
    
    if ((t >= temp_max + 20) || (t > (t_set + 100))) { // Prevent global over heating
        if (mode == POWER_ON) chill = true;     // Turn off the power in main working mode only;
    }
    if (t < temp_max) {                         // Do not save to the history readings when the IRON is disconnected
        if (--h_counter < 0) {
            h_temp.update(t);
            h_counter = h_max_counter;
        } 
    }

    int32_t p = 0;
    switch (mode) {
        case POWER_OFF:
            break;
        case POWER_COOLING:
            if (h_temp.average() < iron_cold)
                mode = POWER_OFF;
            break;
        case POWER_ON:
            if (chill) {
                if (t < (t_set - 2)) {
                    chill = false;
                    resetPID();
                } else {
                    break;
                }
            }
            p = PID::reqPower(t_set, t);
            p = constrain(p, 0, max_power);
            break;
        case POWER_FIXED:
            p = fix_power;
            break;
        default:
            break;
    }
    applied_power = p & 0xff;
    if (h_counter == 1) {
        h_power.update(applied_power);
    }
    fastPWM.duty(applied_power);
}

void IRON::fixPower(uint8_t Power) {
	if (Power == 0) {                         	// To switch off the IRON, set the Power to 0
		fix_power = 0;
		fastPWM.off();
        mode = POWER_COOLING;
		return;
	}

	if (Power > max_fixed_power)
		Power = max_fixed_power;
    fix_power = Power;
    mode = POWER_FIXED;
}

/*
 * Return ambient temperature in Celsius
 * Caches previous result to skip expensive calculations
 */
int16_t IRON::ambientTemp(void) {
static const uint16_t add_resistor  = 10030;                // The additional resistor value (10koHm)
static const float    normal_temp[2]= { 10000, 25 };        // nominal resistance and the nominal temperature
static const uint16_t beta          = 3950;                 // The beta coefficient of the thermistor (usually 3000-4000)
static int32_t  average             = -1;                   // Previous value of ambient temperature (readings on aPIN)
static int      cached_ambient      = ambient_tempC;        // Previous value of the temperature

    uint16_t a_temp = amb_int.read();                       // Average value of ambient temperature
    if (abs(a_temp - average) < 20)
        return cached_ambient;

    average = a_temp;
    if (average < 975) {                                    // prevent division by zero
        // convert the value to resistance
        float resistance = 1023.0 / (float)average - 1.0;
        resistance = (float)add_resistor / resistance;
        float steinhart = resistance / normal_temp[0];      // (R/Ro)
        steinhart = log(steinhart);                         // ln(R/Ro)
        steinhart /= beta;                                  // 1/B * ln(R/Ro)
        steinhart += 1.0 / (normal_temp[1] + 273.15);       // + (1/To)
        steinhart = 1.0 / steinhart;                        // Invert
        steinhart -= 273.15;                                // convert to Celsius
        cached_ambient = round(steinhart);
    } else {                                                // about -30 *C, perhaps, the IRON is disconnected
        cached_ambient  = ambient_tempC;
    }
    return cached_ambient;
}

void IRON::adjust(uint16_t t) {
    if (t > temp_max) t = temp_max;             // Do not allow over heating
    temp_set = t;
}

// If any switch is short, its status is 'true'
void IRON::checkSWStatus(void) {
    if (millis() > check_tilt_ms) {
        check_tilt_ms = millis() + 100;
        if (!disconnected) {                    // Current through the IRON is registered
            uint16_t avg = tilt.read();
            if (300 < avg && avg < 700) {       // Middle state
                avg = tilt.average(analogRead(tPIN));
                if (avg < 300 || avg > 700) {   // Toggle state
                    tilt_toggle = true;
                }
            } else {
                tilt.update(analogRead(tPIN));          
            }
        }
    }
}

bool IRON::isIronTiltSwitch(bool reed) {
    bool ret = tilt_toggle;
    tilt_toggle = false;
    if (reed) {
        return tilt.read() < 300;
    }
    return ret;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
	public:
		SCREEN* next;                           // Pointer to the next screen
		SCREEN* nextL;                          // Pointer to the next Level screen, usually, setup
		SCREEN* main;                           // Pointer to the main screen
		SCREEN* no_iron;                       	// Pointer to the screen when the IRON was disconnected
		SCREEN() {
			next = nextL = main = no_iron = 0;
			update_screen  = 0;
			scr_timeout    = 0;
			time_to_return = 0;
		}
		virtual void init(void)                 { }
		virtual SCREEN* show(void)              { return this; }
		virtual SCREEN* menu(void)              { if (this->next != 0)  return this->next;  else return this; }
		virtual SCREEN* menu_long(void)      { if (this->nextL != 0) return this->nextL; else return this; }
		virtual void rotaryValue(int16_t value) { }
		bool    isSetup(void)                   { return (scr_timeout != 0); }
		void    forceRedraw(void)               { update_screen = 0; }
		virtual SCREEN* returnToMain(void) {
			if (main && scr_timeout && (millis() >= time_to_return)) {
				scr_timeout = 0;
				return main;
			}
			return this;
		}
		void resetTimeout(void) {
			if (scr_timeout > 0)
			time_to_return = millis() + scr_timeout*1000;
		}
		void setSCRtimeout(uint16_t t) {
			scr_timeout = t;
			resetTimeout(); 
		}
		bool wasRecentlyReset(void) {
			uint32_t to = (time_to_return - millis()) / 1000;
			return((scr_timeout - to) < 15);
		}
	protected:
		uint32_t update_screen;             	// Time in ms when the screen should be updated
		uint32_t scr_timeout;                   // Timeout is sec. to return to the main screen, canceling all changes
		uint32_t time_to_return;                // Time in ms to return to main screen
};

//---------------------------------------- class mainSCREEN [the soldering IRON is OFF] ------------------------
class mainSCREEN : public SCREEN {
	public:
		mainSCREEN(IRON* Iron, DSPL* DSP, RENC* ENC, BUZZER* Buzz, IRON_CFG* Cfg) {
			pIron = Iron;
			pD    = DSP;
			pEnc  = ENC;
			pBz   = Buzz;
			pCfg  = Cfg;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value); // Setup the preset temperature
	private:
		IRON*     pIron;                      	// Pointer to the IRON instance
		DSPL*     pD;                           // Pointer to the DSPLay instance
		RENC*     pEnc;                         // Pointer to the rotary encoder instance
		BUZZER*   pBz;                          // Pointer to the simple buzzer instance
		IRON_CFG* pCfg;                         // Pointer to the configuration instance
		uint32_t  clear_used_ms;                // Time in ms when used flag should be cleared (if > 0)
		uint32_t  change_display;               // Time in ms when to switch display between preset temperature and tip name                    
		bool      used;                         // Whether the IRON was used (was hot)
		bool      cool_notified;                // Whether there was cold notification played
		bool      show_tip;                     // Whether show the tip name instead of preset temperature
		const uint16_t period = 1000;           // The period to update the screen
		const uint32_t cool_notify_period = 120000; // The period to display 'cool' message (ms)
		const uint16_t show_temp = 20000;       // The period in ms to show the preset temperature
};

void mainSCREEN::init(void) {
	pIron->switchPower(false);
	uint16_t temp_set = pIron->presetTemp();
    int16_t  ambient  = pIron->ambientTemp();
	temp_set = pCfg->tempToHuman(temp_set, ambient); // The preset temperature in the human readable units
	if (pCfg->isCelsius())
		pEnc->reset(temp_set, temp_minC, temp_maxC, 1, 5);
	else
		pEnc->reset(temp_set, temp_minF, temp_maxF, 1, 5);
	used = !pIron->isCold();
	cool_notified = !used;
	if (used) {                                 // the IRON was used, we should save new data in EEPROM
		pCfg->savePresetTempHuman(temp_set);
	}
	clear_used_ms = 0;
	forceRedraw();
	pD->clear();
	pD->msgOff();
	show_tip = false;
	change_display = millis() + show_temp;
}

void mainSCREEN::rotaryValue(int16_t value) {
    int16_t ambient = pIron->ambientTemp();
	uint16_t temp = pCfg->humanToTemp(value, ambient);
	pIron->setTemp(temp);
	pD->tSet(value, pCfg->isCelsius());
	uint32_t ms = millis();
	update_screen  = ms + period;
	change_display = ms + show_temp;
	show_tip = false;
}

SCREEN* mainSCREEN::show(void) {
	SCREEN* nxt = this;
	if (no_iron && pIron->checkIron()) {      	// Check that the IRON is connected
		nxt = no_iron;
	}
	if (millis() < update_screen) return nxt;
	update_screen = millis() + period;

	if (clear_used_ms && (millis() > clear_used_ms)) {
		clear_used_ms = 0;
		used = false;
	}

	if (millis() > change_display) {
		show_tip = !show_tip;
		change_display = millis() + 2000;
		if (!show_tip) change_display += show_temp;
	}

    int16_t ambient = pIron->ambientTemp();
	if (show_tip) {
		pD->tip(pCfg->tipName(), true);
	} else {
		uint16_t temp_set = pIron->presetTemp();
		temp_set = pCfg->tempToHuman(temp_set, ambient);	// The preset temperature in the human readable units
		pD->tSet(temp_set, pCfg->isCelsius());
	}
	pD->msgOff();
 
	uint16_t temp   = pIron->tempAverage();
	uint16_t tempH  = pCfg->tempToHuman(temp, ambient);
	if (pIron->isCold()) {
		if (used)
			pD->msgCold();
		else
			pD->tCurr(tempH);
		if (used && !cool_notified) {
			pBz->lowBeep();
			cool_notified = true;
			clear_used_ms = millis() + cool_notify_period;
		}
	} else {
		pD->tCurr(tempH);
	}
	return nxt;
}

//---------------------------------------- class tipSCREEN [tip is disconnected, choose new tip] ---------------
class tipSCREEN : public SCREEN {
	public:
		tipSCREEN(IRON* Iron, DSPL* DSP, RENC* ENC, IRON_CFG* Cfg) {
			pIron = Iron;
			pD    = DSP;
			pEnc  = ENC;
			pCfg  = Cfg;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value); // Select the tip
	private:
        uint8_t     old_tip;                    // previous tip index
		IRON*       pIron;                      // Pointer to the IRON instance
		DSPL*       pD;                         // Pointer to the DSPLay instance
		RENC*       pEnc;                       // Pointer to the rotary encoder instance
		IRON_CFG*   pCfg;                       // Pointer to the configuration instance                      
		const uint16_t period = 1000;          	// The period to update the screen
};

void tipSCREEN::init(void) {
	pIron->switchPower(false);
	old_tip = pCfg->tipIndex();
	pEnc->reset(old_tip, 0, pCfg->tipsLoaded(), 1, 1, true);// Select the tip by the rotary encoder
	forceRedraw();
	pD->clear();
	pD->msgSelectTip();
}

void tipSCREEN::rotaryValue(int16_t value) {
    if (value == old_tip) return;
	update_screen   = millis() + period;
    uint8_t index   = pCfg->nextTip(old_tip, value > old_tip);
	uint16_t temp   = pIron->presetTemp();      // Preset temperature in internal units
    int16_t ambient = pIron->ambientTemp();
	temp = pCfg->tempToHuman(temp, ambient);    // The temperature in human readable units (Celsius o Fahrenheit)
	index = pCfg->selectTip(index);
    old_tip = index;
	pEnc->write(index);
	temp = pCfg->humanToTemp(temp, ambient);    // Translate previously set temperature in human readable units into internal value
	pIron->setTemp(temp);                       // Install previously set temperature into the IRON by new tip calibration
    forceRedraw();
}

SCREEN* tipSCREEN::show(void) {
	SCREEN* nxt = this;
	if (no_iron && !pIron->checkIron()) {       // Check that the IRON is disconnected
		nxt = no_iron;
		pIron->initTempHistory();               // The new tip is connected, reset the temp history 
	}
	if (millis() < update_screen) return nxt;
	update_screen = millis() + period;
    
	pD->tip(pCfg->tipName(), false);
	pD->mark('*', !pCfg->isCalibrated());
	return nxt;
}

//---------------------------------------- class actSCREEN [Toggle tip activation ] ----------------------------
class actSCREEN : public SCREEN {
    public:
        actSCREEN(IRON* Iron, DSPL* DSP, RENC* ENC, IRON_CFG* Cfg) {
            pIron = Iron;
            pD    = DSP;
            pEnc  = ENC;
            pCfg  = Cfg;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual SCREEN* menu(void);
        virtual void    rotaryValue(int16_t value); // Select the tip
    private:
        IRON*       pIron;                      // Pointer to the IRON instance
        DSPL*       pD;                         // Pointer to the DSPLay instance
        RENC*       pEnc;                       // Pointer to the rotary encoder instance
        IRON_CFG*   pCfg;                       // Pointer to the configuration instance
        const uint16_t period = 10000;          // The period to update the screen                     
};

void actSCREEN::init(void) {
    pIron->switchPower(false);
    char *n = pCfg->tipName();
    int8_t global_index = pCfg->index(n);       // Find current tip in the global tip array by the name
    if (global_index < 0) global_index = 0;
    pEnc->reset(global_index, 0, pCfg->tipsLoaded(), 1, 1, true);// Select the tip by the rotary encoder
    pD->clear();
    pD->msgActivateTip();
    pD->tip(pCfg->tipName(), false);
}

void actSCREEN::rotaryValue(int16_t value) {
    forceRedraw();
}

SCREEN* actSCREEN::menu(void) {
    uint8_t tip = pEnc->read();
    bool active = pCfg->toggleTipActivation(tip);
    forceRedraw();
    return this;
}

SCREEN* actSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    uint8_t tip = pEnc->read();
    bool active = pCfg->isTipActive(tip);
    char tip_name[tip_name_sz+1];
    pCfg->name(tip_name, tip);
    tip_name[tip_name_sz] = '\0';
    pD->clear();
    pD->tip(tip_name, false);
    pD->mark('x', active);
    return this;
}

//---------------------------------------- class workSCREEN [the soldering IRON is ON] -------------------------
class workSCREEN : public SCREEN {
	public:
		workSCREEN(IRON* Iron, DSPL* DSP, RENC* Enc, BUZZER* Buzz, IRON_CFG* Cfg) {
			update_screen = 0;
			pIron = Iron;
			pD    = DSP;
			pBz   = Buzz;
			pEnc  = Enc;
			pCfg  = Cfg;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value); // Change the preset temperature
		virtual SCREEN* returnToMain(void);    	// Automatic power-off
	private:
        void        adjustPresetTemp(void);
        void        hwTimeout(uint16_t low_temp, bool tilt_active);
		IRON*       pIron;                      // Pointer to the IRON instance
		DSPL*       pD;                         // Pointer to the DSPLay instance
		BUZZER*     pBz;                        // Pointer to the simple Buzzer instance
		RENC*       pEnc;                       // Pointer to the rotary encoder instance
		IRON_CFG*   pCfg;                       // Pointer to the configuration instance
		bool        ready;                      // Whether the IRON have reached the preset temperature
        bool        lowpower_mode   = false;    // Whether hardware low power mode using tilt switch
        uint32_t    lowpower_time   = 0;        // Time when switch to standby power mode
		uint32_t    auto_off_notified;          // The time (in ms) when the automatic power-off was notified
        uint16_t    tempH = 0;                  // The preset temperature in human readable units
		const uint16_t period = 1000;           // The period to update the screen (ms)
};

void workSCREEN::init(void) {
	uint16_t temp_set = pIron->presetTemp();
    int16_t ambient   = pIron->ambientTemp();
	bool is_celsius   = pCfg->isCelsius();
	tempH             = pCfg->tempToHuman(temp_set, ambient);
	if (is_celsius)
		pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
	else
		pEnc->reset(tempH, temp_minF, temp_maxF, 1, 5);
	pIron->switchPower(true);
	ready           = false;
    lowpower_mode   = false;
    lowpower_time   = 0;
    time_to_return  = 0;
	pD->clear();
	pD->tSet(tempH, is_celsius);
	pD->msgOn();
	uint16_t to = pCfg->getOffTimeout() * 60;
	this->setSCRtimeout(to);
	auto_off_notified = 0;
	forceRedraw();
}

void workSCREEN::rotaryValue(int16_t value) {   // Setup new preset temperature by rotating the encoder
    tempH = value;
	ready = false;
    lowpower_mode = false;
	pD->msgOn();
    int16_t ambient = pIron->ambientTemp();
	uint16_t temp = pCfg->humanToTemp(value, ambient); // Translate human readable temperature into internal value
	pIron->setTemp(temp);
	pD->tSet(value, pCfg->isCelsius());
	SCREEN::resetTimeout();
	update_screen = millis() + period;
}

SCREEN* workSCREEN::show(void) {
	SCREEN* nxt = this;
	if (millis() < update_screen) return nxt;
	update_screen = millis() + period;

	int16_t temp        = pIron->tempAverage();
	int16_t temp_set    = pIron->presetTemp();
    int16_t ambient     = pIron->ambientTemp();
	int tempH           = pCfg->tempToHuman(temp, ambient);
	pD->tCurr(tempH);
	uint8_t p = pIron->appliedPower();
	pD->percent(p);

	uint16_t td = pIron->tempDispersion();
	uint16_t pd = pIron->powerDispersion();
	int ap      = pIron->getAvgPower();
    uint16_t low_temp = pCfg->lowTemp();        // 'Standby temperature' setup in the main menu

	if ((abs(temp_set - temp) < 3) && (pIron->tempDispersion() <= 10) && (ap > 0))  {
		if (!ready) {
			pBz->shortBeep();
			ready = true;
			pD->msgReady();
            if (low_temp)
                lowpower_time = millis() + (uint32_t)pCfg->lowTimeout() * 1000;
			update_screen = millis() + (period << 2);
			return this;
		}
	}

    bool tilt_active = false;
    if (low_temp) {
        tilt_active = pIron->isIronTiltSwitch(pCfg->isReedType());
    }

    // If the automatic power-off feature is enabled, check the IRON status
    if (low_temp && ready && pCfg->getOffTimeout()) {       // The IRON has reaches the preset temperature                         
        hwTimeout(low_temp, tilt_active);       // Use hardware tilt switch to turn low power mode
    }

    if (!lowpower_mode && pCfg->isAmbientSensor())
        adjustPresetTemp();
        
	uint32_t to = (time_to_return - millis()) / 1000;
	if (ready) {
		if (scr_timeout > 0 && (to < 100)) {
			pD->timeToOff(to);
			if (!auto_off_notified) {
				pBz->shortBeep();
				auto_off_notified = millis();
			}
		} else if (lowpower_mode) {
            pD->msgStandby();
		} else if (SCREEN::wasRecentlyReset()) {
			pD->msgWorking();
		} else {
			pD->msgReady();
		}
	} else {
		pD->msgOn();
        resetTimeout();
	}
	return nxt;
}

SCREEN* workSCREEN::returnToMain(void) {
	if (main && scr_timeout && (millis() >= time_to_return)) {
		scr_timeout = 0;
		pBz->doubleBeep();
		return main;
	}
	return this;
}

void workSCREEN::adjustPresetTemp(void) {
    uint16_t presetTemp = pIron->presetTemp();              // Preset temperature (internal units)
    int16_t  ambient    = pIron->ambientTemp();
    uint16_t temp       = pCfg->humanToTemp(tempH, ambient); // Expected temperature of IRON in internal units
    if (temp != presetTemp) {                               // The ambient temperature have changed, we need to adjust preset temperature
        pIron->adjust(temp);
    }
}

void workSCREEN::hwTimeout(uint16_t low_temp, bool tilt_active) {
    uint32_t now_ms = millis();
    if (tilt_active) {                                      // If the IRON is used, Reset standby time
        lowpower_time = now_ms + (uint32_t)pCfg->lowTimeout() * 1000; // Convert timeout to milliseconds
        if (lowpower_mode) {                                // If the IRON is in low power mode, return to main working mode
            pIron->lowPowerMode(0);
            lowpower_time   = 0;
            lowpower_mode   = false;
            ready           = false;
            pD->msgOn();
        }
    } else if (!lowpower_mode) {
        if (lowpower_time) {
            if (now_ms >= lowpower_time) {
                int16_t  ambient    = pIron->ambientTemp();
                uint16_t temp_low   = pCfg->lowTemp();
                uint16_t temp       = pCfg->humanToTemp(temp_low, ambient);
                pIron->lowPowerMode(temp);
                auto_off_notified   = false;
                lowpower_mode       = true;
                resetTimeout();                             // Activate automatic power-off
                return;
            }
        } else {
            lowpower_time = now_ms + (uint32_t)pCfg->lowTimeout() * 1000;
        }
    }
}

//---------------------------------------- class powerSCREEN [fixed power to the IRON] -------------------------
class powerSCREEN : public SCREEN {
	public:
		powerSCREEN(IRON* Iron, DSPL* DSP, RENC* Enc, IRON_CFG* CFG) {
			pIron = Iron;
			pD    = DSP;
			pEnc  = Enc;
			pCfg  = CFG;
			on    = false;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value);
		virtual SCREEN* menu(void);
		virtual SCREEN* menu_long(void);
	private:
		IRON*     pIron;                        // Pointer to the IRON instance
		DSPL*     pD;                           // Pointer to the DSPLay instance
		RENC*     pEnc;                         // Pointer to the rotary encoder instance
		IRON_CFG* pCfg;                         // Pointer to the configuration instance
		uint32_t  update_screen;                // Time in ms to update the screen
		bool on;                                // Whether the power of soldering IRON is on
		const uint16_t period = 1000;          	// The period in ms to update the screen
};

void powerSCREEN::init(void) {
	uint8_t p = pIron->getAvgPower();
	uint8_t max_power = pIron->getMaxFixedPower();
	pEnc->reset(p, 0, max_power, 1);
	on = true;                                 	// Do start heating immediately
	pIron->switchPower(false);
	pIron->fixPower(p);
	pD->clear();
	pD->pSet(p);
}

SCREEN* powerSCREEN::show(void) {
	SCREEN* nxt = this;
	if (no_iron && pIron->checkIron()) {       	// Check that the IRON is connected
		nxt = no_iron;
	}
	if (millis() < update_screen) return nxt;
	update_screen = millis() + period;
	uint16_t temp = pIron->tempAverage();
    int16_t ambient = pIron->ambientTemp();
	temp = pCfg->tempToHuman(temp, ambient);
	pD->tCurr(temp);
	return nxt;
}

void powerSCREEN::rotaryValue(int16_t value) {
	pD->pSet(value);
	pIron->fixPower(value);
	on = true;
	update_screen = millis() + (period * 2);
}

SCREEN* powerSCREEN::menu(void) {
	on = !on;
	if (on) {
		uint16_t pos = pEnc->read();
		pIron->fixPower(pos);
		pD->clear();
		pD->pSet(pos);
	} else {
		pIron->fixPower(0);
		pD->clear();
		pD->pSet(0);
	}
	forceRedraw();
	return this;
}

SCREEN* powerSCREEN::menu_long(void) {
	pIron->fixPower(0);
	if (nextL) {
		pIron->switchPower(true);
		return nextL;
	}
	return this;
}

//---------------------------------------- class errorSCREEN [the soldering IRON error detected] ---------------
class errorSCREEN : public SCREEN {
	public:
		errorSCREEN(IRON* Iron, DSPL* DSP, BUZZER* Buzz) {
		pIron = Iron;
		pD    = DSP;
		pBz   = Buzz;
		}
		virtual void init(void) { pIron->switchPower(false); pD->clear(); pD->msgFail(); pBz->failedBeep(); }
	private:
		IRON*    pIron;                       	// Pointer to the IRON instance
		DSPL*    pD;                            // Pointer to the display instance
		BUZZER*  pBz;                           // Pointer to the simple Buzzer instance
};

//---------------------------------------- class configSCREEN [configuration menu] -----------------------------
class configSCREEN : public SCREEN {
	public:
		configSCREEN(IRON* Iron, DSPL* DSP, RENC* Enc, IRON_CFG* Cfg, BUZZER* Buzz) {
			pIron = Iron;
			pD    = DSP;
			pEnc  = Enc;
			pCfg  = Cfg;
            pBz   = Buzz;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value);
		virtual SCREEN* menu(void);
		virtual SCREEN* menu_long(void);
		SCREEN* calib;                          // Pointer to the calibration SCREEN
        SCREEN* activate;                       // Pointer to the tip activation SCREEN
	private:
		IRON*     	pIron;                      // Pointer to the IRON instance
		DSPL*     	pD;                         // Pointer to the DSPLay instance
		RENC*  	    pEnc;                       // Pointer to the rotary encoder instance
		IRON_CFG* 	pCfg;                       // Pointer to the config instance
        BUZZER*     pBz;                        // Pointer to the buzzer instance
		uint8_t 	mode;                       // Which parameter to change
		bool 		tune;                       // Whether the parameter is modifying
		bool 		changed;                    // Whether some configuration parameter has been changed
		bool 		cels;                       // Current Celsius/Fahrenheit;
        bool        buzzer;                     // Buzzer ON/OFF
        bool        reed;                       // The hardware switch type: reed/tilt
        bool        ambient;                    // The ambient temperature sensor enabled/disabled
		uint8_t		off_timeout;                // Automatic switch-off timeout in minutes
        uint16_t    low_temp;                   // Standby temperature
        uint8_t     low_timeout;                // Standby timeout
		const uint16_t period = 10000;          // The period in ms to update the screen
};

void configSCREEN::init(void) {
	mode = 0;
    // 0 - C/F, 1 - buzzer, 2 - switch type, 3 - ambient, 4 - standby temp, 5 - standby time,
    // 6 - off-timeout, 7 - tip calibrate, 8 - atcivate tip, 9 - tune, 10 - save, 11 - cancel
	pEnc->reset(mode, 0, 11, 1, 0, true);
	tune        = false;
	changed     = false;
	cels        = pCfg->isCelsius();
    buzzer      = pCfg->isBuzzer();
    reed        = pCfg->isReedType();
    ambient     = pCfg->isAmbientSensor();
	off_timeout = pCfg->getOffTimeout();
    low_temp    = pCfg->lowTemp();
    low_timeout = pCfg->lowTimeout();
	pD->clear();
	pD->setupMode(0, false, 0);
	this->setSCRtimeout(30);
}

SCREEN* configSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;
	switch (mode) {
		case 0:                                 // C/F
			pD->setupMode(mode, false, cels);
			break;
		case 1:                                 // buzzer
            pD->setupMode(mode, tune, buzzer);
			break;
        case 2:                                 // switch type
            pD->setupMode(mode, tune, reed);
            break;
        case 3:                                 // ambient temperature sensor
            pD->setupMode(mode, tune, ambient);
            break;
		case 4:                                 // standby temp
            pD->setupMode(mode, tune, low_temp);
            break;
        case 5:                                 // standby timeout
            pD->setupMode(mode, tune, low_timeout);
            break;
        case 6:                                 // off-timeout
            pD->setupMode(mode, tune, off_timeout);
            break;
        default:
			pD->setupMode(mode, false, 0);
			break;
	}
	return this;
}

void configSCREEN::rotaryValue(int16_t value) {
	if (tune) {                               	// tune the temperature units
		changed = true;
		switch (mode) {
            case 1:                             // Buzzer
                buzzer = value; 
                break;
            case 2:                             // Switch type
                reed = value;
                break;
            case 3:                             // ambient temperature sensor
                ambient = value;
                break;
            case 4:                             // standby temperature
                if (value > 0)
                    value += 179;
                low_temp = value;
                break;
            case 5:                             // Standby Time
                low_timeout = value;
                break; 
			case 6:                            	// tuning the switch-off timeout
				if (value > 0) value += 2;      // The minimum timeout is 3 minutes
				off_timeout = value;
				break;
			default:
				break;
		}
	} else {
		mode = value;
	}
	forceRedraw();
}

SCREEN* configSCREEN::menu(void) {
	if (tune) {
		tune = false;
		pEnc->reset(mode, 0, 11, 1, 0, true);   // The value has been tuned, return to the menu list mode
	} else {
		switch (mode) {
            case 0:                             // C/F. In-line editing
                cels = !cels;
                changed = true;
                forceRedraw();
                return this;
            case 1:                             // Buzzer
                pEnc->reset(buzzer, 0, 1, 1, 0, true);
                break;
            case 2:                             // Switch type
                pEnc->reset(reed, 0, 1, 1, 0, true);
                break;
            case 3:                             // ambient temperature sensor
                pEnc->reset(ambient, 0, 1, 1, 0, true);
                break;
            case 4:                             // standby temperature
                {
                uint16_t v = low_temp;
                if (v > 0) v -= 179;
                pEnc->reset(v, 0, 120, 1, 5, false);
                }
                break;
            case 5:                             // standby timeout
                pEnc->reset(low_timeout, 10, 255, 1, 5, false);
                break;
            case 6:                             // off-timeout
                {
                int v = off_timeout;
                if (v > 0) v -= 2;
                pEnc->reset(v, 0, 28, 1, 5, false);
                }
                break;
			case 7:
				if (calib) return calib;
				break;
            case 8:                             // Activate tip
                if (activate) return activate;
                break;
			case 9:                           	// Tune potentiometer
				if (next) return next;
				break;
			case 10:                           	// Save configuration data
				menu_long();
			case 11:                            // Return to the main menu
				if (main) return main;
				return this;
		}
		tune = true;
	}
	forceRedraw();
	return this;
}

SCREEN* configSCREEN::menu_long(void) {
	if (nextL) {
		if (changed) {
            pCfg->setLowPower(low_temp, low_timeout, reed);
			pCfg->saveConfig(off_timeout, cels, buzzer, ambient);
            pBz->activate(buzzer);
		}
		return nextL;
	}
	return this;
}

//---------------------------------------- class calibSCREEN [ tip calibration ] -------------------------------
#define MCALIB_POINTS	8
class calibSCREEN : public SCREEN {
	public:
		calibSCREEN(IRON* Iron, DSPL* DSP, RENC* Enc, IRON_CFG* Cfg, BUZZER* Buzz) {
			pIron = Iron;
			pD    = DSP;
			pEnc  = Enc;
			pCfg  = Cfg;
			pBz   = Buzz;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value);
		virtual SCREEN* menu(void);
		virtual SCREEN* menu_long(void);
	private:
		bool		calibrationOLS(uint16_t* tip, uint16_t min_temp, uint16_t max_temp);
		uint8_t 	closestIndex(uint16_t temp);
        void        updateReference(uint8_t indx);
		IRON*		pIron;                      // Pointer to the IRON instance
		DSPL*		pD;                         // Pointer to the DSPLay instance
		RENC*	    pEnc;                       // Pointer to the rotary encoder instance
		IRON_CFG*	pCfg;                       // Pointer to the config instance
		BUZZER*		pBz;                        // Pointer to the buzzer instance
		uint16_t	calib_temp[2][MCALIB_POINTS];	// The calibration data: real temp. [0] and temp. in internal units [1]
		uint8_t		ref_temp_index	= 0;		// Which temperature reference to change: [0-MCALIB_POINTS]
		uint16_t	tip_temp_max	= 0;		// the maximum possible tip temperature in the internal units
		uint16_t	preset_temp;                // The preset temp in human readable units
		bool		cels;                       // Current Celsius/Fahrenheit;
		bool		ready;                      // Ready to enter real temperature
		const uint16_t t_diff = 60;             // The adjustment could be in the interval [t_ref-t_diff; t_ref+2*t_diff]
		const uint32_t period = 1000;          	// Update screen period
		const uint16_t start_int_temp = 200;	// Minimal temperature in internal units, about 100 degrees Celsius
};

void calibSCREEN::init(void) {
	pIron->switchPower(false);
	// Prepare to enter real temperature
	uint16_t min_t 		= 50;
	uint16_t max_t		= 600;
	if (!pCfg->isCelsius()) {
		min_t 	=  122;
		max_t 	= 1111;
	}
	pEnc->reset(0, min_t, max_t, 1, 5, false);
    tip_temp_max = temp_max / 2;
	for (uint8_t i = 0; i < MCALIB_POINTS; ++i) {
		calib_temp[0][i] = 0;					// Real temperature. 0 means not entered yet
		calib_temp[1][i] = map(i, 0, MCALIB_POINTS-1, start_int_temp, tip_temp_max); // Internal temperature
	}
	ready 			= false;                    // Not ready to enter real temperature
	ref_temp_index	= 0;
	pD->tRef(ref_temp_index);
	preset_temp = pIron->presetTemp();          // Save the preset temperature in human readable units
    int16_t ambient = pIron->ambientTemp();
	preset_temp = pCfg->tempToHuman(preset_temp, ambient);
    pD->clear();
    pD->msgOff();
	forceRedraw();
}

SCREEN* calibSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;
	int16_t temp		= pIron->tempAverage(); // Actual IRON temperature
	int16_t temp_set	= pIron->presetTemp();
    int16_t ambient     = pIron->ambientTemp();
    uint16_t tempH      = pCfg->tempToHuman(temp, ambient);
	if (ready) {
		pD->tSet(tempH, cels);
	} else {                                  // Show the current Iron temperature
        pD->tCurr(tempH);
	}
	uint8_t p = pIron->appliedPower();
	if (!pIron->isOn()) p = 0;
	pD->percent(p);
	if (! ready && abs(temp_set - temp) < 4 && pIron->tempDispersion() <= 20)  {
		pBz->shortBeep();
		pD->msgReady();
	    ready = true;
        pEnc->write(tempH);
	}
	if (ready && !pIron->isOn()) {          	// The IRON was switched off by error
		pD->msgOff();
		ready = false;
	}
	return this;
}

void calibSCREEN::rotaryValue(int16_t value) {	// The Encoder rotated
	update_screen = millis() + period;
	if (ready) {                               	// change the real value for the temperature
		pD->tCurr(value);
	}
}

SCREEN* calibSCREEN::menu(void) {				// Rotary encoder pressed
	if (ready) {                                // The real temperature has been entered
		uint16_t r_temp = pEnc->read();
		uint16_t temp   = pIron->tempAverage();	// The temperature of the IRON in internal units
		pIron->switchPower(false);
		pD->msgOff();
		if (!cels)                                // Always save the human readable temperature in Celsius
			r_temp = map(r_temp, temp_minF, temp_maxF, temp_minC, temp_maxC);
		calib_temp[0][ref_temp_index] = r_temp;
		calib_temp[1][ref_temp_index] = temp;
		if (r_temp < temp_maxC - 20) {
			updateReference(ref_temp_index);	// Update reference temperature points
			++ref_temp_index;
			// Try to update the current tip calibration
			uint16_t tip[3];
			if (calibrationOLS(tip, 150, 600)) {
				pCfg->applyCalibration(tip);
			}
		}
		if ((r_temp >= temp_maxC - 20) || ref_temp_index >= MCALIB_POINTS) {
			return menu_long();                 // Finish calibration
		} else {								// Continue calibration
			uint16_t temp = calib_temp[1][ref_temp_index];
			pIron->setTemp(temp);
			pIron->switchPower(true);	
		}
	} else {									// Toggle the power
		if (pIron->isOn()) {
			pIron->switchPower(false);
			pD->msgOff();
		} else {
			pIron->switchPower(true);
			pD->msgOn();
		}
	}
	pD->tRef(ref_temp_index);
	ready = false;
	forceRedraw();
	return this;
}

SCREEN* calibSCREEN::menu_long(void) {    	// Save new tip calibration data
	pIron->switchPower(false);
	uint16_t tip[3];
	if (calibrationOLS(tip, 150, temp_maxC)) {
		uint8_t near_index	= closestIndex(temp_tip[2]);
		tip[2] = map(temp_tip[2], temp_tip[1], calib_temp[0][near_index],
				tip[1], calib_temp[1][near_index]);
		if (tip[2] > temp_max) tip[2] = temp_max;

		pCfg->applyCalibration(tip);
		pCfg->saveCalibrationData(tip, ambient_tempC);
	}
	pCfg->savePresetTempHuman(preset_temp);
    int16_t ambient = pIron->ambientTemp();
	uint16_t temp = pCfg->humanToTemp(preset_temp, ambient);
	pIron->setTemp(temp);
	if (nextL) return nextL;
	return this;
}

/*
 * Calculate tip calibration parameter using linear approximation by Ordinary Least Squares method
 * Y = a * X + b, where
 * Y - internal temperature, X - real temperature. a and b are double coefficients
 * a = (N * sum(Xi*Yi) - sum(Xi) * sum(Yi)) / ( N * sum(Xi^2) - (sum(Xi))^2)
 * b = 1/N * (sum(Yi) - a * sum(Xi))
 */
bool calibSCREEN::calibrationOLS(uint16_t* tip, uint16_t min_temp, uint16_t max_temp) {
	long sum_XY = 0;							// sum(Xi * Yi)
	long sum_X 	= 0;							// sum(Xi)
	long sum_Y  = 0;							// sum(Yi)
	long sum_X2 = 0;							// sum(Xi^2)
	long N		= 0;

	for (uint8_t i = 0; i < MCALIB_POINTS; ++i) {
		uint16_t X 	= calib_temp[0][i];
		uint16_t Y	= calib_temp[1][i];
		if (X >= min_temp && X <= max_temp) {
			sum_XY 	+= X * Y;
			sum_X	+= X;
			sum_Y   += Y;
			sum_X2  += X * X;
			++N;
		}
	}

	if (N <= 2)									// Not enough real temperatures have been entered
		return false;

	double	a  = (double)N * (double)sum_XY - (double)sum_X * (double)sum_Y;
			a /= (double)N * (double)sum_X2 - (double)sum_X * (double)sum_X;
	double 	b  = (double)sum_Y - a * (double)sum_X;
			b /= (double)N;

	for (uint8_t i = 0; i < 3; ++i) {
		double temp = a * (double)temp_tip[i] + b;
		tip[i] = round(temp);
	}
	if (tip[2] > temp_max) tip[2] = temp_max;
	return true;
}

// Find the index of the reference point with the closest temperature
uint8_t calibSCREEN::closestIndex(uint16_t temp) {
	uint16_t diff = 1000;
	uint8_t index = MCALIB_POINTS;
	for (uint8_t i = 0; i < MCALIB_POINTS; ++i) {
		uint16_t X = calib_temp[0][i];
		if (X > 0 && abs(X-temp) < diff) {
			diff = abs(X-temp);
			index = i;
		}
	}
	return index;
}

void calibSCREEN::updateReference(uint8_t indx) {                    // Update reference points
    uint16_t expected_temp  = map(indx, 0, MCALIB_POINTS, temp_minC, temp_maxC);
    uint16_t r_temp         = calib_temp[0][indx];
    if (indx < 5 && r_temp > (expected_temp + expected_temp/4)) {   // The real temperature is too high
        tip_temp_max -= tip_temp_max >> 2;                      // tip_temp_max *= 0.75;
        if (tip_temp_max < temp_max / 4)
            tip_temp_max = temp_max / 4;                        // Limit minimum possible value of the highest temperature
    } else if (r_temp > (expected_temp + expected_temp/8)) {    // The real temperature is biger than expected
        tip_temp_max += tip_temp_max >> 3;                      // tip_temp_max *= 1.125;
        if (tip_temp_max > temp_max)
            tip_temp_max = temp_max;
    } else if (indx < 5 && r_temp < (expected_temp - expected_temp/4)) { // The real temperature is too low
        tip_temp_max += tip_temp_max >> 2;                      // tip_temp_max *= 1.25;
        if (tip_temp_max > temp_max)
            tip_temp_max = temp_max;
    } else if (r_temp < (expected_temp - expected_temp/8)) {    // The real temperature is lower than expected
        tip_temp_max += tip_temp_max >> 3;                      // tip_temp_max *= 1.125;
        if (tip_temp_max > temp_max)
            tip_temp_max = temp_max;
    } else {
        return;
    }
    // rebuild the array of the reference temperatures
    for (uint8_t i = indx+1; i < MCALIB_POINTS; ++i) {
        calib_temp[1][i] = map(i, 0, MCALIB_POINTS-1, start_int_temp, tip_temp_max);
    }

}

//---------------------------------------- class tuneSCREEN [tune the potentiometer ] --------------------------
class tuneSCREEN : public SCREEN {
	public:
		tuneSCREEN(IRON* Iron, DSPL* DSP, RENC* ENC, BUZZER* Buzz) {
			pIron = Iron;
			pD    = DSP;
			pEnc  = ENC;
			pBz   = Buzz;
		}
		virtual void    init(void);
		virtual SCREEN* menu(void);
		virtual SCREEN* menu_long(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value);
	private:
		IRON*    pIron;							// Pointer to the IRON instance
		DSPL*    pD;                            // Pointer to the display instance
		RENC*   pEnc;                          // Pointer to the rotary encoder instance
		BUZZER*  pBz;                           // Pointer to the simple Buzzer instance
		bool     arm_beep;                      // Whether beep is armed
		uint8_t  max_power;                     // Maximum possible power to be applied
		const uint16_t period = 1000;           // The period in ms to update the screen
};

void tuneSCREEN::init(void) {
	pIron->switchPower(false);
	max_power = pIron->getMaxFixedPower();
	pEnc->reset(15, 0, max_power, 1, 5);        // Rotate the encoder to change the power supplied
	arm_beep = false;
	pD->clear();
	pD->msgTune();
	forceRedraw();
}

void tuneSCREEN::rotaryValue(int16_t value) {
	pIron->fixPower(value);
	forceRedraw();
}

SCREEN* tuneSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;
	int16_t temp = pIron->tempAverage();
	pD->tCurr(temp);
	uint8_t power = pEnc->read();				// applied power
	if (!pIron->isOn())
		power = 0;
	else
		power = map(power, 0, max_power, 0, 100);
	pD->percent(power);
	if (arm_beep && (pIron->tempDispersion() < 5)) {
		pBz->shortBeep();
		arm_beep = false;
	}
	return this;
}
  
SCREEN* tuneSCREEN::menu(void) {                // The rotary button pressed
	if (pIron->isOn()) {
		pIron->fixPower(0);
	} else {
		uint8_t power = pEnc->read();			// applied power
		pIron->fixPower(power);
	}
	return this;
}

SCREEN* tuneSCREEN::menu_long(void) {
	pIron->fixPower(0);							// switch off the power
	if (next) return next;
	return this;
}

//---------------------------------------- class pidSCREEN [tune the PID coefficients] -------------------------
class pidSCREEN : public SCREEN {
	public:
		pidSCREEN(IRON* Iron, RENC* ENC) {
			pIron = Iron;
			pEnc  = ENC;
		}
		virtual void    init(void);
		virtual SCREEN* menu(void);
		virtual SCREEN* menu_long(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value);
	private:
		void     showCfgInfo(void);				// show the main config information: Temp set and PID coefficients
		IRON*    pIron;							// Pointer to the IRON instance
		RENC*    pEnc;							// Pointer to the rotary encoder instance
		uint8_t  mode;							// Which temperature to tune [0-3]: select, Kp, Ki, Kd
		uint32_t update_screen;					// Time in ms when update the screen (print nre info)
		int      temp_set;
		const uint16_t period = 500;
};

void pidSCREEN::init(void) {
	temp_set = pIron->presetTemp();
	mode = 0;									// select the element from the list
	pEnc->reset(1, 1, 4, 1, 1, true);			// 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp 
	showCfgInfo();
	Serial.println("");
}

void pidSCREEN::rotaryValue(int16_t value) {
	if (mode == 0) {							// No limit is selected, list the menu
		showCfgInfo();
		switch (value) {
			case 1:
				Serial.println("Kp");
				break;
			case 2:
				Serial.println("Ki");
				break;
			case 4:
				Serial.println(F("Temp"));
				break;
			case 3:
			default:
				Serial.println("Kd");
				break;
		}
	} else {
		switch (mode) {
			case 1:
				Serial.print(F("Kp = "));
				pIron->changePID(mode, value);
				break;
			case 2:
				Serial.print(F("Ki = "));
				pIron->changePID(mode, value);
				break;
			case 4:
				Serial.print(F("Temp = "));
				temp_set = value;
				pIron->setTemp(value);
				break;
			case 3:
			default:
				Serial.print(F("Kd = "));
				pIron->changePID(mode, value);
				break;
		}
		Serial.println(value);
	}
}

SCREEN* pidSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;
	if (pIron->isOn()) {
		char buff[60];
		int temp     = pIron->currTemp();
		uint8_t  pwr = pIron->getAvgPower();
		uint16_t td  = pIron->tempDispersion();
		uint16_t pd  = pIron->powerDispersion();
		sprintf(buff, "%3d: power = %3d, td = %3d, pd = %3d --- ", temp_set - temp, pwr, td, pd);
		Serial.println(buff);
	}
	return this;
}

SCREEN* pidSCREEN::menu(void) {					// The rotary button pressed
	if (mode == 0) {							// select upper or lower temperature limit
		mode = pEnc->read();
		if (mode != 4) {
			int k = pIron->changePID(mode, -1);
			pEnc->reset(k, 0, 10000, 1, 10);
		} else {
			pEnc->reset(temp_set, 0, 970, 1, 5);
		}
	} else {									// upper or lower temperature limit just setup     
		mode = 0;
		pEnc->reset(1, 1, 4, 1, 1, true);		// 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp
	}
	return this;
}

SCREEN* pidSCREEN::menu_long(void) {
	bool on = pIron->isOn();
	pIron->switchPower(!on);
	if (on)
		Serial.println("The iron is OFF");
	else
		Serial.println("The iron is ON");
	return this;
}

void pidSCREEN::showCfgInfo(void) {
	Serial.print(F("Temp set: "));
	Serial.print(temp_set, DEC);
	Serial.print(F(", PID: ["));
	for (uint8_t i = 1; i < 4; ++i) {
		int k = pIron->changePID(i, -1);
		Serial.print(k, DEC);
		if (i < 3) Serial.print(", ");
	}
	Serial.print("]; ");
}
//=================================== End of class declarations ================================================

DSPL		disp;
RENC        encoder(R_MAIN_PIN, R_SECD_PIN, R_BUTN_PIN);
IRON		iron(probePIN, checkPIN, termisPIN, tiltswPIN);
IRON_CFG	ironCfg(MAX_CUSTOM_TIPS);               // See config.h
BUZZER		simpleBuzzer(buzzerPIN);
TIPS        tips;

mainSCREEN		offScr(&iron, &disp, &encoder, &simpleBuzzer, &ironCfg);
tipSCREEN		selScr(&iron, &disp, &encoder, &ironCfg);
workSCREEN		wrkScr(&iron, &disp, &encoder, &simpleBuzzer, &ironCfg);
errorSCREEN		errScr(&iron, &disp, &simpleBuzzer);
powerSCREEN		pwrScr(&iron, &disp, &encoder, &ironCfg);
configSCREEN	cfgScr(&iron, &disp, &encoder, &ironCfg, &simpleBuzzer);
calibSCREEN		tipScr(&iron, &disp, &encoder, &ironCfg, &simpleBuzzer);
actSCREEN       actScr(&iron, &disp, &encoder, &ironCfg);
tuneSCREEN		tuneScr(&iron, &disp, &encoder, &simpleBuzzer);
//pidSCREEN		pidScr(&iron, &encoder);

SCREEN *pCurrentScreen = &offScr;
//SCREEN *pCurrentScreen = &pidScr;

/*
 * The timer1 overflow interrupt handler.
 * Activates the procedure for IRON current check or for IRON temperature check
 * Interrupt routine on Timer1 overflow, @31250 Hz, 32 microseconds is a timer period
 * keepTemp() function takes 353 mks, about 12 ticks of TIMER1;
 * We should wait for 33 timer ticks before checking the temperature after iron was powered off
 */
const uint32_t period_ticks = (31250 * temp_check_period)/1000-33-12;
ISR(TIMER1_OVF_vect) {
	if (iron_off) {									// The IRON is switched off, we need to check the temperature
		if (++tmr1_count >= 33) {					// about 1 millisecond
			TIMSK1 &= ~_BV(TOIE1);					// disable the overflow interrupts
			iron.keepTemp();						// Check the temp. If on, keep the temperature
			tmr1_count = 0;
			iron_off = false;
			TIMSK1 |= _BV(TOIE1);					// enable the the overflow interrupts
		}
	} else {										// The IRON is on, check the current and switch-off the IRON
		if (++tmr1_count >= period_ticks) {
			TIMSK1 &= ~_BV(TOIE1);					// disable the overflow interrupts
			tmr1_count = 0;
			OCR1B      = 0;							// Switch-off the power to check the temperature
			PORTB     &= ~heaterBIT;
			iron_off   = true;
			TIMSK1    |= _BV(TOIE1);				// enable the overflow interrupts
		}
	}
}

// the setup routine runs once when you press reset:
void setup() {
//	Serial.begin(115200);
	disp.init();

	// Load configuration parameters
	ironCfg.init();
	iron.init();
	uint16_t temp   = ironCfg.tempPresetHuman();
    int16_t ambient = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        int16_t amb = iron.ambientTemp();
        if (amb == ambient) break;
        delay(500);
        ambient = amb;
    }
    temp = ironCfg.humanToTemp(temp, ambient);
	iron.setTemp(temp);
    simpleBuzzer.activate(ironCfg.isBuzzer());

	// Initialize rotary encoder
	encoder.init();
	delay(500);
	attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);

	// Initialize SCREEN hierarchy
	offScr.next     = &wrkScr;
	offScr.nextL    = &cfgScr;
	offScr.no_iron  = &selScr;
	wrkScr.next     = &offScr;
	wrkScr.nextL    = &pwrScr;
	wrkScr.main     = &offScr;
	errScr.next     = &offScr;
	errScr.nextL    = &offScr;
	pwrScr.nextL    = &wrkScr;
	pwrScr.no_iron  = &errScr;
	cfgScr.next     = &tuneScr;
	cfgScr.nextL    = &offScr;
	cfgScr.main     = &offScr;
	cfgScr.calib    = &tipScr;
    cfgScr.activate = &actScr;
	tipScr.nextL    = &offScr;
	tuneScr.next    = &cfgScr;
	tuneScr.main    = &offScr;
	selScr.nextL    = &cfgScr;
	selScr.no_iron  = &offScr;
    actScr.main     = &offScr;
    actScr.nextL    = &offScr;
	pCurrentScreen->init();
}

// Encoder interrupt handler
static void rotEncChange(void) {
    encoder.encoderIntr();
}

// The main loop
void loop() {
	static int16_t  old_pos = encoder.read();
  
	SCREEN* nxt = pCurrentScreen->returnToMain();
	if (nxt != pCurrentScreen) {				// return to the main screen by timeout
		pCurrentScreen = nxt;
		pCurrentScreen->init();
	}

	int16_t pos = encoder.read();
	if (old_pos != pos) {
		pCurrentScreen->rotaryValue(pos);
		old_pos = pos;
		if (pCurrentScreen->isSetup())
			pCurrentScreen->resetTimeout();
	}

	uint8_t bStatus = encoder.buttonCheck();
	switch (bStatus) {
		case 2:									// int32_t press;
			nxt = pCurrentScreen->menu_long();
			if (nxt != pCurrentScreen) {
				pCurrentScreen = nxt;
				pCurrentScreen->init();
			} else {
				if (pCurrentScreen->isSetup())
					pCurrentScreen->resetTimeout();
			}
			break;
		case 1:									// short press
			nxt = pCurrentScreen->menu();
			if (nxt != pCurrentScreen) {
				pCurrentScreen = nxt;
				pCurrentScreen->init();
			} else {
				if (pCurrentScreen->isSetup())
					pCurrentScreen->resetTimeout();
			}
			break;
		case 0:									// Not pressed
		default:
			break;
	}

	nxt = pCurrentScreen->show();
	if (nxt && pCurrentScreen != nxt) {			// Be paranoiac, the returned value must not be null 
		pCurrentScreen = nxt;
		pCurrentScreen->init();
	}

    iron.checkSWStatus();
}
