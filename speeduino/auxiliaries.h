#ifndef AUX_H
#define AUX_H

#include "globals.h" /* For BOARD_H. */
#include "auxiliary_pins.h"

#if defined(CORE_AVR)
#include <util/atomic.h>
#endif

void initialiseAuxPWM(void);
void boostControl(void);
void boostDisable(void);
void boostByGear(void);
void vvtControl(void);
void initialiseFan(void);
void initialiseAirCon(void);
void nitrousControl(void);
void fanControl(void);
void airConControl(void);
bool READ_AIRCON_REQUEST(void);
void wmiControl(void);

#define SIMPLE_BOOST_P  1
#define SIMPLE_BOOST_I  1
#define SIMPLE_BOOST_D  1

#define AIRCON_ON() { configPage15.airConCompPol==1 ? AirConComp.off() : AirConComp.on(); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_OFF() { configPage15.airConCompPol==1 ? AirConComp.on() : AirConComp.off(); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }

#define AIRCON_FAN_ON() { configPage15.airConFanPol==1 ? AirConFan.off() : AirConFan.on(); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_FAN); }
#define AIRCON_FAN_OFF() { configPage15.airConFanPol==1 ? AirConFan.on() : AirConFan.off(); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_FAN); }

#define FAN_ON() do { \
  configPage6.fanInv ? Fan.off() : Fan.on(); \
  } while (0)

#define FAN_OFF() do { \
  configPage6.fanInv ? Fan.on() : Fan.off(); \
  } while (0)


#define VVT_TIME_DELAY_MULTIPLIER  50

#define WMI_TANK_IS_EMPTY() (WMIEmpty.is_configured() ? (configPage10.wmiEmptyPolarity == 0) ^ WMIEmpty.read() : 1)

#if defined(PWM_FAN_AVAILABLE)//PWM fan not available on Arduino MEGA
extern uint16_t fan_pwm_max_count; //Used for variable PWM frequency
void fanInterrupt(void);
#endif

extern uint16_t vvt_pwm_max_count; //Used for variable PWM frequency
extern uint16_t boost_pwm_max_count; //Used for variable PWM frequency

void boostInterrupt(void);
void vvtInterrupt(void);

#endif