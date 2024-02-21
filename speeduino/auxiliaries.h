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

#define N2O_STAGE1_PIN_LOW() NitrousStage1.off()
#define N2O_STAGE1_PIN_HIGH() NitrousStage1.on()
#define N2O_STAGE2_PIN_LOW() NitrousStage2.off()
#define N2O_STAGE2_PIN_HIGH() NitrousStage2.on()

#define BOOST_PIN_LOW() Boost.off()
#define BOOST_PIN_HIGH() Boost.on()

#define AIRCON_PIN_LOW() AirConComp.off()
#define AIRCON_PIN_HIGH() AirConComp.on()
#define AIRCON_ON() { configPage15.airConCompPol==1 ? AIRCON_PIN_LOW() : AIRCON_PIN_HIGH(); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_OFF() { configPage15.airConCompPol==1 ? AIRCON_PIN_HIGH() : AIRCON_PIN_LOW(); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }

#define AIRCON_FAN_PIN_LOW() AirConFan.off()
#define AIRCON_FAN_PIN_HIGH() AirConFan.on()

#define AIRCON_FAN_ON() { configPage15.airConFanPol==1 ? AIRCON_FAN_PIN_LOW() : AIRCON_FAN_PIN_HIGH(); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_FAN); }
#define AIRCON_FAN_OFF() { configPage15.airConFanPol==1 ? AIRCON_FAN_PIN_HIGH() : AIRCON_FAN_PIN_LOW(); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_FAN); }

#define FUEL_PUMP_ON() FuelPump.on()
#define FUEL_PUMP_OFF() FuelPump.off()

#define FAN_PIN_LOW() Fan.off()
#define FAN_PIN_HIGH() Fan.on()

#define FAN_ON() do { \
  configPage6.fanInv ? FAN_PIN_LOW() : FAN_PIN_HIGH(); \
  } while (0)

#define FAN_OFF() do { \
  configPage6.fanInv ? FAN_PIN_HIGH() : FAN_PIN_LOW(); \
  } while (0)


#define READ_N2O_ARM_PIN() NitrousArming.read()

#define VVT1_PIN_ON() VVT_1.on()
#define VVT1_PIN_OFF() VVT_1.off()
#define VVT2_PIN_ON() VVT_2.on()
#define VVT2_PIN_OFF() VVT_2.off()
#define VVT_TIME_DELAY_MULTIPLIER  50

#define WMI_TANK_IS_EMPTY() (WMIEmptyEnabled ? (configPage10.wmiEmptyPolarity == 0) ^ digitalRead(pinWMIEmpty) : 1)

#if defined(PWM_FAN_AVAILABLE)//PWM fan not available on Arduino MEGA
extern uint16_t fan_pwm_max_count; //Used for variable PWM frequency
void fanInterrupt(void);
#endif

extern uint16_t vvt_pwm_max_count; //Used for variable PWM frequency
extern uint16_t boost_pwm_max_count; //Used for variable PWM frequency

void boostInterrupt(void);
void vvtInterrupt(void);

#endif