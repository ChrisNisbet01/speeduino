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

#if(defined(CORE_TEENSY) || defined(CORE_STM32))

#define N2O_STAGE1_PIN_LOW() digitalWrite(configPage10.n2o_stage1_pin, LOW)
#define N2O_STAGE1_PIN_HIGH() digitalWrite(configPage10.n2o_stage1_pin, HIGH)
#define N2O_STAGE2_PIN_LOW() digitalWrite(configPage10.n2o_stage2_pin, LOW)
#define N2O_STAGE2_PIN_HIGH() digitalWrite(configPage10.n2o_stage2_pin, HIGH)

#else

static inline void
pin_set(volatile PORT_TYPE &port, PINMASK_TYPE const mask)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    port |= mask;
  }
}

static inline void
pin_clear(volatile PORT_TYPE &port, PINMASK_TYPE const mask)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    port &= ~mask;
  }
}

static inline void
pin_toggle(volatile PORT_TYPE &port, PINMASK_TYPE const mask)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    port &= ~mask;
  }
}

static inline void
bit_set_atomic(uint8_t &value, uint8_t const bit)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    BIT_SET(value, bit);
  }
}

static inline void
bit_clear_atomic(uint8_t &value, uint8_t const bit)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    BIT_CLEAR(value, bit);
  }
}

#define N2O_STAGE1_PIN_LOW() pin_clear(*n2o_stage1_pin_port, n2o_stage1_pin_mask)
#define N2O_STAGE1_PIN_HIGH() pin_set(*n2o_stage1_pin_port, n2o_stage1_pin_mask)

#define N2O_STAGE2_PIN_LOW() pin_clear(*n2o_stage2_pin_port, n2o_stage2_pin_mask)
#define N2O_STAGE2_PIN_HIGH() pin_set(*n2o_stage2_pin_port, n2o_stage2_pin_mask)

#endif

#define BOOST_PIN_LOW() boost.off()
#define BOOST_PIN_HIGH() boost.on()

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


#define READ_N2O_ARM_PIN() ((*n2o_arming_pin_port & n2o_arming_pin_mask) ? true : false)


#define VVT1_PIN_ON() VVT_1.on()
#define VVT1_PIN_OFF() VVT_1.off()
#define VVT2_PIN_ON() VVT_2.on()
#define VVT2_PIN_OFF() VVT_2.off()
#define VVT_TIME_DELAY_MULTIPLIER  50

#define WMI_TANK_IS_EMPTY() (configPage10.wmiEmptyEnabled ? (configPage10.wmiEmptyPolarity == 0) ^ digitalRead(pinWMIEmpty) : 1)

#if defined(PWM_FAN_AVAILABLE)//PWM fan not available on Arduino MEGA
extern uint16_t fan_pwm_max_count; //Used for variable PWM frequency
void fanInterrupt(void);
#endif

extern uint16_t vvt_pwm_max_count; //Used for variable PWM frequency
extern uint16_t boost_pwm_max_count; //Used for variable PWM frequency

void boostInterrupt(void);
void vvtInterrupt(void);

#endif