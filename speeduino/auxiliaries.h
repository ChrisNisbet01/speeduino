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

#define BOOST_PIN_LOW() boost.off()
#define BOOST_PIN_HIGH() boost.on()

#define VVT1_PIN_LOW() digitalWrite(pinVVT_1, LOW)
#define VVT1_PIN_HIGH() digitalWrite(pinVVT_1, HIGH)
#define VVT2_PIN_LOW() digitalWrite(pinVVT_2, LOW)
#define VVT2_PIN_HIGH() digitalWrite(pinVVT_2, HIGH)
#define FAN_PIN_LOW() digitalWrite(pinFan, LOW)
#define FAN_PIN_HIGH() digitalWrite(pinFan, HIGH)
#define N2O_STAGE1_PIN_LOW() digitalWrite(configPage10.n2o_stage1_pin, LOW)
#define N2O_STAGE1_PIN_HIGH() digitalWrite(configPage10.n2o_stage1_pin, HIGH)
#define N2O_STAGE2_PIN_LOW() digitalWrite(configPage10.n2o_stage2_pin, LOW)
#define N2O_STAGE2_PIN_HIGH() digitalWrite(configPage10.n2o_stage2_pin, HIGH)
#define AIRCON_PIN_LOW() digitalWrite(pinAirConComp, LOW)
#define AIRCON_PIN_HIGH() digitalWrite(pinAirConComp, HIGH)
#define AIRCON_FAN_PIN_LOW() digitalWrite(pinAirConFan, LOW)
#define AIRCON_FAN_PIN_HIGH() digitalWrite(pinAirConFan, HIGH)

#define AIRCON_ON() { configPage15.airConCompPol==1 ? AIRCON_PIN_LOW() : AIRCON_PIN_HIGH(); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_OFF() { configPage15.airConCompPol==1 ? AIRCON_PIN_HIGH() : AIRCON_PIN_LOW(); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_FAN_ON() { configPage15.airConFanPol==1 ? AIRCON_FAN_PIN_LOW() : AIRCON_FAN_PIN_HIGH(); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_FAN); }
#define AIRCON_FAN_OFF() { configPage15.airConFanPol==1 ? AIRCON_FAN_PIN_HIGH() : AIRCON_FAN_PIN_LOW(); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_FAN); }

#define FAN_ON() { configPage6.fanInv ? FAN_PIN_LOW() : FAN_PIN_HIGH(); }
#define FAN_OFF() { configPage6.fanInv ? FAN_PIN_HIGH() : FAN_PIN_LOW(); }
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

#define BOOST_PIN_LOW() boost.off()
#define BOOST_PIN_HIGH() boost.on()

#define VVT1_PIN_LOW() pin_clear(*vvt1_pin_port, vvt1_pin_mask)
#define VVT1_PIN_HIGH() pin_set(*vvt1_pin_port, vvt1_pin_mask)

#define VVT2_PIN_LOW() pin_clear(*vvt2_pin_port, vvt2_pin_mask)
#define VVT2_PIN_HIGH() pin_set(*vvt2_pin_port, vvt2_pin_mask)

#define N2O_STAGE1_PIN_LOW() pin_clear(*n2o_stage1_pin_port, n2o_stage1_pin_mask)
#define N2O_STAGE1_PIN_HIGH() pin_set(*n2o_stage1_pin_port, n2o_stage1_pin_mask)

#define N2O_STAGE2_PIN_LOW() pin_clear(*n2o_stage2_pin_port, n2o_stage2_pin_mask)
#define N2O_STAGE2_PIN_HIGH() pin_set(*n2o_stage2_pin_port, n2o_stage2_pin_mask)

#define FAN_PIN_LOW() pin_clear(*fan_pin_port, fan_pin_mask)
#define FAN_PIN_HIGH() pin_set(*fan_pin_port, fan_pin_mask)

#define AIRCON_PIN_LOW()  pin_clear(*aircon_comp_pin_port,aircon_comp_pin_mask)
#define AIRCON_PIN_HIGH() pin_set(*aircon_comp_pin_port, aircon_comp_pin_mask)

#define AIRCON_FAN_PIN_LOW() pin_clear(*aircon_fan_pin_port, aircon_fan_pin_mask)
#define AIRCON_FAN_PIN_HIGH() pin_set(*aircon_fan_pin_port, aircon_fan_pin_mask)

#define AIRCON_ON() do { \
  configPage15.airConCompPol == 1 ? AIRCON_PIN_LOW() : AIRCON_PIN_HIGH(); \
  bit_set_atomic(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); \
  } while(0)
#define AIRCON_OFF() do { \
  configPage15.airConCompPol == 1 ? AIRCON_PIN_HIGH() : AIRCON_PIN_LOW(); \
  bit_clear_atomic(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); \
  } while(0)

#define AIRCON_FAN_ON() do { \
  configPage15.airConFanPol == 1 ? AIRCON_FAN_PIN_LOW() : AIRCON_FAN_PIN_HIGH(); \
  bit_set_atomic(currentStatus.airConStatus, BIT_AIRCON_FAN); \
  } while(0)
#define AIRCON_FAN_OFF() do { \
  configPage15.airConFanPol == 1 ? AIRCON_FAN_PIN_HIGH() : AIRCON_FAN_PIN_LOW(); \
  bit_clear_atomic(currentStatus.airConStatus, BIT_AIRCON_FAN); \
  } while(0)

#define FAN_ON() do { \
  configPage6.fanInv ? FAN_PIN_LOW() : FAN_PIN_HIGH(); \
  } while (0)
#define FAN_OFF() do { \
  configPage6.fanInv ? FAN_PIN_HIGH() : FAN_PIN_LOW(); \
  } while (0)


#endif

#define FUEL_PUMP_ON() FuelPump.on()
#define FUEL_PUMP_OFF() FuelPump.off()

#define READ_N2O_ARM_PIN() ((*n2o_arming_pin_port & n2o_arming_pin_mask) ? true : false)


#define VVT1_PIN_ON() VVT1_PIN_HIGH();
#define VVT1_PIN_OFF() VVT1_PIN_LOW();
#define VVT2_PIN_ON() VVT2_PIN_HIGH();
#define VVT2_PIN_OFF() VVT2_PIN_LOW();
#define VVT_TIME_DELAY_MULTIPLIER  50

#define WMI_TANK_IS_EMPTY() (configPage10.wmiEmptyEnabled ? (configPage10.wmiEmptyPolarity == 0) ^ digitalRead(pinWMIEmpty) : 1)

extern volatile PORT_TYPE *vvt1_pin_port;
extern volatile PINMASK_TYPE vvt1_pin_mask;
extern volatile PORT_TYPE *vvt2_pin_port;
extern volatile PINMASK_TYPE vvt2_pin_mask;
extern volatile PORT_TYPE *fan_pin_port;
extern volatile PINMASK_TYPE fan_pin_mask;

#if defined(PWM_FAN_AVAILABLE)//PWM fan not available on Arduino MEGA
extern uint16_t fan_pwm_max_count; //Used for variable PWM frequency
void fanInterrupt(void);
#endif

extern uint16_t vvt_pwm_max_count; //Used for variable PWM frequency
extern uint16_t boost_pwm_max_count; //Used for variable PWM frequency

void boostInterrupt(void);
void vvtInterrupt(void);

#endif