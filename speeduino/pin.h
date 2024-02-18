#pragma once

#include "globals.h"

#define INVALID_PIN_NUMBER 0xff
/*
 * Probably don't need all these classes - simply wanted to make them equivalent
 * to what's already there.
 * I suspect that digitalRead/digitalWrite can probably be used, even for atomic operations.
 */
class IOPortMaskOutputPin
{
public:
  /*
   * Assumes that port and mask have been assigned before control methods are
   * called.
   */
  void on(void);

  void off(void);

  void toggle(void);

  void configure(byte pin, byte initial_state = LOW);

  bool is_configured(void);

private:

  bool m_inverted;
  byte m_pin = INVALID_PIN_NUMBER;
  volatile PORT_TYPE * m_port = nullptr;
  PINMASK_TYPE m_mask = 0;
  bool m_is_configured = false;
};


class IODigitalWriteOutputPin
{
public:
  void on(void);

  void off(void);

  void write(byte value);

#if defined(CORE_TEENSY) || defined(CORE_STM32)
  void toggle(void);
#endif

  void configure(byte pin, byte initial_state = LOW);

  bool is_configured(void);

private:

  bool m_inverted;
  byte m_pin = INVALID_PIN_NUMBER;
  bool m_is_configured = false;
};

#if defined(CORE_TEENSY) || defined(CORE_STM32)

#else

class IOAtomicWriteOutputPin
{
public:
  /*
   * Assumes that port and mask have been assigned before control methods are
   * called.
   */
  void on(void);

  void off(void);

  void toggle(void);

  void configure(byte pin, byte initial_state = LOW);

  bool is_configured(void);

private:

  byte m_pin = INVALID_PIN_NUMBER;
  volatile PORT_TYPE * m_port = nullptr;
  PINMASK_TYPE m_mask = 0;
  bool m_is_configured = false;
};
#endif

