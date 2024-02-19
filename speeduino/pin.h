#pragma once

#include "globals.h"

#define INVALID_PIN_NUMBER 0
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

  void write(byte val);

  void configure(byte pin, byte initial_state = LOW, byte mode = OUTPUT);

  bool is_configured(void);

private:
  volatile PORT_TYPE * m_port = nullptr;
  PINMASK_TYPE m_mask = 0;
  bool m_is_configured = false;
};

class IOPortMaskInputPin
{
public:
  byte pin = INVALID_PIN_NUMBER;

  /*
   * Assumes that port and mask have been assigned before control methods are
   * called.
   */
  bool read(void);

  void configure(byte pin, byte mode = INPUT);

  bool is_configured(void);

  void setPin(byte pin);

private:
  volatile PORT_TYPE * m_port = nullptr;
  PINMASK_TYPE m_mask = 0;
  bool m_is_configured = false;
};

class IODigitalWriteOutputPin
{
public:
  byte pin = INVALID_PIN_NUMBER;

  void on(void);

  void off(void);

  void write(byte value);

  void toggle(void);

  void configure(byte pin, byte initial_state = LOW, byte mode = OUTPUT);

  bool is_configured(void);

  void setPin(byte pin);

private:
  bool m_is_configured = false;
};

class IODigitalReadInputPin
{
public:

  byte pin = INVALID_PIN_NUMBER;

  bool read(void);

  void configure(byte pin, byte mode = INPUT);

  bool is_configured(void);

  void setPin(byte pin);

private:
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

  void configure(byte pin, byte initial_state = LOW, byte mode = OUTPUT);

  bool is_configured(void);

private:

  volatile PORT_TYPE * m_port = nullptr;
  PINMASK_TYPE m_mask = 0;
  bool m_is_configured = false;
};
#endif

