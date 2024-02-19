#include "pin.h"
#include "auxiliaries.h"

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

void IOPortMaskOutputPin::on(void)
{
  *m_port |= m_mask;
}

void IOPortMaskOutputPin::off(void)
{
  *m_port &= ~m_mask;
}

void IOPortMaskOutputPin::toggle(void)
{
  *m_port ^= m_mask;
}

void IOPortMaskOutputPin:: write(byte val)
{
  val ? on() : off();
}

void IOPortMaskOutputPin::configure(byte pin, byte initial_state, byte mode)
{
  m_port = portOutputRegister(digitalPinToPort(pin));
  m_mask = digitalPinToBitMask(pin);
  /* Set the pin before configuring as an output. */
  if (mode == OUTPUT)
  {
    if (initial_state == LOW)
    {
      off();
    }
    else if (initial_state == HIGH)
    {
      on();
    }
    else
    {
      /* Do_nothing. */
    }
  }
  pinMode(pin, mode);
  m_is_configured = true;
}

bool IOPortMaskOutputPin::is_configured(void)
{
  return m_port != nullptr && m_is_configured;
}

bool IOPortMaskInputPin::read(void)
{
  return (*m_port & m_mask) != 0;
}

void IOPortMaskInputPin::configure(byte pin, byte mode)
{
  m_pin = pin;
  m_port = portOutputRegister(digitalPinToPort(pin));
  m_mask = digitalPinToBitMask(pin);
  pinMode(pin, mode);
  m_is_configured = true;
}

bool IOPortMaskInputPin::is_configured(void)
{
  return m_port != nullptr && m_is_configured;
}

byte IOPortMaskInputPin::getPin(void)
{
  return m_pin;
}

void IOPortMaskInputPin::setPin(byte pin)
{
  m_pin = pin;
}

void IODigitalWriteOutputPin::on(void)
{
  digitalWrite(m_pin, HIGH);
}

void IODigitalWriteOutputPin::off(void)
{
  digitalWrite(m_pin, LOW);
}

void IODigitalWriteOutputPin::write(byte value)
{
  digitalWrite(m_pin, value);
}

void IODigitalWriteOutputPin::toggle(void)
{
#if defined(CORE_TEENSY) || defined(CORE_STM32)
  digitalToggle(m_pin);
#else
  write(!digitalRead(m_pin));
#endif
}

void IODigitalWriteOutputPin::configure(byte pin, byte initial_state, byte mode)
{
  m_pin = pin;
  /* Set the pin before configuring as an output. */
  if (mode == OUTPUT)
  {
    if (initial_state == LOW)
    {
      off();
    }
    else if (initial_state == HIGH)
    {
      on();
    }
    else
    {
      /* Do_nothing. */
    }
  }

  pinMode(m_pin, mode);
  m_is_configured = true;
}

bool IODigitalWriteOutputPin::is_configured(void)
{
  return m_is_configured;
}

byte IODigitalWriteOutputPin::getPin(void)
{
  return m_pin;
}

void IODigitalWriteOutputPin::setPin(byte pin)
{
  m_pin = pin;
}

bool IODigitalReadInputPin::read(void)
{
  return digitalRead(m_pin);
}

void IODigitalReadInputPin::configure(byte pin, byte mode)
{
  m_pin = pin;

  pinMode(m_pin, mode);
  m_is_configured = true;
}

bool IODigitalReadInputPin::is_configured(void)
{
  return m_is_configured;
}

byte IODigitalReadInputPin::getPin(void)
{
  return m_pin;
}

void IODigitalReadInputPin::setPin(byte pin)
{
  m_pin = pin;
}

#if defined(CORE_TEENSY) || defined(CORE_STM32)

#else

void IOAtomicWriteOutputPin::on(void)
{
  pin_set(*m_port, m_mask);
}

void IOAtomicWriteOutputPin::off(void)
{
  pin_clear(*m_port, m_mask);
}

void IOAtomicWriteOutputPin::toggle(void)
{
  pin_toggle(*m_port, m_mask);
}

void IOAtomicWriteOutputPin::configure(byte pin, byte initial_state, byte mode)
{
  m_port = portOutputRegister(digitalPinToPort(pin));
  m_mask = digitalPinToBitMask(pin);

  /* Set the pin before configuring as an output. */
  if (mode == OUTPUT)
  {
    if (initial_state == LOW)
    {
      off();
    }
    else if (initial_state == HIGH)
    {
      on();
    }
    else
    {
      /* Do_nothing. */
    }
  }

  pinMode(pin, mode);
  m_is_configured = true;
}

bool IOAtomicWriteOutputPin::is_configured(void)
{
  return m_is_configured;
}
#endif

