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

void IOPortMaskOutputPin::configure(byte initial_state, byte mode)
{
  if (pin == INVALID_PIN_NUMBER)
  {
    return;
  }
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

void IOPortMaskInputPin::configure(byte mode)
{
  if (pin == INVALID_PIN_NUMBER)
  {
    return;
  }
  m_port = portOutputRegister(digitalPinToPort(pin));
  m_mask = digitalPinToBitMask(pin);
  pinMode(pin, mode);
  m_is_configured = true;
}

bool IOPortMaskInputPin::is_configured(void)
{
  return m_port != nullptr && m_is_configured;
}

void IOPortMaskInputPin::setPin(byte pin_)
{
  pin = pin_;
}

void IODigitalWriteOutputPin::on(void)
{
  digitalWrite(pin, HIGH);
}

void IODigitalWriteOutputPin::off(void)
{
  digitalWrite(pin, LOW);
}

void IODigitalWriteOutputPin::write(byte value)
{
  digitalWrite(pin, value);
}

void IODigitalWriteOutputPin::toggle(void)
{
#if defined(CORE_TEENSY) || defined(CORE_STM32)
  digitalToggle(pin);
#else
  write(!digitalRead(pin));
#endif
}

void IODigitalWriteOutputPin::configure(byte initial_state, byte mode)
{
  if (pin == INVALID_PIN_NUMBER)
  {
    return;
  }
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

bool IODigitalWriteOutputPin::is_configured(void)
{
  return m_is_configured;
}

void IODigitalWriteOutputPin::setPin(byte pin_)
{
  pin = pin_;
}

bool IODigitalReadInputPin::read(void)
{
  return digitalRead(pin);
}

void IODigitalReadInputPin::configure(byte mode)
{
  if (pin == INVALID_PIN_NUMBER)
  {
    return;
  }
  pinMode(pin, mode);
  m_is_configured = true;
}

bool IODigitalReadInputPin::is_configured(void)
{
  return m_is_configured;
}

void IODigitalReadInputPin::setPin(byte pin_)
{
  pin = pin_;
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

void IOAtomicWriteOutputPin::configure(byte initial_state, byte mode)
{
  if (pin == INVALID_PIN_NUMBER)
  {
    return;
  }
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

