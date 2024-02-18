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

bool IOPortMaskOutputPin::read(void)
{
  return (*m_port & m_mask) != 0;
}

void IOPortMaskOutputPin::configure(byte pin, byte initial_state, byte mode)
{
  m_pin = pin;
  if (m_pin == INVALID_PIN_NUMBER)
  {
    return;
  }

  m_port = portOutputRegister(digitalPinToPort(m_pin));
  m_mask = digitalPinToBitMask(m_pin);
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

bool IOPortMaskOutputPin::is_configured(void)
{
  return m_port != nullptr && m_is_configured;
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

bool IODigitalWriteOutputPin::read(void)
{
  return digitalRead(m_pin);
}

#if defined(CORE_TEENSY) || defined(CORE_STM32)
void IODigitalWriteOutputPin::toggle(void)
{
  digitalToggle(m_pin);
}
#endif

void IODigitalWriteOutputPin::configure(byte pin, byte initial_state, byte mode)
{
  m_pin = pin;
  if (m_pin == INVALID_PIN_NUMBER)
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

  pinMode(m_pin, mode);
  m_is_configured = true;
}

bool IODigitalWriteOutputPin::is_configured(void)
{
  return m_is_configured;
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
  m_pin = pin;
  if (m_pin == INVALID_PIN_NUMBER)
  {
    return;
  }
  m_port = portOutputRegister(digitalPinToPort(m_pin));
  m_mask = digitalPinToBitMask(m_pin);

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

bool IOAtomicWriteOutputPin::is_configured(void)
{
  return m_is_configured;
}
#endif

