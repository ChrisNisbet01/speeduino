#include "pin.h"
#include "auxiliaries.h"

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

void IOPortMaskOutputPin::configure(byte pin, byte initial_state)
{
  m_port = portOutputRegister(digitalPinToPort(pin));
  m_mask = digitalPinToBitMask(pin);
  /* Set the pin before configuring as an output. */
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
    /* do_nothing */
  }
  pinMode(pin, OUTPUT);
  m_is_configured = true;
}

bool IOPortMaskOutputPin::is_configured(void)
{
  return m_port != nullptr && m_is_configured;
}


#if defined(CORE_TEENSY) || defined(CORE_STM32)
void IODigitalWriteOutputPin::on(void)
{
  digitalWrite(m_pin, HIGH);
}

void IODigitalWriteOutputPin::off(void)
{
  digitalWrite(m_pin, LOW);
}

void IODigitalWriteOutputPin::toggle(void)
{
  digitalToggle(m_pin);
}

void IODigitalWriteOutputPin::configure(byte pin, byte initial_state)
{
  m_pin = pin;
  /* Set the pin before configuring as an output. */
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
    /* do_nothing */
  }

  pinMode(pin, OUTPUT);
  m_is_configured = true;
}

bool IODigitalWriteOutputPin::is_configured(void)
{
  return m_is_configured;
}

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

void IOAtomicWriteOutputPin::configure(byte pin, byte initial_state)
{
  m_port = portOutputRegister(digitalPinToPort(pin));
  m_mask = digitalPinToBitMask(pin);
  /* Set the pin before configuring as an output. */
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
    /* do_nothing */
  }

  pinMode(pin, OUTPUT);
  m_is_configured = true;
}

bool IOAtomicWriteOutputPin::is_configured(void)
{
  return m_is_configured;
}
#endif

