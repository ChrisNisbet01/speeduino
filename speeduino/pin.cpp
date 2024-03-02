#include "pin.h"
#include "auxiliaries.h"
#include "bit_macros.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)
#define HAVE_ATOMIC_BLOCK 0
#else
#define HAVE_ATOMIC_BLOCK 1
#endif

#if 0
/*
  Need something like this to support ATOMIC_BLOCK on those platforms that don't
  already support it.
 */
static uint32_t interrupts_disable_return_previous_state(void)
{
  uint32_t const old_primask = __get_PRIMASK();

  __disable_irq();

  return old_primask;
}

static void interrupts_state_restore(uint32_t previous_state)
{
  __set_PRIMASK(previous_state);
}
#endif

#if HAVE_ATOMIC_BLOCK
static inline void
atomic_pin_set(volatile PORT_TYPE &port, PINMASK_TYPE const mask)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    port |= mask;
  }
}

static inline void
atomic_pin_clear(volatile PORT_TYPE &port, PINMASK_TYPE const mask)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    port &= ~mask;
  }
}

static inline bool
atomic_pin_read(volatile PORT_TYPE &port, PINMASK_TYPE const mask)
{
  bool state;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    state = (port & mask) != 0;
  }

  return state;
}

static inline void
atomic_pin_toggle(volatile PORT_TYPE &port, PINMASK_TYPE const mask)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    port &= ~mask;
  }
}
#endif

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

void IOPortMaskOutputPin::configure(byte initial_state)
{
  if (pin == INVALID_PIN_NUMBER)
  {
    return;
  }
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
    /* Do_nothing. */
  }
  pinMode(pin, OUTPUT);
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

bool IODigitalWriteOutputPin::read(void)
{
  return digitalRead(pin);
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

void IODigitalWriteOutputPin::configure(byte initial_state)
{
  if (pin == INVALID_PIN_NUMBER)
  {
    return;
  }
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
    /* Do_nothing. */
  }

  pinMode(pin, OUTPUT);
  m_is_configured = true;
}

bool IODigitalWriteOutputPin::is_configured(void)
{
  return m_is_configured;
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

#if defined(CORE_TEENSY) || defined(CORE_STM32)

#else

bool IOAtomicWriteOutputPin::read(void)
{
  return atomic_pin_read(*m_port, m_mask);
}

void IOAtomicWriteOutputPin::on(void)
{
  atomic_pin_set(*m_port, m_mask);
}

void IOAtomicWriteOutputPin::off(void)
{
  atomic_pin_clear(*m_port, m_mask);
}

void IOAtomicWriteOutputPin:: write(byte val)
{
  val ? on() : off();
}

void IOAtomicWriteOutputPin::toggle(void)
{
  atomic_pin_toggle(*m_port, m_mask);
}

void IOAtomicWriteOutputPin::configure(byte initial_state)
{
  if (pin == INVALID_PIN_NUMBER)
  {
    return;
  }
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
    /* Do_nothing. */
  }

  pinMode(pin, OUTPUT);
  m_is_configured = true;
}

bool IOAtomicWriteOutputPin::is_configured(void)
{
  return m_is_configured;
}
#endif

