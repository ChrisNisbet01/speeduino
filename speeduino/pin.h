#pragma once

#include "globals.h"

class IOPin
{
public:
  /*
   * Assumes that port and mask have been assigned before control methods are
   * called.
   */
  void on(void)
  {
    *m_port |= m_mask;
  }

  void off(void)
  {
    *m_port &= ~m_mask;
  }

  void toggle(void)
  {
    *m_port ^= m_mask;
  }

  void configure(byte pin)
  {
    pinMode(pin, OUTPUT);
    m_port = portOutputRegister(digitalPinToPort(pin));
    m_mask = digitalPinToBitMask(pin);
    m_is_configured = true;
  }

  bool is_configured(void)
  {
    return m_port != nullptr && m_is_configured;
  }

private:
  bool m_is_configured = false;
  volatile PORT_TYPE * m_port = nullptr;
  PINMASK_TYPE m_mask = 0;
};

