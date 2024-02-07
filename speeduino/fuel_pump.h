#pragma once

#include "auxiliaries.h"

class FuelPump
{
public:
  void turnOn(void)
  {
    m_state = true;
    FUEL_PUMP_ON();
  }

  void turnOff(void)
  {
    m_state = false;
    FUEL_PUMP_OFF();
  }

  bool isOn(void)
  {
    return m_state;
  }

  void startPriming(void)
  {
    m_is_primed = false;
  }

  void stopPriming(void)
  {
    m_is_primed = true;
  }

  bool isPrimed(void)
  {
    return m_is_primed;
  }

  void resetPriming(void)
  {
    m_is_primed = false;
  }

private:
  bool m_state = false;
  bool m_is_primed = false;
};

class FuelPriming
{
public:

  void start(byte const start_time)
  {
    m_are_priming = true;
    m_priming_time = start_time;
  }

  void complete(void)
  {
    m_are_priming = false;
  }

  bool arePriming(void)
  {
    return m_are_priming;
  }

  /*
   * Update the priming state.
   * current_time: The current timestamp.
   * duration: The amount of time to prime the pump for.
   */
  void update(byte const current_time, byte const duration)
  {
    if (m_are_priming)
    {
      bool const priming_is_completed = current_time - m_priming_time >= duration;

      if (priming_is_completed)
      {
        m_are_priming = false;
      }
    }
  }

private:
  bool m_are_priming = false;
  ///< The time (in seconds, based on @ref statuses.secl) that the fuel pump started priming
  byte m_priming_time = 0;
};

extern FuelPump fuelPump;

extern FuelPriming fuelPriming;
