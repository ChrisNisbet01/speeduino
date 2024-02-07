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
    m_is_primed = false;
    m_priming_time = start_time;
  }

  void complete(void)
  {
    m_is_primed = true;
  }

  bool isCompleted(void)
  {
    return m_is_primed;
  }

  bool durationIsCompleted(byte const current_time, byte const duration)
  {
    return current_time - m_priming_time >= duration;
  }

private:
  bool m_is_primed = false;
  ///< The time (in seconds, based on @ref statuses.secl) that the fuel pump started priming
  byte m_priming_time = 0;
};

extern FuelPump fuelPump;

extern FuelPriming fuelPriming;
