#pragma once

#include "globals.h"

struct TPSDOT
{
public:

  int16_t update(byte newTPS, uint32_t timestamp_micros, byte minChange)
  {
    if (!m_initialised)
    {
      m_initialised = true;
      m_last_tps = newTPS;
      m_last_timestamp = timestamp_micros;
    }

    uint32_t const delta_t = timestamp_micros - m_last_timestamp;
    byte const delta_tps = newTPS - m_last_tps;

    if (delta_t == 0)
    {
      m_dot = 0;
      m_last_tps = newTPS;
    }
    else if (abs(delta_tps) < minChange)
    {
      m_dot = 0;

      /*
       * Don't update the TPS reading though, so slow but continuous changes
       * eventually result in minChange getting exceeded.
       */
      m_last_timestamp = timestamp_micros;
    }
    else
    {
      m_dot = (MICROS_PER_SEC / delta_t * delta_tps) / 2;

      m_last_tps = newTPS;
      m_last_timestamp = timestamp_micros;
    }

    return m_dot;
  }

  int16_t DOT(void)
  {
    return m_dot;
  }

  void reset(void)
  {
    m_last_tps = 0;
    m_last_timestamp = 0;
    m_dot = 0;
    m_initialised = false;
  }

private:

  byte m_last_tps = 0;
  uint32_t m_last_timestamp = 0;
  int16_t m_dot = 0;
  bool m_initialised = false;
};

extern struct TPSDOT tpsDOT;

