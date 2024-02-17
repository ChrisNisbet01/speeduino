#pragma once

#include <globals.h>

struct MAPDOT
{
public:

  int16_t update(byte newMAP, uint32_t timestamp_micros, byte minChange)
  {
    if (!m_initialised)
    {
      m_initialised = true;
      m_last_map = newMAP;
      m_last_timestamp = timestamp_micros;
    }

    uint32_t const delta_t = timestamp_micros - m_last_timestamp;
    byte const delta_map = newMAP - m_last_map;

    if (delta_t == 0)
    {
      m_dot = 0;
      m_last_map = newMAP;
    }
    else if (abs(delta_map) < minChange)
    {
      m_dot = 0;

      /*
       * Don't update the MAP reading though, so slow but continuous changes
       * eventually result in minChange getting exceeded.
       */
      m_last_timestamp = timestamp_micros;
    }
    else
    {
      m_dot = (MICROS_PER_SEC / delta_t * delta_map);

      m_last_map = newMAP;
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
    m_last_map = 0;
    m_last_timestamp = 0;
    m_dot = 0;
    m_initialised = false;
  }

private:

  byte m_last_map = 0;
  uint32_t m_last_timestamp = 0;
  int16_t m_dot = 0;
  bool m_initialised = false;
};

extern struct MAPDOT mapDOT;

