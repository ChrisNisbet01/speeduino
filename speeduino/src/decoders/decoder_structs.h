#pragma once

#include "globals.h"
#include <stdint.h>

#if defined(CORE_SAMD21)
typedef PinStatus interrupt_mode_t;
#else
typedef byte interrupt_mode_t;
#endif

typedef void (*trigger_setup_fn)(bool initialisationComplete);
typedef void (*trigger_handler_fn)(void);
typedef uint16_t (*trigger_get_rpm_fn)(void);
typedef int (*trigger_get_crank_angle_fn)(void);
typedef void (*trigger_set_end_teeth_fn)(void);

typedef struct decoder_handler_st
{
  trigger_setup_fn setup;
  trigger_handler_fn primaryToothHandler;
  trigger_handler_fn secondaryToothHandler;
  trigger_handler_fn tertiaryToothHandler;

  trigger_get_rpm_fn get_rpm;
  trigger_get_crank_angle_fn get_crank_angle;
  trigger_set_end_teeth_fn set_end_teeth;
} decoder_handler_st;

typedef struct decoder_context_st
{
  decoder_handler_st const * handler;

  trigger_handler_fn primaryToothHandler;
  trigger_handler_fn secondaryToothHandler;
  trigger_handler_fn tertiaryToothHandler;

  void attach_primary_interrupt(byte pin, trigger_handler_fn handler, interrupt_mode_t edge);
  void attach_secondary_interrupt(byte pin, trigger_handler_fn handler, interrupt_mode_t edge);
  void attach_tertiary_interrupt(byte pin, trigger_handler_fn handler, interrupt_mode_t edge);
} decoder_context_st;

