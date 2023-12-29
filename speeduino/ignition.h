#ifndef IGNITION_H__
#define IGNITION_H__

#include "ignition_id.h"

typedef void (*init_ignition_fn)(void);
typedef void (*ignition_begin_charge_id_fn)(ignition_id_t coil);
typedef void (*ignition_end_charge_id_fn)(ignition_id_t coil);
typedef void (*ignition_toggle_id_fn)(ignition_id_t coil);

typedef struct ignition_st
{
  init_ignition_fn init;
  ignition_begin_charge_id_fn begin_charge;
  ignition_end_charge_id_fn end_charge;
  ignition_toggle_id_fn toggle;
} ignition_st;

#endif /* IGNITION_H__ */

