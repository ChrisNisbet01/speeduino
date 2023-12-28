#ifndef IGNITION_H__
#define IGNITION_H__

typedef void (*init_ignition_fn)(void);
typedef void (*ignition_begin_charge_fn)(void);
typedef void (*ignition_end_charge_fn)(void);
typedef void (*ignition_toggle_fn)(void);

typedef enum ignition_id_t
{
  ignition_id_1 = 0,
  ignition_id_2,
  ignition_id_3,
  ignition_id_4,
  ignition_id_5,
  ignition_id_6,
  ignition_id_7,
  ignition_id_8,
  ignition_id_COUNT,
} ignition_id_t;

typedef struct ignition_control_st
{
  ignition_begin_charge_fn begin_charge;
  ignition_end_charge_fn end_charge;
  ignition_toggle_fn toggle;
} ignition_control_st;

typedef struct ignition_st
{
  init_ignition_fn init;
  ignition_control_st const * const control;
} ignition_st;

#endif /* IGNITION_H__ */

