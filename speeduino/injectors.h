#ifndef INJECTORS_H__
#define INJECTORS_H__

typedef void (*init_injectors_fn)(void);
typedef void (*open_injector_fn)(void);
typedef void (*close_injector_fn)(void);
typedef void (*toggle_injector_fn)(void);

typedef enum injector_id_t
{
  injector_id_1 = 0,
  injector_id_2,
  injector_id_3,
  injector_id_4,
  injector_id_5,
  injector_id_6,
  injector_id_7,
  injector_id_8,
  injector_id_COUNT,
} injector_id_t;

typedef struct injector_control_st
{
  open_injector_fn open;
  close_injector_fn close;
  toggle_injector_fn toggle;
} injector_control_st;

typedef struct injectors_st
{
  init_injectors_fn init;
  injector_control_st const * const control;
} injector_st;

#endif /* INJECTORS_H__ */

