#ifndef INJECTORS_H__
#define INJECTORS_H__

#include "injector_id.h"

typedef void (*init_injectors_fn)(void);
typedef void (*open_injector_id_fn)(injector_id_t injector);
typedef void (*close_injector_id_fn)(injector_id_t injector);
typedef void (*toggle_injector_id_fn)(injector_id_t injector);

typedef struct injectors_st
{
  init_injectors_fn init;
  open_injector_id_fn open;
  close_injector_id_fn close;
  toggle_injector_id_fn toggle;
} injectors_st;

#endif /* INJECTORS_H__ */

