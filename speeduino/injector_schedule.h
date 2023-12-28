#ifndef INJECTOR_SCHEDULE_H__
#define INJECTOR_SCHEDULE_H__

#include "types.h"

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

void injectorControlMethodAssign(OUTPUT_CONTROL_TYPE control_method);

void openInjector1(void);
void closeInjector1(void);
void injector1Toggle(void);

void openInjector2(void);
void closeInjector2(void);
void injector2Toggle(void);

void openInjector3(void);
void closeInjector3(void);
void injector3Toggle(void);

void openInjector4(void);
void closeInjector4(void);
void injector4Toggle(void);

void openInjector5(void);
void closeInjector5(void);
void injector5Toggle(void);

void openInjector6(void);
void closeInjector6(void);
void injector6Toggle(void);

void openInjector7(void);
void closeInjector7(void);
void injector7Toggle(void);

void openInjector8(void);
void closeInjector8(void);
void injector8Toggle(void);

// These are for Semi-Sequential and 5 Cylinder injection
void openInjector1and3(void);
void closeInjector1and3(void);
void openInjector2and4(void);
void closeInjector2and4(void);
void openInjector1and4(void);
void closeInjector1and4(void);
void openInjector2and3(void);
void closeInjector2and3(void);

void openInjector3and5(void);
void closeInjector3and5(void);

void openInjector2and5(void);
void closeInjector2and5(void);
void openInjector3and6(void);
void closeInjector3and6(void);

void openInjector1and5(void);
void closeInjector1and5(void);
void openInjector2and6(void);
void closeInjector2and6(void);
void openInjector3and7(void);
void closeInjector3and7(void);
void openInjector4and8(void);
void closeInjector4and8(void);

#endif /* INJECTOR_SCHEDULE_H__ */

