#ifndef INJECTOR_SCHEDULE_H__
#define INJECTOR_SCHEDULE_H__

#include "types.h"

void injectorControlMethodAssign(OUTPUT_CONTROL_TYPE control_method);

/* Must be called _after_ the control method has been assigned. */
void injector_pins_init(void);

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

