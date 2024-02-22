#pragma once

//Handy bit setting macros
#define BIT(x) (1 << (x))
#define BIT_SET(a, b) ((a) |= BIT((b)))
#define BIT_CLEAR(a, b) ((a) &= ~BIT((b)))
#define BIT_CHECK(var, pos) (!!((var) & BIT((pos))))
#define BIT_TOGGLE(var, pos) ((var) ^= BIT((pos)))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))

