/*
These are some utility functions and variables used through the main code
*/
#pragma once
#include "globals.h"

#include <Arduino.h>

#define COMPARATOR_EQUAL 0
#define COMPARATOR_NOT_EQUAL 1
#define COMPARATOR_GREATER 2
#define COMPARATOR_GREATER_EQUAL 3
#define COMPARATOR_LESS 4
#define COMPARATOR_LESS_EQUAL 5
#define COMPARATOR_AND 6
#define COMPARATOR_XOR 7

#define BITWISE_DISABLED 0
#define BITWISE_AND 1
#define BITWISE_OR 2
#define BITWISE_XOR 3

#define REUSE_RULES 240

extern uint8_t ioOutDelay[sizeof(configPage13.outputPin)];
extern uint8_t ioDelay[sizeof(configPage13.outputPin)];
extern uint8_t pinIsValid;
extern uint8_t currentRuleStatus;

byte pinTranslate(byte rawPin);
byte pinTranslateAnalog(byte rawPin);
void initialiseProgrammableIO(void);
void checkProgrammableIO(void);
int16_t ProgrammableIOGetData(uint16_t index);

#if !defined(UNUSED)
#define UNUSED(x) (void)(x)
#endif

#define _countof(x) (sizeof((x)) / sizeof ((x)[0]))
#define _end_range_address(array) ((array) + _countof((array)))
#define _end_range_byte_address(array) (((byte*)(array)) + sizeof((array)))

// Pre-processor arithmetic increment (pulled from Boost.Preprocessor)
#define PP_INC(x) PP_INC_I(x)
#define PP_INC_I(x) PP_INC_ ## x

#if !defined(MIN)
#define MIN(a, b) \
    ({\
        const __typeof__(a) _a = (a); \
        const __typeof__(b) _b = (b); \
        (_a < _b) ? _a : _b; \
    })
#endif

#define TENTHS_TO_MS(v) ((v) * 100)

#define PP_INC_0 1
#define PP_INC_1 2
#define PP_INC_2 3
#define PP_INC_3 4
#define PP_INC_4 5
#define PP_INC_5 6
#define PP_INC_6 7
#define PP_INC_7 8
#define PP_INC_8 9
#define PP_INC_9 10
#define PP_INC_10 11
#define PP_INC_11 12
#define PP_INC_12 13

