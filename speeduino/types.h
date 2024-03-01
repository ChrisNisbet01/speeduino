#pragma once

#include <stdint.h>

typedef enum {
    OUTPUT_CONTROL_DIRECT = 0,
    OUTPUT_CONTROL_MC33810 = 10,
} OUTPUT_CONTROL_TYPE;

/** @brief Byte type. This is not defined in any C or C++ standard header. */
typedef uint8_t byte;

