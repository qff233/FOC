#ifndef __FOC_UTILS_H
#define __FOC_UTILS_H

#include <stdint.h>

// sign function
#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))
#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x)-0.5f))
#define _constrain(amt, low, high)                                             \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _sqrt(a) (_sqrtApprox(a))

#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f

// dq voltage structs
struct DQVoltage {
    float d = 0.0f;
    float q = 0.0f;
};

// dq current structure
struct DQCurrent {
    float d = 0.0f;
    float q = 0.0f;
};

struct PhaseCurrent {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
};

struct PhaseVoltage {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
};

/**
 *  Function approximating the sine calculation by using fixed size array
 * - execution time ~40us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _sin(float a);
/**
 * Function approximating cosine calculation by using fixed size array
 * - execution time ~50us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _cos(float a);

/**
 * normalizing radian angle to [0,2PI]
 * @param angle - angle to be normalized
 */
float _normalizeAngle(float angle);

/**
 * Electrical angle calculation
 *
 * @param shaft_angle - shaft angle of the motor
 * @param pole_pairs - number of pole pairs
 */
float _electricalAngle(float shaft_angle, int pole_pairs);

/**
 * Function approximating square root function
 *  - using fast inverse square root
 *
 * @param value - number
 */
float _sqrtApprox(float value);

uint64_t get_microsecond();

#endif //__FOC_UTILS_H
