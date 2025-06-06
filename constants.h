#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <math.h>
#define _USE_MATH_DEFINES
#ifndef M_PI
        #define M_PI 3.14159265358979323846
#endif

#define KP 0.005f//da trovare
#define KI 0.0005f//da trovare

#define PERIOD 20 //in millisecondi, ergo 50Hz

#define CLAMP_LIMIT 30 //limite superiore al numero di saturazioni, in caso fermo tutto e.g. mi sono schiantato

#define WHEEL_RAY_MM 33.f
#define TICKS_PER_REVOLUTION 16
#define TRANSMISSION_RATIO 120

#define BASELINE 120.0 // distanza tra le ruote in mm

const float mmsPerTick = (WHEEL_RAY_MM * 2 * M_PI) / (TICKS_PER_REVOLUTION * TRANSMISSION_RATIO);


#endif