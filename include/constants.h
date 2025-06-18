#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <math.h>
#define _USE_MATH_DEFINES
#ifndef M_PI
        #define M_PI 3.14159265358979323846
#endif

#define KP_L 0.004f//da trovare
#define KI_L 0.005f//da trovare
#define KP_R 0.004f//da trovare
#define KI_R 0.007f//da trovare

#define PERIOD 20 //in millisecondi, ergo 50Hz

#define CLAMP_LIMIT 30 //limite superiore al numero di saturazioni, in caso fermo tutto e.g. mi sono schiantato

#define WHEEL_RAY_MM 33.f
#define TICKS_PER_REVOLUTION 16
#define TRANSMISSION_RATIO 120

static const float mmsPerTick = (WHEEL_RAY_MM * 2 * M_PI) / (TICKS_PER_REVOLUTION * TRANSMISSION_RATIO);


#define STEP_LEFT 0.1263372089                              // DISTANZA (mm) per TICK (sinistra)
#define STEP_RIGHT 0.1288004121                             // DISTANZA (mm) per TICK (destra)

#define MAX_PWM 0.6                                         // VALORE MASSIMO (trovato sperimentalmente)
#define MIN_PWM 0.2                                         // VALORE MINIMO (trovato sperimentalmente)

#define LOOP_DURATION_SEC 5                                 // DURATA del MOVIMENTO (s)
#define LOOP_DURATION_NSEC (LOOP_DURATION_SEC * 1e9)        // DURATA del MOVIMENTO (ns)
#define LOOP_PERIOD_NSEC 2e7                                // INTERVALLO di CAMPIONAMENTO (ns)
#define NUM_CYCLES (LOOP_DURATION_NSEC / LOOP_PERIOD_NSEC)  // NUMERO DI CICLI

#define SPEED 50                                            // VELOCITA' (mm/s)

#define B 120                                               // DISTANZA tra le RUOTE

#define THRESHOLD 0.005                                     // SOGLIA per DISTINGUERE tra DRITTO o CURVANDO
                                                            // (ticks sinistra e destra mai esattamente uguali, anche se dritto)

#endif
