#ifndef CONTROLLO_H
#define CONTROLLO_H

#include <pthread.h>
#include "libcoderbot/include/motor.h"
#include "libcoderbot/include/encoder.h"
#include "libcoderbot/include/cbdef.h"
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <math.h>
#include "include/concurrent.h"
#include "include/constants.h"

// Struct for passing arguments to the control thread
typedef struct {
    pthread_mutex_t* clamp_mutex;  // Mutex for clamp counter
    float* targetSpeed_mm_s;
    float dutyCycle;
    cbMotor_t* motor;
    cbEncoder_t* encoder;
    int* clamp_counter;
} controllo_args_t;

// Clamp function: clamps the duty cycle and increments the clamp counter if out of bounds
float clamp(float dutyCycle, int* clamp_counter);

// Control thread function
void* controllo(void* args);

#endif // CONTROLLO_H