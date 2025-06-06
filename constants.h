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

const float mmsPerTick = (WHEEL_RAY_MM * 2 * M_PI) / (TICKS_PER_REVOLUTION * TRANSMISSION_RATIO);


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
coderbot@emb-cb03:~/besoLesly/progetto $ cat controllo.h
#ifndef CONTROLLO_H
#define CONTROLLO_H
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include "libcoderbot/include/motor.h"
#include "libcoderbot/include/cbdef.h"
#include "libcoderbot/include/encoder.h"
#include "concurrent.h"
#include "constants.h"
#include <math.h>

extern pthread_mutex_t clamp_mutex;     //mutex di clamp counter

float clamp(float dutyCycle,int* clamp_counter){
        mutex_lock(&clamp_mutex);

        if (*clamp_counter>=CLAMP_LIMIT){
                mutex_unlock(&clamp_mutex);
                exit(EXIT_FAILURE);
        }
        if (dutyCycle > 1.0f) {
                *(clamp_counter)++;
                mutex_unlock(&clamp_mutex);
                return 1.0f;
        }
        else if (dutyCycle < 0.0f){
                *(clamp_counter)++;
                mutex_unlock(&clamp_mutex);
                return 0.1f;
        }

        mutex_unlock(&clamp_mutex);
        return dutyCycle;
}

typedef struct {
        float targetSpeed_mm_s;
        float dutyCycle;
        cbMotor_t* motor;
        cbEncoder_t* encoder;
        int* clamp_counter;
} controllo_args_t;

void* controllo(void *args){
        controllo_args_t *controlArgs = (controllo_args_t *)args;
        cbMotor_t* motor = (controlArgs->motor);
        float dutyCycle = controlArgs->dutyCycle;
        float targetSpeed_mm_s = controlArgs->targetSpeed_mm_s;
        cbEncoder_t* encoder = (controlArgs->encoder);
        // da stabilire i valori
        int* clamp_counter = controlArgs->clamp_counter;

        set_sched_deadline(15*1000*1000,19*1000*1000,20*1000*1000);

        int tick = 0;
        int prevTick = 0;
        float travelledDistance = 0.0f;
        float speed = 0.0f;
        float error = 0.0f;
        float newDutyCycle = 0.0f;
        float erroreAccumulato = 0.0f;

        cbDir_t direction=forward;

        cbMotorMove(motor, direction, dutyCycle);

        while(1){

                tick = encoder->ticks;
                travelledDistance = (tick - prevTick)*mmsPerTick;

                prevTick = tick;

                speed = (travelledDistance / PERIOD) * 1000; // il periodo è in millisecondi, per avere la velocità in mm/s mi riporto ai secondi

                error = targetSpeed_mm_s - speed;
                erroreAccumulato += error; //sorrenti dice che nell'integrale va considerato l'errore corrente, non so quanto mi convince

                if(encoder->pin_a==PIN_ENCODER_LEFT_A){
                        newDutyCycle = KP_L*error + KI_L*erroreAccumulato;

                }else if(encoder->pin_a==PIN_ENCODER_RIGHT_A){
                        //sono nel destro
                        newDutyCycle = KP_R*error + KI_R*erroreAccumulato;
                }
                newDutyCycle = fabs(newDutyCycle);
                //controllo saturazione
                newDutyCycle = clamp(newDutyCycle,clamp_counter);
                // se errore positivo -> troppo lento
                // se error negativo -> troppo veloce

                if (error > 0){
                        direction = forward;
                }else if (error < 0){
                        direction = backward;
                }
                cbMotorMove(motor, direction, newDutyCycle);
                pthread_testcancel();
                sched_yield(); //segnalo allo scheduler che ho finito
        }
}
#endif