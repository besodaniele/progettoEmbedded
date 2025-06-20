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
#include <stdatomic.h>

extern pthread_mutex_t clamp_mutex;     //mutex di clamp counter

float clamp(float dutyCycle,_Atomic(int*) clamp_counter){
        mutex_lock(&clamp_mutex);

	if (*(atomic_load(&clamp_counter))>=CLAMP_LIMIT){
                mutex_unlock(&clamp_mutex);
                exit(EXIT_FAILURE);
        }
        if (dutyCycle > 1.0f) {
         	atomic_fetch_add(&clamp_counter, 1);
	        mutex_unlock(&clamp_mutex);
                return 1.0f;
        }

        else if (dutyCycle < 0.0f){
                atomic_fetch_add(&clamp_counter, 1);
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
