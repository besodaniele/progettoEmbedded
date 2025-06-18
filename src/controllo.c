#include "include/controllo.h"

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <math.h>
#include <errno.h>
#include "libcoderbot/include/cbdef.h"
#include "include/concurrent.h"
#include "include/constants.h"


// Clamp the duty cycle and track how many times clamping occurs
float clamp(float dutyCycle, int* clamp_counter) {

	if (*clamp_counter >= CLAMP_LIMIT) {
		exit(EXIT_FAILURE);
	}
	if (dutyCycle > 1.0f) {
		(*clamp_counter)++;
		return 1.0;
	} else if (dutyCycle < 0.0f) {
		(*clamp_counter)++;
		return 0.1f;
	}
	return dutyCycle;
}

// Control loop thread
void* controllo(void* args) {
    controllo_args_t *controlArgs = (controllo_args_t *)args;
    cbMotor_t* motor = controlArgs->motor;
    float dutyCycle = controlArgs->dutyCycle;
    float* targetSpeed_mm_s = controlArgs->targetSpeed_mm_s;
    cbEncoder_t* encoder = controlArgs->encoder;
    int* clamp_counter = controlArgs->clamp_counter;
    pthread_mutex_t* clamp_mutex = controlArgs->clamp_mutex;
    set_sched_deadline(15 * 1000 * 1000, 19 * 1000 * 1000, 20 * 1000 * 1000);

    int tick = 0;
    int prevTick = 0;
    float travelledDistance = 0.0f;
    float speed = 0.0f;
    float error = 0.0f;
    float newDutyCycle = 0.0f;
    float erroreAccumulato = 0.0f;
	int ret;
    cbDir_t direction = forward;

    cbMotorMove(motor, direction, dutyCycle);

    while (1) {
        tick = encoder->ticks;
        travelledDistance = (tick - prevTick) * mmsPerTick;
        prevTick = tick;

        speed = (travelledDistance / PERIOD) * 1000; // Convert to mm/s

        error = *targetSpeed_mm_s - speed;
	printf("%f\n",error);

        erroreAccumulato += error;

        if (encoder->pin_a == PIN_ENCODER_LEFT_A) {
            newDutyCycle = KP_L * error + KI_L * erroreAccumulato;
        } else if (encoder->pin_a == PIN_ENCODER_RIGHT_A) {
            newDutyCycle = KP_R * error + KI_R * erroreAccumulato;
        }

        newDutyCycle = fabs(newDutyCycle);
/*
	ret = pthread_mutex_trylock(clamp_mutex);
	if(ret!=0){
                if(ret==EBUSY){
                        //risorsa occupata
                        //puts("trovato risorsa bloccata\n");
                        pthread_testcancel();
			sched_yield();
                }else{
			errno = ret;
			if(pthread_mutex_unlock(clamp_mutex)!=0){
				perror("task controllo: pthread_mutex_unlock");
                        	exit(EXIT_FAILURE);
                	}
			perror("task controllo: pthread_mutex_unlock");
                        exit(EXIT_FAILURE);
        	}
        }else{
	        newDutyCycle = clamp(newDutyCycle, clamp_counter); //clamp blocca tutto
		if(pthread_mutex_unlock(clamp_mutex)!=0){
                        perror("task controllo: pthread_mutex_unlock");
                        exit(EXIT_FAILURE);
                }
	}
*/
ret = pthread_mutex_trylock(clamp_mutex);
if(ret != 0) {
    if(ret == EBUSY) {
        // Resource busy - mutex is locked by another thread
        pthread_testcancel();
        sched_yield();
    } else {
        // Other error occurred during trylock
        errno = ret;
        perror("task controllo: pthread_mutex_trylock");
        exit(EXIT_FAILURE);
    }
} else {
    // Successfully acquired the lock
    newDutyCycle = clamp(newDutyCycle, clamp_counter);
    if(pthread_mutex_unlock(clamp_mutex) != 0) {
        perror("task controllo: pthread_mutex_unlock");
        exit(EXIT_FAILURE);
    }
}




	if (error > 0) {
            direction = forward;
        } else if (error < 0) {
            direction = backward;
        }

        cbMotorMove(motor, direction, newDutyCycle);
        pthread_testcancel();
        sched_yield();
    }

    return NULL;
}