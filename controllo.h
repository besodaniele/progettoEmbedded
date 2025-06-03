#include <stdio.h>
#include <stdlib.h>
//#include <pigpio.h>
#include "libcoderbot/include/motor.h"
#include "libcoderbot/include/cbdef.h"
#include "libcoderbot/include/encoder.h"
#include "concurrent.h"
#define _USE_MATH_DEFINES
#include <math.h>
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
const float mmsPerTick = (WHEEL_RAY_MM * 2 * M_PI) / (TICKS_PER_REVOLUTION * TRANSMISSION_RATIO);

extern int clamp_counter = 0; //incrementata da entrambi i task del controllo dei motori, da proteggere con mutex

float clamp(float dutyCycle){
	if (clamp_counter>=CLAMP_LIMIT) exit(EXIT_FAILURE);
	if (dutyCycle > 1.0f) {
		clamp_counter++;
		return 1.0f;
	}
	else if (dutyCycle < 0.0f){
		clamp_counter++;
		return 0.1f;
	}
	else return dutyCycle;
}

typedef struct {
	float goal_mm;
	float targetSpeed_mm_s;
	float dutyCycle;
	cbMotor_t motor;
	cbEncoder_t encoder;
} controllo_args_t;

void controllo(void *args){
	controllo_args_t *controlArgs = (controllo_args_t *)args;
    float goal_mm = controlArgs->goal_mm;
    cbMotor_t motor = controlArgs->motor;
    float dutyCycle = controlArgs->dutyCycle;
	float targetSpeed_mm_s = controlArgs->targetSpeed_mm_s;
	cbEncoder_t encoder = controlArgs->encoder;
    //set_sched_deadline(80*1000,15*1000,200*1000);
    // da stabilire i valori

	int tick = 0;
	int prevTick = 0;
	float travelledDistance = 0.0f;
	float speed = 0.0f;
	float error = 0.0f;
	float newDutyCycle = 0.0f;
	float erroreAccumulatoL = 0.0f, erroreAccumulatoR = 0.0f;

	cbDir_t direction=forward;

	cbMotorMove(&motor, direction, dutyCycle);

	while(goal_mm > 0.0f){

		tick = encoder.ticks;

		travelledDistance = (tick - prevTick)*mmsPerTick;

		prevTick = tick;

		speed = (travelledDistance / PERIOD) * 1000; // il periodo è in millisecondi, per avere la velocità in mm/s mi riporto ai secondi

		error = targetSpeed_mm_s - speed;

		erroreAccumulatoL += error; //sorrenti dice che nell'integrale va considerato l'errore corrente, non so quanto mi convince
		
		newDutyCycle = KP*error + KI*erroreAccumulatoL;
		newDutyCycle = fabs(newDutyCycle);

		//controllo saturazione
		newDutyCycle = clamp(newDutyCycle);
		// se errore positivo -> troppo lento
		// se error negativo -> troppo veloce

		if (error > 0){
			direction = forward;
		}else if (error < 0){
			direction = backward;
			newDutyCycle = newDutyCycle *0.1f;
			newDutyCycle = clamp(newDutyCycle);
		}
		cbMotorMove(&motor, direction, newDutyCycle);
		goal_mm -= travelledDistance;
        pthread_testcancel();
        sched_yield(); //segnalo allo scheduler che ho finito
	}
}

