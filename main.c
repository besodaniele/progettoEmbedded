#include <concurrent.h>
#include "libcoderbot/include/motor.h"
#include "libcoderbot/include/cbdef.h"
#include "libcoderbot/include/encoder.h"
#include "controllo.h"
#include "odometria.h"
cbMotor_t motorL = {PIN_LEFT_FORWARD, PIN_LEFT_BACKWARD, forward};
cbMotor_t motorR = {PIN_RIGHT_FORWARD, PIN_RIGHT_BACKWARD, forward};

cbEncoder_t encoderL = {PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B, GPIO_PIN_NC, 0, 0, 0};
cbEncoder_t encoderR = {PIN_ENCODER_RIGHT_A, PIN_ENCODER_RIGHT_B, GPIO_PIN_NC, 0, 0, 0};

void init(){
	if(gpioInitialise() < 0)
		exit(EXIT_FAILURE);
	// Left
	cbMotorGPIOinit(&motorL);
	cbEncoderGPIOinit(&encoderL);
	cbEncoderRegisterISRs(&encoderL, 50);
	// Right
	cbMotorGPIOinit(&motorR);
	cbEncoderGPIOinit(&encoderR);
	cbEncoderRegisterISRs(&encoderR, 50);
}
void kill() {
    cbMotorReset(&motorL);
    cbMotorReset(&motorR);
    cbEncoderCancelISRs(&encoderL);
    cbEncoderCancelISRs(&encoderR);
    gpioTerminate();
}

int main(){
    init();
    atexit(kill);
    task_t controlloL = {.entry_point=controllo};
    task_t controlloR = {.entry_point=controllo};
    controllo_args_t argsL = {
        .goal_mm = 1000.0f,
        .targetSpeed_mm_s = 100.0f,
        .dutyCycle = 0.5f,
        .motor = motorL,
        .encoder = encoderL
    };
    controllo_args_t argsR = {
        .goal_mm = 1000.0f,
        .targetSpeed_mm_s = 100.0f,
        .dutyCycle = 0.5f,
        .motor = motorR,
        .encoder = encoderR
    };
    
    odometria_args_t odom_args = {
        .encoder_left = encoderL,
        .encoder_right = encoderR
    };
    task_t odom = {.entry_point=odometria};

    create_task(&odom, (void*)&odom_args);
    create_task(&controlloL, (void*)&argsL);
    create_task(&controlloR, (void*)&argsR);
    join_task(&controlloL);
    join_task(&controlloR);
    cancel_task(&odom);
    
}

