#include <concurrent.h>
#include "libcoderbot/include/motor.h"
#include "libcoderbot/include/cbdef.h"
#include "libcoderbot/include/encoder.h"
#include "controllo.h"
#include "odometria.h"


pthread_mutex_t clamp_mutex = PTHREAD_MUTEX_INITIALIZER;    //inizializzo clamp_mutex

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
void turnOff() {
    cbMotorReset(&motorL);
    cbMotorReset(&motorR);
    cbEncoderCancelISRs(&encoderL);
    cbEncoderCancelISRs(&encoderR);
    gpioTerminate();
}
void gpio_terminate() {
    gpioTerminate();
    
}

int main(){
    init();
    atexit(gpio_terminate);
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
    
    clamp_counter = 0;
    odometria_args_t odom_args = {
        .encoder_left = encoderL,
        .encoder_right = encoderR
    };
    task_t odom = {.entry_point=odometria};
    
    if (mutex_init(&clamp_mutex) != 0) {
        failure("Failed to initialize clamp mutex");
    }
    
    if(create_task(&odom, (void*)&odom_args)!=0){
        failure("Failed to create odometry task");
    }
    if(create_task(&controlloL, (void*)&argsL)!=0){
        failure("Failed to create left control task");
    }
    if(create_task(&controlloR, (void*)&argsR)!=0){
        failure("Failed to create right control task");
    }
    // Attendo il completamento dei task di controllo
    // e cancello il task di odometria
    if (join_task(&controlloL) != 0) {
        failure("Failed to join left control task");
    }
    
    if (join_task(&controlloR) != 0) {
        failure("Failed to join right control task");
    }
    if (cancel_task(&odom) != 0) {
        failure("Failed to cancel odometry task");
    }
    
}

