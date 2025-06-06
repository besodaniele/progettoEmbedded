#include "concurrent.h"
#include "libcoderbot/include/motor.h"
#include "libcoderbot/include/cbdef.h"
#include "libcoderbot/include/encoder.h"
#include "controllo.h"
//#include "odometria.h"
#include <pigpio.h>

pthread_mutex_t clamp_mutex = PTHREAD_MUTEX_INITIALIZER;    //inizializzo clamp_mutex
typedef struct motorEncoderPackage {
    cbMotor_t* motorR;
    cbMotor_t* motorL;
    cbEncoder_t* encoderR;
    cbEncoder_t* encoderL;
} motorEncoderPackage_t;

void init(motorEncoderPackage_t *package) {
        cbEncoder_t* encoderL = package->encoderL;
        cbEncoder_t* encoderR = package->encoderR;
        cbMotor_t* motorL = package->motorL;
        cbMotor_t* motorR = package->motorR;

        if(gpioInitialise() < 0)
                exit(EXIT_FAILURE);
        // Left
        cbMotorGPIOinit(motorL);
        cbEncoderGPIOinit(encoderL);
        cbEncoderRegisterISRs(encoderL, 50);
        // Right
        cbMotorGPIOinit(motorR);
        cbEncoderGPIOinit(encoderR);
        cbEncoderRegisterISRs(encoderR, 50);
}
void kill_robot(motorEncoderPackage_t *package) {
    cbEncoder_t* encoderL = package->encoderL;
    cbEncoder_t* encoderR = package->encoderR;
    cbMotor_t* motorL = package->motorL;
    cbMotor_t* motorR = package->motorR;

    puts("exiting");
    cbMotorReset(motorL);
    cbMotorReset(motorR);
    cbEncoderCancelISRs(encoderL);
    cbEncoderCancelISRs(encoderR);
}
void gpio_terminate(){
        gpioTerminate();
}
int main(){

    cbMotor_t motorL = {PIN_LEFT_FORWARD, PIN_LEFT_BACKWARD, forward};
    cbMotor_t motorR = {PIN_RIGHT_FORWARD, PIN_RIGHT_BACKWARD, forward};

    cbEncoder_t encoderL = {PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B, GPIO_PIN_NC, 0, 0, 0};
    cbEncoder_t encoderR = {PIN_ENCODER_RIGHT_A, PIN_ENCODER_RIGHT_B, GPIO_PIN_NC, 0, 0, 0};

    motorEncoderPackage_t package = {
        .motorR = &motorR,
        .motorL = &motorL,
        .encoderR = &encoderR,
        .encoderL = &encoderL
    };

    init(&package);


    atexit(gpio_terminate);
    int clamp_counter = 0;
    task_t controlloL = {.tid=0,.entry_point=controllo};
    task_t controlloR = {.tid=1,.entry_point=controllo};
    controllo_args_t argsL = {
        .targetSpeed_mm_s = 1000.0f,
        .dutyCycle = 0.5f,
        .motor = &motorL,
        .encoder = &encoderL,
        .clamp_counter = &clamp_counter
    };
    controllo_args_t argsR = {
        .targetSpeed_mm_s = 1000.0f,
        .dutyCycle = 0.5f,
        .motor = &motorR,
        .encoder = &encoderR,
        .clamp_counter = &clamp_counter
    };
        /*
    odometria_args_t odom_args = {
        .encoder_left = encoderL,
        .encoder_right = encoderR
    };

    task_t odom = {.entry_point=odometria};
        */
    if (mutex_init(&clamp_mutex) != 0) {
        failure("Failed to initialize clamp mutex");
    }
/*
    if(create_task(&odom, (void*)&odom_args)!=0){
        failure("Failed to create odometry task");
    }
*/
    if(create_task(&controlloL, (void*)&argsL)!=0){
        failure("Failed to create left control task");
    }
    if(create_task(&controlloR, (void*)&argsR)!=0){
        failure("Failed to create right control task");
    }
    // Attendo il completamento dei task di controllo
    // e cancello il task di odometria
        sleep(10);
        cancel_task(&controlloL);
        cancel_task(&controlloR);

    if (join_task(&controlloL) != 0) {
        kill_robot(&package);
        failure("Failed to join left control task");
    }

    if (join_task(&controlloR) != 0) {
        failure("Failed to join right control task");
        kill_robot(&package);
    }
        kill_robot(&package);
/*
    if (cancel_task(&odom) != 0) {
        failure("Failed to cancel odometry task");
    }
*/
        exit(EXIT_SUCCESS);
}