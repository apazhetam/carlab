/* PID.h */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "project.h"  // Project-specific header
#include <math.h>     // Math functions
#include <stdbool.h>  // Booleans
#include <stdio.h>    // Standard I/O functions
#include <stdlib.h>   // Standard library functions

typedef struct {
    // Controller gains
    float Kp;
    float Ki;
    float Kd;

    // Controller "memory"
    float integrator;
    float differentiator;

    // Required for integrator
    float prevError;
    float totalError;
} PIDController;

/*----------------------------------------------------------------------------*/

// PID Objects
PIDController pidMotor;
PIDController pidServo;

// PID controller constants for Motor
float PID_KP_MOTOR;
float PID_KI_MOTOR;
float PID_KD_MOTOR;

// PID controller constants for Servo
float PID_KP_SERVO;
float PID_KI_SERVO;
float PID_KD_SERVO;

/*----------------------------------------------------------------------------*/

const uint16 MAX_TIME_16B; // Maximum time value (16-bit timer)
const float CLOCK_FREQ;    // Clock frequency 
const float RADIUS;        // Radius of our wheel in ft
const float TARGETMOTOR;   // Target speed in ft/sec
const float TARGETSERVO;   // Target time in ms to view black line

// Track previous time for speed calculation
uint16 prevTime;

// PWM_MOTOR control range to prevent overspeeding
const int16 MIN_PWM_MOTOR;
const int16 MAX_PWM_MOTOR;

// Bias for the motor PWM, found to hold the car's speed close to 4 ft/sec
const int16 PWM_MOTOR_BIAS;

// Servo bias for initial steering addition
const int16 PWM_SERVO_BIAS;

// Servo PID additional variables
const int16 CANNOT_SEE_BLACK; // Fixed maximum value
int16 prevServoPWM;

// Booleans to control which values are printed
bool printMotor;
bool printServo;
bool printdt;
bool printErr;
bool printPrp;
bool printInt;
bool printDif;
bool printPWM;
bool printUpd;

/*----------------------------------------------------------------------------*/

/* Initializes a PID controller */
void PIDController_Init(PIDController *pid, float kp, float ki, float kd);

/* Update the PIDMotor controller */
void PIDMotor_Update(PIDController *pidMotor, float current, float deltaTime);

/* Update the PIDServo controller */
void PIDServo_Update(PIDController *pidServo, float deltaTime);

#endif

/* [] END OF FILE */