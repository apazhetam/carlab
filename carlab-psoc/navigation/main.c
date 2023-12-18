/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
/* This is a C program for controlling motor speed using a PID controller. */

/* Include necessary header files */
#include "project.h"  // Project-specific header
#include "PID.h"      // PID controller header
#include <stdio.h>    // Standard I/O functions
#include <math.h>     // Math functions
#include <stdlib.h>   // Standard library functions
#include <stdbool.h>   // Booleans

const uint16 MAX_TIME_16B = 65535;  // Maximum time value (16-bit timer)
const float CLOCK_FREQ = 10000; // Clock frequency 
const float RADIUS = 0.1;      // Radius of our wheel in ft
const float TARGETMOTOR = 4;        // Target speed in ft/sec
const float TARGETSERVO = 705;      // Target time in ms to view black line

// PID Objects
PIDController pidMotor;
PIDController pidServo;

// Track previous time for speed calculation
uint16 prevTime;

// PID controller constants for Motor
float PID_KP_MOTOR = 100;
float PID_KI_MOTOR = 0.3;
float PID_KD_MOTOR = 0;

// PID controller constants for Servo
float PID_KP_SERVO = 0.05;
float PID_KI_SERVO = 0;
float PID_KD_SERVO = 0.04;

// PWM_MOTOR control range to prevent overspeeding
int16 MIN_PWM_MOTOR = 0;
int16 MAX_PWM_MOTOR = 150;
 
// PWM_SERVO control range for max steer
int16 MIN_PWM_SERVO = 0;
int16 MAX_PWM_SERVO = 255;

// Bias for the motor PWM, found to hold the car's speed close to 4 ft/sec
const int16 pwmMotorBias = 6;

// Servo bias for initial steering addition
const int16 pwmServoBias = 127;

// Servo PID additional variables
const int16 cannotSeeBlack = 1410; //Fixed maximum value
int16 prevServoPWM = 127;

// Booleans to control which values are printed
bool printMotor = false;
bool printServo = true;

bool printdt = true;
bool printErr = true;
bool printPrp = true;
bool printDif = true;
bool printPWM = true;
bool printUpd = true;

/* Initializes a PID controller */
void PIDController_Init(PIDController *pid, float kp, float ki, float kd)
{
    /* Set PID controller constants */
    pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

    /* Initialize other controller variables */
	pid->integrator = 0.0f;
	pid->differentiator  = 0.0f;
	pid->prevError  = 0.0f;
    pid->totalError = 0.0f;
	pid->out = 0.0f;
}

/* Update the PIDMotor controller */
void PIDMotor_Update(PIDController *pidMotor, float current, float deltaTime) 
{   
    float speedUpdate;
    char strbuf[16];
    int16 pwmValue;
    
	/* Find error between current speed and target speed */
    float error = TARGETMOTOR - current;
    
    /* Print error to UART */
    if (printMotor) {
        sprintf(strbuf, "Err: %i.%i\r\n", (int16)(error), ((int16)(error*100))%100);
        UART_PutString(strbuf);
    }

	/* Calculate Proportional Value */
    float proportional = pidMotor->Kp * error;

    /* Print proportional value to UART */
    if (printMotor) {
        sprintf(strbuf, "Prp: %i.%i\r\n", (int16)(proportional), ((int16)(proportional*100))%100);
        UART_PutString(strbuf);
    }
	
    /* Calculate Integral Value */
    pidMotor->totalError = pidMotor->totalError + (error * (deltaTime / CLOCK_FREQ)); // Formula from class
    pidMotor->integrator = pidMotor->Ki * pidMotor->totalError; // Multiply by our constant
    
    /* Print integral value to UART */
    if (printMotor) {
        sprintf(strbuf, "Int: %i.%i\r\n", (int16)(pidMotor->integrator), ((int16)(pidMotor->integrator*100))%100);
        UART_PutString(strbuf);
    }
    
	/* Calculate Differential Value */
    pidMotor->differentiator = pidMotor->Kd * ((error - pidMotor->prevError) / (deltaTime / CLOCK_FREQ));
    
    /* Print differential value to UART */
    if (printMotor) {
        sprintf(strbuf, "Dif: %i.%i\r\n", (int16)(pidMotor->differentiator), ((int16)(pidMotor->differentiator*100))%100);
        UART_PutString(strbuf);
    }

	/* Compute the PID update */
    speedUpdate = proportional + pidMotor->integrator + pidMotor->differentiator;

    /* Apply a minimum and maximum speed threshold to prevent minor updates */
    if (speedUpdate > 0 && speedUpdate < 80) {
        speedUpdate = 0;
    }

	/* Store current error for later use */
    pidMotor->prevError = error;

    /* Convert the speed update to a PWM update */
    pwmValue = pwmMotorBias + (int16) roundf(speedUpdate);

    /* Print PWM value to UART */
    if (printMotor) {
        sprintf(strbuf, "PWM: %i.%i\r\n", pwmValue, pwmValue*100%100);
        UART_PutString(strbuf);
    }

    /* Apply PWM value limits */
    if (pwmValue > MAX_PWM_MOTOR) pwmValue = MAX_PWM_MOTOR;
    else if (pwmValue < MIN_PWM_MOTOR) pwmValue = MIN_PWM_MOTOR;

    /* Print modified PWM value to UART */
    if (printMotor) {
        sprintf(strbuf, "PWM: %i.%i\r\n", pwmValue, pwmValue*100%100);
        UART_PutString(strbuf);
    }

    /* Print speed update to UART */
    if (printMotor) {
        sprintf(strbuf, "Upd: %i.%i\r\n", (int16)(speedUpdate), ((int16)(speedUpdate*100))%100);
        UART_PutString(strbuf);
    }

    /* Send changes to motor */
    PWM_Motor_WriteCompare(pwmValue);
}

/* Interrupt Service Routine triggered by Hall Effect sensor */
CY_ISR(halleffect_inter) 
{    
    uint16 currentTime;
    uint16 deltaTime;
    float currentSpeed;
    char strbuf[32];
    
    /* Read current time */
    currentTime = HallEffect_Timer_ReadCapture();    

    /* Calculate deltaTime */
    if (prevTime <= currentTime)
        deltaTime = prevTime + MAX_TIME_16B - currentTime; // When the timer resets
    else
        deltaTime = prevTime - currentTime; // Normal Case

    /* Calculate speed in ft/sec */
    currentSpeed = (2 * M_PI * RADIUS * 10000) / (deltaTime * 6);
    
    /* Print current speed to UART */
    if (printMotor) {
        sprintf(strbuf, "Speed: %i.%i\r\n", (int16)(currentSpeed), ((int16)(currentSpeed*100))%100);
        UART_PutString(strbuf);
    }

    /* CALL PID UPDATE FUNCTION */ 
    PIDMotor_Update(&pidMotor, currentSpeed, deltaTime);

    prevTime = currentTime;
    HallEffect_Timer_ReadStatusRegister();
}

/* Update the PIDServo controller */
void PIDServo_Update(PIDController *pidServo, float deltaTime) 
{
    float steerUpdate;
    char strbuf[16];
    int16 pwmValue;
    
	/* Calculate error between target time to find black tape and current deltaTime */
    float error = deltaTime - TARGETSERVO;
    
    /* Print error to UART */
    if (printServo && printErr) {
        sprintf(strbuf, "Err: %i.%i\r\n", (int16)(error), ((int16)(error*100))%100);
        UART_PutString(strbuf);
    }

	/* Calculate Proportional Value */
    float proportional = pidServo->Kp * error;

    /* Print proportional value to UART */
    if (printServo && printPrp) {
        sprintf(strbuf, "Prp: %i.%i\r\n", (int16)(proportional), ((int16)(proportional*100))%100);
        UART_PutString(strbuf);
    }
	
    /* Calculate Integral Value */
    pidServo->totalError = pidServo->totalError + (error * (deltaTime / CLOCK_FREQ));
    pidServo->integrator = pidServo->Ki * pidServo->totalError; // Multiply by our constant
    
	/* Calculate Differential Value */
    pidServo->differentiator = pidServo->Kd * (error - pidServo->prevError);
    
    /* Print differential value to UART */
    if (printServo && printDif) {
        sprintf(strbuf, "Dif: %i.%i\r\n", (int16)(pidServo->differentiator), ((int16)(pidServo->differentiator*100))%100);
        UART_PutString(strbuf);
    }

	/* Compute the PID update */
    steerUpdate = proportional + pidServo->integrator + pidServo->differentiator;
    
	/* Store current error for later use */
    pidServo->prevError = error;

    /* Convert the speed update to a PWM update */
    pwmValue = pwmServoBias + (int16) roundf(steerUpdate);

    // If out of frame, then keep the previous value and update
    if (deltaTime >= cannotSeeBlack) {
        pwmValue = prevServoPWM;
    }
    prevServoPWM = pwmValue;

    /* Print PWM value to UART */
    if (printServo && printPWM) {
        sprintf(strbuf, "PWM: %i.%i\r\n", pwmValue, pwmValue*100%100);
        UART_PutString(strbuf);
    }

    /* Print steer update to UART */
    if (printServo && printUpd) {
        sprintf(strbuf, "Upd: %i.%i\r\n", (int16)(steerUpdate), ((int16)(steerUpdate*100))%100);
        UART_PutString(strbuf);
    }

    /* Send changes to motor */
    PWM_Servo_WriteCompare(pwmValue);
}

/* Interrupt Service Routine triggered when Camera detects the black tape */
CY_ISR(camera_inter) 
{
    uint16 currentTime;
    uint16 deltaTime;
    char strbuf[16];
    
    /* Read current time */
    currentTime = Camera_Timer_ReadCapture();    

    /* Calculate deltaTime */
    deltaTime = MAX_TIME_16B - currentTime; 

    if (printServo && printdt) {
        sprintf(strbuf, "dT : %i\r\n", deltaTime);
        UART_PutString(strbuf);
    }
    
    /* CALL SERVO UPDATE FUNCTION */
    PIDServo_Update(&pidServo, deltaTime);

    Camera_Timer_ReadStatusRegister();
}   


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    /* Initialize peripherals */
    UART_Start();
    LCD_Start(); 

    PWM_Motor_Start();
    HallEffect_Timer_Start();
    
    PWM_Servo_Start();
    Camera_Timer_Start();
    Color_Compare_Start();
    DAC_Start();

    /* Configure interrupts */
    HallEffect_Interrupt_Start();
    Camera_Interrupt_Start();
    HallEffect_Interrupt_SetVector(halleffect_inter);
    Camera_Interrupt_SetVector(camera_inter);

    /* Display a startup message on LCD and XBee */
    LCD_Position(0,0);
    LCD_PrintString("Start!");
    UART_PutString("HELLO!");
    
    /* Initialize the PID controllers */
    PIDController_Init(&pidMotor, PID_KP_MOTOR, PID_KI_MOTOR, PID_KD_MOTOR);
    PIDController_Init(&pidServo, PID_KP_SERVO, PID_KI_SERVO, PID_KD_SERVO);

    for(;;)
    {
        /* Application loop - unneeded as we use the counter interrupt */
    }
}

/* [] END OF FILE */
