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

const uint16 MAX_TIME = 65535;  // Maximum time value (16-bit timer)
const float CLOCK_FREQ = 10000; // Clock frequency 
const float RADIUS = 0.1;      // Radius of our wheel in ft
const float TARGET = 4;        // Target speed in ft/sec

// PID Object
PIDController pid;

// Track previous time
uint16 prevTime;

// PID controller constants
float PID_KP = 130;
float PID_KI = 0.3;
float PID_KD = 0;

// PWM control range to prevent overspeeding
int16 MIN_PWM = 0;
int16 MAX_PWM = 150;
 
// Our bias, found to hold the car's speed close to 4 ft/sec
const int16 pwmBias = 6;

/* Function to initialize the PID controller */
void PIDController_Init(PIDController *pid)
{
    /* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
    pid->totalError = 0.0f;

	pid->out = 0.0f;

    /* Set PID controller constants */
    pid->Kp  = PID_KP;
	pid->Ki  = PID_KI;
	pid->Kd  = PID_KD;
	// pid->tau = PID_TAU;
}

/* Function to update the motor speed */
void MotorUpdate(float value)
{
    PWM_WriteCompare(value);
}

/* Main function loop to update the PID controller */

void PIDController_Update(PIDController *pid, float current, float deltaTime) 
{   
    float speedUpdate;
    char strbuf[16];
    int16 pwmValue;
    
	/* Error signal */
    float error = TARGET - current;
    
    sprintf(strbuf, "Err: %i.%i\r\n", (int16)(error), ((int16)(error*100))%100);
    UART_PutString(strbuf);

	/* Calculate Proportional Value */
    float proportional = pid->Kp * error;

    /* Print proportional value to UART */
    sprintf(strbuf, "Prp: %i.%i\r\n", (int16)(proportional), ((int16)(proportional*100))%100);
    UART_PutString(strbuf);

	
    /* Calculate Integral Value */
    pid->totalError = pid->totalError + (error * (deltaTime / CLOCK_FREQ)); // Formula from class
    pid->integrator = pid->Ki * pid->totalError; // Multiply by our constant
    
    /* Print integral value to UART */
    sprintf(strbuf, "Int: %i.%i\r\n", (int16)(pid->integrator), ((int16)(pid->integrator*100))%100);
    UART_PutString(strbuf);
    
	/* Calculate Differential Value */
    pid->differentiator = pid->Kd * ((error - pid->prevError) / (deltaTime / CLOCK_FREQ));
    
    /* Print differential value to UART */
    sprintf(strbuf, "Dif: %i.%i\r\n", (int16)(pid->differentiator), ((int16)(pid->differentiator*100))%100);
    UART_PutString(strbuf);

	/* Compute Output */
    speedUpdate = proportional + pid->integrator + pid->differentiator;

    /* Apply a minimum and maximum speed threshold to prevent minor updates */
    if (speedUpdate > 0 && speedUpdate < 80) {
        speedUpdate = 0;
    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = current;

    /* Convert the speed update to a PWM update */
    pwmValue = pwmBias + (int16) roundf(speedUpdate);

    /* Print PWM value to UART */
    sprintf(strbuf, "PWM: %i.%i\r\n", pwmValue, pwmValue*100%100);
    UART_PutString(strbuf);

    /* Apply PWM value limits */
    if (pwmValue > MAX_PWM) pwmValue = MAX_PWM;
    else if (pwmValue < MIN_PWM) pwmValue = MIN_PWM;

    /* Print modified PWM value to UART */
    sprintf(strbuf, "PWM: %i.%i\r\n", pwmValue, pwmValue*100%100);
    UART_PutString(strbuf);

    /* Print speed update to UART */
    sprintf(strbuf, "Upd: %i.%i\r\n", (int16)(speedUpdate), ((int16)(speedUpdate*100))%100);
    UART_PutString(strbuf);

    /* Send changes to motor */
    MotorUpdate(pwmValue);
}



/* Interrupt Service Routine for timer */
CY_ISR(counter_inter) 
{    
    uint16 currentTime;
    uint16 deltaTime;
    float currentSpeed;
    char strbuf[16];
    char strbuf2[32];
    char strbuf3[32];
    
    /* Read current time */

    currentTime = Timer_ReadCapture();
    

    /* Calculate deltaTime */
    if (prevTime <= currentTime)
        deltaTime = prevTime + MAX_TIME - currentTime; // When the timer resets
    else
        deltaTime = prevTime - currentTime; // Normal Case

    /* Calculate speed in ft/sec */
    currentSpeed = (2 * M_PI * RADIUS * 10000) / (deltaTime * 6);
    
    // sprintf(strbuf, "Speed: %f", currentSpeed);
    sprintf(strbuf3, "Speed: %i.%i\r\n", (int16)(currentSpeed), ((int16)(currentSpeed*100))%100);
    UART_PutString(strbuf3);

    // TODO: CALL PID UPDATE FUNCTION HERE
    PIDController_Update(&pid, currentSpeed, deltaTime);

    prevTime = currentTime;
    Timer_ReadStatusRegister();
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    /* Initialize peripherals */
    UART_Start();
    LCD_Start();
    PWM_Start();
    Timer_Start();
    Counter_Interrupt_Start();
    Counter_Interrupt_SetVector(counter_inter);

    /* Display a startup message on LCD and XBEE */
    LCD_Position(0,0);
    LCD_PrintString("Start!");
    UART_PutString("HELLO!");
    
    /* Initialize the PID controller */
    PIDController_Init(&pid);

    /* Start the motor with an initial PWM bias */
    MotorUpdate(pwmBias);
    
    
    for(;;)
    {
        /* Application loop - unneeded as we use the counter interrupt */
    }
}

/* [] END OF FILE */
