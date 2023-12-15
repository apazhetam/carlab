/* PID.c */
/* Implement PID control for the motor & servo PWM outputs. */

#include "PID.h"      // PID controller header

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
    if (printMotor && printErr) {
        sprintf(strbuf, "Err: %i.%i\r\n", (int16)(error), ((int16)(error*100))%100);
        UART_PutString(strbuf);
    }

	/* Calculate Proportional Value */
    float proportional = pidMotor->Kp * error;

    /* Print proportional value to UART */
    if (printMotor && printPrp) {
        sprintf(strbuf, "Prp: %i.%i\r\n", (int16)(proportional), ((int16)(proportional*100))%100);
        UART_PutString(strbuf);
    }
	
    /* Calculate Integral Value */
    pidMotor->totalError = pidMotor->totalError + (error * (deltaTime / CLOCK_FREQ)); // Formula from class
    pidMotor->integrator = pidMotor->Ki * pidMotor->totalError; // Multiply by our constant
    
    /* Print integral value to UART */
    if (printMotor && printInt) {
        sprintf(strbuf, "Int: %i.%i\r\n", (int16)(pidMotor->integrator), ((int16)(pidMotor->integrator*100))%100);
        UART_PutString(strbuf);
    }
    
	/* Calculate Differential Value */
    pidMotor->differentiator = pidMotor->Kd * ((error - pidMotor->prevError) / (deltaTime / CLOCK_FREQ));
    
    /* Print differential value to UART */
    if (printMotor && printDif) {
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
    pwmValue = PWM_MOTOR_BIAS + (int16) roundf(speedUpdate);

    /* Print PWM value to UART */
    if (printMotor && printPWM) {
        sprintf(strbuf, "PWM: %i.%i\r\n", pwmValue, pwmValue*100%100);
        UART_PutString(strbuf);
    }

    /* Apply PWM value limits */
    if (pwmValue > MAX_PWM_MOTOR) pwmValue = MAX_PWM_MOTOR;
    else if (pwmValue < MIN_PWM_MOTOR) pwmValue = MIN_PWM_MOTOR;

    /* Print modified PWM value to UART */
    if (printMotor && printPWM) {
        sprintf(strbuf, "PWM: %i.%i\r\n", pwmValue, pwmValue*100%100);
        UART_PutString(strbuf);
    }

    /* Print speed update to UART */
    if (printMotor && printUpd) {
        sprintf(strbuf, "Upd: %i.%i\r\n", (int16)(speedUpdate), ((int16)(speedUpdate*100))%100);
        UART_PutString(strbuf);
    }

    /* Send changes to motor */
    PWM_Motor_WriteCompare(pwmValue);
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
    pwmValue = PWM_SERVO_BIAS + (int16) roundf(steerUpdate);

    // If out of frame, then keep the previous value and update
    if (deltaTime >= CANNOT_SEE_BLACK) {
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

    /* Send changes to servo */
    PWM_Servo_WriteCompare(pwmValue);
}

/* [] END OF FILE */