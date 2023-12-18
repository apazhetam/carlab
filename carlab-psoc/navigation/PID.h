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

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Controller "memory" */
	float integrator;
	float differentiator;

	float prevError;		/* Required for integrator */
    float totalError;		/* Required for integrator */
    
	/* Controller output */
	float out;

} PIDController;

void PIDController_Init(PIDController *pid, float kp, float ki, float kd);
void PIDController_Update(PIDController *pid, float current, float deltaTime);

#endif

/* [] END OF FILE */
