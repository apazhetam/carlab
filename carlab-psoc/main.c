/* main.c */

#include "PID.h"      // PID controller header
#include <string.h>   // String functions

#define MAX_STRING_LENGTH 32    // Input coming via UART
#define FORWARD_VAL 1           // H_bridge inverter setting for forward
#define BACKWARD_VAL 2          // H_bridge inverter setting for backwards
#define SERVO_PWM_LEFT 5        // Fixed PWM for Left Turn
#define SERVO_PWM_RIGHT 250     // Fixed PWM for Right Turn
#define SERVO_PWM_FORWARD 127   // Fixed PWM for Forward or Backward
#define FORWARD_TIME 5          // Execution time for wait() state in seconds
#define TURN_TIME 3          // Execution time for wait() state in seconds
#define STATE_CLOCK_FREQ 10

/*----------------------------------------------------------------------------*/

#define RADIUS 0.1     // Radius of our wheel in ft


/*----------------------------------------------------------------------------*/

// PID controller constants for Motor
float PID_KP_MOTOR = 100;
float PID_KI_MOTOR = 0.3;
float PID_KD_MOTOR = 0;

// PID controller constants for Servo
float PID_KP_SERVO = 0.05;
float PID_KI_SERVO = 0;
float PID_KD_SERVO = 0.04;

/*----------------------------------------------------------------------------*/

const uint16 MAX_TIME_16B = 65535;  // Maximum time value (16-bit timer)
const float CLOCK_FREQ = 10000;    // Clock frequency 
const float TARGETMOTOR = 5;       // Target speed in ft/sec
const float TARGETSERVO = 705;     // Target time in ms to view black line

// PWM_MOTOR control range to prevent overspeeding
const int16 MIN_PWM_MOTOR = 5;
// const int16 MAX_PWM_MOTOR = 150;
const int16 MAX_PWM_MOTOR = 254;

// Bias for the motor PWM, found to hold the car's speed close to 4 ft/sec
const int16 PWM_MOTOR_BIAS = 140;
// const int16 PWM_MOTOR_BIAS = 180;

// Servo bias for initial steering addition
const int16 PWM_SERVO_BIAS = 127;

int16 prevServoPWM = 127;

// Booleans to control which values are printed
bool printMotor = false;
bool printServo = false;
bool printdt = false;
bool printErr = false;
bool printPrp = false;
bool printInt = false;
bool printDif = false;
bool printPWM = true;
bool printUpd = false;

/*----------------------------------------------------------------------------*/

/* Define constants */
enum Statetype {FORWARD, BACKWARD, LEFT, RIGHT, STOP};

/*----------------------------------------------------------------------------*/

void StartMotor();
void StopMotor();
void StartServo();
void StopServo();
void waitSeconds(float waitTime);
void handleForwardState(enum Statetype prevState);
void handleBackwardState(enum Statetype prevState);
void handleLeftState(enum Statetype prevState);
void handleRightState(enum Statetype prevState);
void handleStopState();
void execute();


// All declared volatile because of quick changes
volatile int continueExecute = false;

volatile int hallSignal = false;
volatile int raspberrySignal = false;

volatile int hallEnabled = false;
volatile int raspberryEnabled = true;

volatile int firstInstruction = true;
volatile int actionsComplete = false;

volatile int delayComplete = false;
volatile int turnTimerComplete = false;

volatile int turning = false;

volatile uint16 servoPWM = SERVO_PWM_FORWARD;

volatile int turningRight = true;
volatile int turnDone = false;
volatile int wentForward = false;

/*----------------------------------------------------------------------------*/

/* Define constants */
// enum Statetype {FORWARD, BACKWARD, LEFT, RIGHT, STOP};

uint16 currentServoPWM = 127;

// Variable to hold directional command string
char receivedString[MAX_STRING_LENGTH];
uint16 stringPointer = 0;

/*----------------------------------------------------------------------------*/
/* Interrupt service routines for different states*/

/* UNUSED: Interrupt Service Routine triggered when Camera detects the black tape */
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

/* Interrupt Service Routine triggered by Hall Effect sensor */
CY_ISR(halleffect_inter) 
{    
    // UART_PutString("hall int\r\n");
    if (hallEnabled) {
        hallSignal = true;
    }
    PWM_Servo_WriteCompare(servoPWM);
}

/* Interrupt Service Routine triggered once movement is complete */
CY_ISR(state_inter)
{   
    UART_PutString("state int\r\n");
    delayComplete = true;
    State_Timer_ReadStatusRegister();
    State_Interrupt_ClearPending();
}
/* Interrupt Service Routine triggered once turn is complete */
CY_ISR(turn_inter)
{   
    UART_PutString("turn int\r\n");
    turnTimerComplete = true;
    Turn_Timer_ReadStatusRegister();
    Turn_Interrupt_ClearPending();
}

/* Interrupt Service Routine triggered once extra wait is complete */
CY_ISR(state_1_inter)
{
    State_Timer_1_Stop();
    UART_PutString("waited 1 second\r\n");
    UART_RXBITCTR_CONTROL_REG |= (uint8) UART_CNTR_ENABLE;    // ENABLE UART RX
    State_Timer_1_ReadStatusRegister();
}

/* Interrupt Service Routine triggered to allow UART to come through */
CY_ISR(raspberry_inter)
{
    if (raspberryEnabled) raspberrySignal = true;
}


/*----------------------------------------------------------------------------*/


/* Read UART Information from Raspberry */
void RaspberryFunction() {
    char strbuf[60]; // Buffer to hold commands

    UART_PutString("Rx:    ");
    char receivedByte = (char) UART_ReadRxData(); // Receieves one byte
    UART_PutChar(receivedByte);
    UART_PutString("\r\n");

    // Halt byte. If receieved, then stop command collection
    if (receivedByte == 'X') {
        stringPointer = 0; // Reset pointer

        sprintf(strbuf, "receivedString: %s\r\n", receivedString);
        UART_PutString(strbuf);

        UART_RXBITCTR_CONTROL_REG &= (uint8) ~UART_CNTR_ENABLE;    // Disable UART RX
        raspberryEnabled = false;
        continueExecute = true;
    }
    else {
        receivedString[stringPointer] = receivedByte; // Store
        receivedString[stringPointer + 1] = ',';
        receivedString[stringPointer + 2] = '\0';
        stringPointer += 2; // Increment to next byte
    }
}

/* Speed Control function through hall effect sensor*/
void HallFunction() {
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

/*----------------------------------------------------------------------------*/

/* Commands for specific motions. */

void StartMotor()
{
    PIDController_Init(&pidMotor, PID_KP_MOTOR, PID_KI_MOTOR, PID_KD_MOTOR);
    PIDController_Init(&pidServo, PID_KP_SERVO, PID_KI_SERVO, PID_KD_SERVO);
    HallEffect_Timer_Start();
    PWM_Motor_Start();
    hallEnabled = true;
    HallEffect_Interrupt_SetPending();
}

void StopMotor()
{
    hallEnabled = false;
    HallEffect_Interrupt_ClearPending();
    HallEffect_Timer_Stop();
    PWM_Motor_Stop();
}

void StartServo()
{
    PWM_Servo_Start();
}

void StopServo()
{
    PWM_Servo_Stop();
}

// To be used in future implementation: command to follow line
void StartNagivation()
{
    Camera_Interrupt_Start();
    Camera_Interrupt_SetVector(camera_inter);
    Camera_Timer_Start();
    PWM_Servo_Start();
    Color_Compare_Start();
    DAC_Start();
}

// To be used in future implementation: command to stop following line
void StopNagivation()
{
    Camera_Interrupt_Stop();
    Camera_Timer_Stop();
    PWM_Servo_Stop();
    Color_Compare_Stop();
    DAC_Stop();
}

/*----------------------------------------------------------------------------*/
/* State Machine Implementation*/

/* Implement the FORWARD state. */
void handleForwardState(enum Statetype prevState) 
{
    StartMotor();
    StartServo();
    HBridge_Reg_Write(FORWARD_VAL);
    UART_PutString("Start Forward\r\n");
    PWM_Servo_WriteCompare(SERVO_PWM_FORWARD);
    servoPWM = SERVO_PWM_FORWARD;
}

/* Implement the BACKWARD state. */
void handleBackwardState(enum Statetype prevState) 
{
    StartMotor();
    StartServo();
    HBridge_Reg_Write(BACKWARD_VAL);
    UART_PutString("Start Backward\r\n");
    PWM_Servo_WriteCompare(SERVO_PWM_FORWARD);
    servoPWM = SERVO_PWM_FORWARD;
}

/* Implement the LEFT state. */
void handleLeftState(enum Statetype prevState) 
{
    StartMotor();
    StartServo();
    HBridge_Reg_Write(FORWARD_VAL);
    UART_PutString("Start Left\r\n");
    PWM_Servo_WriteCompare(SERVO_PWM_LEFT);
    servoPWM = SERVO_PWM_LEFT;
}

/* Implement the RIGHT state. */
void handleRightState(enum Statetype prevState) 
{
    StartMotor();
    StartServo();
    HBridge_Reg_Write(FORWARD_VAL);
    UART_PutString("Start Right\r\n");
    PWM_Servo_WriteCompare(SERVO_PWM_RIGHT);
    servoPWM = SERVO_PWM_RIGHT;

}

/* Implement the STOP state. */
void handleStopState() 
{
    HBridge_Reg_Write(FORWARD_VAL); // To reset HBRidge to default
    PWM_Servo_WriteCompare(SERVO_PWM_FORWARD);
    UART_PutString("Stopping....\r\n");
    StopMotor();
    StopServo();
}

void execute() 
{
    enum Statetype state = STOP;

    char * nextAction; 
    char strbuf[32];
    UART_PutString("test.\r\n");

    sprintf(strbuf, "test receivedString: %s\r\n", receivedString);
    UART_PutString(strbuf); // Save UART string into string buffer

    // Marker for first char in sequence
    if (firstInstruction) {
        UART_PutString("first char.\r\n"); 
        nextAction = strtok(receivedString,",");
    }
    else {
        nextAction = strtok(NULL,",");
    }

    // STATE MACHINE IMPLEMENTATION
    if (nextAction != NULL) {
        sprintf(strbuf, "%c\r\n", *nextAction); // Take next action
        UART_PutString(strbuf); 

        switch (*nextAction) {
            case 'F':
                handleForwardState(state);
                wentForward = true;
                turning = false;
                state = FORWARD;
                break;
            case 'B':
                handleBackwardState(state);
                state = BACKWARD;
                turning = false;
                break;
            case 'L':
                if (wentForward) {
                    wentForward = false;
                    handleLeftState(state);
                    turning = true;
                    turningRight = false;
                    turnDone = true;
                }
                else {
                    handleForwardState(state);
                    turning = true;
                    turningRight = false;
                    turnDone = false;
                }
                state = LEFT;
                break;
            case 'R':
                if (wentForward) {
                    wentForward = false;
                    handleRightState(state);
                    turning = true;
                    turningRight = true;
                    turnDone = true;
                }
                else {
                    handleForwardState(state);
                    turning = true;
                    turningRight = true;
                    turnDone = false;  
                }
                state = RIGHT;
                break;
            case 'S':
                handleStopState(state);
                turning = false;
                state = STOP;
                break;
            default: // Skip movement if unsure
                sprintf(strbuf, "Invalid direction: %c\r\n", nextAction);
                UART_PutString(strbuf);
        }
    }
    else {
        firstInstruction = false;
        UART_PutString("All actions completed!\r\n");
        handleStopState(state);

        // Reset LCD Dispaly
        LCD_WriteControl(LCD_CLEAR_DISPLAY); 
        LCD_Position(0,0);
        LCD_PrintString("Ready!");

        // Clear buffer, ready for next input from Pi
        UART_ClearRxBuffer();
        UART_RXBITCTR_CONTROL_REG |= (uint8) UART_CNTR_ENABLE;    // Enable UART RX
        raspberryEnabled = true;

        actionsComplete = true;
        turning = false;
    }  
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    /* Initialize peripherals */
    UART_Start();
    LCD_Start();

    Raspberry_Interrupt_Start();
    Raspberry_Interrupt_SetVector(raspberry_inter);

    HallEffect_Interrupt_Start();
    HallEffect_Interrupt_SetVector(halleffect_inter);

    State_Interrupt_Start();
    State_Interrupt_SetVector(state_inter);

    Turn_Interrupt_Start();
    Turn_Interrupt_SetVector(turn_inter);
    
    State_Interrupt_1_Start();
    State_Interrupt_1_SetVector(state_1_inter);

    /* Display a startup message on LCD and XBee */
    LCD_Position(0,0);
    LCD_PrintString("Hello!");
    UART_PutString("HELLO!\r\n");

    /* ------------------------------------------------------------------- */

    UART_ClearRxBuffer();
    UART_RXBITCTR_CONTROL_REG &= (uint8) ~UART_CNTR_ENABLE;   // DISABLE UART RX
    State_Timer_1_Start();

    /* ------------------------------------------------------------------- */
    /* Set STOP as the starting state */
    enum Statetype state = STOP;
    handleStopState(); 

    for(;;)
    {
        // Runs the command in our buffer
        if (continueExecute) {
            continueExecute = false;
            execute();
            UART_PutString("End Action.\r\n");
            firstInstruction = false;

            // If reached the end, then reset beginning
            if (actionsComplete) {
                firstInstruction = true; // So that the next string's first char is recorded
            }
            else if (turning) {
                Turn_Timer_Start();
                Turn_Timer_ReadStatusRegister();    
                Turn_Interrupt_ClearPending();
            }
            else {
                State_Timer_Start();
                State_Timer_ReadStatusRegister();    
                State_Interrupt_ClearPending();
            }

            actionsComplete = false;
        }


        // If finished previous string, then allow new input
        if (raspberrySignal) {
            raspberrySignal = false;
            UART_PutString("Raspberry Signal\r\n");
            RaspberryFunction();
        }

        // Determins whether to use speed control PID
        if (hallSignal) {
            hallSignal = false;
            HallFunction();
        }

        // If at the end of the delay, then continue execution
        if (delayComplete) {
            delayComplete = false;
            State_Timer_Stop(); 
            // Move to next set of command
            continueExecute = true;
        }

        // Separate times for turn. If ready to turn, turn.
        if (turnTimerComplete) {
            turnTimerComplete = false;
            Turn_Timer_Stop(); 
            // If turn not complete yet, then turn
            if (!turnDone) {
                turnDone = true;
                if (turningRight) {
                    handleRightState(RIGHT);
                }
                else {
                    handleLeftState(LEFT);
                }

                Turn_Timer_Start();
                Turn_Timer_ReadStatusRegister();    
                Turn_Interrupt_ClearPending(); 
            }
            // Move to next set of command
            else {
                continueExecute = true;
            }
        }
    }
}

/* [] END OF FILE */