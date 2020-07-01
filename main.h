#include <avr/io.h> // for pin definitions
#include <stdint.h> // for data types

// define macros and constants
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define TRUE 1
#define FALSE 0
#define HOME_OFF 0 // homing states
#define HOME_SEARCH 1
#define HOME_FOUND 2
#define HOME_ON 3
#define UART_BAUD 9600UL
#define UART_UBRR (F_CPU/(8*UART_BAUD)-1) // calculate register value for baudrate for U2Xn = 1
#define UART_MAXSTRLEN 64
#define STR_TERM '\n'
#define RADIX 10 // for itoa
#define F_TINT 8000 // timer interrupt frequency
#define C_PRESC 64 // control frequency that much slower than F_TINT
#define CMD_ID "AX" STR(AX_ID) // axis specific commands must start with AX<id>
#define PIN_ENABLE PA6 // motor enable pin (low active)
#define PIN_STEP PB1 // motor step pin
#define PIN_DIR PB0 // motor direction pin
#define PIN_LED PA0 // status or testing LED pin
#define PIN_HOME PA4 // homing switch pin (low active)
#define PIN_SS1 PA7 // substep bit 1
#define PIN_SS2 PB2 // substep bit 2
#define PIN_SS3 PA3 // substep bit 3

// structs
typedef struct {
	// for a P control loop
	int32_t tarStepPos;
	int32_t curStepPos;
	int32_t oldStepPos;
	uint16_t tic;
	uint16_t stepPeriod;
	int8_t tarDir;
} pCtrl_t;

// variables
char uartReceiveStr[UART_MAXSTRLEN] = "";
char uartSendStr[UART_MAXSTRLEN] = "";
uint8_t gotCommand = FALSE;
volatile uint8_t gotControlReq = FALSE;
uint8_t substeps = 4; // num of microsteps which make 1 full step
uint16_t maxStepRate, minStepRate; // step rate limits
uint16_t kAcc = 100; // acceleration proportion
uint16_t kDec= 10; // decceleration proportion
uint8_t homing = HOME_OFF; // homing state
pCtrl_t ctrlState;

// prototypes
void GPIO_Init();
void USART0_Init();
void USART0_ReceiveChar();
void USART0_SendChar(const char c);
void USART0_SendString(const char* strP);
void ParseCommand(const char* strP);
void Timer0_Init();
int32_t DegToSteps(int32_t deg);
int32_t StepsToDeg(int32_t steps);
void SendNum(int16_t num);
void Control(pCtrl_t* ctrlP);
void PhyStep(pCtrl_t* ctrlP);
void SetSubsteps(uint8_t ss);
int32_t DegStrToSteps(const char* strP);
void SendStepsAsDeg(int32_t steps);