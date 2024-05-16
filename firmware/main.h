#include <avr/io.h> // for microcontroller specific peripheral definitions
#include <stdint.h> // for uint8_t, uint16_t, etc.
#include <avr/eeprom.h> // for storing axis ID permanently

// defines
#define FSTEP_REV 200 // number of full steps of the motor
#define UART_BAUD_RATE(BAUD_RATE) ((float)(F_CPU*64/(16*(float)BAUD_RATE))+0.5)
#define STR_TERM '\n'
#define UART_MAXSTRLEN 64
#define RADIX 10 // for itoa
#define TRUE 1
#define FALSE 0
#define F_TINT 8000 // timer interrupt frequency
#define C_PRESC 64 // control frequency that much slower than F_TINT
#define PIN_STEP PIN3_bm
#define PIN_DIR PIN4_bm
#define PIN_ENABLE PIN5_bm // low active
#define PIN_MS1 PIN6_bm
#define PIN_MS2 PIN7_bm
#define PIN_STALL PIN0_bm
#define PIN_HOME PIN1_bm // low active, needs pullup configuration

// own types
struct control {
	int32_t tarStepPos;
	int32_t curStepPos;
	int32_t oldStepPos;
	uint16_t tic;
	uint16_t stepPeriod;
	int8_t tarDir;
} ctrlState;

enum home {
	off, 
	search, 
	found
};

enum mode {
	pos, 
	vel
};

// variables
char uartRecvStr[UART_MAXSTRLEN] = "";
char uartSendStr[UART_MAXSTRLEN] = "";
char cmdID[] = "AX1"; // axis specific commands must start with AX<id>
uint8_t EEMEM eAxisIDAddr; // address pointer for axis ID stored in eeprom
uint8_t gotCommand = FALSE;
uint8_t substeps = 8; // num of microsteps which make 1 full step
uint16_t maxStepRate, minStepRate; // step rate limits
int32_t stepVel; // step velocity
uint16_t kAcc = 100; // acceleration proportion
uint16_t kDec= 10; // decceleration proportion
enum home homing = off; // homing state
enum mode moving = pos; // movement mode

// prototypes
void configClock();
void configGPIO();
void configUART();
void sendUARTChar(char c);
void sendUARTString(const char* strP);
void configTCA();
void parseCommand(const char* strP);
int32_t degToSteps(int32_t deg);
int32_t stepsToDeg(int32_t steps);
void sendNum(int16_t num);
void controlStep();
void phyStep();
void setSubsteps(uint8_t ss);
int32_t degStrToSteps(const char* strP);
void sendStepsAsDeg(int32_t steps);
void move(int32_t speed);