#include <avr/io.h> // for pin definitions
#include <stdint.h> // for shorter types
#include <avr/interrupt.h> // for sei(), cli() and ISR()
#include <stdlib.h>	// for itoa, atoi
#include <string.h>	// for string operations like cmp and cat

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
	uint16_t tic;
	uint16_t stepPeriod;
	int8_t tarDir;
} pCtrl_t;

// variables
volatile char uartReceiveStr[UART_MAXSTRLEN] = "";
char uartSendStr[UART_MAXSTRLEN] = "";
volatile uint8_t gotUARTReq, gotControlReq = FALSE;
uint8_t substeps = 4; // num of microsteps which make 1 full step
uint16_t maxStepRate, minStepRate; // step rate limits
uint16_t kP = 10; // proportional control factor
uint8_t homing = HOME_OFF; // homing state
pCtrl_t P1;
pCtrl_t P2;

// prototypes
void GPIO_Init();
void USART0_Init();
void USART0_SendChar(const char c);
void USART0_SendString(const char* strP);
void ParseCommand(const char* strP);
void Timer0_Init();
int32_t DegToSteps(int32_t deg);
int32_t StepsToDeg(int32_t steps);
int16_t ParseNum(const char* strP);
void SendNum(int16_t num);
void Control(pCtrl_t* ctrlP);
void PhyStep(pCtrl_t* ctrlP);
void SetSubsteps(uint8_t ss);
int32_t DegStrToSteps(const char* strP);
void SendStepsAsDeg(int32_t steps);

ISR(TIMER0_COMPA_vect) {
	// timer 0 compare interrupt
	gotControlReq = TRUE;
}

ISR(USART0_RX_vect) {
	// receiving chars via UART Rx
	static uint8_t uartStrCount = 0;
	char nextChar = UDR0;
    if (nextChar != '\n' && nextChar != '\r' && uartStrCount < UART_MAXSTRLEN) {
    	// still getting valid chars
		uartReceiveStr[uartStrCount++] = nextChar;
    } else {
    	// end of string
		uartReceiveStr[uartStrCount] = '\0';
	  	uartStrCount = 0;
		gotUARTReq = TRUE;
    }
}

int main() {
	// initialization
	maxStepRate = DegToSteps(360);
	minStepRate = DegToSteps(10);
	GPIO_Init();
	Timer0_Init();
	USART0_Init();
	sei(); // enabling interrupts
	// starting infinity loop
	while(TRUE) {
		// waiting for instruction or query commands from the host
		if (gotUARTReq) {
			// copy usart string buffer because parsing takes a while
			static char bufCpy[UART_MAXSTRLEN];
			char* bufCpyP = strcpy(bufCpy, (char*)uartReceiveStr);
			// parse commands
			ParseCommand(bufCpyP);
			gotUARTReq = FALSE;
		}
		// control loop step
		if (gotControlReq) {
			static uint16_t tic = 0;
			if (tic++ >= C_PRESC) {
				// control steps
				tic = 0;
				Control(&P1);
				Control(&P2);
			}
			if (P1.tic++ >= P1.stepPeriod) {
				// intermediate follow step
				P1.tic = 0;
				P1.curStepPos += P1.tarDir;
				P2.tarStepPos = P1.curStepPos;
			}
			if (P2.tic++ >= P2.stepPeriod) {
				// perform step physically here
				P2.tic = 0;
				PhyStep(&P2);
			}
			gotControlReq = FALSE;
		}
		// show whether motor is on target position
		if (P2.curStepPos != P1.tarStepPos) {
			PORTA |= (1 << PIN_LED);
		} else {
			PORTA &= ~(1 << PIN_LED);
			if (homing == HOME_FOUND) {
				// homing settled
				homing = HOME_ON;
				// reset positions
				P1.curStepPos = 0;
				P1.tarStepPos = 0;
				P2.curStepPos = 0;
				P2.tarStepPos = 0;
			}
		}
		// check for home
		if ((homing == HOME_SEARCH) && (!(PINA & (1 << PIN_HOME)))) {
			// switch activated, remember home position
			P1.tarStepPos = P2.curStepPos;
			homing = HOME_FOUND;
		}
	}
	return 0;
}

void Control(pCtrl_t* ctrlP) {
	// control loop step
	int32_t v = kP*(ctrlP->tarStepPos - ctrlP->curStepPos);
	// sign
	uint32_t stepRate;
	if (v > 0) {
		ctrlP->tarDir = 1;
		stepRate = v;
	} else if (v < 0) {
		ctrlP->tarDir = -1;
		stepRate = -v;
	} else {
		ctrlP->tarDir = 0;
		stepRate = 0;
	}
	// limit rate
	if (stepRate > maxStepRate) stepRate = maxStepRate;
	if (stepRate < minStepRate) stepRate = minStepRate;
	// update step period
	ctrlP->stepPeriod = F_TINT/stepRate;
}

inline void PhyStep(pCtrl_t* ctrlP) {
	// perform a step
	if (ctrlP->tarDir != 0) {
		// set direction
		if (ctrlP->tarDir > 0) {
			PORTB &= ~(1 << PIN_DIR); // physical +
		} else {
			PORTB |= (1 << PIN_DIR); // physical -
		}
		// step
		PORTB |= (1 << PIN_STEP);
		ctrlP->curStepPos += ctrlP->tarDir;
		PORTB &= ~(1 << PIN_STEP);
	}
}

int32_t DegToSteps(int32_t deg) {
	// same as steps = deg*substeps/stepangle
	return deg*substeps*FSTEP_REV/360;
}

int32_t StepsToDeg(int32_t steps) {
	// same as deg = steps*stepangle/substeps
	return steps*360/(substeps*FSTEP_REV);
}

int16_t ParseNum(const char* strP) {
	// parses the string for a decimal number
	char *nxt;
	return strtol(strP, &nxt, RADIX);
}

void SendNum(int16_t num) {
	// converts a decimal number to string and sends it over USART
	itoa(num, uartSendStr, RADIX);
	USART0_SendString(uartSendStr);
}

int32_t DegStrToSteps(const char* strP) {
	// parses the string for float degree and converts to steps
	char *nxt;
	int32_t num = strtol(strP, &nxt, RADIX);
	if (*nxt == '.') {
		// float detected
		// get 2 fractional digits
		uint8_t i;
		for (i=0; i<2; i++) {
			nxt++;
			char c = *nxt;
			if ((c >= 48) && (c <= 57)) {
				num = num*10+c-48;
			} else {
				num = num*10;
			}
		}
		// convert to steps
		return DegToSteps(num)/100;
	} else {
		return DegToSteps(num);
	}
}

void SendStepsAsDeg(int32_t steps) {
	// converts steps to degree in float and sends it over USART
	int32_t num = StepsToDeg(steps*100); // for 2 fractional digits
	ldiv_t deg = ldiv(num, 100);
	// quotient
	itoa(deg.quot, uartSendStr, RADIX);
	strcat(uartSendStr, "."); // float point
	// fractional digits (remainder)
	char fracDigs[3];
	itoa(abs(deg.rem), fracDigs, RADIX);
	strcat(uartSendStr, fracDigs);
	// send
	USART0_SendString(uartSendStr);
}

void ParseCommand(const char* strP) {
	// parse command (SCPI style, but not protocol conform)
	// search entry point: our axis with ID ("AX<ID>")
	char* cmdP = strstr(strP, CMD_ID);
	if (cmdP) {
		cmdP += 4; // skip "AX<ID>:"
		if (strncmp(cmdP, "POW", 3) == 0) {
			// enable/disable power
			cmdP += 3;
			if (*cmdP == '?') {
				// host wants to know if the motor power is enabled (low active)
				if (PORTA & (1 << PIN_ENABLE)) {
					USART0_SendString("OFF");
				} else {
					USART0_SendString("ON");
				}
			} else {
				// host wants to set the motor power
				/*
				note: loosing power results in unknown state of the position, so reset it
				Same goes when powering on because there is no absolute encoder
				*/
				P1.curStepPos = 0;
				P1.tarStepPos = 0;
				P2.curStepPos = 0;
				P2.tarStepPos = 0;
				homing = HOME_OFF;
				cmdP += 2; // skip " O"
				if (*cmdP == 'N') {
					PORTA &= ~(1 << PIN_ENABLE); // enable (low active)
				} else {
					PORTA |= (1 << PIN_ENABLE); // disable
				}
			}
		} else if (strncmp(cmdP, "KP", 2) == 0) {
			// proportional control loop parameter
			cmdP += 2;
			if (*cmdP == '?') {
				SendNum(kP);
			} else {
				kP = ParseNum(cmdP);
			}
		} else if (strncmp(cmdP, "HOME", 4) == 0) {
			// start homing
			cmdP += 4;
			if (*cmdP == '?') {
				// requested for home status (1 = on home and motor stopped, 0 not)
				SendNum((homing == HOME_ON) ? 1 : 0);
			} else {
				// set homing heading
				int8_t homeDir = ParseNum(cmdP);
				P1.tarStepPos = (homeDir > 0) ? DegToSteps(36000) : DegToSteps(-36000);
				homing = HOME_SEARCH;
			}
		} else if (strncmp(cmdP, "POS", 3) == 0) {
			// target position
			cmdP += 3;
			if (*cmdP == '?') {
				SendStepsAsDeg(P2.curStepPos);
			} else {
				P1.tarStepPos = DegStrToSteps(cmdP);
				homing = HOME_OFF;
			}
		} else if (strncmp(cmdP, "SUB", 3) == 0) {
			// substeps
			cmdP += 3;
			if (*cmdP == '?') {
				SendNum(substeps);
			} else {
				substeps = ParseNum(cmdP);
				SetSubsteps(substeps);
			}
		} else if (strncmp(cmdP, "LIM", 3) == 0) {
			// rate limit
			cmdP += 4; // skip "LIM:"
			if (strncmp(cmdP, "MAX", 3) == 0) {
				// max rate
				cmdP += 3;
				if (*cmdP == '?') {
					SendStepsAsDeg(maxStepRate);
				} else {
					maxStepRate = DegStrToSteps(cmdP);
				}
			}
			if (strncmp(cmdP, "MIN", 3) == 0) {
				// min rate
				cmdP += 3;
				if (*cmdP == '?') {
					SendStepsAsDeg(minStepRate);
				} else {
					minStepRate = DegStrToSteps(cmdP);
				}
			}
		} else {
			// no valid command
			strcpy(uartSendStr, "No valid command: ");
			strcat(uartSendStr, cmdP);
			USART0_SendString(uartSendStr);
		}
	} else {
		// no axis specific command
		cmdP = strstr(strP, "*IDN?");
		if (cmdP) {
			// requested for device name
			USART0_SendString(CMD_ID);
		}
	}
}

void SetSubsteps(uint8_t ss) {
	// clear substep pins
	PORTA &= ~((1 << PIN_SS1) | (1 << PIN_SS3));
	PORTB &= ~(1 << PIN_SS2);
	// set substep pins
	if (ss == 2) {
		PORTA |= (1 << PIN_SS1);
	} else if (ss == 4) {
		PORTB |= (1 << PIN_SS2);
	} else if (ss == 8) {
		PORTA |= (1 << PIN_SS1);
		PORTB |= (1 << PIN_SS2);
	} else if (ss == 16) {
		PORTA |= (1 << PIN_SS1) | (1 << PIN_SS3);
		PORTB |= (1 << PIN_SS2);
	}
}

void GPIO_Init() {
	// output pins
	DDRA = (1 << PIN_ENABLE) | (1 << PIN_SS1) | (1 << PIN_LED);
	DDRB = (1 << PIN_SS2) | (1 << PIN_SS3) | (1 << PIN_STEP) | (1 << PIN_DIR);
	// disable motor (low active)
	PORTA |= (1 << PIN_ENABLE);
	// pull-up on input pins
	PUEA |= (1 << PIN_HOME);
	// set substeps
	SetSubsteps(substeps);
}

void Timer0_Init() {
	// configure 8 bit timer 0
	TCCR0A = (1 << WGM01); // CTC Modus
	TCCR0B |= (1 << CS01); // prescale 8
	// OCR0A = (8000000HzCPU/8prescale/8000Interrupts/s)-1cycle = 125-1
	OCR0A = (F_CPU/8/F_TINT)-1;
	// compare-interrupt
	TIMSK0 |= (1 << OCIE0A);
}

void USART0_Init() {
	// set baud rate
	UBRR0H = (uint8_t)(UART_UBRR >> 8);
	UBRR0L = (uint8_t)UART_UBRR;
	// enable receiver and transmitter pins
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	// enable receiver interrupt
	UCSR0B |= (1 << RXCIE0);
	// frame format: 8N1 (is initially set anyway)
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	// double speed because the error is too high (-7%) for baud 9600 without U2X
	UCSR0A = (1 << U2X0);
}

void USART0_SendChar(const char c) {
	// wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));
	// put char into buffer
	UDR0 = c;
}

void USART0_SendString(const char* strP) {
	// for all characters in string until '\0'
	while(*strP) {
		USART0_SendChar(*strP);
		strP++;
	}
	// append cr and lf
	USART0_SendChar('\r');
	USART0_SendChar('\n');
}