#include <avr/io.h> // for pin definitions
#include <stdint.h> // for data types
#include <avr/interrupt.h> // for sei(), cli() and ISR()
#include <stdlib.h>	// for itoa, atoi
#include <string.h>	// for string operations like cmp and cat
#include "main.h" // for project specific definitions

ISR(TIMER0_COMPA_vect) {
	// timer 0 compare interrupt
	gotControlReq = TRUE;
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
		USART0_ReceiveChar();
		if (gotCommand) {
			// parse commands
			ParseCommand((char*)(uartReceiveStr));
			gotCommand = FALSE;
		}
		// control loop
		if (gotControlReq) {
			static uint16_t tic = 0;
			if (tic++ >= C_PRESC) {
				// perform control step
				tic = 0;
				Control(&ctrlState);
			}
			if (ctrlState.tic++ >= ctrlState.stepPeriod) {
				// perform physical step
				ctrlState.tic = 0;
				PhyStep(&ctrlState);
			}
			gotControlReq = FALSE;
		}
		// show whether motor is on target position
		if (ctrlState.curStepPos != ctrlState.tarStepPos) {
			PORTA |= (1 << PIN_LED);
		} else {
			PORTA &= ~(1 << PIN_LED);
			if (homing == HOME_FOUND) {
				// homing settled
				homing = HOME_ON;
				// reset positions
				ctrlState.curStepPos = 0;
				ctrlState.tarStepPos = 0;
			}
			// get rested position
			ctrlState.oldStepPos = ctrlState.curStepPos;
		}
		// check for home
		if ((homing == HOME_SEARCH) && (!(PINA & (1 << PIN_HOME)))) {
			// switch activated, remember home position
			ctrlState.tarStepPos = ctrlState.curStepPos;
			homing = HOME_FOUND;
		}
	}
	return 0;
}

void Control(pCtrl_t* ctrlP) {
	// control loop step
	int32_t tarVel = kDec*(ctrlP->tarStepPos - ctrlP->curStepPos);
	int32_t oldVel = kAcc*(ctrlP->curStepPos - ctrlP->oldStepPos);
	// sign
	uint32_t tarRate;
	uint32_t oldRate;
	if (tarVel > 0) {
		ctrlP->tarDir = 1;
		tarRate = tarVel;
		oldRate = oldVel;
	} else if (tarVel < 0) {
		ctrlP->tarDir = -1;
		tarRate = -tarVel;
		oldRate = -oldVel;
	} else {
		ctrlP->tarDir = 0;
		tarRate = 0;
		oldRate = 0;
	}
	// limit rate
	tarRate = (tarRate < oldRate) ? tarRate : oldRate; // choose acceleration or decceleration ramp
	if (tarRate > maxStepRate) tarRate = maxStepRate;
	if (tarRate < minStepRate) tarRate = minStepRate;
	// update step period
	ctrlP->stepPeriod = F_TINT/tarRate;
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

inline void USART0_ReceiveChar() {
	// collect newly arrived bytes
	if (!(UCSR0A & (1 << RXC0))) return; // no new char received yet
	// new char available
	static uint8_t uartStrCount = 0;
	char nextChar = UDR0;
	if (nextChar != STR_TERM && uartStrCount < UART_MAXSTRLEN) {
		// still getting valid chars
		uartReceiveStr[uartStrCount++] = nextChar;
	} else {
		// end of string
		uartReceiveStr[uartStrCount] = STR_TERM;
	  	uartStrCount = 0;
		gotCommand = TRUE;
	}
}

inline void ParseCommand(const char* strP) {
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
				ctrlState.curStepPos = 0;
				ctrlState.tarStepPos = 0;
				ctrlState.oldStepPos = 0;
				homing = HOME_OFF;
				cmdP += 2; // skip " O"
				if (*cmdP == 'N') {
					PORTA &= ~(1 << PIN_ENABLE); // enable (low active)
				} else {
					PORTA |= (1 << PIN_ENABLE); // disable
				}
			}
		} else if (strncmp(cmdP, "ACC", 3) == 0) {
			// acceleration proportion
			cmdP += 3;
			if (*cmdP == '?') {
				SendNum(kAcc);
			} else {
				kAcc = (uint16_t)(strtoul(cmdP, NULL, RADIX));
			}
		} else if (strncmp(cmdP, "DEC", 3) == 0) {
			// decceleration proportion
			cmdP += 3;
			if (*cmdP == '?') {
				SendNum(kDec);
			} else {
				kDec = (uint16_t)(strtoul(cmdP, NULL, RADIX));
			}
		} else if (strncmp(cmdP, "HOME", 4) == 0) {
			// start homing
			cmdP += 4;
			if (*cmdP == '?') {
				// requested for home status (1 = on home and motor stopped, 0 not)
				SendNum((homing == HOME_ON) ? 1 : 0);
			} else {
				// set homing heading
				int8_t homeDir = (int8_t)(strtol(cmdP, NULL, RADIX));
				ctrlState.tarStepPos = (homeDir > 0) ? DegToSteps(36000) : DegToSteps(-36000);
				homing = HOME_SEARCH;
			}
		} else if (strncmp(cmdP, "POS", 3) == 0) {
			// target position
			cmdP += 3;
			if (*cmdP == '?') {
				SendStepsAsDeg(ctrlState.curStepPos);
			} else {
				ctrlState.tarStepPos = DegStrToSteps(cmdP);
				homing = HOME_OFF;
			}
		} else if (strncmp(cmdP, "SUB", 3) == 0) {
			// substeps
			cmdP += 3;
			if (*cmdP == '?') {
				SendNum(substeps);
			} else {
				substeps = (uint8_t)(strtoul(cmdP, NULL, RADIX));
				SetSubsteps(substeps);
			}
		} else if (strncmp(cmdP, "RATE", 4) == 0) {
			// rate (shorthand for LIM:MAX)
			cmdP += 4;
			if (*cmdP == '?') {
				SendStepsAsDeg(maxStepRate);
			} else {
				maxStepRate = DegStrToSteps(cmdP);
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
	DDRA = (1 << PIN_ENABLE) | (1 << PIN_SS1) | (1 << PIN_SS3) | (1 << PIN_LED);
	DDRB = (1 << PIN_SS2) | (1 << PIN_STEP) | (1 << PIN_DIR);
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
	// append string termination
	USART0_SendChar(STR_TERM);
}