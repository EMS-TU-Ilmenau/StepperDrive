#include <avr/io.h> // for microcontroller specific peripheral definitions
#include <stdint.h> // for uint8_t, uint16_t, etc.
#include <string.h>	// for string operations like cmp and cat
#include <stdlib.h>	// for itoa, atoi
#include <avr/interrupt.h>	// for sei(), cli() and ISR():
#include <util/delay.h> // for waiting
#include "main.h" // for application specific definitions

ISR(USART0_RXC_vect) {
	char nextChar = USART0.RXDATAL; // new char available

	// track received string
	static uint8_t charCnt = 0;
	if (nextChar != STR_TERM && charCnt < UART_MAXSTRLEN) {
		// still getting valid chars
		uartRecvStr[charCnt++] = nextChar;
	} else {
		// end of string
		uartRecvStr[charCnt] = '\0';
		charCnt = 0;
		gotCommand = TRUE;
	}
}

int main() {
	// setup
	configClock();

	// init
	maxStepRate = degToSteps(360);
	minStepRate = degToSteps(10);
	
	// read axis ID
	uint8_t oldID = 0;
	uint8_t storedID;
	for (uint8_t nTries = 3; nTries > 0; nTries--) {
		_delay_ms(50); // let voltage stabilize, otherwise eeprom read is corrupted
		storedID = eeprom_read_byte(&eAxisIDAddr);
		if ((oldID != 0) && (storedID == oldID)) break;
		oldID = storedID;
	}
	if ((storedID < 48) || (storedID > 57)) {
		// initially store ID from default command string
		storedID = cmdID[2];
		eeprom_write_byte(&eAxisIDAddr, storedID);
	}
	cmdID[2] = storedID;

	// continue setup
	configGPIO();
	configUART();
	configTCA();
	sei();
	
	// main program loop
	while (TRUE) {
		// parse incoming
		if (gotCommand) {
			parseCommand((char*)uartRecvStr);
			gotCommand = FALSE;
		}

		// timer interrupt flag
		if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) {
			TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm; // clear flag
			if (moving == pos) {
				// position control loop
				static uint16_t tic = 0;
				if (tic++ >= C_PRESC) {
					// perform control step
					tic = 0;
					controlStep();
				}
			}
			if (ctrlState.tic++ >= ctrlState.stepPeriod) {
				// perform physical step
				ctrlState.tic = 0;
				phyStep();
			}
		}

		// check for home
		if ((homing == search) && (!(PORTB.IN & PIN_HOME))) {
			// switch activated, stop motor
			ctrlState.curStepPos = 0;
			ctrlState.tarStepPos = 0;
			ctrlState.oldStepPos = 0;
			moving = pos;
			homing = found;
		}
	}
	return 0;
}

void parseCommand(const char* strP) {
	// parse command (SCPI style, but not protocol conform)
	// search entry point: our axis with ID ("AX<ID>")
	char* cmdP = strstr(strP, cmdID);
	if (cmdP) {
		cmdP += 4; // skip "AX<ID>:"
		if (strncmp(cmdP, "ID", 2) == 0) {
			// change axis ID
			cmdP += 3; // skip "ID "
			uint8_t id = *cmdP;
			if ((id >= 48) && (id <= 57)) {
				cmdID[2] = id;
				for (uint8_t nTries = 3; nTries > 0; nTries--) {
					eeprom_update_byte(&eAxisIDAddr, id);
					uint8_t storedID = eeprom_read_byte(&eAxisIDAddr);
					if (storedID == id) break;
				}
			}
		} else if (strncmp(cmdP, "POW", 3) == 0) {
			// enable/disable power
			cmdP += 3;
			if (*cmdP == '?') {
				// host wants to know if the motor power is enabled (low active)
				sendUARTString((PORTA.OUT & PIN_ENABLE) ? "OFF" : "ON");
			} else {
				// host wants to set the motor power
				/*
				note: loosing power results in unknown state of the position, so reset it
				Same goes when powering on because there is no absolute encoder
				*/
				ctrlState.curStepPos = 0;
				ctrlState.tarStepPos = 0;
				ctrlState.oldStepPos = 0;
				cmdP += 2; // skip " O"
				if (*cmdP == 'N') {
					PORTA.OUTCLR = PIN_ENABLE;
				} else {
					PORTA.OUTSET = PIN_ENABLE;
				}
			}
		} else if (strncmp(cmdP, "ACC", 3) == 0) {
			// acceleration proportion
			cmdP += 3;
			if (*cmdP == '?') {
				sendNum(kAcc);
			} else {
				kAcc = (uint16_t)(strtoul(cmdP, NULL, RADIX));
			}
		} else if (strncmp(cmdP, "DEC", 3) == 0) {
			// decceleration proportion
			cmdP += 3;
			if (*cmdP == '?') {
				sendNum(kDec);
			} else {
				kDec = (uint16_t)(strtoul(cmdP, NULL, RADIX));
			}
		} else if (strncmp(cmdP, "HOME", 4) == 0) {
			// start homing
			cmdP += 4;
			if (*cmdP == '?') {
				// requested for home status (1 = on home and motor stopped, 0 not)
				sendNum((homing == found) ? 1 : 0);
			} else {
				// set homing velocity (with sign)
				stepVel = degStrToSteps(cmdP);
				move(stepVel);
				homing = search;
			}
		} else if (strncmp(cmdP, "VEL", 3) == 0) {
			// velocity (moving without target position)
			cmdP += 3;
			if (*cmdP == '?') {
				sendStepsAsDeg(stepVel);
			} else {
				stepVel = degStrToSteps(cmdP);
				move(stepVel);
				homing = off;
			}
		} else if (strncmp(cmdP, "POS", 3) == 0) {
			// target position
			cmdP += 3;
			if (*cmdP == '?') {
				sendStepsAsDeg(ctrlState.curStepPos);
			} else {
				ctrlState.tarStepPos = degStrToSteps(cmdP);
				homing = off;
				moving = pos;
			}
		} else if (strncmp(cmdP, "SUB", 3) == 0) {
			// substeps
			cmdP += 3;
			if (*cmdP == '?') {
				sendNum(substeps);
			} else {
				substeps = (uint8_t)(strtoul(cmdP, NULL, RADIX));
				setSubsteps(substeps);
			}
		} else if (strncmp(cmdP, "RATE", 4) == 0) {
			// rate (shorthand for LIM:MAX)
			cmdP += 4;
			if (*cmdP == '?') {
				sendStepsAsDeg(maxStepRate);
			} else {
				maxStepRate = degStrToSteps(cmdP);
			}
		} else if (strncmp(cmdP, "LIM", 3) == 0) {
			// rate limit
			cmdP += 4; // skip "LIM:"
			if (strncmp(cmdP, "MAX", 3) == 0) {
				// max rate
				cmdP += 3;
				if (*cmdP == '?') {
					sendStepsAsDeg(maxStepRate);
				} else {
					maxStepRate = degStrToSteps(cmdP);
				}
			}
			if (strncmp(cmdP, "MIN", 3) == 0) {
				// min rate
				cmdP += 3;
				if (*cmdP == '?') {
					sendStepsAsDeg(minStepRate);
				} else {
					minStepRate = degStrToSteps(cmdP);
				}
			}
		} else {
			// no valid command
			strcpy(uartSendStr, "No valid command: ");
			strcat(uartSendStr, cmdP);
			sendUARTString(uartSendStr);
		}
		// query operation complete
		cmdP = strstr(cmdP, "*OPC?");
		if (cmdP) {
			sendUARTString("1");
		}
	} else {
		// no axis specific command
		cmdP = strstr(strP, "*IDN?");
		if (cmdP) {
			// requested for device name
			sendUARTString(cmdID);
		}
	}
}

void configClock() {
	// configure CPU frequency to 10 MHz
	CCP = CCP_IOREG_gc; // disable protection to configure clock frequency
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc; // use 20 MHz internal clock as source
	CCP = CCP_IOREG_gc; // disable protection to configure clock frequency
	CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm; // prescale divider
}

void configGPIO() {
	// inputs
	PORTB.DIRCLR = PIN_STALL | PIN_HOME;
	PORTB.PIN1CTRL |= PORT_PULLUPEN_bm; // enable pullup for home

	// outputs
	PORTA.DIRSET = PIN_ENABLE | PIN_STEP | PIN_DIR | PIN_MS1 | PIN_MS2;
	PORTA.OUTSET = PIN_ENABLE; // disable motor power
	PORTA.OUTCLR = PIN_MS1 | PIN_MS2; // configure substeps to 1/8
}

void configUART() {
	USART0.BAUD = (uint16_t)UART_BAUD_RATE(9600); // set baudrate
	USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm; // enable RX | TX
	USART0.CTRLA |= USART_RXCIF_bm; // enable interrupt on receive complete
	// use alternative UART pins
	PORTMUX_CTRLB |= PORTMUX_USART0_bm; // enable alternative UART pins
	PORTA.DIRCLR = PIN2_bm; // PA2 as input (RX)
	PORTA.DIRSET = PIN1_bm; // PA1 as output (TX)
}

void sendUARTChar(const char c) {
	while (!(USART0.STATUS & USART_DREIF_bm)); // wait until the transmit DATA register is empty
	USART0.TXDATAL = c; // put char into buffer
}

void sendUARTString(const char* strP) {
	// for all characters in string until '\0'
	while(*strP && (*strP != STR_TERM)) {
		sendUARTChar(*strP++);
	}
	// append string termination
	sendUARTChar(STR_TERM);
}

void configTCA() {
	// configures 16 bit timer counter A
	// set normal mode (default)
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm; // count with prescale 2 and enable timer
	TCA0.SINGLE.PER = (F_CPU/2/F_TINT)-1; // set TOP (10000000 Hz CPU / 2 timer prescale / 8000 Interrupts/s) - 1 cycle = 624
}

void setSubsteps(uint8_t ss) {
	PORTA.OUTCLR = PIN_MS1 | PIN_MS2; // ss == 8
	// set substep pins
	if (ss == 16) {
		PORTA.OUTSET = PIN_MS1 | PIN_MS2;
	} else if (ss == 32) {
		PORTA.OUTSET = PIN_MS1;
	} else if (ss == 64) {
		PORTA.OUTSET = PIN_MS2;
	}
}

int32_t degToSteps(int32_t deg) {
	// same as steps = deg*substeps/stepangle
	return deg*substeps*FSTEP_REV/360;
}

int32_t stepsToDeg(int32_t steps) {
	// same as deg = steps*stepangle/substeps
	return steps*360/(substeps*FSTEP_REV);
}

int32_t degStrToSteps(const char* strP) {
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
		return degToSteps(num)/100;
	} else {
		return degToSteps(num);
	}
}

void sendStepsAsDeg(int32_t steps) {
	// converts steps to degree in float and sends it over USART
	int32_t num = stepsToDeg(steps*100); // for 2 fractional digits
	ldiv_t deg = ldiv(num, 100);
	// quotient
	itoa(deg.quot, uartSendStr, RADIX);
	strcat(uartSendStr, "."); // float point
	// fractional digits (remainder)
	char fracDigs[3];
	itoa(abs(deg.rem), fracDigs, RADIX);
	strcat(uartSendStr, fracDigs);
	// send
	sendUARTString(uartSendStr);
}

void sendNum(int16_t num) {
	// converts a decimal number to string and sends it over USART
	itoa(num, uartSendStr, RADIX);
	sendUARTString(uartSendStr);
}

void controlStep() {
	// control loop step
	int32_t tarVel = kDec*(ctrlState.tarStepPos - ctrlState.curStepPos);
	int32_t oldVel = kAcc*(ctrlState.curStepPos - ctrlState.oldStepPos);
	// sign
	uint32_t tarRate;
	uint32_t oldRate;
	if (tarVel > 0) {
		ctrlState.tarDir = 1;
		tarRate = tarVel;
		oldRate = oldVel;
	} else if (tarVel < 0) {
		ctrlState.tarDir = -1;
		tarRate = -tarVel;
		oldRate = -oldVel;
	} else {
		ctrlState.tarDir = 0;
		tarRate = 0;
		oldRate = 0;
	}
	// limit rate
	tarRate = (tarRate < oldRate) ? tarRate : oldRate; // choose acceleration or decceleration ramp
	if (tarRate > maxStepRate) tarRate = maxStepRate;
	if (tarRate < minStepRate) tarRate = minStepRate;
	// update step period
	ctrlState.stepPeriod = F_TINT/tarRate;
}

inline void phyStep() {
	// perform a step
	if (ctrlState.tarDir != 0) {
		// set direction
		if (ctrlState.tarDir > 0) {
			PORTA.OUTSET = PIN_DIR; // physical +
		} else {
			PORTA.OUTCLR = PIN_DIR; // physical -
		}
		// step
		PORTA.OUTSET = PIN_STEP;
		ctrlState.curStepPos += ctrlState.tarDir;
		PORTA.OUTCLR = PIN_STEP;
	}
}

void move(int32_t vel) {
	ctrlState.stepPeriod = F_TINT/abs(vel);
	if (vel == 0) {
		ctrlState.tarDir = 0;
		ctrlState.tarStepPos = ctrlState.curStepPos;
		ctrlState.oldStepPos = ctrlState.curStepPos;
	} else if (vel > 0) {
		ctrlState.tarDir = 1;
	} else if (vel < 0) {
		ctrlState.tarDir = -1;
	}
	moving = vel;
}