/**
 * @file HIB.cpp
 * @brief Brief of HIB.cpp
 * @author Manuel Barranco, Alberto Ballesteros
 * @date December 2016 - March 2017
 *
 * @details Detailed description of this file.
 */

#include "HIB.h"

#include <LiquidCrystal.h>
//#include <mcp_can.h>
#include <SPI.h>


// Pins
/** @name Keypad pins */
//@{
#define HIB_KEY_C1_PIN A4
#define HIB_KEY_C2_PIN A3
#define HIB_KEY_C3_PIN A2

#define HIB_KEY_R1_PIN 10
#define HIB_KEY_R2_PIN 11
#define HIB_KEY_R3_PIN 12
#define HIB_KEY_R4_PIN 13

#define HIB_KEY_INT_PIN 18 // INT3
//@}

/** @name LCD pins */
//@{
#define HIB_LCD_RS_PIN A5
#define HIB_LCD_EN_PIN A0
#define HIB_LCD_DB4_PIN 10
#define HIB_LCD_DB5_PIN 11
#define HIB_LCD_DB6_PIN 12
#define HIB_LCD_DB7_PIN 13
// R/W needs to be connected to GND
//@}

/** @name LED pins and pin values */
//@{
#define HIB_LED1_PIN 3
#define HIB_LED2_PIN 4
#define HIB_LED3_PIN 5
#define HIB_LED4_PIN 6
#define HIB_LED5_PIN 7
#define HIB_LED6_PIN 8

#define HIB_LED_ON 1
#define HIB_LED_OFF 0

//@}

/** @name ADC pin and adc modes */
//@{
#define HIB_ADC_PIN A1
#define ADC_POLLING_DRIVEN 0
#define ADC_AUTO_DRIVEN 1
#define ADC_TIMER_DRIVEN 2
//@}

/** @name Buzzer pins */
//@{
#define HIB_BUZ_PIN A1
//@}

/** @name CAN pins */
//@{
#define HIB_CAN_CS_PIN 9
//@}

/** @name 7-segment displays pins and defines /*
//@{
	// Macros definitions for 7-segment displays

 * U:   Upper lines
 * L/R: Left/Right
 * L:   Lower lines
 * C:   Center line
 *

 * Visual representation of the lines' indexes
      U
    _____
UL |     | UR
   |  C  |
    ―――――
LL |     | LR
   |     |
    ‾‾‾‾‾. DOT
      L
*/

#define U_1 33
#define UR_1 31
#define LR_1 27
#define L_1 25
#define LL_1 23
#define UL_1 35
#define C_1 37
#define DOT_1 29

#define U_2 24
#define UR_2 22
#define LR_2 43
#define L_2 41
#define LL_2 39
#define UL_2 26
#define C_2 28
#define DOT_2 45

// Indexes for pins array
#define U 0
#define UR 1
#define LR 2
#define L 3
#define LL 4
#define UL 5
#define C 6
#define DOT 7
//@}

/** @name Temperature sensor & LDR pins */
//@{
#define LEFT_TEM_PIN A8
#define RIGHT_TEM_PIN A9

#define LEFT_LDR_PIN A6
#define RIGHT_LDR_PIN A7
//@}

// Define cbi() and sbi() for clearing and setting bits in the
// ADC registers.
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

////////////////////////////////////////////////////////////////////////////////
// DEFINITION AND INITIALIZATION OF STATIC VARIABLES OF HIB CLASS
////////////////////////////////////////////////////////////////////////////////

// Define and initialize pointer to the active HIB object
HIB *HIB::pHIB = NULL;

////////////////////////////////////////////////////////////////////////////////
// LOCAL VARIABLES
////////////////////////////////////////////////////////////////////////////////

// LEDS
int led_pins[] = {
	HIB_LED1_PIN, HIB_LED2_PIN,
	HIB_LED3_PIN, HIB_LED4_PIN,
	HIB_LED5_PIN, HIB_LED6_PIN};

static byte led_values[] = {
	HIB_LED_OFF, HIB_LED_OFF,
	HIB_LED_OFF, HIB_LED_OFF,
	HIB_LED_OFF, HIB_LED_OFF};


// KEYPAD

// Pointer to the hook invoked by keyISR 
volatile bool keyInterruptEnabled = false;
volatile bool keyFallingEdgeByLCD  = false; // indicates that the key falling edge is due to row reconfiguration done by LCD
void (*pKeyHook)(uint8_t value) = NULL;
volatile long pressedKeyTimeStamp;
volatile long keystrokeTimeStamp;
volatile uint8_t minimumInterKeystrokeTime;


// LCD
static LiquidCrystal lcd = LiquidCrystal(
	HIB_LCD_RS_PIN,
	HIB_LCD_EN_PIN,
	HIB_LCD_DB4_PIN,
	HIB_LCD_DB5_PIN,
	HIB_LCD_DB6_PIN,
	HIB_LCD_DB7_PIN);


// CAN
/*static MCP_CAN can = MCP_CAN(
	HIB_CAN_CS_PIN
);*/


// ADC
// Pointer to the hook invoked by adcISR --> registerAdcValue
// after it register the adc acquired value
void (*pAdcHook)(uint16_t value) = NULL;
int analogRef = DEFAULT;
volatile uint16_t adcAcquiredValue = ADC_VALUE_NOT_READY;
byte ADC_MODE = ADC_POLLING_DRIVEN;
bool adcLastPolledReadValue = true;

volatile bool timerDrivenConvInPro = false; // timer-driven adc conversion in progress
volatile bool readOnOtherAdcSensor = false; // a read operation has been done on a sensor other than the potentiometer


/** 7-segment displays
 * @brief Provides quick access to pins for turning all on/off
 * First 8 are for display 1, the other 8 are for display 2
 * Display 1 is the one on the right on the board!
 */
uint8_t segs_pins[16] = {U_1, UR_1, LR_1, L_1, LL_1, UL_1, C_1, DOT_1, U_2, UR_2, LR_2, L_2, LL_2, UL_2, C_2, DOT_2};


////////////////////////////////////////////////////////////////////////////////
// AUXILIARY FUNCTIONS DECLARATION
////////////////////////////////////////////////////////////////////////////////

void _configPinLed(void);
void _configPinAdc(void);
void _configPinBuz(void);
void _configPinLCD(void);
void _configPinKey(void);
void _configPinKeyOutputForKeyInt(void);
void _configPin7Segs(void);
void _configPinsTem(void);
void _configPinsLDR(void);
//void _configPinCAN (void);

void _wait_us(uint32_t time_us);

////////////////////////////////////////////////////////////////////////////////
// CLASS FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

HIB::HIB()
{
	pHIB = this;
}

void HIB::begin()
{
	// Init LEDs
	_configPinLed();

	// Init LCD
	_configPinLCD();
	lcd.begin(LCD_COLS, LCD_LINS);

	// ATTENTION!!!!!!
	// It is mandatory to configure the pins for the keypad after doing so for the LCD
	// Since LCD and keypad share some lines, it is necessary to:
	// (1) configure keypad columns and rows appropriately AND
	// (2) leave all keypad columns to LOW to be able to
	// Otherwise it would not be possible to detect falling edge for keypad interrupt
	_configPinKey();

	// Init CAN
	//while (can.begin(CAN_500KBPS) != CAN_OK){ delay(100); }

	// Init 7 segments
	_configPin7Segs();

	// Init temperature sensor
	_configPinsTem();

	// Init LDR
	_configPinsLDR();
}

/*******************************************************************************
 * LEDs
 *******************************************************************************
 * The pins related to the LEDs are not shared. Consequently, they are only
 * configured once, in the constructor.
 ******************************************************************************/

void HIB::ledOn(uint8_t num_led)
{
	if (num_led < 0 || num_led > NUM_LEDS - 1)
		return;
	digitalWrite(led_pins[num_led], HIGH);
	led_values[num_led] = HIB_LED_ON;
}

void HIB::ledOff(uint8_t num_led)
{
	if (num_led < 0 || num_led > NUM_LEDS - 1)
		return;
	digitalWrite(led_pins[num_led], LOW);
	led_values[num_led] = HIB_LED_OFF;
}

void HIB::ledToggle(uint8_t num_led)
{
	if (num_led < 0 || num_led > NUM_LEDS - 1)
		return;

	switch (led_values[num_led])
	{
	case HIB_LED_ON:
		digitalWrite(led_pins[num_led], LOW);
		led_values[num_led] = HIB_LED_OFF;
		break;

	case HIB_LED_OFF:
		digitalWrite(led_pins[num_led], HIGH);
		led_values[num_led] = HIB_LED_ON;
		break;

	default:
		break;
	}
}



void HIB::ledPrintNum(unsigned char number)
{
	unsigned char maskLed = 0x01;
	unsigned char led;

	maskLed=0x01;
	for (led=0; led <= 5; led++)
	{
		if (number & maskLed)
		{
			pHIB->ledOn(led);
		}
		else
		{
			pHIB->ledOff(led);
		}
		maskLed = maskLed << 1;
	}

}


/*******************************************************************************
 * Keypad
 *******************************************************************************
 *
 ******************************************************************************/

uint8_t HIB::keyRead(bool block)
{
	uint8_t wait_ms = 2;

	_configPinKey();

	do
	{
		// Scan first column
		digitalWrite(HIB_KEY_C1_PIN, LOW);
		digitalWrite(HIB_KEY_C2_PIN, HIGH);
		digitalWrite(HIB_KEY_C3_PIN, HIGH);

		delay(wait_ms);

		if (digitalRead(HIB_KEY_R1_PIN) == LOW)
			return 0;
		if (digitalRead(HIB_KEY_R2_PIN) == LOW)
			return 3;
		if (digitalRead(HIB_KEY_R3_PIN) == LOW)
			return 6;
		if (digitalRead(HIB_KEY_R4_PIN) == LOW)
			return 9;

		// Scan second column
		digitalWrite(HIB_KEY_C1_PIN, HIGH);
		digitalWrite(HIB_KEY_C2_PIN, LOW);
		digitalWrite(HIB_KEY_C3_PIN, HIGH);

		delay(wait_ms);

		if (digitalRead(HIB_KEY_R1_PIN) == LOW)
			return 1;
		if (digitalRead(HIB_KEY_R2_PIN) == LOW)
			return 4;
		if (digitalRead(HIB_KEY_R3_PIN) == LOW)
			return 7;
		if (digitalRead(HIB_KEY_R4_PIN) == LOW)
			return 10;

		// Scan third column
		digitalWrite(HIB_KEY_C1_PIN, HIGH);
		digitalWrite(HIB_KEY_C2_PIN, HIGH);
		digitalWrite(HIB_KEY_C3_PIN, LOW);

		delay(wait_ms);

		if (digitalRead(HIB_KEY_R1_PIN) == LOW)
			return 2;
		if (digitalRead(HIB_KEY_R2_PIN) == LOW)
			return 5;
		if (digitalRead(HIB_KEY_R3_PIN) == LOW)
			return 8;
		if (digitalRead(HIB_KEY_R4_PIN) == LOW)
			return 11;
	} while (block == true);

	return NO_KEY;
}




void HIB::keySetIntDriven(uint8_t minInterKeystrokeTime, void (*pHook)(uint8_t newKey) )
{
	// Register that keypad INT is enabled
	keyInterruptEnabled = true;

	// Itilialize keystroke timestamp and set minimum expected time between keystrokes
	keystrokeTimeStamp = millis();
	minimumInterKeystrokeTime = minInterKeystrokeTime;

	// Set keypad ISR Hook
	pKeyHook = pHook;

	// PINs SET UP	
	_configPinKey();

	// Set comlumns to LOW
	// to make AND gate able to catpure keystroke for keypad int
	_configPinKeyOutputForKeyInt();

	delay(200);

	attachInterrupt(digitalPinToInterrupt(HIB_KEY_INT_PIN), ISR_KEYPAD, FALLING); 
}




// KEYPAD ISR.
void HIB::ISR_KEYPAD()
{
	uint8_t pressedKey;
	bool processIsrKey;
	char auxSREG;

	// Save the AVR Status Register by software
	// since the micro does not do it automatically
	auxSREG = SREG;

    	////////////////////////////////////////
   	// Detection based on detection of minimum time between keystrokes
   	////////////////////////////////////////
	//Serial.println("KEYPAD ISR");


	processIsrKey = false;	// we will set to true if:
				// ISR has not been called due to interaction with LCD

	if(keyFallingEdgeByLCD)	// check if isr has been called due to interaction with LCD
	{
	//	Serial.println("KEYPAD by LCD");
		keyFallingEdgeByLCD = false;
	}
	else
	{
		// Now check if minimum time between keystrokes has elapsed

		pressedKeyTimeStamp = millis();
		if ( (pressedKeyTimeStamp - keystrokeTimeStamp) > minimumInterKeystrokeTime )
		{
			keystrokeTimeStamp = millis();
			processIsrKey = true;
		}
	}

	if (processIsrKey)
	{
		//Serial.println("KEYPAD ISR: processIsrKey");

   		// Temporarily disable keypad interrupt.
		// This is necessary since keyRead generates falling edges
		// when scanning the keypad matrix
		//detachInterrupt(digitalPinToInterrupt(HIB_KEY_INT_PIN)); 

		// Scan keypad matrix	
		pressedKey = pHIB->keyRead(false);

		// Set comlumns to LOW
		// to make AND gate able to catpure keystroke for keypad int
		_configPinKeyOutputForKeyInt();

		// Re-attach keypad int
		//attachInterrupt(digitalPinToInterrupt(HIB_KEY_INT_PIN), ISR_KEYPAD, FALLING);
	
		// Call the HIB KEY ISR handler
		if (pressedKey != pHIB->NO_KEY && pKeyHook != NULL)
		{
			pKeyHook(pressedKey);
		}
	
		// Rehabilitar keypad interrupt
		//attachInterrupt(digitalPinToInterrupt(HIB_KEY_INT_PIN), ISR_KEYPAD, FALLING); 
	} // end if processIsrKey

	// Restore the AVR Status Register by software
	// since the micro does not do it automatically
	auxSREG = SREG;


    		/*
   		 ////////////////////////////////////////
   		 // Detection based on detection of minimum number of rebounds
   		 ////////////////////////////////////////
   	 
   		 if(pressedKey == antPressedKey)
   		 {      
   		   numRebotes++;
   		 }
   		 else
   		 {
   		   numRebotes = 0;
   		 }
   		 
    		if(numRebotes > 30)
   		 {
   	   numRebotes = 0;
    		  key = pressedKey;
    		  Serial.println(key);
		 }
   		 antPressedKey = pressedKey;
	
   		 */
}



/*******************************************************************************
 * Buzzer
 *******************************************************************************
 *
 ******************************************************************************/

void HIB::buzzPlay(uint16_t time_ms, uint16_t frec_hz)
{
	uint32_t cycle_us;
	uint32_t half_cycle_us;
	uint16_t num_cycles;

	if (time_ms < 1 || time_ms > 6500)
		return;
	if (frec_hz < 1 || frec_hz > 10000)
		return;

	// Make sure the ADC is disabled
	cbi(ADCSRA, ADEN);

	// Make sure the pin connected to buzz is configured as output
	_configPinBuz();

	/*******************
	* Do buzzPlay stuff
	********************/
	cycle_us = (1.0 / frec_hz) * 1000000;
	half_cycle_us = cycle_us / 2;

	num_cycles = (time_ms * 1000L) / cycle_us;

	while (num_cycles > 0)
	{
		digitalWrite(HIB_BUZ_PIN, HIGH);
		_wait_us(half_cycle_us);
		digitalWrite(HIB_BUZ_PIN, LOW);
		_wait_us(half_cycle_us);

		num_cycles--;
	}
	/*****************
	******************/

	// Make sure the pin connected to adc is configured as input
	_configPinAdc();

	// Reenable and restart ADC conversion if needed

	switch (ADC_MODE)
	{
	case ADC_POLLING_DRIVEN:
		sbi(ADCSRA, ADEN);
		if (!adcLastPolledReadValue)
		{
			sbi(ADCSRA, ADSC);
		}
		break;

	case ADC_TIMER_DRIVEN:
		sbi(ADCSRA, ADEN);
		sbi(ADCSRA, ADSC);

		// Manually Clear Input Capture Interrupt Flag
		// in Timer/Counter i Interrupt Flag register (TIFRi)
		//
		// TIFRi bits:
		//	- - ICFi - OCFiC OCFiB OCFiA TOVi
		//
		// to clear an int flag it is necessary to write a '1'
		// TIFRi = TIFRi | 0010 0000
		//
		// We clear this bit because if the timer expired while
		// playing the buzzer then:
		//    the ICFi flag of the timer is set
		//    the ADC ISR is not triggered, since ADC int is disabled while playing buzzer
		//    thus the ADC ISR has not manually clear ICFi
		//    thus the timer interrupt will never trigger again
		//    and thus timer interrput will not trigger a new ADC conversion anylonger
		//
		// RIGHT now, we are using timer 1 for ADC
		TIFR1 = TIFR1 | 0x20;

		break;

	case ADC_AUTO_DRIVEN:
		sbi(ADCSRA, ADEN);
		sbi(ADCSRA, ADSC);
		break;

	default:
		break;
	}
}

/*******************************************************************************
 * LCD
 *******************************************************************************
 *
 ******************************************************************************/

// ATTENTION!!!!!!
//
// If keypad int is enabled by user, in each lcd funcion to the following.
// (1) at the beginning dettach keypad isr int.
// (1) at the end: set back to 'key pin mode' all pins associated to keypad.
// (2) at the end: leave all keypad columns to LOW to make AND gate to catpure keystroke for keypad int.
// (3) set keyFallingEdgeByLCD = true to indicate that _configPinLCD and/or lcd write/read operations
// cause an isr key int.
//
// In particular note that if keyapd int is enabled it is necessary
// to dettach keypad int to not trigger this interrupt when calling _configPinLCD
// or when writtin/reading from LCD.
// HOWEVER note that
// detaching INT does not prevent INT from triggering when the INT is re-attached.
// This is so becuase changing rows pin mode to OUTPUT and then to INPUT_PULLUP
// generates falling edge that is 'buffered' (even when interrupt is detached)
//  --> to cope with this problem we set keyFallingEdgeByLCD = true
//  	--> this makes ISR_KEYPAD to 'ignore' itself
//
//
// * Also note that keyRead generates a falling edge when it drives to HIGH
// a column where a key is pressed. This scenario does not affect lcd functions,
// but it is noteworthy to report it here.
//
// All this is strictly necessary only when keypad int is enabled.
// However, we always do so
// Changing rows pin mode to OUTPUT and then to INPUT_PULLUP
// generates falling edge that is 'buffered' (even when interrupt is detached).

// Auxiliary lcd funtions

void _lcdEntryKeypadIntInteraction(void);
void _lcdEntryKeypadIntInteraction(void)
{
	if(keyInterruptEnabled)
		detachInterrupt(digitalPinToInterrupt(HIB_KEY_INT_PIN)); 
}


void _lcdExitKeypadIntInteraction(void (*pISR)(void));
void _lcdExitKeypadIntInteraction(void (*pISR)(void))
{
	if(keyInterruptEnabled)
	{
		_configPinKey();
		_configPinKeyOutputForKeyInt();
		keyFallingEdgeByLCD = true;
		attachInterrupt(digitalPinToInterrupt(HIB_KEY_INT_PIN), pISR, FALLING);
	}
}


// LCD functions

void HIB::lcdClear()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.clear();
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdMoveHome()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.home();
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdPrint(char *text)
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.print(text);
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdSetCursorFirstLine()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.setCursor(0, 0);
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdSetCursorSecondLine()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.setCursor(0, 1);
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdSetCursorToLeft()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.command(0x10);
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdSetCursorToRight()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.command(0x14);
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdSetCursor(uint8_t lin, uint8_t col)
{
	if (lin < 0 || lin > LCD_LINS - 1)
		return;
	if (col < 0 || col > LCD_COLS - 1)
		return;

	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.setCursor(col, lin);
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdScrollToLeft()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.scrollDisplayLeft();
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdScrollToRight()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.scrollDisplayRight();
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdTurnOnCursor()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.cursor();
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdHideCursor()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.noCursor();
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdBlinkCursor()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.blink();
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

void HIB::lcdNoBlinkCursor()
{
	_lcdEntryKeypadIntInteraction();
	_configPinLCD();
	lcd.noBlink();
	_lcdExitKeypadIntInteraction(ISR_KEYPAD);
}

/*******************************************************************************
 * Timer 5
 *******************************************************************************
 ******************************************************************************/

void HIB::setUpTimer5(uint16_t topCount, tClkPreFactType preFact, void (*pHook)(void))
{

	char auxSREG;

	// Save SREG and disable interrupts
	auxSREG = SREG;
	noInterrupts();

	// Set timer 5 hook
	pTimer5Hook = pHook;

	// Timer 5 is clocked by the clkI/O clock
	// CKSEL fuses = 0010 by default --> the system Source clock is selected
	//	to be the internal Calibrated RC Oscillator, which works at 8 MHz
	// CKDIV8 fuse = 0 by default
	//	--> CLKPS bits of CLKPR (CLK Prescale Register) = 0011 at startup
	//	thus the Source clock is divided by 8, resulting in a 1 Mhz
	//	"final" Source Clock
	// This "final" Source Clock is routed through different "clock domains"
	//	Each clock domain is implemented as a "separated copy" of the
	//	"final" Source Clock
	//
	// Timer/Counter 5 uses the referred to as clkI/O clock
	//
	// This low clock freq is used to wisely be able to configure all the micro
	// 	parts
	//
	// HOWEVER NOTE THAT
	// 	we have measured that after micro initialization the compiler
	// 	sets CLKPS = 0000, thus the Source clock is divided by 1, resulting
	//	in a 8 Mhz "final" Source Clock
	//
	// Thus, we must consider that clkI/O clock works at 8 MHz

	//  When accessing the registers it is necessary to
	//  save/restore SREG and disable/enable interrupts
	//  Almost all registers share a temp 8-bit buffer for high
	//  byte. Thus if access is programmed in assembler:
	//	write: 1st ICR5H and then ICR5L
	//	red: 1st ICR5L and then ICR5H
	//	see 17.3 of datasheet

	// Stop Timer/Counter 5
	TCCR5B = TCCR5B & 0xF8; // Clear bits CS52 CS51 CS50 (bits 2,1,0)

	// Clear Timer/Counter 5 Interrupt Flag register (TIFR5)
	//
	// TIFR5 bits:
	//	- - ICF5 - OCF5C OCF5B OCF5A TOV5
	// individually set when the corresponding event/conditions happens
	// each cleared by hardware when exiting corresponding ISR
	// each can also be cleared "manually" by software
	// We use only ICF5 (see configuration of TIMSK5 just above)

	// to clear an int flag it is necessary to write a '1'
	// TIFR5 = TIFR5 | 0010 1111
	TIFR5 = TIFR5 | 0x2F; // CHANGE THIS IN ORIGINAL

	// Configure part of the WGM5 (Wafeform Generation Mode of timer/counter 5) mode
	// 	and disconnect Output Comparator modules of timer/counter 5 from
	// 	I/O pins
	//
	// We want to set WGM5 = 1100 --> timer 5 mode = 12
	// 	this means we configure timer 5
	//	in the CTC (Clear Timer on Compare Match) mode 12
	//	so that it will reset when reaching a TOP value specified in
	//	the Input Capture Register ICR5
	//
	// Timer/Counter Control register 5 A (TCCR5A)
	// TCCR5A bits:
	//	COM5A1 COM5A0 COM5B1 COM5B0 COM5C1 COM5C0 WGM51 WGM50
	// TCCR5A value: 00 00 00 00
	// thus:
	// 	COM5A1 COM5A0 = 00 --> OC5A disconnected from I/O
	// 	COM5B1 COM5B0 = 00 --> OC5B disconnected from I/O
	// 	COM5C1 COM5C0 = 00 --> OC5C disconnected from I/O
	//	WGM51 WGM50 = 00 <-- WGM5_1..0 = 00 <-- WGM53 WGM52 WGM51 WGM50 = 1100 <-- WGM5 = mode 12

	TCCR5A = 0x00;

	// Do not configure Comparator Modules of timer/counter 5 (we won't use
	// 	them)
	//
	// Output Compare registers 5 A, B and C High and Low bytes (OCR5xL/H)
	// OCR5AH OCR5AL = x x since we do not use output this output comparator
	// OCR5BH OCR5BL = x x since we do not use output this output comparator
	// OCR5CH OCR5CL = x x since we do not use output this output comparator

	// Configure the TOP value at which timer/counter 5 must reset
	//	note that we have we set mode 12 -->
	//	thus Input Capture register of timer/counter 5 specifies the
	//	TOP value at which the timer must reset
	//
	// Input Capture register 5 High and Low bytes (ICR5L/H)
	// ICR5H ICR5L value: since

	// for instance, if we want to count 1 s, then:
	// (1) bellow we will set the Timer/Counter 5 clock (tT5clk) = 32 us
	// (2) thus topCount must be (1 s / 32 us) = 31250 = 0x7A12
	//
	// ICR5H = 0x7A;
	// ICR5L = 0x12;

	ICR5H = (topCount >> 8);
	ICR5L = (topCount & 0x00FF);

	// Initialize timer/counter 5 to 0
	//
	// Timer/Counter N register 5 High and Low bytes (TCNT5L/H)
	// TCNT5H TCNT5L = timer 5 current count value
	TCNT5H = 0x00;
	TCNT5L = 0x00;

	// Enable just input capture interrupt of timer/counter 5 (we use
	//	it to trigger an ISR when the timer/counter 5 reaches the TOP
	//	specified in ICR5L/H
	//
	// Timer/Counter Interrupt MaSK register 5 (TIMSK5)
	// TIMSK5 bits:
	//	- - ICIE5 - OCIE5C OCI5B OCIE5A TOIE5
	// TIMSK5 value: - - 1 - 0 0 0 0
	// thus:
	//	ICIE5 = 1 --> enable timer5 input capture interrupt
	//	OCIE5_A_B_C = 0 0 0 --> disable timer 5 output compare match
	//		interrupt for output comparators A, B and C (we do not
	//		use them)
	//	TOIE5: 0 --> disable timer 5 overflow interrupt (we do
	//	not use it)

	// clear bits 0,1,2,3,5 then set bit 5
	// i.e. TIMSK5 = (TIMSK5 & 1101 0000)) | 0010 0000;
	TIMSK5 = (TIMSK5 & 0xD0) | 0x20;

	// Complete configuration of WGM5 and start counting by selecting
	// appropriate prescaler output
	//
	// Timer/Counter Control register 5 B (TCCR5B)
	// TCCR5B bits:
	//	ICNC5 ICES5 - WGM53 WGM52 CS52 CS51 CS50
	// TCCR5B value: x x - 1 1 1 0 0
	// thus:
	// 	ICNC5 = x
	// 	ICES5 = x
	//	WGM5_3..2 = 11 <-- WGM53 WGM52 WGM51 WGM50 = 1100 <-- WGM5 = mode 12
	//	CS52 CS51 CS50 = prefact
	//
	// For instance, to count 1s we select from the prescaler unit
	//	TclkI/O prescaled by 256
	// thereby we obtain a Timer/Counter 5 clock (tT5clk) = TclkI/O / 256.
	//
	// To be more specific in this example:
	//	preFact = T_PRESCALE_256 = 0b100
	// 	--> prescale by 256 --> timer 5 will be clocked at 8/256 Mhz
	//				--> fT5clk = 8/256 Mhz --> tT5clk = 32 us
	//
	// First, clear bits 0,1,2,3,4 using mask 0xE0
	// Second, set bits 3,4 as well as the bits specified in preFact

	TCCR5B = (TCCR5B & 0xE0) | (0b00011000 | preFact);
	timer5CurrentPrescale = preFact; // register current prescaler factor

	// Retore SREG and enable interrupts
	SREG = auxSREG;
	interrupts();
}

void HIB::pauseTimer5()
{
	// Stop Timer/Counter 5
	TCCR5B = TCCR5B & 0xF8; // Clear bits CS52 CS51 CS50 (bits 2,1,0)
}

void HIB::continueTimer5()
{
	TCCR5B = TCCR5B | (0b00000000 | timer5CurrentPrescale);
}

void HIB::timer5ISR()
{

	if (pHIB->pTimer5Hook != NULL)
	{
		pHIB->pTimer5Hook();
	}
}

// TIMER5 CAPT (Timer/Counter 5 Capture Event)
//	we call it "input capture interrupt of timer/counter 5"
ISR(TIMER5_CAPT_vect)
{

	char auxSREG;

	// Save the AVR Status Register by software
	// since the micro does not do it automatically
	auxSREG = SREG;

	// Call the HIB timer 5 ISR handler
	HIB::timer5ISR();

	// Restore the AVR Status Register by software
	// since the micro does not do it automatically
	auxSREG = SREG;
}

/*******************************************************************************
 * ADC
 *******************************************************************************
 *
 ******************************************************************************/

// Disables and Resets ADC
void disAndResetADC()
{

	// Set ADCSRA = 0000 0111 (disabled ADC, NO auto trigger enable,
	//    ADC interrupt flag cleared, ADC interrupt DISabled,
	// preescaler divides XTAL by 128)
	ADCSRA = 0x07;

	// Set ADCSRB = 0000 0000 (trigger source is Free Running Mode;
	//    but since auto trigger is still set as disabled in ADCSRA,
	//    the ADC is still configured as Single Conversion Mode)
	ADCSRB = 0x00;

	// Set ADMUX = 0000 0000 (voltage reference = AREF,
	//    ADC right adjust result, input channel is ADC0)
	ADMUX = 0x00;

	// Set voltage reference for ADC (DEFAULT = 1,
	// which means that it uses AVCC with external capacitor at AREF pin
	ADMUX = ADMUX | (analogRef << 6);

	// Set Analog Channel and Gain Selection Bits (MUX4:0)
	// Mask with 7 to restrict the selection to ADC0..7
	// Substract 54 because A0 to A7 are represented from 54 to 61
	// Note that MUX5 bit in ADCSRB is set to 0
	ADMUX = ADMUX | ((HIB_ADC_PIN - 54) & 7);
}

void HIB::adcSetPollingDriven()
{

	// Make sure the pin connected to adc is configured as input
	_configPinAdc();

	// Stop Timer/Counter 1 if necessary
	if (ADC_MODE == ADC_TIMER_DRIVEN)
		TCCR1B = TCCR1B & 0xF8; // Clear bits CS52 CS51 CS50 (bits 2,1,0)

	// Update ADC mode flag
	ADC_MODE = ADC_POLLING_DRIVEN;

	// Disable and reset ADC
	disAndResetADC();

	// Enable the ADC
	sbi(ADCSRA, ADEN);
	delay(1);
}

void HIB::adcSetTimerDriven(uint16_t topCount, tClkPreFactType preFact, void (*pHook)(uint16_t value))
{

	char auxSREG;

	// Make sure the pin connected to adc is configured as input
	_configPinAdc();

	// Stop Timer/Counter 1 if necessary
	if (ADC_MODE == ADC_TIMER_DRIVEN)
		TCCR1B = TCCR1B & 0xF8; // Clear bits CS52 CS51 CS50 (bits 2,1,0)

	// Update ADC mode flag
	ADC_MODE = ADC_TIMER_DRIVEN;

	// Set adc Hook
	pAdcHook = pHook;

	// Disable and reset ADC
	disAndResetADC();

	// Enable the ADC
	sbi(ADCSRA, ADEN);
	delay(1);

	// Enable ADC complete interrupt
	sbi(ADCSRA, ADIE);

	// Enable ADC auto trigger in ADCSRA
	sbi(ADCSRA, ADATE);

	// Since all bits of ADCSRB has been cleared
	// by disAndResetADC(), then bits ADTS2:0 = 000
	// This means the the source of auto trigger is
	// "Free running Mode"
	// Thus, we now set the source to be
	// "Timer/Counter1 Capture Event"
	ADCSRB = ADCSRB | 0x07;

	// NOW, configure Timer/counter 1 appropriately
	// to trigger ADC conversions periodically

	// Save SREG and disable interrupts
	auxSREG = SREG;
	noInterrupts();

	// Stop Timer/Counter 1
	TCCR1B = TCCR1B & 0xF8; // Clear bits CSi2 CSi1 CSi0 (bits 2,1,0)

	// Clear Timer/Counter 1 Interrupt Flag register (TIFR1)
	// to clear an int flag it is necessary to write a '1'
	// TIFR1 = TIFR5 | 0010 1111
	TIFR1 = TIFR1 | 0x2F;

	// Configure part of the WGM1 (Wafeform Generation Mode of timer/counter 1) mode
	// 	and disconnect Output Comparator modules of timer/counter 1 from
	// 	I/O pins
	//
	// We want to set WGM1 = 1100 --> timer 1 mode = 12
	// 	this means we configure timer 1
	//	in the CTC (Clear Timer on Compare Match) mode 12
	//	so that it will reset when reaching a TOP value specified in
	//	the Input Capture Register ICR1
	TCCR1A = 0x00;

	// Do not configure Comparator Modules of timer/counter 5 (we won't use
	// 	them)
	//
	// Output Compare registers 1 A, B and C High and Low bytes (OCR1xL/H)
	// OCR1AH OCR1AL = x x since we do not use output this output comparator
	// OCR1BH OCR1BL = x x since we do not use output this output comparator
	// OCR1CH OCR1CL = x x since we do not use output this output comparator

	// Configure the TOP value at which timer/counter 1 must reset
	//	note that we have we set mode 12 -->
	//	thus Input Capture register of timer/counter 1 specifies the
	//	TOP value at which the timer must reset
	//
	// Input Capture register 1 High and Low bytes (ICR1L/H)
	// ICR1H ICR1L value:
	ICR1H = (topCount >> 8);
	ICR1L = (topCount & 0x00FF);

	// Initialize timer/counter 1 to 0
	//
	// Timer/Counter N register 1 High and Low bytes (TCNT1L/H)
	// TCNT1H TCNT1L = timer 1 current count value
	TCNT1H = 0x00;
	TCNT1L = 0x00;

	// Disable all Timer/counter 1 interrupts, except the
	// input capture event interrupt: we will use it to capture
	// the timer 1 reaching a top counting value
	TIMSK1 = (TIMSK1 & 0x00) | 0x20;

	// Complete configuration of WGM5 and start counting by selecting
	// appropriate prescaler output
	//
	// Timer/Counter Control register 1 B (TCCR1B)
	// TCCR1B bits:
	//	ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
	// TCCR1B value: x x - 1 1 1 0 0
	// thus:
	// 	ICNC1 = x
	// 	ICES1 = x
	//	WGM1_3..2 = 11 <-- WGM13 WGM12 WGM11 WGM10 = 1100 <-- WGM1 = mode 12
	//	CS12 CS11 CS10 = prefact

	TCCR1B = (TCCR1B & 0xE0) | (0b00011000 | preFact);
	timer1CurrentPrescale = preFact; // register current prescaler factor

	// Retore SREG and enable interrupts
	SREG = auxSREG;
	interrupts();
}

void HIB::adcSetAutoDriven(void (*pHook)(uint16_t value))
{
	// Make sure the pin connected to adc is configured as input
	_configPinAdc();

	// Stop Timer/Counter 1 if necessary
	if (ADC_MODE == ADC_TIMER_DRIVEN)
		TCCR1B = TCCR1B & 0xF8; // Clear bits CS52 CS51 CS50 (bits 2,1,0)

	// Update ADC mode flag
	ADC_MODE = ADC_AUTO_DRIVEN;

	// Set adc adc Hook
	pAdcHook = pHook;

	// Disable and reset ADC
	disAndResetADC();

	// Enable the ADC
	sbi(ADCSRA, ADEN);
	delay(1);

	// Enable ADC complete interrupt
	sbi(ADCSRA, ADIE);

	// Enable ADC auto trigger in ADCSRA
	sbi(ADCSRA, ADATE);

	// Since all bits of ADCSRB has been cleared
	// by disAndResetADC(), then bits ADTS2:0 = 000
	// This means the the source of auto trigger is
	// "Free running Mode"

	// Trigger first ADC conversion
	sbi(ADCSRA, ADSC);
}

void HIB::adcRequestPoll()
{
	if (ADC_MODE == ADC_POLLING_DRIVEN)
	{

		// Trigger ADC conversion
		sbi(ADCSRA, ADSC);

		// Register that requested poll value has not been read yet
		adcLastPolledReadValue = false;
	}
}

uint16_t HIB::adcReadPolledValue(bool adcPollReadBlocking)
{

	byte auxADIF;

	if ((ADC_MODE != ADC_POLLING_DRIVEN) || adcLastPolledReadValue)
	{
		return ADC_VALUE_NOT_READY;
	}

	auxADIF = ADCSRA & 0x10;

	if (!adcPollReadBlocking)
	{
		if (auxADIF == 0)
		{
			return ADC_VALUE_NOT_READY;
		}
	}
	else
	{
		// Wait until ADC acquires value
		while (auxADIF == 0)
		{
			auxADIF = ADCSRA & 0x10;
		}
	}

	// Register ADC acquired value
	adcRegisterAdcValue();

	// Register that last requested poll value has been read
	adcLastPolledReadValue = true;

	return adcAcquiredValue;
}

uint16_t HIB::adcReadLastAcquiredValue()
{
	return adcAcquiredValue;
}

void HIB::adcRegisterAdcValue()
{

	// We must read ADCL first, which locks ADCH until it is read.
	uint8_t low = ADCL;
	uint8_t high = ADCH;

	adcAcquiredValue = (high << 8) | low;

	if (pAdcHook != NULL)
	{
		pAdcHook(adcAcquiredValue);
	}
}


void HIB::adcISR()
{
	bool aux;
       
	aux = readOnOtherAdcSensor;
	readOnOtherAdcSensor = false;

	if (!aux)
	{
		pHIB->adcRegisterAdcValue();

		pHIB->debug++;
	}


	// Manually Clear Input Capture Interrupt Flag
	// in Timer/Counter i Interrupt Flag register (TIFRi)
	//
	// TIFRi bits:
	//	- - ICFi - OCFiC OCFiB OCFiA TOVi
	//
	// Note that we need to manually clear this bit becuase:
	// timer/counter interrupt flags are individually set when
	// the corresponding event/conditions happens
	// each cleared by hardware when exiting corresponding ISR
	// each can also be cleared "manually" by software
	// We use only ICFi (see configuration of TIMSKi por ADC just above)

	// to clear an int flag it is necessary to write a '1'
	// TIFRi = TIFRi | 0010 0000
	TIFR1 = TIFR1 | 0x20;

	// It is not necessary to manually clear the ADC Interrupt Flag
	// ADIF bit of register ADCSRA – ADC Control and Status Register A,
	// since this bit is automatically cleared by hardware when exiting the ADC ISR
}

// ADC ISR.
ISR(ADC_vect)
{

	char auxSREG;

	// Save the AVR Status Register by software
	// since the micro does not do it automatically
	auxSREG = SREG;


  	// Conversion request is automatically triggered when
  	// Timer 1 reaches the configured top: ISR(TIMER1_CAPT_vect)
	// When the conversion finishes, this ISR(ADC_vect) is called.
  	// Thus, indicate here that the conversion has finished.
  	// This piece of info is necessary for the functions that
  	// read other sensors connected to the adc (temperature and light)
	timerDrivenConvInPro = false;

	// Call the HIB ADC ISR handler
	HIB::adcISR();

	// Restore the AVR Status Register by software
	// since the micro does not do it automatically
	auxSREG = SREG;
}


ISR(TIMER1_CAPT_vect)
{

  char auxSREG;

  // Save the AVR Status Register by software
  // since the micro does not do it automatically
  auxSREG = SREG;

  // Conversion request is automatically triggered when
  // Timer 1 reaches the configured top
  // Thus, indicate that a conversion is tacking place.
  // This piece of info is necessary for the functions that
  // read other sensors connected to the adc (temperature and light)
  timerDrivenConvInPro = true;

  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  auxSREG = SREG;
}

/*******************************************************************************
 * 7-segment displays
 *******************************************************************************
 *
 ******************************************************************************/

void HIB::d7sAllOn(uint8_t display)
{
	// Calculate index in array to see which pins to use
	uint8_t index = (8 * (display - 1));
	for (uint8_t i = index; i < 8 + index; i++)
	{
		digitalWrite(segs_pins[i], HIGH);
	}
}

void HIB::d7sAllOff(uint8_t display)
{
	// Calculate index in array to see which pins to use
	uint8_t index = (8 * (display - 1));
	for (uint8_t i = index; i < 8 + index; i++)
	{
		digitalWrite(segs_pins[i], LOW);
	}
}

void HIB::d7sDotOn(uint8_t display)
{
	if (display == RIGHT_7SEG_DIS)
		digitalWrite(DOT_1, HIGH);
	else
		digitalWrite(DOT_2, HIGH);
}

void HIB::d7sDotOff(uint8_t display)
{
	if (display == RIGHT_7SEG_DIS)
		digitalWrite(DOT_1, LOW);
	else
		digitalWrite(DOT_2, LOW);
}

void HIB::d7sPrintDigit(uint8_t digit, uint8_t display)
{
	d7sAllOff(display); // To avoid possible overlapping
	// Calculate index in array to see which pins to use
	uint8_t index = (8 * (display - 1));
	switch (digit)
	{
	case 0:
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		break;
	case 1:
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		break;
	case 2:
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		break;
	case 3:
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		break;
	case 4:
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		break;
	case 5:
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		break;
	case 6:
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		break;
	case 7:
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		break;
	case 8:
		d7sAllOn(display);
		d7sDotOff(display);
		break;
	case 9:
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		break;
	case 10:
	case 'a':
	case 'A':
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		break;
	case 11:
	case 'b':
	case 'B':
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		break;
	case 12:
	case 'c':
	case 'C':
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		break;
	case 13:
	case 'd':
	case 'D':
		digitalWrite(segs_pins[UR + index], HIGH);
		digitalWrite(segs_pins[LR + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		break;
	case 14:
	case 'e':
	case 'E':
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		digitalWrite(segs_pins[L + index], HIGH);
		break;
	case 15:
	case 'f':
	case 'F':
		digitalWrite(segs_pins[U + index], HIGH);
		digitalWrite(segs_pins[UL + index], HIGH);
		digitalWrite(segs_pins[C + index], HIGH);
		digitalWrite(segs_pins[LL + index], HIGH);
		break;
	}
}


void HIB::d7sPrint2DigHex(uint8_t hexNumber)
{
	uint8_t lower = hexNumber / 16;
	uint8_t upper = hexNumber % 16;

	d7sPrintDigit(upper, 1);
	d7sPrintDigit(lower, 2);
}


void HIB:: d7sPrint2DigDec(uint8_t decNumber)
{
	uint8_t lower = decNumber / 10;
	uint8_t upper = decNumber % 10;

	d7sPrintDigit(upper, 1);
	d7sPrintDigit(lower, 2);
}




/*******************************************************************************
 * Auxiliary function for temperature and ldr sensors
 *******************************************************************************
 *
 ******************************************************************************/

// Auxiliar funtion for temperature and ldr sensors
// It manages interaction when potentiometer is being adquired
// by the adc in timer-driven mode 

uint16_t HIB::readSensorWhileAdcPotTimerDriven(uint8_t adcInputPin)
{
	uint16_t adcReading;

	// Pause Timer/Counter 1
	TCCR1B = TCCR1B & 0xF8; // Clear bits CS12 CS11 CS10 (bits 2,1,0)

	// Just in case Timer/counter 1 has just expired,
	// wait until potentiometer conversion finishes
	while (timerDrivenConvInPro);

	readOnOtherAdcSensor = true;

	adcReading = analogRead(adcInputPin);

	// analogRead from temperature sensor pin changes ADMUX and ADCSRB
	// Specifically:
	// 	If MUX5 bit of ADCSRB == 1
 	//	 --> the ADC input is A8 or A9 (left or right temp sensor)
 	// 	else
 	// 	--> the ADC input is A1 (potentiometer)
	//
	//	If MUX5 bit == 1 and MUX4:0 of ADMUX = 0000 --> ADC input is A8
	//	If MUX5 bit == 1 and MUX4:1 of ADMUX = 0001 --> ADC input is A9
	//
	//Therefore, we must:
	//
	// 		set MUX5 bit of ADCSRB to '0' 
	// 		set MUX3:0 of ADMUX to 001
	//		for selecting the potentiometer pin
	ADCSRB = ADCSRB & 0xF7;
	ADMUX = (ADMUX & 0xE0) | 0x01;
	
	//ADMUX = ADMUX | ((HIB_ADC_PIN - 54) & 7);
	// Mask with 7 to restrict the selection to ADC0..7
	// Substract 54 because A0 to A7 are represented from 54 to 61
	// Note that MUX5 bit in ADCSRB is set to 0

	// Continue timer 1
	TCCR1B = TCCR1B | (0b00000000 | timer1CurrentPrescale);

	// Return raw value adquired by the adc
	return adcReading;
}




/*******************************************************************************
 * Temperature sensors
 *******************************************************************************
 *
 ******************************************************************************/

uint16_t HIB::temReadAdc(uint8_t temSensor)
{
	uint8_t adcInputPin;

	if (temSensor != LEFT_TEM_SENS && temSensor != RIGHT_TEM_SENS)
		return ADC_VALUE_NOT_READY; // ERROR

	if (temSensor == LEFT_TEM_SENS)
		adcInputPin = LEFT_TEM_PIN;

	if (temSensor == RIGHT_TEM_SENS)
		adcInputPin = RIGHT_TEM_PIN;

	return readSensorWhileAdcPotTimerDriven(adcInputPin);
}


float HIB::temReadVoltage(uint8_t temSensor)
{
	uint16_t adcReading;

	adcReading = temReadAdc(temSensor);
	
	// Return adquired temperatura sensor value.
	// The voltage input range the ADC has into account is [0, 5] V.
	// The ADC returns a value (adcReading) in the range [0, 1023] --> 1024 different values
	// --> ADC reads voltage with a resolution of 5/1024
	// --> Thus the voltage is adcReading · 5/1024
	// Anyway, note that the voltage output range from the temperature sensor
	// is approximately [100, 175] mV = [0.1, 1.75] V (it does not reaches 5V)
	// Thus, adcReading · 5/1024 will actually be in the range [100, 175] mV
	// -->  
	return (adcReading*5.0 / 1024.0);
}



float HIB::temReadCelsius(uint8_t temSensor)
{
	// The voltage is proportional to the temperature --> line equation: V = m·T + a
	// Each unit of temperature (each Celsius degree) implies an inc/dec of 0.01V = 10mV
	// The voltage output range from the temperature sensor is approximately [0.1, 1.75] V
	// The temperature range is [-40, 125] ºC --> range of 165 units.
	//
	// The line crosses the ordenadas axis (V axis) at the point (0ºC, 0.5V)
	//
	// Thus, the line equation expression is:
	//
	// 	V = m·T + a --> V = [ (1.75-0.1) / 165 ] + 0.5
	// 	V = 1/100 + 0.5
	//
	// We can use this equation to calculate T based on the voltage:
	// 	T = (V-a) / m --> T = (V-0.5) / (1/100) --> T = (V-0.5) · 100
	//
	return (temReadVoltage(temSensor) - 0.5) * 100.0;
}




/*******************************************************************************
 * LDR sensors
 *******************************************************************************
 *
 ******************************************************************************/

uint16_t HIB::ldrReadAdc(uint8_t ldrSensor)
{
	uint8_t adcInputPin;

	if (ldrSensor != LEFT_LDR_SENS && ldrSensor != RIGHT_LDR_SENS)
		return ADC_VALUE_NOT_READY; // ERROR

	if (ldrSensor == LEFT_LDR_SENS)
		adcInputPin = LEFT_LDR_PIN;

	if (ldrSensor == RIGHT_LDR_SENS)
		adcInputPin = RIGHT_LDR_PIN;

	return readSensorWhileAdcPotTimerDriven(adcInputPin);
}



////////////////////////////////////////////////////////////////////////////////
// AUXILIARY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void _configPinLed(void)
{
	uint8_t i;

	for (i = 0; i < (sizeof(led_pins) / sizeof(int)); i++)
	{
		pinMode(led_pins[i], OUTPUT);
	}
}

void _configPinBuz(void)
{
	pinMode(HIB_BUZ_PIN, OUTPUT);
}

void _configPinAdc(void)
{
	pinMode(HIB_ADC_PIN, INPUT);
}

void _configPinKey(void)
{
	pinMode(HIB_KEY_R1_PIN, INPUT_PULLUP);
	pinMode(HIB_KEY_R2_PIN, INPUT_PULLUP);
	pinMode(HIB_KEY_R3_PIN, INPUT_PULLUP);
	pinMode(HIB_KEY_R4_PIN, INPUT_PULLUP);

	pinMode(HIB_KEY_C1_PIN, OUTPUT);
	pinMode(HIB_KEY_C2_PIN, OUTPUT);
	pinMode(HIB_KEY_C3_PIN, OUTPUT);
}



void _configPinKeyOutputForKeyInt(void)
{
	digitalWrite(HIB_KEY_C1_PIN, LOW);
	digitalWrite(HIB_KEY_C2_PIN, LOW);
	digitalWrite(HIB_KEY_C3_PIN, LOW);	
}



void _configPinLCD(void)
{
	pinMode(HIB_KEY_R1_PIN, OUTPUT);
	pinMode(HIB_KEY_R2_PIN, OUTPUT);
	pinMode(HIB_KEY_R3_PIN, OUTPUT);
	pinMode(HIB_KEY_R4_PIN, OUTPUT);

	pinMode(HIB_KEY_C1_PIN, INPUT);
	pinMode(HIB_KEY_C2_PIN, INPUT);
	pinMode(HIB_KEY_C3_PIN, INPUT);
}

void _configPin7Segs(void)
{
	for (uint8_t i = 0; i < 16; i++)
	{
		pinMode(segs_pins[i], OUTPUT);
	}
}

void _configPinsTem(void)
{
	pinMode(LEFT_TEM_PIN, INPUT);
	pinMode(RIGHT_TEM_PIN, INPUT);
}

void _configPinsLDR(void)
{
	pinMode(LEFT_LDR_PIN, INPUT);
	pinMode(RIGHT_LDR_PIN, INPUT);
}

//void HIB::_configPinCAN(void){}

/*
	According to https://www.arduino.cc/en/Reference/DelayMicroseconds the
	delayMicroseconds() function is accurate from 3 to 16.383 usecs.
*/
void _wait_us(uint32_t time_us)
{
	uint16_t _time_ms;
	uint16_t _time_us;

	_time_ms = time_us / 16383;
	_time_us = time_us % 16383;

	delay(_time_ms);
	delayMicroseconds(_time_us);
}
