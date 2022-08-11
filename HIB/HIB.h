/**
 * @file HIB.h
 * @brief Brief of HIB.h
 * @authors Manuel Barranco, Alberto Ballesteros, Adrian Moreno, Izar Castorina
 * @institution: Departament de Ciencies Matematiques i Informatica, Escola Politecnica Superior, Universitat de les Illes Balears
 * @last update April 2021
 *
 * @details Detailed description of this file.
 */


#ifndef HIB_H
#define HIB_H

#include <Arduino.h>

// Timer Clock Prescaler Factor type
enum tClkPreFactType
{
	T_PRESCALE_1 = 0b001,
	T_PRESCALE_8 = 0b010,
	T_PRESCALE_64 = 0b011,
	T_PRESCALE_256 = 0b100,
	T_PRESCALE_1024 = 0b101
};



class HIB
{

private:
	/**
	 * @brief Pointer to the active HIB object.
	 * Only	one instantiation of HIB can be used to
	 * interact with the HIB board at a time
	 */
	static HIB *pHIB;


	/**
	 * @brief ISR of the KEYPAD.
	 * member function keySetIntDriven attatches it to 'exernal pin int'
	 * Only	one instantiation of HIB can be used to
	 * interact with the HIB board at a time
	 */
	static void ISR_KEYPAD();

	/**
	 * @brief Pointer to the timer 5 hook defined by the user.
	 * If the user actually defines this hook and provides its
	 * address to setUpTimer5, then timer5ISR
	 * invokes it
	 *
	 * @details timer5ISR is the HIB static function that the
	 * Timer 5 CAPT ISR calls. Since timer5ISR is static, it needs
	 * to use the static pointer pHIB to access the HIB
	 * instantation. In turn, pTimerHook needs to be declared here as a
	 * member of HIB in order to be accessed by means of pHIB.
	 * 
	 * @see pHIB, setUpTimer5, timer5ISR
	 *
	*/
	void (*pTimer5Hook)(void) = NULL;

	unsigned char timer5CurrentPrescale = 0; // Timer 5 current prescaler factor
	unsigned char timer1CurrentPrescale = 0; // Timer  current prescaler factor

	/** @brief Function called by adcISR() to
 	 * process the adc value acquired by the ADC.
	 * Additionally, if the user defines a hook for the
	 * adc adc and provides its address to adcSetAutoDriven, then
	 * adcRegisterAdcValue invokes that hook
	 *
	 * @details adcISR is the HIB static function that the
	 * ADC ISR calls. Since adcISR is static, it needs
	 * to use the static pointer pHIB to access the HIB
	 * instantation. In turn, adcRegisterAdcValue needs to
	 * be declared here as a member of HIB in order to be
	 * accessed by means of pHIB.
	 * 
	 * @see pHIB, adcSetAutoDriven, adcRegisterAdcValue, adcAdcISR
	 */
	void adcRegisterAdcValue();


	// Auxiliar funtion for temperature and ldr sensors
	// It manages interaction when potentiometer is being adquired
	// by the adc in timer-driven mode 
	uint16_t readSensorWhileAdcPotTimerDriven(uint8_t adcInputPin);

public:

	uint16_t debug = 0;

	/** @brief Number of LEDs */
	const uint8_t NUM_LEDS = 6;

	/** @brief Number of LCD lines */
	const uint8_t LCD_LINS = 2;

	/** @brief Number of LCD columns */
	const uint8_t LCD_COLS = 16;

	/** @brief Value returned by keyRead() when no key is pressed */
	const uint8_t NO_KEY = 255;

	#define ADC_VALUE_NOT_READY 65535

	/** @brief Indicates left-handed or right-handed 7-seg display */
	const uint8_t LEFT_7SEG_DIS = 2;
	const uint8_t RIGHT_7SEG_DIS = 1;

	/** @brief Indicates left-handed or right-handed temperature sensor */
	const uint8_t LEFT_TEM_SENS = 0;
	const uint8_t RIGHT_TEM_SENS = 1;

	/** @brief Indicates left-handed or right-handed ldr sensor */
	const uint8_t LEFT_LDR_SENS = 0;
	const uint8_t RIGHT_LDR_SENS = 1;

	/**
	 * @brief Initializes all the components.
	 *
	 * Configures the pins of the peripherals as input or outputs, accordingly,
	 * and initializes the peripherals.
	 */

	HIB();

	void begin();

	/**************************************************************************/
	/** @name LEDs ************************************************************/
	/**************************************************************************/
	//@{

	/**
	 * @brief Switch on one specific led
	 * @param [in] num_led Number of the led (from 0 to NUM_LEDS-1)
	 * @see NUM_LEDS, ledOff, ledToggle
	 */

	void ledOn(uint8_t num_led);

	/**
	 * @brief Switch off one specific led
	 * @param [in] num_led Number of the led (from 0 to NUM_LEDS-1)
	 * @see NUM_LEDS, ledOn, ledToggle
	 */

	void ledOff(uint8_t num_led);

	/**
	 * @brief Toggles one specific led
	 * @param [in] num_led Number of the led (from 0 to NUM_LEDS-1)
	 * @see NUM_LEDS, ledOn, ledOff
	 */

	void ledToggle(uint8_t num_led);

	/**
	 * @brief Light up leds to visualy represent the argument as BNSS number
	 * @param [in] number Natural number
	 * @see NUM_LEDS, ledOn, ledOff
	 */

	void ledPrintNum(unsigned char number);

	//@}

	/**************************************************************************/
	/** @name Keypad **********************************************************/
	/**************************************************************************/
	//@{

	/**
	 * @brief Read the keypad for any key pressed.
	 *
	 * @param [in] block Block the execution until a key is pressed.
	 *
	 * @return The code of the pressed key. If block is <i>false</i> and no key
	 * is pressed NO_KEY is returned.
	 * 
	 * @see NO_KEY
	 */

	uint8_t keyRead(bool block);


	/**
	 * @brief Setup the keypad to trigger an interruption (INT3) when a key 
	 * is pressed.
	 *
	 * @param [in] minInterKeystrokeTime Minimum expected time between keystrokes.
	 * You can play with this parameter to adjust keypad int 'sensitivity'
	 *
	 * @param [in] pHook User-defined hook to be invoked by the KEYPAD ISR
	 *
	 * @see ISR_KEYPAD
	 */
	void keySetIntDriven(uint8_t minInterKeystrokeTime, void (*pHook)(uint8_t newKey) );


	//@}

	/**************************************************************************/
	/** @name Buzzer **********************************************************/
	/**************************************************************************/
	//@{

	/**
	 * @brief Produces a buzz of a given frequency during a given time.
	 *
	 * @param [in] time_ms Time in ms during which the buzz sounds. This value
	 * has to be between 1 and 6500.
	 *
	 * @param [in] frec_hz Frequency in hertz of the buzz. The buzzer is
	 * designed to buzz in a frequency between 3500 and 4500 hertz. However, a
	 * frequency between 1 and 10000 is also valid. A frequency outside the
	 * ideal range will make the sound weaker.
	 */

	void buzzPlay(uint16_t time_ms, uint16_t frec_hz);

	//@}

	//@}

	/**************************************************************************/
	/** @name LCD *************************************************************/
	/**************************************************************************/
	//@{

	/**
	 * @brief Clear the LCD and move cursor to initial position.
	 * @details Clean DDRAM and move cursor to (0,0).
	 */

	void lcdClear();

	/**
	 * @brief Move cursor to initial position.
	 * @details Does not clean DDRAM, just move cursor to (0,0).
	 */

	void lcdMoveHome();

	/**
	 * @brief Print a given text into de the LCD starting from the cursor position.
	 * @details Write string into the DDRAM starting from the cursor position.
	 * @param [in] text String containing the text.
	 */

	void lcdPrint(char *text);

	/**
	 * @brief Functions to move cursor to a given position of the LCD
	 */

	void lcdSetCursorFirstLine();

	void lcdSetCursorSecondLine();

	void lcdSetCursorToLeft();

	void lcdSetCursorToRight();

	/**
	 * @brief Move cursor to a given row and column of the LCD.
	 * @details Move cursor to a specific position of the LCD. Since the LCD has
	 * LCD_LINS lines and LCD_COLS columns, the values of the coordinates are
	 * restricted.
	 * @param [in] lin Line. Valid value between 0 and LCD_LINS-1.
	 * @param [in] col Column. Valid value between 0 and LCD_COLS-1.
	 * @see LCD_LINS, LCD_COLS
	 */

	void lcdSetCursor(uint8_t lin, uint8_t col);

	/**
	 * @brief Functions for scrolling within the LCD
	 */

	void lcdScrollToLeft();

	void lcdScrollToRight();

	/**
	 * @brief Functions for changing the LCD cursor behavior (mode)
	 */

	void lcdTurnOnCursor();

	void lcdHideCursor();

	void lcdBlinkCursor();

	void lcdNoBlinkCursor();

	//@}

	/**************************************************************************/
	/** @name Timer 5 *********************************************************/
	/**************************************************************************/
	//@{

	/**
	 * @brief Stop time, then configure it, then start it up
	 *
	 * @param [in] topCount Top number of timer ticks to count; the timer wraps
	 * around to zero when reaching this value
	 * @param [in] preFact Prescaler value
	 * @param [in] pHook User-defined hook to be invoked by the Timer 5 ISR
	 */

	void setUpTimer5(uint16_t topCount, tClkPreFactType preFact, void (*pHook)(void));

	/**
	 * @brief Pause the timer (it keeps its current counting value)
	 */

	void pauseTimer5();

	/**
	 * @brief Allow the timer to continue counting
	 */

	void continueTimer5();

	/**
	 * @brief Function called by the Timer 5 CAPT ISR.
	 *
	 * @details This function additionally calls a user-defined
	 * timer 5 hook (if the hook is actually defined and its
	 * address is provided to setUpTimer5).
	 * Also note that this function needs to be static so that
	 * the timer 5 capt ISR can call it without using an HIB instantiation.
	 *
	 * @see setUpTimer5
	 */

	static void timer5ISR();

	/**************************************************************************/
	/** @name ADC *************************************************************/
	/**************************************************************************/
	//@{

	/**
	 * @brief Configure the adc to work in polling (manual) mode.
	 *
	 * @details No conversion (acquisition) request is carried out.
	 * No interrupt is risen when the ADC finishes converting a requested value.
	 *
	 * @see adcRequestPoll, adcReadPolledValue, adcReadLastAcquiredValue
	 */

	void adcSetPollingDriven();

	/**
	 * @brief Configure and START the adc to work in timer-driven mode.
	 *
	 * @details Timer 1 input capture event (event of timer 1 reaching configured TOP value)
	 * is used to automatically trigger the adc conversion in a periodic fashion.
	 * The ADC interrupt is risen when the ADC finishes each conversion.
	 * The ADC ISR calls pHook if this is different from NULL
	 *
	 * @param [in] topCount Top number of ticks for timer 1 to count before triggering the ADC conversion;
	 * the timer wraps around to zero when reaching this value
	 * @param [in] preFact Prescaler value
	 * @param [in] pHook User-defined hook to be invoked by the ADC ISR
	 *             NOTE: the hook is provided with the just converted value through an input parameter
	 *
	 * @see adcReadLastAcquiredValue
	 */

	void adcSetTimerDriven(uint16_t topCount, tClkPreFactType preFact, void (*pHook)(uint16_t value));

	/**
	 * @brief Configure and START the adc to work in "Free running Mode", so that it
	 * automatically triggers a new conversion (acquisition) when finishing the previous one.
	 *
	 * @details The ADC is configured to operate in auto triggered free running mode so the
	 * adc is read continuously.
	 * The ADC interrupt is risen when the ADC finishes each conversion.
	 * The ADC ISR calls pHook if this is different from NULL
	 * The input clock of the ADC is prescaled as XTAL / 128 and each conversion takes
	 * about 25 clock input cycles to complete
	 *
	 * @param [in] pHook User-defined hook to be invoked by the ADC ISR.
	 *             NOTE: the hook is provided with the just converted value through an input parameter
	 *
	 * @see adcReadLastAcquiredValue
	 */

	void adcSetAutoDriven(void (*pHook)(uint16_t value));

	/**
	 * @brief Request the ADC to convert (acquire) a new value from of the
	 * adc.
	 *
	 * @details
	 *
	 * If the ADC is not configured to work in
	 * this function does nothing.
	 *
	 * @see adcSetPollingDriven
	 */

	void adcRequestPoll();

	/**
	 * @brief Read the value acquired from the adc after
	 * the last call to adcRequestPoll.

	 * @param [in] adcPollReadBlocking Bool indicating whether or
	 * not the read attempt is blocking.
	 *   If true, the function will actively wait until the polled value is ready.
	 *   If false and the polled value is ready, the function will return it.
	 *   If false and the polled value is not ready, it will return ADC_VALUE_NOT_READY
	 *   to indicate it.

	 * @details
	 * 
	 * If this function is invoked without having configured the
	 * ADC in polling fashion, it returns
	 * ADC_VALUE_NOT_READY to indicate an error.
	 * 
	 * If this function is invoked without having called
	 * adcRequestPoll anytime, it returns ADC_VALUE_NOT_READY
	 * to indicate an error.
	 * 
	 * If adcRequestPoll is not invoked in between two consecutive
	 * calls to adcReadPolledValue, adcReadPolledValue returns
	 * ADC_VALUE_NOT_READY to indicate an error.
	 *
	 *
	 * @return The value set by the adc codified as an
	 * unsigned integer between 0 and 1023, or ADC_VALUE_NOT_READY.
	 *
	 * @see adcRequestPoll
	 */

	uint16_t adcReadPolledValue(bool adcPollReadBlocking);

	/**
	 * @brief Read last value acquired from the adc
	 * REGARDLESS of the method used to do so, i.e:
	 * polling driveb, timer driven, or auto driven.
	 */

	uint16_t adcReadLastAcquiredValue();

	/**
	 * @brief function called by the ADC ISR.
	 *
	 * @details This function additionally calls
	 * adcRegisterAdcValue, which in turn calls a user-defined
	 * adc hook (if the user defines it and provides its address
	 * to adcSetAutoDriven).
	 * Also note that adcISR needs to be static so that
	 * the ADC ISR can call it without using an HIB instantiation.
	 *
	 * @see adcRegisterAdcValue, adcSetAutoDriven
	 */

	static void adcISR();

	//@}

	/**************************************************************************/
	/** @name 7-segment displays **********************************************/
	/**************************************************************************/
	//@{
	/**
 	 * @brief   Turns all segments of the specified display on. 
 	 * @param   [in] display  LEFT_7SEG_DIS or RIGHT_7SEG_DIS,
	 * 			  depending on which display to use
	 * 			  left-handed or right-handed respectively
 	 */
	void d7sAllOn(uint8_t display);

	/**
 	 * @brief   Turns all segments of the specified display off. 
 	 * @param   [in] display  LEFT_7SEG_DIS or RIGHT_7SEG_DIS,
	 * 			  depending on which display to use
	 * 			  left-handed or right-handed respectively
 	 */
	void d7sAllOff(uint8_t display);

	/**
	 * @brief   Turns on the dot of the specified display. 
 	 * @param   [in] display  LEFT_7SEG_DIS or RIGHT_7SEG_DIS,
	 * 			  depending on which display to use
	 * 			  left-handed or right-handed respectively
 	 */
	void d7sDotOn(uint8_t display);

	/**
 	 * @brief   Turns off the dot of the specified display. 
 	 * @param   [in] display  LEFT_7SEG_DIS or RIGHT_7SEG_DIS,
	 * 			  depending on which display to use
	 * 			  left-handed or right-handed respectively
 	 */
	void d7sDotOff(uint8_t display);

	/**
 	 * @brief   Prints the specified digit on the specified display. 
 	 * @param   [in] digit	unsigned integer from 0 to 15 (F);
	 * 			or the ascii codde of alfa hex symbols: 'a', 'A' ... 'f', 'F' 
 	 * @param   [in] display  LEFT_7SEG_DIS or RIGHT_7SEG_DIS,
	 * 			  depending on which display to use
	 * 			  left-handed or right-handed respectively
 	 */
	void d7sPrintDigit(uint8_t digit, uint8_t display);

	/**
 	 * @brief   Prints the specified hexadecimal number using both displays. 
 	 * @param   [in] hexNumber  unsigned integer from 0 to FF (255)
 	 */
	void d7sPrint2DigHex(uint8_t hexNumber);

	//@}

	/**
 	 * @brief   Prints the specified decimal number using both displays. 
 	 * @param   [in] decNunmer  unsigned integer from 0 to 99
 	 */
	void d7sPrint2DigDec(uint8_t decNumber);


	/**************************************************************************/
	/** @name Temperature sensors *********************************************/
	/**************************************************************************/
	//@{

	/**
 	 * @brief   Returns the 'raw' digital value that the adc adquires from the temperature sensor. 
	 * @param   [in] temSensor specifies which sensor to read from
	 * 		 (LEFT_TEM_SENS: lefth-handed sensor or RIGH_TEM_SENS: right-handed sensor) 
	 * @param   [out] 10-bit value the adc adquires from the sensor (range = [20, 358]). 
 	 */		
	uint16_t temReadAdc(uint8_t temSensor);

	/**
 	 * @brief   Returns the voltage value coming from the temperature sensor. 
	 * @param   [in] temSensor specifies which sensor to read from
	 * 		 (LEFT_TEM_SENS: lefth-handed sensor or RIGH_TEM_SENS: right-handed sensor) 
	 * @param   [out] voltage from the sensor (range = [0.1, 1.17] V ). 
 	 */		
	float temReadVoltage(uint8_t temsensor);

	/**
 	 * @brief   Returns the temperature coming from the temperature sensor in Celsius degrees. 
	 * @param   [in] temSensor specifies which sensor to read from
	 * 		 (LEFT_TEM_SENS: lefth-handed sensor or RIGH_TEM_SENS: right-handed sensor) 
	 * @param   [out] temperature from the sensor (range = [-40, 125] ºC ). 
 	 */		
	float temReadCelsius(uint8_t temSensor);

	//@}

	/**************************************************************************/
	/** @name LDR sensors (Light-Dependent Resitor aka photoresistor) *********/
	/**************************************************************************/
	//@{

	/**
 	 * @brief   Returns the 'raw' digital value that the adc adquires from the ldr sensor. 
	 * @param   [in] ldrSensor specifies which sensor to read from
	 * 		 (LEFT_LDR_SENS: lefth-handed sensor or RIGH_LDR_SENS: right-handed sensor) 
	 * @param   [out] 10-bit value the adc adquires from the sensor (range = [0, 1023]). 
	 *
	 * NOTE:    ldr response delay to ligth change: around 50 ms
 	 */		
	uint16_t ldrReadAdc(uint8_t ldrSensor);

	//@}


};

#endif
