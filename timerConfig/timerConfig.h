#include <HIB.h>

/*
 * Timers configuration header
 *
 * Timers 0, 1, 3, 4, 5 can be configured to be fed with any of the prescaler generated clocks
 * In turn, the prescaler is configured to be clocked with the system clock, which is configured
 * at 8 Mhz.
 */

//////////////////////////////////////////////////////////////////////////////////
// Calculo del valor del preescaler y del periodo para diferentes velocidades
//////////////////////////////////////////////////////////////////////////////////

// Ejemplo:
//
// Queremos que el Timer cuente 1s
//
// Tenemos que hallar dos valores:
//
//	- Valor de prescalado
//	- Valor del número de tiks a contar (timer ticks)
//
// La frecuencia del system clock (CLK_I/O) es 16 Mhz = 16.000.000 Hz
//
// Con un valor de prescalado de 1:1
//
// 	F_CLK_IO = 16.000.000 Hz -> T_CLK_IO = 62,5·E-9 s
//  	F_CLK_Timer (1:1) = 16.000.000 Hz -> T_CLK_Timer (1:1) = 62,5·E-9 s
//
// 	T_CLK_Timer * TIMER_TICKS = tiempo
// 	TIMER_TICKS = tiempo / T_CLK_Timer
//	TIMER_TICKS = 1s / 62,5·E-9 s = 16.000.000
//
// Como el timer/contador es de 16 bits, solo puede contar hasta 65535 ticks.
// Por tanto, TIMER_TICKS no cabe en el registro
// del timer que se utiliza para programar su periodo (ICR5)
//
//	¿Cual crees que deberia ser el valor del prescalado para que
//	el TIMER_TICKS sea un numero lo suficientemente bajo como para
//	poder programar el timer 5 con el?
//
//	Una vez hayas encontrado el valor de prescalado apropiado, debes descomentar
//	las constantes
//
//	TIMER_PSCALER_FOR_1s y TIMER_TICKS_FOR_1s
//
//	y asignarles el valor de prescalado y TIMER_TICKS respectivamente


const uint16_t TIMER_PSCALER_FOR_1s = T_PRESCALE_1024;
const uint16_t TIMER_TICKS_FOR_1s = 0x3D09; // 15625 (dec)
  
const uint16_t TIMER_PSCALER_FOR_500ms = T_PRESCALE_256;
const uint16_t TIMER_TICKS_FOR_500ms = 0x7A12; // 31250 (dec)

const uint16_t TIMER_PSCALER_FOR_250ms = T_PRESCALE_256;
const uint16_t TIMER_TICKS_FOR_250ms =0x3D09; // 15625 (dec)

const uint16_t TIMER_PSCALER_FOR_125ms = T_PRESCALE_64;
const uint16_t TIMER_TICKS_FOR_125ms = 0x7A12; // 31250 (dec)

const uint16_t TIMER_PSCALER_FOR_62_5ms = T_PRESCALE_64;
const uint16_t TIMER_TICKS_FOR_62_5ms = 0x3D09; // 15625 (dec)

const uint16_t TIMER_PSCALER_FOR_50ms = T_PRESCALE_256;
const uint16_t TIMER_TICKS_FOR_50ms = 0xC35; // 3125 (dec)

const uint16_t TIMER_PSCALER_FOR_10ms = T_PRESCALE_256;
const uint16_t TIMER_TICKS_FOR_10ms = 0x271; // 625 (dec)

const uint16_t TIMER_PSCALER_FOR_1ms = T_PRESCALE_64;
const uint16_t TIMER_TICKS_FOR_1ms = 0xFA; // 250 (dec)