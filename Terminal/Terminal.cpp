/**
 * @file Terminal.cpp
 * @brief Brief of Terminal.cpp
 * @authors Alberto Ballesteros, Miller Espinosa and Manuel Barranco 
 * @date December 2016 - March 2017
 *
 * @details Detailed description of this file.
 */

#include "Terminal.h"
#include <math.h>
#include <stdio.h>

using namespace std;

#define ESC 27 // Escape
#define BELL  7 // Bell

Terminal::Terminal(){

}


void Terminal::begin(unsigned long baudRate){
	Serial.begin(baudRate);
}


char Terminal::getChar(bool block){

	char byteAva;

	byteAva = Serial.available();

	while ( block && !byteAva )
	{
		byteAva = Serial.available();
	}

	if (byteAva)
	{
	
		return Serial.read();
	}
	else
	{
		return NO_CHAR_RX_UART;
	}
}


void Terminal::print(byte number){
	Serial.write(number);
}

void Terminal::print(char c){
	Serial.write(c);
}


void Terminal::print(int integer){
	Serial.print(integer);
}

void Terminal::print(double real){
	Serial.print(real);
}

void Terminal::print(char *str){
	Serial.print(str);
}

void Terminal::printHEX(unsigned long number){
	Serial.print(number, HEX);
}

void Terminal::println(char *str){
	print(str);
	newLine();
}

void Terminal::println(unsigned long number){
	Serial.println(number);
}



void Terminal::moveHome(void){
    Serial.write(ESC);
    Serial.write('[');
    Serial.write('H');
}

void Terminal::moveUp(void){
	Serial.write(ESC);
	Serial.write('[');
	Serial.write('A');
}

void Terminal::moveDown(void){
	Serial.write(ESC);
	Serial.write('[');
	Serial.write('B');
}

void Terminal::moveRight(void){
	Serial.write(ESC);
	Serial.write('[');
	Serial.write('C');
}

void Terminal::moveLeft(void){
	Serial.write(ESC);
	Serial.write('[');
	Serial.write('D');
}

void Terminal::newLine(void){
	Serial.print("\n\r");
}

void Terminal::move(char row, char col){
	char row1, row2;
	char col1, col2;

	if (row < 0 || row > TERM_ROWS) return;
	if (col < 0 || col > TERM_COLS) return;

	// Row
	row1 = (row/10) + 48;
	row2 = (row%10) + 48;

	// Col
	col1 = (col/10) + 48;
	col2 = (col%10) + 48;

	Serial.write(ESC);
	Serial.write('[');
	Serial.write(row1);
	Serial.write(row2);
	Serial.write(';');
	Serial.write(col1);
	Serial.write(col2);
	Serial.write('f');
}


void Terminal::clear(){
	Serial.write(12);
}


void Terminal::deleteToEndLine(void){
	Serial.write(ESC);
	Serial.write('[');
	Serial.write('K');
}


void Terminal::bell(){
	Serial.write(BELL);
}


void Terminal::waitTxComplete(){
	Serial.flush();
}




//*************Métodos privados******************

//Realiza una potencia en base 10 y exponente 'exp'
int Terminal::pow10(int exp){
	int result = 1;

	for (int i = 0; i < exp; i++) result*=10;

	return result;
}

