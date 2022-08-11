#include <stdio.h>
#include "HIB.h"
#include "timerConfig.h"
#include "SO.h"
#include <SPI.h>
#include "Terminal.h"
#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>

///////////////////////////////////////////////////////////////////////////////
// Global constants
///////////////////////////////////////////////////////////////////////////////
#define PERIOD_PAD 3
#define PERIOD_WRITE_UART 3
#define PERIOD_LCD 10
#define PERIOD_AUXILIAR 20

#define PRIO_SNSCONTROL 1
#define PRIO_UART 2
#define PRIO_PAD 3
#define PRIO_LCD 4
#define PRIO_AUXILIAR 1

#define NUM_SENSORES 3
#define FORMAT_LCD1 "#%d %s: %s"
#define FORMAT_LCD2 "Umbral: %s"

#define TERM_HEADER "ID\tSensor\t\tValor\tUmbral\tSuperado"
#define TERM_SEPARATOR "------------------------------------------------"
#define TERM_FORMAT "%d\t%s\t%s\t%s\t%c"
#define TERM_HELP "Escriba la ID de un sensor para modificar su umbral."
#define TERM_NEWVAL "Nuevo valor: "

#define CAN_ID_SENSOR 1

const char* sensorNombresLargos[3] = {
  "Luminosidad",
  "Temperatura",
  "Calidad de aire"
};
const char* sensorNombresCortos[3] = {"Luz", "Temp.", "CO2"};


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

///////////////////////////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////////////////////////

HIB hib;
SO so;
Terminal term;

typedef struct canMsg_t {
  uint8_t sns_id;
  float sns_val;
};

canMsg_t rxSensor;

// Shared
volatile uint8_t sensorSelect = 0;
volatile double snsValores[NUM_SENSORES] = {0.0, 0.0, 0.0};
volatile double snsUmbrales[NUM_SENSORES] = {600.0, 26.0, 2.5};

// Semaphores
Sem sPrint2;
Sem sSensorSelect;

// CAN
MCP_CAN CAN(SPI_CS_PIN);

/********************************
  Declaration of flags and masks
*********************************/
  // flag (set of bits) for CAN reception events
Flag fCANevent;

  // refers to the LSB bit of fCANevent
const unsigned char maskRxCANevent = 0x01; // represents reception of sensor via CAN

/*****************
  TICKISR
******************/ 

// Hook FOR TICKISR
void timer5Hook () { 
  so.updateTime(); // Call SO to update time (tick) and manage tasks
}

/*****************
  CANISR
******************/ 
void isrCAN() {
  char auxSREG;

  // Save the AVR Status Register by software
  // since the micro does not do it automatically
  auxSREG = SREG;
  ////////

  if (CAN.rxInterrupt()) {
    
    CAN.readRxMsg();
    switch(CAN.getRxMsgId()) {
      case CAN_ID_SENSOR:
        CAN.getRxMsgData((byte*) &rxSensor);
//        Serial.println(rxSensor.sns_id);
//        Serial.println(rxSensor.sns_val);
        so.setFlag(fCANevent, maskRxCANevent);
        break;
      default: break;
    }
  }

  /////
  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  SREG = auxSREG;
}

/******************************************
  TASKS declarations and implementations
*******************************************/ 

 /* Task states

  DESTROYED / uninitalized
  BLOCKED / WAITING FOR SEMAPHORE-MAILBOX-FLAG
  AUTO-SUSPENDED BY TIME
  ACTIVE / ELIGIBLE / READY
  RUNNING

*/

void P_LEERPAD() {
  unsigned long nextActivationTick;
  
  uint8_t key = hib.NO_KEY;
  uint8_t prev_key = hib.NO_KEY;

  nextActivationTick = so.getTick();
  while(true) {
    // Avoid multiple triggers in non-blocking environment
    prev_key = key;
    key = hib.keyRead(false);
    
    if(key != hib.NO_KEY && key != prev_key) {
      if(key < NUM_SENSORES) {
        so.waitSem(sSensorSelect);
        sensorSelect = key; // add mutex
        so.signalSem(sSensorSelect);
      }
    }

    nextActivationTick += PERIOD_PAD;
    so.delayUntilTick(nextActivationTick);
  }
}

void P_PRINT_P2() {
  unsigned long nextActivationTick;
  uint8_t select;
  char msg[17];
  double valor;
  double umbral;
  char str_valor[10];
  char str_umbral[10];
  
  nextActivationTick = so.getTick();
  
  while(true) {
    hib.lcdClear();
    hib.lcdSetCursorFirstLine();
    
    // Get message
    so.waitSem(sSensorSelect);
    select = sensorSelect;
    so.signalSem(sSensorSelect);

    // Get variables
    so.waitSem(sPrint2);
    valor = snsValores[select];
    umbral = snsUmbrales[select];
    so.signalSem(sPrint2);

    // String format
    dtostrf(valor, 5, 2, str_valor);
    dtostrf(umbral, 5, 2, str_umbral);
    
    sprintf(msg, FORMAT_LCD1, select, sensorNombresCortos[select], str_valor);
    hib.lcdPrint(msg);
    hib.lcdSetCursorSecondLine();
    sprintf(msg, FORMAT_LCD2, str_umbral);
    hib.lcdPrint(msg);
    

    nextActivationTick += PERIOD_LCD; // Calculate next activation time
    so.delayUntilTick(nextActivationTick);
  }
}

void E_SNSCONTROL() {
  
  double valor;
  double umbral;
  bool activar;
  
  while(true) {
    so.waitFlag(fCANevent, maskRxCANevent);
    so.clearFlag(fCANevent, maskRxCANevent);

    valor = rxSensor.sns_val;
    
    so.waitSem(sPrint2);
    snsValores[rxSensor.sns_id] = rxSensor.sns_val;
    umbral = snsUmbrales[rxSensor.sns_id];
    so.signalSem(sPrint2);


    activar = false;
    switch(rxSensor.sns_id) {
      case 0: // comprobar si es menor
        activar = valor < umbral;
        break;
      case 1: case 2: // comprobar si es mayor
        activar = valor > umbral;
        break;
      default: break;
    }

    if(activar && (CAN.checkPendingTransmission() != CAN_TXPENDING)) {
      CAN.sendMsgBufNonBlocking(
        CAN_ID_SENSOR, CAN_STDID, sizeof(uint8_t), (INT8U*) &rxSensor.sns_id);
    }
  }
}

void P_AUXILIAR() {
  // Simula recepción de mensajes del bus CAN
  unsigned long nextActivationTick;
  
  nextActivationTick = so.getTick();
  while(true) {
    
    so.waitSem(sPrint2);
    snsValores[0] += random(-50, 50) / 100.0;
    snsValores[0] = snsValores[0] >= 0.0 ? (
      snsValores[0] <= 100.0 ? snsValores[0] : 100.0) : 0.0;
      
    snsValores[1] += random(-50, 50) / 100.0;
    snsValores[1] = snsValores[1] >= 0.0 ? (
      snsValores[1] <= 100.0 ? snsValores[1] : 100.0) : 0.0;
      
    snsValores[2] += random(-50, 50) / 100.0;
    snsValores[2] = snsValores[2] >= 0.0 ? (
      snsValores[2] <= 100.0 ? snsValores[2] : 100.0) : 0.0;
    so.signalSem(sPrint2);
    
    nextActivationTick += PERIOD_AUXILIAR; // Calculate next activation time
    so.delayUntilTick(nextActivationTick);
  }
  
}

void P_CONTROLUART() {
  unsigned long nextActivationTick;

  // Output
  char msg[50];
  double valor[NUM_SENSORES];
  double umbral[NUM_SENSORES];
  char str_valor[10];
  char str_umbral[10];

  // Input
  char c = term.NO_CHAR_RX_UART;
  char prev_c = term.NO_CHAR_RX_UART;
  uint8_t select = 0;
  bool changing = false;
  char input[10] = "";
  uint8_t head = 0;
  
  nextActivationTick = so.getTick();
  while(true) {
    // Print output
    so.waitSem(sPrint2);
    memcpy(valor,  snsValores,  NUM_SENSORES*sizeof(double));
    memcpy(umbral, snsUmbrales, NUM_SENSORES*sizeof(double));
    so.signalSem(sPrint2);

    term.clear();
    term.println("Imprimiendo estado de los sensores...\n");
    term.println(TERM_HEADER);
    term.println(TERM_SEPARATOR);
    for(uint8_t i = 0; i < NUM_SENSORES; i++) {
      // String format
      dtostrf(valor[i],  5, 2, str_valor);
      dtostrf(umbral[i], 5, 2, str_umbral);
      
      sprintf(msg, TERM_FORMAT, i, sensorNombresLargos[i], str_valor, str_umbral, 
        valor[i] >= umbral[i] ? '*' : ' '); 
      term.println(msg);
    }
    term.println("");
    if(!changing) {
      term.println(TERM_HELP);
    } else {
      term.print(TERM_NEWVAL);
      term.print(input);
    }
    // ---------------

    // User input
    prev_c = c;
    c = term.getChar(false);

    if(c != term.NO_CHAR_RX_UART && c != prev_c) {
      if(changing) {
        if(c == '\r') {
          so.waitSem(sPrint2);
          snsUmbrales[select] = atof(input);
          so.signalSem(sPrint2);
          input[0] = "\0";
          head = 0;
          changing = false;
        } else {
          // Send the received character back through the UART
          term.print(c);
          input[head] = c;
          head++;
          input[head] = '\0';
        }
      } else {
        select = c - '0';
        if(0 <= select && select < NUM_SENSORES) {
          changing = true;
        }
      }
    }
    
    nextActivationTick += PERIOD_WRITE_UART; // Calculate next activation time
    so.delayUntilTick(nextActivationTick);
  }
  
}


///////////////////////////////////////////////////////////////////////////////
// Auxiliary functions
///////////////////////////////////////////////////////////////////////////////

// Imprime una cadena con saltos de línea
void lcdPrintNewline(const char *string) {
  char copy[100];
  strcpy(copy, string);
  char *ptr = strchr(copy, '\n');
  if(*ptr == '\0') {
    hib.lcdPrint(copy);
  } else {
    *ptr = '\0';
    ptr++;
    hib.lcdPrint(copy);
    hib.lcdSetCursorSecondLine();
    hib.lcdPrint(ptr);
  }
}


///////////////////////////////////////////////////////////////////////////////
// Main functions
///////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  term.begin(115200);
  
  hib.begin();
  hib.lcdClear();

  so.begin();
  
  // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }

  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  attachInterrupt(0, isrCAN, FALLING);
}

void loop() {
  fCANevent = so.defFlag();
  
  sPrint2 = so.defSem(1);
  sSensorSelect = so.defSem(1);

  so.defTask(E_SNSCONTROL, PRIO_SNSCONTROL);
  so.defTask(P_CONTROLUART, PRIO_UART);
  so.defTask(P_LEERPAD, PRIO_PAD);
  so.defTask(P_PRINT_P2, PRIO_LCD);
  //so.defTask(P_AUXILIAR, 2);

  hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);
  so.enterMultiTaskingEnvironment();
  
}
