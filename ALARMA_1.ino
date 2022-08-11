#include "HIB.h"
#include "timerConfig.h"
#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include "SO.h"
#include <SPI.h>

const int SPI_CS_PIN = 9;

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;

/********************************************
  Declaración de flags y máscaras
*********************************************/
Flag fPADEvent;
Flag fCANEvent;

const unsigned char maskPADEvent = 0x01;
const unsigned char maskCANEvent = 0x02;


/********************************************
  Declaración de semáforos
*********************************************/
Sem sTemp;
Sem sLuz;
Sem sPot;
Sem sCAN;
Sem sAlarma;


/********************************************
  Declaración de mailboxes
*********************************************/
MBox mbPrint1;


/********************************************
  Declaración de variables globales y const.
*********************************************/
#define CAN_ID 1

#define PERIOD_TASK_SNS 20;
#define PERIOD_TASK_CAN_TX 2; 
#define PERIOD_TASK_ZUMBADOR 10;

volatile uint8_t padValue = 0;
volatile float potValue = 0;
volatile float lightValue = 0;
volatile float celsiusValue = 0;
volatile float txSensor [3];

typedef struct canMsg_t {
  uint8_t sns_id;
  float sns_val;
};

uint8_t rxSensor;

bool alarmaAct = false;

/********************************************
  ISR's
*********************************************/

/* keyHook:
  Recoge nuevo valor de teclado y hace set del flag fPADEvent
*/
void keyHook(uint8_t newKey)
{
  padValue = newKey;
  so.setFlag(fPADEvent, maskPADEvent);
}

/* timer5Hook:
  Actualiza el tiempo del SO
*/
void timer5Hook ()
{
  so.updateTime();
}

/* adcHook:
  Recoge un valor del ADC y lo mapea entre 0 y 5
*/
void adcHook(uint16_t newAdcTimerOrAutoDrivenValue)
{
  potValue = ((float) newAdcTimerOrAutoDrivenValue) * 0.004887585;

}

/* isrCAN:
  Recibe una ID de la placa 2 a través de CAN
*/
void isrCAN() {

  char auxSREG;
  auxSREG = SREG;

  // Si hay interrupción por CAN leemos el mensaje
  if (CAN.rxInterrupt()) {

    CAN.readRxMsg();
    // Recogemos valor según ID de CAN y lo guardamos en variable global
    // a continuación, hacemos set del flag fCANEvent
    switch (CAN.getRxMsgId()) {
      case CAN_ID:
        CAN.getRxMsgData((byte*) &rxSensor);
        Serial.println("CAN RECEIVE");
        so.setFlag(fCANEvent, maskCANEvent);
        break;

      default: break;
    }
  }
  SREG = auxSREG;
}

/********************************************
  TAREAS
*********************************************/

/*********** PRIORIDADES DE TAREAS **********/

#define PRIO_TASK_CONTROL 1
#define PRIO_TASK_ACTIVAR_ALARMA 1
#define PRIO_TASK_SNS_LUZ 2
#define PRIO_TASK_SNS_TEMP 2
#define PRIO_TASK_SNS_POT 2
#define PRIO_TASK_ZUMBADOR 1
#define PRIO_TASK_PRINT_P1 1
#define PRIO_TASK_CANTX 1

/*********** DECLARACIÓN DE TAREAS **********/

/*** Tareas de Control de la Alarma ***/

/* E_CONTROL:
  Lee una tecla y activa/desactiva sensorización. También puede apagar
  la alarma si se ha encendido.
*/
void E_CONTROL() {

  bool luzActiva  = false;
  bool tempActiva = false;
  bool potActiva  = false;
  bool alarmaActiva = false;

  while (1)
  {
    // Esperamos a que algún bit del flag del PAD se ponga a '1'
    so.waitFlag(fPADEvent, maskPADEvent);

    // Despejamos flag, para no procesarlo dos veces
    so.clearFlag(fPADEvent, maskPADEvent);
    
    // Miramos que tecla se ha pulsado
    switch (padValue) {

      //Activación de Sensores

      case 0:
        // SNS Luz
        if (!luzActiva)
        {
          Serial.println("LUZ ON");
          so.signalSem(sLuz);
          hib.ledOn(0);
          luzActiva = true;
        } 
        break;

      case 1:
        // SNS Temperatura
        if(!tempActiva){
          Serial.println("TEMP ON");
          so.signalSem(sTemp);
          hib.ledOn(1);
          tempActiva = true;
        } 
        break;

      case 2:
        // SNS Potencia
        if(!potActiva){
          Serial.println("POTENCIA ON");
          so.signalSem(sPot);
          hib.ledOn(2);
          potActiva = true;
        } 
        break;


      // Desactivación de Sensores

      case 3:
        // SNS Luz
        if(luzActiva) {
          Serial.println("LUZ OFF");
          so.waitSem(sLuz);
          hib.ledOff(0);
          luzActiva = false;
        }
        break;

      case 4:
        // SNS Temperatura
        if(tempActiva) {
          Serial.println("TEMP OFF");
          so.waitSem(sTemp);
          hib.ledOff(1);
          tempActiva = false;
        }
        break;

      case 5:
        // SNS Potencia
        if(potActiva) {
          Serial.println("POTENCIA OFF");
          so.waitSem(sPot);
          hib.ledOff(2);
          potActiva = false;
        }
        break;

      // Desactivar Alarma

      case 7:
        // Recogemos valor de la alarma
        so.waitSem(sAlarma);
          alarmaActiva = alarmaAct;
        so.signalSem(sAlarma);

        // Si esta activa, la desactivamos
        if (alarmaActiva){
          E_DESACTIVAR_ALARMA();
          if(luzActiva ) hib.ledOn(0);
          if(tempActiva) hib.ledOn(1);
          if(potActiva ) hib.ledOn(2);
        }
        break;

      /*
      ///// DEBUG: ACTIVAR ALARMA MANUALMENTE /////
      case 10:
        so.waitSem(sAlarma);
        alarmaAct = true;
        so.signalSem(sAlarma);
        Serial.println("ALARMA ACTIVADA !!!");
        break;
      ////////////////////////////////////////////
      */

      default:
        break;
    }
  }
}

/* E_DESACTIVAR_ALARMA:
  Desactiva la alarma y con ella, los periféricos encendidos.
*/
void E_DESACTIVAR_ALARMA() {

  // Cambiamos estado de la alarma a false
  so.waitSem(sAlarma);
    alarmaAct = false;
  so.signalSem(sAlarma);
  Serial.println("ALARMA DESACTIVADA");

  // Desactivamos actuadores

  // Despejamos LCD
  hib.lcdClear();

  // Apagamos LEDS
  for (int i = 0; i < 7 ; i++) {
    hib.ledOff(i);
  }

  // Apagamos 7-SEG
  hib.d7sAllOff(hib.RIGHT_7SEG_DIS);
  
}

/* E_ACTIVAR_ALARMA:
  Espera al flag del bus CAN y cuando ha recibido algo, activa la alarma,
  y envia por Mailbox el número de sensor activo a E_PRINT_P1
*/
void E_ACTIVAR_ALARMA() {

  uint8_t valor;
  bool alarmaChange = false;

  while (1) {
    // Esperamos a que algún bit del flag del PAD se ponga a '1'
    so.waitFlag(fCANEvent, maskCANEvent);

    // Despejamos flag, para no procesarlo dos veces
    so.clearFlag(fCANEvent, maskCANEvent);

    // Si la alarma no esta activa, la activamos y decimos que ha cambiado
    so.waitSem(sAlarma);
    if(!alarmaAct) {
      alarmaChange = true;
      alarmaAct = true;
    }
    so.signalSem(sAlarma);
    Serial.println("ALARMA ACTIVADA POR CAN !!!");

    // Recogemos valor del sensor (ID)
    valor = rxSensor;

    // Si la alarma acaba de cambiar, enviamos el ID por Mailbox
    if(alarmaChange) {
      so.signalMBox(mbPrint1, (byte*) &valor);
      alarmaChange = false;
    }
  }
}

/* E_PRINT_P1:
  Si alarma activa: Encender LEDs, avisar de alarma activa por LCD y indicar la alarma
	en 7-SEG.
*/
void E_PRINT_P1() {

  uint8_t valorId;
  uint8_t * valorIdReal;
  uint8_t number;
  bool alarma;

  while (1) {
    // Esperamos mensaje del Mailbox mbPrint1
    so.waitMBox(mbPrint1, (byte**) &valorIdReal);
    valorId = *valorIdReal;
    
    // Recogemos valor de alarma
    so.waitSem(sAlarma);
    alarma = alarmaAct;
    so.signalSem(sAlarma);

    
    // Si la alarma está activa
    if (alarma) {

      // Anunciamos por LCD
      hib.lcdClear();
      hib.lcdSetCursorFirstLine();
      hib.lcdPrint("ALARMA");
      hib.lcdSetCursorSecondLine();
      hib.lcdPrint("ACTIVA");

      // Encendemos LEDS
      for (int i = 0; i < 7 ; i++) {
        hib.ledOn(i);
      }

      // Encendemos 7-SEG con el número de la ID del Sensor
      hib.d7sAllOff(hib.RIGHT_7SEG_DIS);
      switch (valorId) {

        case 0:
          // SNS Luz
          number = 0;
          hib.d7sPrintDigit((uint8_t) number, hib.RIGHT_7SEG_DIS);
          break;

        case 1:
          // SNS Temperatura
          number = 1;
          hib.d7sPrintDigit((uint8_t) number, hib.RIGHT_7SEG_DIS);
          break;

        case 2:
          // SNS Potencia
          number = 2;
          hib.d7sPrintDigit((uint8_t) number, hib.RIGHT_7SEG_DIS);
          break;

        default:
          break;
      }
    }
  }
}

/*** Tareas de actuadores ***/

/* P_ZUMBADOR:
  Si alarmaAct esta a true, (alarma activada), hace pitar el zumbador.
*/
void P_ZUMBADOR() {
  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();
  bool activa;

  while (1) {
    // Comprobamos que alarma está activa
    so.waitSem(sAlarma);
      activa = alarmaAct;
    so.signalSem(sAlarma);

    // Si está activa, hacemos pitar el Zumbador 100ms a 500Hz
    if (activa)
    {
      hib.buzzPlay(100,500);
    }
    
    // Calculamos próxima activación
    nextActivationTick = nextActivationTick + PERIOD_TASK_ZUMBADOR;
    so.delayUntilTick(nextActivationTick);
  }
}

/*** Tareas de sensorización ***/

/* P_SNS_LUZ:
  Lee el sensor de luz si tiene permiso y lo escribe en variable global 
  para CAN.
*/
void P_SNS_LUZ() {

  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();

  while (1)
  {
    // Desbloqueamos tarea con su semáforo
    so.waitSem(sLuz);
    so.signalSem(sLuz);

    // Leemos valor del sensor de luz
    lightValue = hib.ldrReadAdc(hib.LEFT_LDR_SENS);

    // Escribimos en variable global
    so.waitSem(sCAN);
    txSensor[0] = lightValue;
    so.signalSem(sCAN);

    // Anunciamos valor por serial
    Serial.print("LIGHT Sensor = ");
    Serial.println(txSensor[0]);

    // Calculamos próxima activación
    while(so.getTick() >= nextActivationTick) {
      nextActivationTick = nextActivationTick + PERIOD_TASK_SNS;
    }
    so.delayUntilTick(nextActivationTick);

  }
}

/* P_SNS_LUZ:
  Lee el sensor de temperatura si tiene permiso y lo escribe en variable global 
  para CAN.
*/
void P_SNS_TEMP() {

  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();

  while (1)
  {
    // Desbloqueamos tarea con su semáforo
    so.waitSem(sTemp);
    so.signalSem(sTemp);

    // Leemos valor del sensor de temperatura
    celsiusValue = hib.temReadCelsius(hib.LEFT_TEM_SENS);

    // Escribimos en variable global
    so.waitSem(sCAN);
    txSensor[1] = celsiusValue;
    so.signalSem(sCAN);

    // Anunciamos valor por serial
    Serial.print("TEMP Sensor = ");
    Serial.println(txSensor[1]);

    // Calculamos próxima activación
    while(so.getTick() >= nextActivationTick) {
      nextActivationTick = nextActivationTick + PERIOD_TASK_SNS;
    }
    so.delayUntilTick(nextActivationTick);
  }
}

/* P_SNS_LUZ:
  Lee el sensor de potencia si tiene permiso y lo escribe en variable global 
  para CAN.
*/
void P_SNS_POT() {

  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();

  while (1)
  {
    // Desbloqueamos tarea con su semáforo
    so.waitSem(sPot);
    so.signalSem(sPot);

    // Leemos valor del sensor de potencia mediante ISR
    hib.adcSetTimerDriven(TIMER_TICKS_FOR_250ms, TIMER_PSCALER_FOR_250ms, adcHook);

    // Escribimos en variable global
    so.waitSem(sCAN);
    txSensor[2] = potValue;
    so.signalSem(sCAN);

    // Anunciamos valor por serial
    Serial.print("POWER Sensor = ");
    Serial.println(txSensor[2]);

    // Calculamos próxima activación
    while(so.getTick() >= nextActivationTick) {
      nextActivationTick = nextActivationTick + PERIOD_TASK_SNS;
    }
    so.delayUntilTick(nextActivationTick);
  }
}

/*** Tarea de transmisión por CAN ***/

/* P_CAN_TX:
  Publica el id sensor y valor en el bus CAN si ha cambiado su valor
*/
void P_CAN_TX() {

  volatile float senPrev [3] = {0.0, 0.0, 0.0};
  volatile boolean senPending [3] = {false, false, false};
  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();

  while (1)
  {
    // Miramos si hay algún envio pendiente y actualizamos array de bools
    so.waitSem(sCAN);
    for (uint8_t i = 0; i < 3; i++) {
      if (!senPending[i]) {
        senPending[i] = senPrev[i] != txSensor[i];
      }
      senPrev[i] = txSensor[i];
    }
    so.signalSem(sCAN);

    // Para cada sensor, si hay envio pendiente, lo enviamos a través de CAN
    for (uint8_t i = 0; i < 3; i++) {
      canMsg_t aux = {i, senPrev[i]};
      if ((CAN.checkPendingTransmission() != CAN_TXPENDING) && senPending[i]) {
        CAN.sendMsgBufNonBlocking(CAN_ID, CAN_STDID, sizeof(canMsg_t), (INT8U *)&aux);
        senPending[i] = false;
        Serial.println("CAN SEND");
      }
    }

    // Calculamos próxima activación
    while(so.getTick() >= nextActivationTick) {
      nextActivationTick = nextActivationTick + PERIOD_TASK_CAN_TX;
    }
    so.delayUntilTick(nextActivationTick);
  }

}

/********************************************
  PROGRAMA PRINCIPAL
*********************************************/

void setup() {
  // Inicializaciones
  Serial.begin(115200);
  hib.begin();
  so.begin();

  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(100);
  }

  attachInterrupt(0, isrCAN, FALLING);
}

void loop() {

  // Declaración e inicialización de tareas
  so.defTask(E_CONTROL, PRIO_TASK_CONTROL);
  so.defTask(E_ACTIVAR_ALARMA, PRIO_TASK_ACTIVAR_ALARMA);
  so.defTask(E_PRINT_P1, PRIO_TASK_PRINT_P1);
  so.defTask(P_ZUMBADOR, PRIO_TASK_ZUMBADOR);
  so.defTask(P_SNS_LUZ, PRIO_TASK_SNS_LUZ);
  so.defTask(P_SNS_TEMP, PRIO_TASK_SNS_TEMP);
  so.defTask(P_SNS_POT, PRIO_TASK_SNS_POT);
  so.defTask(P_CAN_TX, PRIO_TASK_CANTX);

  // Declaración e inicialización de semáforos
  sLuz = so.defSem(0);
  sTemp = so.defSem(0);
  sPot = so.defSem(0);
  sCAN = so.defSem(1);
  sAlarma = so.defSem(1);
  
  // Declaración e inicialización de mailboxes
  mbPrint1 = so.defMBox();

  // Declaración e inicialización de flags
  fPADEvent = so.defFlag();
  fCANEvent = so.defFlag();

  // Interrupción KeyPAD
  hib.keySetIntDriven(100, keyHook);

  // Timer 5 para que SO recupere el control en cada TICK
  hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);

  // Entramos en MultiTasking
  so.enterMultiTaskingEnvironment();
}
