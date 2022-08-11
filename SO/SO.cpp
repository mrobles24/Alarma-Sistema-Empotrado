/*********************************************************************************
	Authors:
		Adrián Moreno Coca,
		Izar Maria Prieto Castorina,
		Manuel Barranco

	Institution:
		Departament de Ciències Matemàtiques i Informàtica (DMI)
		Escola Politècnica Superior (EPS)
		Universitat de les Illes Balears (UIB)

	Last update: 2021, Apr, 17th

/*********************************************************************************/

#include "SO.h"

#define noSemaphore 255
#define noMailbox 255
#define noFlag 255

// Context Switch values
#define INSIDE_SCH 15
#define OUTSIDE_SCH 0

////////////////////////////////////////////////////////////////////////////////
// DEFINITION AND INITIALIZATION OF STATIC VARIABLES OF HIB CLASS
////////////////////////////////////////////////////////////////////////////////

// Define and initialize pointer to the active HIB object
SO *SO::pSO = NULL;

////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
////////////////////////////////////////////////////////////////////////////////

volatile byte auxSP_L;   // (auxiliar var): LSB of the SP of the task when it is put into exe
volatile byte auxSP_H;   // (auxiliar var): MSB of the SP of the task when it is put into exe
                         // Pointer to TCB.SP_L of the task being executed
volatile byte pExeSP_LL; // LSB of that pointer
volatile byte pExeSP_LH; // MSB of that pointer
                         // Pointer to TCB.SP_H of the task being executed
volatile byte pExeSP_HL; // LSB of that pointer
volatile byte pExeSP_HH; // MSB of that pointer

volatile unsigned char indexActiveTasks; // index to choose the current task

volatile byte CntxtSwitch;            // var to know if we are in sch
volatile unsigned char tcbPoolIndex;  // Amount of Task Control Blocks saved
volatile unsigned char scbPoolIndex;  // Amount of Semaphores declared
volatile unsigned char mbcbPoolIndex; // Amount of Mailboxes delcared

volatile unsigned char fcbPoolIndex; // Amount of declared Flags

volatile unsigned long int Time = 0;  //Var to increment ISR tick --> unsigned long (32) bits --> more time available
volatile unsigned int TotalTask = 0;
volatile unsigned char signaledSemaphore = noSemaphore; // pointer to the semaphore that has done the signal
volatile unsigned char signaledMailbox = noMailbox;     // pointer to the mailbox that has done the signal
//volatile unsigned char signaledFlag = noFlag;     // pointer to the mailbox that has done the signal

// Type and variable to track the reason why the sch is invoked

enum schCallReasonType
{
  SCH_NOCALL = 0,
  SCH_BY_START_MULTITASKING = 1,
  SCH_BY_UPDATE_TIME = 2,
  SCH_BY_DELAY_TIME = 3,
  SCH_BY_SEM_WAIT = 4,
  SCH_BY_SEM_SIGNAL = 5,
  SCH_BY_MB_WAIT = 6,
  SCH_BY_MB_SIGNAL = 7,
  SCH_BY_FLAG_WAIT = 8,
  SCH_BY_FLAG_SIGNAL = 9
};

volatile schCallReasonType schCallReason = SCH_BY_START_MULTITASKING;


////////////////////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
////////////////////////////////////////////////////////////////////////////////
void idleTask(void);

////////////////////////////////////////////////////////////////////////////////
// CLASS FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

SO::SO()
{
  pSO = this;
}

/******************************* Begin***************************************
*****************************************************************************/
void SO::begin()
{
  CntxtSwitch = INSIDE_SCH;     // protect scheduler , in setup();
  tcbPoolIndex = 0;     // Index to tcb
  scbPoolIndex = 0;     // Index to scb
  mbcbPoolIndex = 0;    // Index to mcbc

  fcbPoolIndex = 0;	// Index to fcb

  defTask(idleTask, 0); // define idle task at the begining
}

/******************************* Deftask*************************************
*****************************************************************************/
void SO::defTask(void (*pTask)(void), int prty)
{
#if SO_debug == 1
  Serial.print(" DEFtASK:");
  Serial.print(tcbPoolIndex);
#endif

  uint16_t indexStack;
  uint16_t i;

  //Vars to store pc of TaskXi
  byte pc_H;
  byte pc_M;
  byte pc_L;

  if (pTask != idleTask)
  {
    //COUNT NUMBER OF TOTAL TASk
    TotalTask++;
  }

  //ASIGN ID TO THE TASK
  TCB_pool[tcbPoolIndex].id = tcbPoolIndex;

  //ASIGN PRIORITY TO THE TASK
  TCB_pool[tcbPoolIndex].priority = prty;

  //SET TASK AS A ACTIVE (READY) TASK
  TCB_pool[tcbPoolIndex].Status = ACTIVE;

  //THE POINTER TO SEMAPHORES POINT TO NULL
  TCB_pool[tcbPoolIndex].S = noSemaphore;

  //THE POINTER TO MAILBOXES POINT TO NULL
  TCB_pool[tcbPoolIndex].MB = noMailbox;

  //THE POINTER TO FLAGS POINT TO NULL
  TCB_pool[tcbPoolIndex].F = noFlag;

  //PROGRAM DIRECTION OF TASKxi
  pc_H = (byte)((((unsigned long int)pTask) & 0xFF0000) >> 16);
  pc_M = (byte)((((unsigned long int)pTask) & 0x00FF00) >> 8);
  pc_L = (byte)((((unsigned long int)pTask) & 0x0000FF));

  //INIT STACK TASK
  indexStack = STACKSIZE - 1;
  for (i = 1; i <= 36; i++)
  {
#if SO_debug == 1
    //Serial.print("DEFTASK: la direccion del elemento de pila num ");Serial.print(indexStack);Serial.print(" es: ");
    //Serial.println((unsigned long)&(TCB_pool[tcbPoolIndex].pila[indexStack]));
#endif

    if (indexStack == (STACKSIZE - 1))
    {
      TCB_pool[tcbPoolIndex].pila[indexStack] = pc_L; // PC LSB
    }
    if (indexStack == (STACKSIZE - 2))
    {
      TCB_pool[tcbPoolIndex].pila[indexStack] = pc_M;
    }
    if (indexStack == (STACKSIZE - 3))
    {
      TCB_pool[tcbPoolIndex].pila[indexStack] = pc_H; //PC MSB
    }
    if (indexStack < (STACKSIZE - 3))
    {
      TCB_pool[tcbPoolIndex].pila[indexStack] = 0; // INICIALIZE REGISTERS
    }

    indexStack--;
  }

  TCB_pool[tcbPoolIndex].SP_H = (byte)((((unsigned int)&(TCB_pool[tcbPoolIndex].pila[STACKSIZE - 37])) & 0xFF00) >> 8);
  TCB_pool[tcbPoolIndex].SP_L = (byte)((((unsigned int)&(TCB_pool[tcbPoolIndex].pila[STACKSIZE - 37])) & 0x00FF));

  /*Serial.println(" ");
    Serial.print("DEFtASK: el valor del SP es ");
    Serial.print("SP_H =");
    Serial.print(TCB_pool[tcbPoolIndex].SP_H);
    Serial.print("  SP_L =");
    Serial.print(TCB_pool[tcbPoolIndex].SP_L);*/

  tcbPoolIndex++;
}

/******************************* updateTime **********************************
*****************************************************************************/
void SO::updateTime()
{
  /*
  #if SO_debug == 1
    Serial.println("TIMEUPDATE: TIMEUPDATE");
  #endif
  */

	// NOTE that we ALWAYS need to increase Time when updateTime is invoked,
	// even if sch or any other primitive that reads Time is already executing!
	// Thus, the sch and these other primitives must protect Time with cli, then latch it then use it.
	//
	// Since Time is protecte with cli in sch and these other primitives,
	// here we can modify it without any problem.
  Time++; // Increase TICK

  // Since updateTime can be invoked from an ISR when sch is already executing;
  // we cannot take it from granted that CntxtSwitch is free here
  // --> we need to check it before occuping it. 
  // However, this check/occuping is not atomic --> thus we need to disable interrupts.
  asm volatile("cli \n\t");

  // If we are in a non protected zone we must go to scheduler
  if (CntxtSwitch == OUTSIDE_SCH)
  {
    CntxtSwitch = INSIDE_SCH;
  /*
  #if SO_debug == 1
      Serial.println("TIMEUPDATE: he encontrado SCH libre.");
  #endif
  */

    asm volatile("sei \n\t");

    schCallReason = SCH_BY_UPDATE_TIME;
    SaveContext();
  }
  /*
  #if SO_debug == 1
      Serial.println("TIMEUPDATE: he encontrado scheduler ocupado.");
  #endif
  */
  // If we are in a protected zone we must return without a scheduler call
  asm volatile("sei \n\t");
}


/******************************* enterMultiTaskingEnvironment ***************
*****************************************************************************/

void SO::enterMultiTaskingEnvironment()
{
	sch();
}


/******************************* Scheduler***********************************
*****************************************************************************/
void SO::sch()
{
#if SO_debug == 1
  Serial.println(" SCH:SCHEDULER ");
  switch (schCallReason)
  {

  case SCH_BY_START_MULTITASKING:
    Serial.print("SCH: called by START MULTITASKING. ");
    Serial.print("Time = ");
    Serial.println(Time);
    break;

  case SCH_BY_UPDATE_TIME:
    Serial.print("SCH: called by UPDATE TIME. ");
    Serial.print("Time = ");
    Serial.println(Time);
    break;

  case SCH_BY_DELAY_TIME:
    Serial.println("SCH: called by DELAY TIME. ");
    break;

  case SCH_BY_SEM_WAIT:
    Serial.println("SCH: called by SEM WAIT");
    break;

  case SCH_BY_SEM_SIGNAL:
    Serial.println("SCH: called by SEM SIGNAL");
    break;

  case SCH_BY_MB_WAIT:
    Serial.println("SCH: called by MAILBOX WAIT");
    break;

  case SCH_BY_MB_SIGNAL:
    Serial.println("SCH: called by MAILBOX SIGNAL");
    break;

  case SCH_BY_FLAG_WAIT:
    Serial.println("SCH: called by FLAG WAIT");
    break;

  case SCH_BY_FLAG_SIGNAL:
    Serial.println("SCH: called by FLAG SIGNAL");
    break;

  default:
    Serial.println("SCH: called by UNKNOWN REASON!!");
    break;
  }
#endif

  unsigned long int latchedTime;
  unsigned char MaxPriority = 0;
  unsigned char i;
  unsigned char NoActiveTask = 0;
  unsigned char indexUnblockTask;
  unsigned char indexedFlag; // Index aux var for visiting flags.
  unsigned char indexedFlagValue;
  bool indexedFlagSignaled;



  // SEMAPHORE SIGNAL HANDLING
  // If a semaphore has been signalled --> and its tail is not empty, the blocked task with the highest priority has to be unbloked
  if (schCallReason == SCH_BY_SEM_SIGNAL)
  {

    MaxPriority = 0;
    indexUnblockTask = 0;
    for (i = 1; i <= TotalTask; i++)
    {

      //If the task:
      // (1) is blocked in a semaphore AND
      // (2) is blocked in the semaphore that has done the signal AND
      // (3) its priority is higher than the priority of the other tasks blocked at this semaphore

      if ((TCB_pool[i].Status == BLOCKED) && (TCB_pool[i].S == signaledSemaphore) && (TCB_pool[i].priority > MaxPriority))
      {
        MaxPriority = TCB_pool[i].priority;
        indexUnblockTask = i;
      }
    }

    // If we have found a task block in the semaphore --> unblock it
    if (indexUnblockTask > 0)
    {
#if SO_debug == 1
      Serial.print(" SCH: UNBLOCK BY SEM SIGNAL, task id = ");
      Serial.println(TCB_pool[indexUnblockTask].id);
#endif
      TCB_pool[indexUnblockTask].Status = ACTIVE;       // unblock the task
      TCB_pool[indexUnblockTask].S = noSemaphore; // reset the reference to the semaphore
    }

    signaledSemaphore = noSemaphore; // reset the reference to the signaled semaphore
  }                                  // End SEM_SIGNAL if




  // MAILBOX SIGNAL HANDLING
  // If a mailbox has been signalled --> and its tail is not empty, the blocked task with the highest priority has to be unbloked
  if (schCallReason == SCH_BY_MB_SIGNAL)
  {
    MaxPriority = 0;
    indexUnblockTask = 0;
    for (i = 1; i <= TotalTask; i++)
    {

      //If the task:
      // (1) is blocked in a mailbox AND
      // (2) is blocked in the mailbox that has done the signal AND
      // (3) its priority is higher than the priority of the other tasks blocked at this mailbox

      if ((TCB_pool[i].Status == BLOCKED) && (TCB_pool[i].MB == signaledMailbox) && (TCB_pool[i].priority > MaxPriority))
      {
        MaxPriority = TCB_pool[i].priority;
        indexUnblockTask = i;
      }
    }

    // If we have found a task block in the mailbox --> unblock it
    if (indexUnblockTask > 0)
    {
#if SO_debug == 1
      Serial.print(" SCH: UNBLOCK BY MB SIGNAL, task id = ");
      Serial.println(TCB_pool[indexUnblockTask].id);
#endif
      TCB_pool[indexUnblockTask].Status = ACTIVE;       // unblock the task

      (*(TCB_pool[indexUnblockTask].pRxMsg)) = MCB_pool[signaledMailbox].pTxMsg; // tranfer message: write it (the pointer) into pointer var of the task
      TCB_pool[indexUnblockTask].MB = noMailbox; // reset the reference to the mailbox

     // Serial.print(" SCH: pointer value = ");
     // Serial.println((unsigned int)MCB_pool[signaledMailbox].pTxMsg);

      MCB_pool[signaledMailbox].newMsg = false; // by simply setting this bool to false, we do not need to "erase" the message
      MCB_pool[signaledMailbox].entailedTasks--;
    }

    signaledMailbox = noMailbox; // reset the reference to the signaled mailbox
  }                              // End MB_SIGNAL if




  // FLAGS HANDLING
  // For each flag: check whether or not it has been signalled
  // If so, then unblock ALL tasks for which the condition of that flag is satisfied
  for (indexedFlag = 0; indexedFlag < fcbPoolIndex; indexedFlag++)
  {
	  // If setFlag is invoked from interrupt
	  // then the following flag attributes may change while sch is executing: value, signaled
	  // Thus, protect and latch/write them; then use latched values if necessary 
    asm volatile("cli \n\t");
    indexedFlagValue = FCB_pool[indexedFlag].value;
    indexedFlagSignaled = FCB_pool[indexedFlag].signaled;
    FCB_pool[indexedFlag].signaled = false;
    asm volatile("sei \n\t");

    // If the flag has been signaled, process it
    // NOTE:
    //   If the flag is signaled again from an interrupt (after interrupts are re-enabled), it will be processed next time the sch is invoked.
    //   Moreover: If a new flag value is signaled from a new interrupt before the last flag value has been processed, then that last value is overwritten;
    //   and thus the corresponding event is 'missed'
    if(indexedFlagSignaled)
    {
      for (i = 1; i <= TotalTask; i++)
      {
        //If the task:
        // (1) is blocked for the flag that has been signaled (set) AND
        // (2) the waiting condition is satisfied: if any bit of the flag indicated by the associated mask is '1'
        //
        // then unblock it

        if ( (TCB_pool[i].Status == BLOCKED) && (TCB_pool[i].F == indexedFlag) && ((indexedFlagValue & TCB_pool[i].Fmask) != 0) )
        {
          #if SO_debug == 1
          Serial.print(" SCH: UNBLOCK BY FLAG SIGNAL, task id = ");
          Serial.print(TCB_pool[i].id);
          Serial.print(" flag id = ");
          Serial.println(indexedFlag);
          #endif

        TCB_pool[i].Status = ACTIVE;        // unblock the task
        TCB_pool[i].F = noFlag; // Task no longer waiting for a flag value

	  // setFlag reads flag attribute entailedTasks
	  // However, it does so only if the sch is not ocupied.
	  // Thus, here we can modify it without any problem
        FCB_pool[indexedFlag].entailedTasks--;

        } // end if flag condition satisfied
      } // end task loop
    } // end if indexed flag signaled

  } // End loop FLAG HANDLING





  // Wake up (activate) taks that are autosuspended by time, if needed
  for (i = 1; i <= TotalTask; i++)
  {
#if SO_debug == 1
    //Serial.print("SCH: el estado de la tarea "); Serial.print(i); Serial.print(" es: ");
    //Serial.println(TCB_pool[i].Status);
    //Serial.print("y su instante de activación es ");
    //Serial.println(TCB_pool[i].Time);
#endif

	  // If updateTime is invoked from interrupt
	  // then variable Time may change while sch is executing.
	  // Thus, protect it, read it, latch it, then use latched Time 
    asm volatile("cli \n\t");
    latchedTime = Time;
    asm volatile("sei \n\t");

    // If the task has supersed its period tick, it has to be activated
    if ((TCB_pool[i].Status == TIMESUSP) && (TCB_pool[i].Time <= latchedTime))
    {

#if SO_debug == 1
      Serial.print(" SCH: ACTIVATE FROM TIME task id = ");
      Serial.println(TCB_pool[i].id);
#endif
      TCB_pool[i].Status = ACTIVE;
      TCB_pool[i].Time = 0;
    } // End Activation check if
  }   // End AutoSuspended tasks for loop




  // From all active task, select the one with the highest priority (to put into execution)
  MaxPriority = 0;
  for (i = 1; i <= TotalTask; i++)
  {
    if ((TCB_pool[i].Status == ACTIVE) && (TCB_pool[i].priority > MaxPriority))
    {
      NoActiveTask = 1; // there is at least one active task
      MaxPriority = TCB_pool[i].priority;
      indexActiveTasks = i;
    }
  }

  // If there is no active task, select idleTask to put into execution
  if (NoActiveTask == 0)
  {
    indexActiveTasks = 0;
  }

  // Reset reason why sch was called
  schCallReason = SCH_NOCALL;

#if SO_debug == 1
  Serial.print("SCH: la tarea a poner en exe es id = ");
  Serial.println(TCB_pool[indexActiveTasks].id);
#endif

#if SO_debug == 1
  for (i = 1; i <= TotalTask; i++)
  {
    Serial.print("SCH: los campos de la tarea id = ");
    Serial.print(TCB_pool[i].id);
    Serial.print(" son: ");
    Serial.print(" priority = ");
    Serial.print(TCB_pool[i].priority);
    Serial.print(" Status = ");
    Serial.print(TCB_pool[i].Status);
    Serial.print(" Time = ");
    Serial.print(TCB_pool[i].Time);
    Serial.print(" S = ");
    Serial.println(TCB_pool[i].S);
    Serial.print(" MB = ");
    Serial.println(TCB_pool[i].MB);
    Serial.print(" F = ");
    Serial.println(TCB_pool[i].F);
  }
#endif

  asm volatile("cli \n\t");
  // Update the variables that are used to point to TCB.SP_L
  // of the task that is goint to be put into execution
  pExeSP_LL = (byte)(((unsigned int)&(TCB_pool[indexActiveTasks].SP_L)) & 0x00FF);
  pExeSP_LH = (byte)((((unsigned int)&(TCB_pool[indexActiveTasks].SP_L)) & 0xFF00) >> 8);
  // Update the variables that are used to point to TCB.SP_H
  // of the task that is goint to be put into execution
  pExeSP_HL = (byte)(((unsigned int)&(TCB_pool[indexActiveTasks].SP_H)) & 0x00FF);
  pExeSP_HH = (byte)((((unsigned int)&(TCB_pool[indexActiveTasks].SP_H)) & 0xFF00) >> 8);
  // Write into the corresponding auxiliar variables the LSB and MSB of the SP
  // of the task that is going to be put into execution
  // We just need these two auxiliary variables to be able to store into the X register
  // these just-mentioned LSB and MSB (and to transfer them to the CPU's SP_L SP_H).
  auxSP_L = TCB_pool[indexActiveTasks].SP_L;
  auxSP_H = TCB_pool[indexActiveTasks].SP_H;
  asm volatile("sei \n\t");

  asm volatile( // go to the higher priority active task
      "lds r26, auxSP_L \n\t"
      "lds r27, auxSP_H \n\t"
      "out __SP_L__, r26 \n\t"
      "out __SP_H__, r27 \n\t"
      "pop r31 \n\t"
      "pop r30 \n\t"
      "pop r29 \n\t"
      "pop r28 \n\t"
      "pop r27 \n\t"
      "pop r26 \n\t"
      "pop r25 \n\t"
      "pop r24 \n\t"
      "pop r23 \n\t"
      "pop r22 \n\t"
      "pop r21 \n\t"
      "pop r20 \n\t"
      "pop r19 \n\t"
      "pop r18 \n\t"
      "pop r17 \n\t"
      "pop r16 \n\t"
      "pop r15 \n\t"
      "pop r14 \n\t"
      "pop r13 \n\t"
      "pop r12 \n\t"
      "pop r11 \n\t"
      "pop r10 \n\t"
      "pop r9 \n\t"
      "pop r8 \n\t"
      "pop r7 \n\t"
      "pop r6 \n\t"
      "pop r5 \n\t"
      "pop r4 \n\t"
      "pop r3 \n\t"
      "pop r2 \n\t"
      "pop r1 \n\t"
      "pop r0 \n\t"
      "out __SREG__, r0 \n\t"
      "pop r0 \n\t");
  CntxtSwitch = OUTSIDE_SCH;

  asm volatile(
      "reti \n\t"); // STACK POINTER IS IN a[127], AFTER GET PC (3 BYTES) // reti is the same as sei + ret
}



/******************************* Get SO current time in ticks ****************
*****************************************************************************/
unsigned long SO::getTick()
{
  unsigned long latchedTime;

  	  // If updateTime is invoked from interrupt
	  // then variable Time may change while getTick is executing.
	  // Thus, protect it, read it, latch it, then use latched Time 
  asm volatile("cli \n\t");
    latchedTime = Time;
  asm volatile("sei \n\t");

  return latchedTime;
}


/******************************* Autosuspend for interval of time (in ticks) *
*****************************************************************************/

void SO::delayTicks(unsigned long Time_2) //Subroutine to sleep(synchronize) the tasks
{
  unsigned long latchedTime;

  CntxtSwitch = INSIDE_SCH; // From hereon we are inside the scheduler

          // HOWEVER:
  	  // If updateTime is invoked from interrupt
	  // then variable Time may change while getTick is executing.
	  // Thus, protect it, read it, latch it, then use latched Time 
  asm volatile("cli \n\t");
    latchedTime = Time;
  asm volatile("sei \n\t");

  TCB_pool[indexActiveTasks].Time = Time_2 + latchedTime; // Set the instant of time (tick) at which the task must be activated
  TCB_pool[indexActiveTasks].Status = TIMESUSP;    // asleep the task
  schCallReason = SCH_BY_DELAY_TIME;
  SaveContext();
}



/******************************* Autosuspend until a given instant of time (tick)
*****************************************************************************/

void SO::delayUntilTick(unsigned long Time_2)
{
  CntxtSwitch = INSIDE_SCH;                        // From hereon we are inside the scheduler
  TCB_pool[indexActiveTasks].Time = Time_2;        // Set the instant of time (tick) at which the task must be activated
  TCB_pool[indexActiveTasks].Status = TIMESUSP;    // asleep the task
  schCallReason = SCH_BY_DELAY_TIME;
  SaveContext();
}

/******************************* Wait (Semaphore)*****************************
*****************************************************************************/
void SO::waitSem(unsigned char S) // Subroutine to do the wait instruction
{
  CntxtSwitch = INSIDE_SCH; // Since this instruction we are inside the scheduler
  if (SCB_pool[S].S == 0)
  {                                              // if S = 0
    TCB_pool[indexActiveTasks].Status = BLOCKED; // BLOCK THE TASK
    TCB_pool[indexActiveTasks].S = S;            // save in tcb of the task the pointer that points to the semaphore that has asleep the task
    SCB_pool[S].tail++;                          // increment to know that the tail is not empty (!= 0)
    schCallReason = SCH_BY_SEM_WAIT;
    SaveContext(); // Save the context and go to scheduler to decide
  }
  else
  {
    SCB_pool[S].S = 0; // if s!=0 the task must execute the rest of the code
    CntxtSwitch = OUTSIDE_SCH;   // WE DONT GO TO SCH
  }
}

/******************************* Signal (Semaphore) **************************
*****************************************************************************/

void SO::signalSem(unsigned char S) // Subroutine to do the signal instruction
{
  CntxtSwitch = INSIDE_SCH; // Since this instruction we are inside the scheduler
  if (SCB_pool[S].tail > 0)
  {                        // if the tail is not empty --> one or some task are waiting
    SCB_pool[S].tail--;    //Since one task (higher priority) is going to be unblock, the tail decrement in one --> othe option is to do this once we have unblock the task in the scheduler
    signaledSemaphore = S; //The pointer that points the semaphore that is going to unblock a task after do a signal
    schCallReason = SCH_BY_SEM_SIGNAL;
    SaveContext(); // Save the context and go to scheduler to decide
  }
  else
  {
    SCB_pool[S].S = 1; // if the tail is empty the task must execute the rest of the code --> there is not a task waiting to be unblock
    CntxtSwitch = OUTSIDE_SCH;   // WE DONT GO TO SCH
  }
}

/******************************* Wait (Mailbox)*******************************
*****************************************************************************/
void SO::waitMBox(unsigned char MB, byte **pMsg_in) // Subroutine to do the wait instruction
{
  CntxtSwitch = INSIDE_SCH; // Since this instruction we are inside the scheduler
  if (MCB_pool[MB].newMsg)
  {
     (*pMsg_in) = MCB_pool[MB].pTxMsg;
     MCB_pool[MB].newMsg = false;
     CntxtSwitch = OUTSIDE_SCH;
  }
  else
  {
    TCB_pool[indexActiveTasks].Status = BLOCKED; // BLOCK THE TASK
    TCB_pool[indexActiveTasks].MB = MB;          // save in tcb of the task the pointer that points to the mailbox that has asleep the task
    TCB_pool[indexActiveTasks].pRxMsg = pMsg_in; // save pointer to point to rxvar of task  
    MCB_pool[MB].entailedTasks++;		 // increase number of entailed tasks            
    schCallReason = SCH_BY_MB_WAIT;
    SaveContext(); // Save the context and go to scheduler to decide
  }
}


/******************************* Signal (Mailbox) ****************************
*****************************************************************************/

void SO::signalMBox(unsigned char MB, byte *pMsg_out) // Subroutine to do the signal instruction
{
  CntxtSwitch = INSIDE_SCH;          // Since this instruction we are inside the scheduler
  //MCB_pool[MB].newMsg = true;
  //MCB_pool[MB].pTxMsg = pMsg_out;

  if (MCB_pool[MB].entailedTasks > 0)
  { 
   // MCB_pool[MB].tail--;      // Since one task (higher priority) is going to be unblock, the tail decrement in one --> othe option is to do this once we have unblock the task in the scheduler


    signaledMailbox = MB; //The pointer that points the semaphore that is going to unblock a task after do a signal
    MCB_pool[MB].pTxMsg = pMsg_out; // Insert msg into MBox, so that unblock task can read it when sch unblocks it
    schCallReason = SCH_BY_MB_SIGNAL;
    SaveContext(); // Save the context and go to scheduler to decide
  }
  else
  {
    MCB_pool[MB].newMsg = true;
    MCB_pool[MB].pTxMsg = pMsg_out; // Insert msg into MBox
    CntxtSwitch = OUTSIDE_SCH;      // WE DONT GO TO SCH
  }
}



/********************************* Wait (Flag) ******************************
*****************************************************************************/
void SO::waitFlag(unsigned char F, const unsigned char mask)
{
  unsigned char latchedValue;

  CntxtSwitch = INSIDE_SCH; // From hereon we are inside the scheduler

          // HOWEVER:
  	  // If setFlag is invoked from interrupt
	  // then the flag's value may change while readFlag is executing.
	  // Thus, protect it, read it, latch it, then use latched value
  asm volatile("cli \n\t");
  latchedValue = FCB_pool[F].value;
  asm volatile("sei \n\t");

  if ( (latchedValue & mask) != 0 )
  {
     //Serial.println("waitFlag: la condicion se cumple, NO me bloqueo");
     CntxtSwitch = OUTSIDE_SCH;
  }
  else
  {
    //Serial.println("waitFlag: la condicion no se cumple, SI me bloqueo");
    TCB_pool[indexActiveTasks].Status = BLOCKED; // BLOCK THE TASK

    TCB_pool[indexActiveTasks].F = F;          // save in tcb of the task the pointer that points to the flag
    TCB_pool[indexActiveTasks].Fmask = mask;  
   
    FCB_pool[F].entailedTasks++;		 // increase number of entailed tasks

    //Serial.println("waitFlag: voy a llamar a sch");

    schCallReason = SCH_BY_FLAG_WAIT;
    SaveContext(); // Save the context and go to scheduler to decide
  }
}



/***************************** Set/Signal (Flag) *****************************
*****************************************************************************/
void SO::setFlag(unsigned char F, const unsigned char mask)
{
  //Serial.println("setFlag: me han invocado.");

  // BE CAREFUL HERE: unlike all the other sincro/comm primitives, setFlag (like updateTime) is allowed to be invoked from an ISR 	

	// NOTE that we ALWAYS need to set the flag and mark it as signaled when setFlag is invoked,
	// even if sch or any other primitive that reads/writes the flag is already executing!
	// Thus, the sch and these other primitives must protect
	// the flag attributes we modifify here (value and signaled)
	// with cli, then latch/write them, then use latched values if necessary.
	//
	// The sch protects flag attributes's value and signaled with cli.
	// Thus here we can modify them without any problem.
  FCB_pool[F].value = FCB_pool[F].value | mask;
  FCB_pool[F].signaled = true; // indicate that the flag has been signaled (set)

  	// Since setFlag can be invoked from an ISR when sch is already executing;
	// we cannot take it from granted that CntxtSwitch is free here
	// --> we need to check it before occuping it. 
	// However, this check/occuping is not atomic --> thus we need to disable interrupts.
  asm volatile("cli \n\t");
  if (CntxtSwitch == OUTSIDE_SCH)
  {

    CntxtSwitch = INSIDE_SCH;
    
    asm volatile("sei \n\t");

    if (FCB_pool[F].entailedTasks > 0)
    {
      //Serial.println("setFlag: hay al menos una tareas esperando por el flag, voy a invocar al scheduler.");    
      schCallReason = SCH_BY_FLAG_SIGNAL;
      SaveContext(); // Save the context and go to scheduler to decide
    }
    else
    {
      // if sch is occupied, we cannot invoke it
      // --> sch will treat signaled flag next time it is invoked for any reason
      CntxtSwitch = OUTSIDE_SCH;
    }
  }
  asm volatile("sei \n\t");
}



 

/**************************** Clear/Reset (Flag) *****************************
*****************************************************************************/
void SO::clearFlag(unsigned char F, const unsigned char mask)
{
  CntxtSwitch = INSIDE_SCH; // From hereon we are inside the scheduler

          // HOWEVER:
  	  // If setFlag is invoked from interrupt
	  // then the flag's value may change while clearFlag is executing.
	  // Thus, protect it.
  asm volatile("cli \n\t");
  FCB_pool[F].value = FCB_pool[F].value & (~mask);
  asm volatile("sei\n\t");

  CntxtSwitch = OUTSIDE_SCH; // WE DONT GO TO SCH
}


/**************************** Read (Flag) *****************************
*****************************************************************************/
// Read flag current value --> NOTE: it does not call to sch
unsigned char SO::readFlag(unsigned char F)
{
  unsigned char latchedValue;

  CntxtSwitch = INSIDE_SCH; // From hereon we are inside the scheduler

          // HOWEVER:
  	  // If setFlag is invoked from interrupt
	  // then the flag's value may change while readFlag is executing.
	  // Thus, protect it, read it, latch it, then use latched value
  asm volatile("cli \n\t");
  latchedValue = FCB_pool[F].value;
  asm volatile("sei\n\t");

  CntxtSwitch = OUTSIDE_SCH; // WE DONT GO TO SCH

  return latchedValue;
}


/****************************** Save context *********************************
*****************************************************************************/
void SO::SaveContext()
{
  // subroutine to save the context before go to scheduler
  // RETURN DIRECCTION IS IN STACK, THEN WE SAVE REGISTERS AND FLAGS
  asm volatile(
      "push r0 \n\t"
      "in r0, __SREG__ \n\t"
      "push r0 \n\t"
      "push r1 \n\t"
      "push r2 \n\t"
      "push r3 \n\t"
      "push r4 \n\t"
      "push r5 \n\t"
      "push r6 \n\t"
      "push r7 \n\t"
      "push r8 \n\t"
      "push r9 \n\t"
      "push r10 \n\t"
      "push r11 \n\t"
      "push r12 \n\t"
      "push r13 \n\t"
      "push r14 \n\t"
      "push r15 \n\t"
      "push r16 \n\t"
      "push r17 \n\t"
      "push r18 \n\t"
      "push r19 \n\t"
      "push r20 \n\t"
      "push r21 \n\t"
      "push r22 \n\t"
      "push r23 \n\t"
      "push r24 \n\t"
      "push r25 \n\t"
      "push r26 \n\t"
      "push r27 \n\t"
      "push r28 \n\t"
      "push r29 \n\t"
      "push r30 \n\t"
      "push r31 \n\t"
      "lds r30, pExeSP_LL \n\t" // tcb.sp = sp
      "lds r31, pExeSP_LH \n\t"
      "in r0,__SP_L__ \n\t"
      "st z,r0 \n\t"
      "lds r30, pExeSP_HL \n\t"
      "lds r31, pExeSP_HH \n\t"
      "in r0,__SP_H__ \n\t"
      "st z,r0 \n\t");

  sch();
}

/********************************** defSem ***********************************
*****************************************************************************/
Sem SO::defSem(int initialValue)
{
#if SO_debug == 1
  Serial.print(" DefSem:");
  Serial.print(scbPoolIndex);
#endif

  unsigned char index;
  SCB_pool[scbPoolIndex].S = initialValue;
  index = scbPoolIndex;
  scbPoolIndex++;
  return (index);
}

/******************************* defMBox *************************************
*****************************************************************************/
Sem SO::defMBox()
{
#if SO_debug == 1
  Serial.print(" DefMBox:");
  Serial.print(mbcbPoolIndex);
#endif

  unsigned char index;
  index = mbcbPoolIndex;
  mbcbPoolIndex++;

  return (index);
}



/******************************* defFlag *************************************
*****************************************************************************/
Sem SO::defFlag()
{
#if SO_debug == 1
  Serial.print(" DefFlag:");
  Serial.print(fcbPoolIndex);
#endif

  unsigned char index;
  index = fcbPoolIndex;
  fcbPoolIndex++;

  return (index);
}


////////////////////////////////////////////////////////////////////////////////
// AUXILIARY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void idleTask(void)
{
  int i = 1;
  while (1)
  {
    i = (i + 1) % 2;
    //Serial.println("IDLE: I am running.");
    /*switch(i){
     case 0:  Serial.print( " idle1 " );break;
     case 1: Serial.print( " idle2 " );break;
    default : break;
    }*/
    delay(500);
  }
}
