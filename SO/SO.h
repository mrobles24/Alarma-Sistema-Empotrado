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


#ifndef SO_H
#define SO_H

#include <Arduino.h>

typedef unsigned char Sem;
typedef unsigned char MBox;
typedef unsigned char Flag;

#define MAXTASK 8 // Maximum tasks --> NOTE: up to 254 (if enough memory)
#define MAXSEM 5  // Maximum semaphore --> NOTE: up to 255 (if enough memory)s
#define MAXMB 4	  // Maximum mailboxe --> NOTE: up to 255 (if enough memory)s
#define MAXFLAGS 4 // Maximum flag --> NOTE: up to 255 (if enough memory)s

	// Stack size of each task (in bytes)
	// Revise this value according to the maximum space
	// a task will need for its local vars and calls
#define STACKSIZE 256

// Set SO_debug to 0 or 1, so as to disable/enable sch printout for debugging
#define SO_debug 0

class SO
{

public:

	//__attribute__ ( ( naked) );
	//	__attribute__( ( naked, optimize("-O0") ) );
	
	
	// constructor
	SO();

	// begin function of SO class
	void begin();

	// function to declare tasks
	void defTask(void (*pTask)(void), int prty);


	// **** TIME (TICK) primitives ****

	// function to call inside the timer hook
	void updateTime();

	// Get SO current time in ticks
	unsigned long getTick(); // MAN

	// Autosuspend for interval of time (in ticks)
	void delayTicks(unsigned long Time_2);

	// Autosuspend until a given instant of time (tick)
	void delayUntilTick(unsigned long Time_2);


	// **** Semaphore primitives ****

	// define and initalize semaphore
	Sem defSem(int initialValue);

	// Wait Semaphore
	void waitSem(unsigned char S);
	// Signal Semaphore
	void signalSem(unsigned char S);


	// **** Mailbox primitives ****

	// define and initialize mailbox
	MBox defMBox();

	// Wait Mailbox --> NOTE: task gets blocked until message available
	void waitMBox(unsigned char MB, byte **pMsg_in);

	// Signal Mailbox --> NOTE: task does not get blocked
	void signalMBox(unsigned char MB, byte *pMsg_out);


	// **** Flag primitives ****
	
	// define and initialize flag
	Flag defFlag();
	
	// Wait Flag
	void waitFlag(unsigned char F, const unsigned char mask);

	// Set Flag
	void setFlag(unsigned char F, const unsigned char mask);

	// Clear Flag --> NOTE: it does not call to sch
	void clearFlag(unsigned char F, const unsigned char mask);

	// Read flag current value --> NOTE: it does not call to sch
	unsigned char readFlag(unsigned char F);


	// **** Dispatcher / scheduler call ***

	// start multitasking --> NOTE: program never returns from this function
	void enterMultiTaskingEnvironment();

	// Save Context
	// If you want to directly call sch from a TASK, invoke this function directly
	void SaveContext() __attribute__((naked));


private:
	enum taskStatusType
	{
		ACTIVE = 0,
		BLOCKED = 1,
		TIMESUSP = 2,
	};

	/*****************
		SEMAFORO
	******************/
	// Semaphore control block
	struct SCB
	{
		unsigned int S;		   // Semaphore value
		unsigned int tail = 0; // Number of blocked (queued) processes
	};

	/*****************
		MAILBOX
	******************/
	// Mailbox control block
	struct MCB 
	{
		byte * pTxMsg; // pointer to the data to be 'transmitted'
		bool newMsg = false; // indicates when the Mbox has a new message
 		unsigned char entailedTasks = 0; // Number of tasks queued in the tail 
	};

	/*****************
		FLAG
	******************/
	// Flag control block
	struct FCB
	{
		unsigned char value = 0x00;
				// Number of tasks queued in the tail.
				// It allows to save unnecessary call to sch
				// if there is no task waiting
				// (consider eliminate this attribute to simplify code).
 		unsigned char entailedTasks = 0; 
				// Indicates whether or not the flag has been set (signaled)
				// and thus, that the flag is pending to be processed by the sch.
				// Since setFlag can be invoked from ISR,
				// setFlag may find the sch occupied.
				// For the sch to not miss the treatment of a setFlag in such a scenario,
				// we can follow 2 strategies:
				// (1) process all flags everytime the sch is invoked
				// (2) let setFlag register/mark that the flag has been signaled and then,
				//     everytime the the sch is invoked, only process the
				//     flags that are marked as signaled
				// We prefer the 2nd option, since it is more efficient,
				// this is why we have this attribute here.
		bool signaled = false; 
	};

	/*****************
		TCB
	******************/
	// Task control block
	struct TCB
	{
		unsigned char id;
		byte pila[STACKSIZE];
		byte SP_L; // LSB of the task SP
		byte SP_H; // MSB of the task SP
		unsigned char priority;
		unsigned char Status;
		unsigned long Time;
		unsigned char S;	// ID of the associated semaphore

			// Datat associated to MBox where task is blocked.
			// PENDING: pack these 2 vars into a "tcbMboxDescriptor" struct
		unsigned char MB;	// ID of the associated Mailbox
		byte ** pRxMsg; 	// pointer to the space where 'transmitted' data is to be stored

		unsigned char F;	// ID of the Flag the task is waiting for
		unsigned char Fmask;	// Mask associated to the Flag the task is waiting for
	};

	static SO *pSO;

	volatile struct TCB TCB_pool[MAXTASK];	   // Array of tcb's
	volatile struct SCB SCB_pool[MAXSEM];      // Array of scb's
	volatile struct MCB MCB_pool[MAXMB];	   // Array of mbcb's
	volatile struct FCB FCB_pool[MAXFLAGS];    // Array of task control block MAN


	// scheduler
	void sch();
};

#endif
