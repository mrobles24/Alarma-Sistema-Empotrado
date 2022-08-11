/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:Loovee
  2014-1-16
  
  Contributor: 
  
  Cory J. Fowler
  Latonita
  Woodward1
  Mehtajaghvi
  BykeBlast
  TheRo0T
  Tsipizic
  ralfEdmund
  Nathancheek
  BlueAndi
  Adlerweb
  Btetz
  Hurvajs
  Manuel Barranco
  
  The MIT License (MIT)

  Copyright (c) 2013 Seeed Technology Inc.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_uib_dfs.h"

#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{
public:

	MCP_CAN(INT8U _CS);


	/**
	 * @brief Initializa CAN controller
	 *
	 * @param [in] speedset Bit rate, specify this parameter by using the corresponding macros of mcp_can_uib_dfs 
	 * @param [in] mode Mode of operation [MODE_LOOPBACK, MODE_NORMAL]
	 * @param [in] enaRxInt Enable/disable CAN reception interrupt (true, false)
	 * @param [in] enaTxInt Enable/disable CAN transmission interrupt (true, false)
	 *
	 * @return CAN_OK if the initialization succeded, CAN_FAILINIT otherwise
	 *
	 * @details	Controller is driven into configuration mode while configuring the mask
	 *		Configures controller to use both RX buffers with rollover
	 *		Masks and filters are reset, but filters are configured to be applied to EXID
	 *		Enables/disables filtering depending on the value of flag DEBUG_RXANY in mcp_can_uib_dfs
	 */

	INT8U begin(INT8U speedset, INT8U mode, bool enaRxInt, bool enaTxInt);


	/**************************************************************************/
	/** @name Transmission ****************************************************/
	/**************************************************************************/
	//@{
		
	/**
	 * @brief Functions to request a frame transmission
	 * 
	 * @param [in] id CAN identifier of the frame.
	 * @param [in] ext Type of CAN identifier, i.e. standard or extended, [CAN_STDID, CAN_EXTID]
	 * @param [in] rtr Remote frame request [0, 1]
	 * @param [in] len Lengh of the data field in bytes [0, 8]
	 * @param [out] *buf Pointer to list of bytes to be transmitted
	 *
	 * @details The two first functions are blocking (actively wait until frame
	 *          is successfully transmitted), whereas the third one is not
	 *          blocking.
	 *
	 *          Recall that we have modified these functions to use only TXB0
	 *
	 * @see checkPendingTransmission
	 */

	INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U rtr, INT8U len, INT8U *buf);
   	INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);
 
	INT8U sendMsgBufNonBlocking(INT32U id, INT8U ext, INT8U len, INT8U *buf);  
   	
	
	/**
	 * @brief Returns whether or not there is a requested frame to be transmitted (pending tx request)
	 *
	 * @return INT8U that indicates whether or not there is a transmission request pending.
	 *			CAN_OK: no transmission request is pending
	 *			CAN_TXPENDING: a transmission request is still pending
	 *
	 * @details Recall that we have restricted this library to just use TXB0 for
	 *          transmitting. Thus, this function is programmed accordingly.
	 *
	 * @see sendMsgBuf
	 */

	INT8U checkPendingTransmission();


	/**
	 * @brief Check if there is a pending transmission interrupt
	 */

   	bool txInterrupt();
	
	/**
	 * @brief Crear transmission flag interrupt
	 */

	void clearTxInterrupt();
	
	/**
	 * @brief returns the number of highest-prioritized pending CAN interrupt
	 *        Currently it only considers transmission and reception CAN interrupts
	 */

   	INT8U readIntSource();
	
	//@}
	
	/**************************************************************************/
	/** @name Reception *******************************************************/
	/**************************************************************************/
	//@{

	/**
	 * @brief Configure Mak
	 *
	 * @param [in] num Mask number. Mask 0 is associated to RXB0; Mask 1 is associated to RXB1
	 * @param [in] ext (1: apply assuming extended id; not 1: apply assuming standard id)
	 * @param [in] ulData Mask value.
	 *		If extended id is assumed, the 29 less significant bits of Mask apply to the whole ext. id. of 29 bits
	 *		If standard id is assumed, the 16 less significatn bits of Mask apply to the 2 first bytes of the DATA FIELD
	 *
	 * @details controller is driven into configuration mode while configuring the mask
	 */

	INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData);           


	/**
	 * @brief Configure Filter 
	 * @param [in] num Filter number 0 to 5.
	 *		Filters 0 and 1 are associated to RXB0; filters 2-5 are associated to RXB1
	 *		If rollover is enabled, filters 0 and 1 are associated to RXB1 when the rollover occurs
	 *
	 * @param [in] ext (1: apply assuming extended id; not 1: apply assuming standard id)
	 *
	 * @param [in] ulData Filter value.
	 *		If extended id is assumed, the 29 less significant bits of Mask apply to the whole ext. id. of 29 bits
	 *		If standard id is assumed, the 16 less significatn bits of Mask apply to the 2 first bytes of the DATA FIELD
	 *
	 * @details controller is driven into configuration mode while configuring the filter
	 */
	
	INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData);
	
	/**
	 * @brief  Read the message allocated in the RX buffer(s), if any; then release it (and clear its interrupt falag)
	 * 
	 * @return INT8U value that indicates the result, as defined in mcp_can_uib_dfs.h:
	 *		(1) CAN_OK: a messages has been successfully read from the RX buffers
	 *		(2) CAN_NOMSG: both RX buffers are free, no message has been read
	 *
	 * @details
	 *
	 * When a frame is received in a given RX buffer, RXBi, the reception interrupt flag
	 * of that buffer, RXiIF, is set in the CAN controller. IT IS IMPORTANT TO NOTE that is is done
	 * even if reception interrupts are disable in the CAN controller
	 *
 	 * A RXBi is considered as occupied (it will not receive a new frame) until its
	 * recepction interrupt flag, RXiIF is not cleared.
	 *
	 * IMPORTANT: Each one of the two next read functions clears the RXiIF of the RXBi buffer it reads the frame from.
	 * In this way, the function fulfills two objectives:
	 *	(1) It frees RXBi, so that a new frame can be received in that buffer. 
	 *	(2) It "acknowledges" to the CAN controller the reception event that generated the reception
	 *	    interrupt (in case the reception interrupt is enabled for that buffer).
	 *
	 * Note that, as "checkReceive" function does (see bellow), these "read" functions first checks if RXB0
	 * is occupied and, in case it is not, it checks RXB1. This means that the function gives more priority
	 * to read from RXB0 than from RXB1. 
	 *
	 * @see checkReceive
	 */
	 
	INT8U readRxMsg();


	/**
	 * @brief Returns the value of the CAN extended id of the received message readed by last call to readRxMsg 
	 * 
	 * @return INTU32 CAN extended id
	 *
	 * @see readRxMsg, checkReceive
	 */
	 
	INT32U getRxMsgId();


	/**
	 * @brief Returns the value of the DLC of the received message readed by last call to readRxMsg 
	 * 
	 * @param [out] INTU8 value of the DLC
	 *
	 * @see readRxMsg, checkReceive
	 */

	INT8U getRxMsgDlc();


	/**
	 * @brief Reads the payload of the received message readed by last call to readRxMsg 
	 * 
	 * @param [in] buf Pointer to the list of bytes to be filled with the bytes that constitute the payload
	 *             of the received message readed by last call to readRxMsg
	 *
	 * @see readRxMsg, checkReceive
	 */

	void getRxMsgData(INT8U buf[]);


	/**
	 * @brief Return whether or not there is a new received frame
	 *
	 * @details It first checks RXB0; if no new message is received there, then it checks RXB1
	 *
	 * @return INTU8 value that indicates the result, as defined in mcp_can_uib_dfs.h:
	 *		(1) CAN_MSGAVAIL: a new message has been receiveda in any of the RX buffers (both are free)
	 *		(2) CAN_NOMSG: a new message has been received in an RX buffer (at least one of them is occupied)
	 *
	 * @see readMsgBuf, readMsgBufID
	 */

   	INT8U checkReceive(void);
	
   	INT8U checkError(void);
   	INT8U isRemoteRequest(void);
    INT8U isExtendedFrame(void);
	
	
	/**
	 * @brief Check if there is a pending reception interrupt
	 */

   	bool rxInterrupt();
	
	//@}


	/**************************************************************************/
	/** @name Additional functions ********************************************/
	/**************************************************************************/
	
	/* read mcp2515's register      */
	INT8U mcp2515_readRegister(const INT8U address);

	/* set bit of one register      */
	void mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data);


private:
   
	// Use separated message member vars for tx and rx
	// message member vars for rx as original
	// add new message member vars for tx (and uptdate cpp accordingly)
	
    INT8U   m_nExtFlg;                   /* identifier xxxID             */
                                         /* either extended (the 29 LSB) */
                                         /* or standard (the 11 LSB)     */
    INT32U  m_nID;                       /* can id                       */
    INT8U   m_nDlc;                      /* data length:                 */
    INT8U   m_nDta[MAX_CHAR_IN_MESSAGE]; /* data                         */
    INT8U   m_nRtr;                      /* rtr                          */
    INT8U   m_nfilhit;


    INT8U   m_nTxExtFlg;                   /* identifier xxxID             */
                                           /* either extended (the 29 LSB) */
                                           /* or standard (the 11 LSB)     */
    INT32U  m_nTxID;                       /* can id                       */
    INT8U   m_nTxDlc;                      /* data length:                 */
    INT8U   m_nTxDta[MAX_CHAR_IN_MESSAGE]; /* data                         */
    INT8U   m_nTxRtr;                      /* rtr                          */
    INT8U   m_nTxfilhit;

    INT8U   SPICS;
    bool    rxIntEnabled;
    bool    txIntEnabled;

private:
	/***************************************************************************
	** mcp2515 driver function 
	***************************************************************************/

    void mcp2515_reset(void);

    //INT8U mcp2515_readRegister(const INT8U address);
    
    void mcp2515_readRegisterS(const INT8U address, INT8U values[], const INT8U n);
    void mcp2515_setRegister(const INT8U address, const INT8U value);

    void mcp2515_setRegisterS(const INT8U address, const INT8U values[], const INT8U n);
    
    void mcp2515_initCANBuffers(void);
    
   // void mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data);

    INT8U mcp2515_readStatus(void);
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);
    INT8U mcp2515_configRate(const INT8U canSpeed);

	// UIB modified to add mode, enaRxInt and enaTxInt
    INT8U mcp2515_init(const INT8U canSpeed, INT8U mode, bool enaRxInt, bool enaTxInt);

    void mcp2515_write_id(const INT8U mcp_addr, const INT8U ext, const INT32U id);
    void mcp2515_read_id(const INT8U mcp_addr, INT8U* ext, INT32U* id);

    void mcp2515_write_canMsg(const INT8U buffer_sidh_addr );
    void mcp2515_read_canMsg(const INT8U buffer_sidh_addr);
	
    void mcp2515_start_transmit(const INT8U mcp_addr);
    INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n);


	/***************************************************************************
	** CAN operator function
	***************************************************************************/

    INT8U setMsg(INT32U id, INT8U ext, INT8U len, INT8U rtr, INT8U *pData);
    INT8U setMsg(INT32U id, INT8U ext, INT8U len, INT8U *pData);
	
	// UIB modified/added to dif between rx/tx message member vars
    INT8U clearRxMsg();
    INT8U clearTxMsg();

	// UIB modified
    INT8U sendMsg(byte blocking);

	// UIB: shift to private area
    INT8U readMsgBuf(INT8U *len, INT8U *buf);
    INT8U readMsgBufID(INT32U *ID, INT8U *len, INT8U *buf);
    INT32U getCanId(void);
};

#endif
