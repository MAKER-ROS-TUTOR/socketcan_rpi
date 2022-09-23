/*
    socketcan_rpi.h
  	

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	
*/


#ifndef SOCKETCAN_RPI_H
#define SOCKETCAN_RPI_H

#ifdef __arm__
#include <wiringPi.h>

#endif

#include <stdio.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "socketcan_rpi/socket_can_dfs_rpi.h"
#define MAX_CHAR_IN_MESSAGE 8

#define CAN_MODEL_NUMBER 10000

class SOCKET_CAN
{
    private:
    

    int can;
 
    INT8U   m_nExtFlg;                                                  // Identifier Type
                                                                        // Extended (29 bit) or Standard (11 bit)
    INT32U  m_nID;                                                      // CAN ID
    INT8U   m_nDlc;                                                     // Data Length Code
    INT8U   m_nDta[MAX_CHAR_IN_MESSAGE];                            	// Data array
    INT8U   m_nRtr;                                                     // Remote request flag
    INT8U   m_nfilhit;                                                  // The number of the filter that matched the message
    //INT8U   MCPCS;  (NOT NEEDED, wiringPi already handles CS pin)     // Chip Select pin number 
    INT8U   mcpMode;                                                    // Mode to return to after configurations are performed.
    
    
    INT8U gpio_can_interrupt;
    
   // private:
   private:
      
   
/*********************************************************************************************************
 *  CAN operator function
 *********************************************************************************************************/
    INT8U setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData);        // Set message
    INT8U clearMsg();                                                   // Clear all message to zero
    INT8U readMsg();                                                    // Read message
    INT8U sendMsg();                                                    // Send message
    
  

public:
    SOCKET_CAN(INT8U gpio_can_interrupt);
    ~SOCKET_CAN();

    int16_t motor1ReadPos();
   
    INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData);               // Initilize Mask(s)
    INT8U init_Mask(INT8U num, INT32U ulData);                          // Initilize Mask(s)
    INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData);               // Initilize Filter(s)
    INT8U init_Filt(INT8U num, INT32U ulData);                          // Initilize Filter(s)

    INT8U setMode(INT8U opMode);           

    INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);      // Send message to transmit buffer
    INT8U sendMsgBuf(INT32U id, INT8U len, INT8U *buf);                 // Send message to transmit buffer
    INT8U readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U *buf);   // Read message from receive buffer
    INT8U readMsgBuf(INT32U *id, INT8U *len, INT8U *buf);
    
    INT8U readMsgBuf2(INT32U *id, INT8U *len, INT8U *buf); 

    INT8U checkReceive(void);                                           // Check for received data
    INT8U checkError(void);                                             // Check for errors
    INT8U getError(void);                                               // Check for errors
    INT8U errorCountRX(void);                                           // Get error count
    INT8U errorCountTX(void);                                           // Get error count
    INT8U enOneShotTX(void);                                            // Enable one-shot transmission
    INT8U disOneShotTX(void);                                           // Disable one-shot transmission
    
 
    bool setupInterruptGpio();
    bool setupCan();
    bool canReadData();
    
};

#endif
