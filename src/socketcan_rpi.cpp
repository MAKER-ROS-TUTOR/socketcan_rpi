/*
    socketcan_rpi.cpp
  	

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

#include "socketcan_rpi/socketcan_rpi.h"

SOCKET_CAN::SOCKET_CAN(INT8U gpio_can_interrupt)
{
     this->gpio_can_interrupt = gpio_can_interrupt;
     can = -1;	
}
SOCKET_CAN::~SOCKET_CAN()
{
   if( can > 0) close(can);
}
bool SOCKET_CAN::setupCan()
{
#ifdef __arm__
    int ret;
    struct sockaddr_can addr;
    struct ifreq ifr;
     //1.Create socket
     // before use socket ,system have /sys/bus/spi/devices/spi0.0/net
     // and sudo ip link set can0 up type can bitrate 500000
     
    can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can < 0) {
        printf("--> CAN socket PF_CAN failed");
        return false;
    }
    
    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(can, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        printf("--> CAN ioctl failed");
        return false;
    }
    
    //3.Bind the socket to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(can, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        printf("--> CAN bind failed");
        return false;
    }
    printf("----> Successful cocketCAN::CAN BUS created! <-----");	
    return true;
#else
    printf("Can't use CAN0 on non-ARM processor");
    return false;
#endif
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            Set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
INT8U SOCKET_CAN::setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData)
{
    int i = 0;
    m_nID     = id;
    m_nRtr    = rtr;
    m_nExtFlg = ext;
    m_nDlc    = len;
    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
        m_nDta[i] = *(pData+i);
	
    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            Set all messages to zero
*********************************************************************************************************/
INT8U SOCKET_CAN::clearMsg()
{
    m_nID       = 0;
    m_nDlc      = 0;
    m_nExtFlg   = 0;
    m_nRtr      = 0;
    m_nfilhit   = 0;
    for(int i = 0; i<m_nDlc; i++ )
      m_nDta[i] = 0x00;

    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            Send message
*********************************************************************************************************/
INT8U SOCKET_CAN::sendMsg()
{
    int nbytes,i = 0;
    INT8U res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));

    frame.can_id  = m_nID;
    frame.can_dlc = m_nDlc;
    
    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
             frame.data[i] = m_nDta[i]; 
     
    nbytes = write(can, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
           printf("CAN BUS Send Error frame[0]!\r\n"); 
           return CAN_SENDMSGTIMEOUT;  
      }
    return CAN_OK;
}
/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Read message
*********************************************************************************************************/
INT8U SOCKET_CAN::readMsg()
{
    INT8U stat, res;
    int i=0, nbytes;
    struct can_frame frame;

    nbytes = read(can,&frame, sizeof(struct can_frame));
    
    if( nbytes < 0){
        res = CAN_NOMSG;
        return res;
    }
    if( nbytes < sizeof( struct can_frame)){
        res = CAN_NOMSG;
        return res;
    }
    else{

        m_nID  = frame.can_id; 
        m_nDlc = frame.can_dlc;
        m_nRtr = 0;

        for( i=0; i< frame.can_dlc;i++)
             m_nDta[i] = frame.data[i];  

        res = CAN_OK; 
    }


    return res;
}
/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
INT8U SOCKET_CAN::sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf)
{
    INT8U res;
	
    setMsg(id, 0, ext, len, buf);
    res = sendMsg();
    
    return res;
}

INT8U SOCKET_CAN::sendMsgBuf(INT32U id, INT8U len, INT8U *buf)
{
    INT8U ext = 0, rtr = 0;
    INT8U res;
    
    if((id & 0x80000000) == 0x80000000)
        ext = 1;
 
    if((id & 0x40000000) == 0x40000000)
        rtr = 1;
        
    setMsg(id, rtr, ext, len, buf);
    res = sendMsg();
    
    return res;
}
/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
INT8U SOCKET_CAN::readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U buf[])
{
   // if(readMsg() == CAN_NOMSG)
//	return CAN_NOMSG;

 //   int  nbytes;
 //   struct can_frame frame;

  //  nbytes = read(can,&frame, sizeof(struct can_frame));

	
    *id  = m_nID;
    *len = m_nDlc;
    *ext = m_nExtFlg;
    for(int i = 0; i<m_nDlc; i++)
        buf[i] = m_nDta[i];

    return CAN_OK;
}



INT8U SOCKET_CAN::readMsgBuf(INT32U *id, INT8U *len, INT8U buf[])
{
  //  if(readMsg() == CAN_NOMSG)
  //	return CAN_NOMSG;

    int  nbytes;
    struct can_frame frame;

    nbytes = read(can,&frame, sizeof(struct can_frame));
    
    if( frame.can_dlc == 4 )
    {    

     m_nID  = frame.can_id; 
     m_nDlc = frame.can_dlc; 

  	
    *id  = m_nID;
    *len = m_nDlc;
    
    for(int i = 0; i<frame.can_dlc; i++)
        buf[i] = frame.data[i];
  
    return CAN_OK;
   }
   return CAN_NOMSG;
}

INT8U SOCKET_CAN::readMsgBuf2(INT32U *id, INT8U *len, INT8U buf[])
{
  //  if(readMsg() == CAN_NOMSG)
//	return CAN_NOMSG;

      int  nbytes;
    struct can_frame frame;

    nbytes = read(can,&frame, sizeof(struct can_frame));

	
    


    if (m_nExtFlg)
        m_nID |= 0x80000000;

    if (m_nRtr)
        m_nID |= 0x40000000;
	
    *id  = m_nID;
    *len = nbytes;
    
    for(int i = 0; i<m_nDlc; i++)
        buf[i] = m_nDta[i];

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           setupInterruptGpio
** Descriptions:            Setups interrupt GPIO pin as input on Raspberry Pi (using wiringPi)
*********************************************************************************************************/
bool SOCKET_CAN::setupInterruptGpio()
{
#ifdef __arm__
    int result = wiringPiSetupGpio();
    if (!result) {
        printf("Gpio started\n");
    }
    else {
        printf("Gpio startup fail\n");
        return false;
    }
    
    pinMode(gpio_can_interrupt, INPUT);
    nanosleep((const struct timespec[]){{0, 500000L}}, NULL);
    return true;
#else
    printf("Can't use GPIO on non-ARM processor");
    return false;
#endif
}
/*********************************************************************************************************
** Function name:           canReadData
** Descriptions:            Checks GPIO interrupt pin to see if data is available (using wiringPi)
*********************************************************************************************************/
bool SOCKET_CAN::canReadData()
{
        
    int i, nbytes;
    struct can_frame frame;

    nbytes = read(can,&frame, sizeof(struct can_frame));
    
    if( (nbytes < 0) || ( nbytes < sizeof( struct can_frame)) ){
       
        return false;
    }

    
        m_nID  = frame.can_id; 
        m_nDlc = frame.can_dlc;
        m_nRtr = 0;

        for( i=0; i< frame.can_dlc;i++)
             m_nDta[i] = frame.data[i];  
  

    
   return true;
}
/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
INT8U SOCKET_CAN::init_Mask(INT8U num, INT8U ext, INT32U ulData)
{
    INT8U res = MCP2515_OK;

    printf("Starting to Set Mask!\r\n");
    printf("Setting Mask Successful!\r\n");

    return res;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
INT8U SOCKET_CAN::init_Mask(INT8U num, INT32U ulData)
{
    INT8U res = MCP2515_OK;
    INT8U ext = 0;

    printf("Starting to Set Mask!\r\n");
    printf("Setting Mask Successful!\r\n");

    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
INT8U SOCKET_CAN::init_Filt(INT8U num, INT8U ext, INT32U ulData)
{
    INT8U res = MCP2515_OK;

    printf("Starting to Set Filter!\r\n");
    
    printf("Setting Filter Successfull!\r\n");

    
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
INT8U SOCKET_CAN::init_Filt(INT8U num, INT32U ulData)
{
    INT8U res = MCP2515_OK;
    INT8U ext = 0;

    printf("Starting to Set Filter!\r\n");
    

    printf("Setting Filter Successfull!\r\n");

    
    return res;
}

/*********************************************************************************************************
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
INT8U SOCKET_CAN::setMode(const INT8U opMode)
{
  //  mcpMode = opMode;
    return 0;//mcp2515_setCANCTRL_Mode(mcpMode);
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            Public function, Checks for received data.  (Used if not using the interrupt output)
*********************************************************************************************************/
INT8U SOCKET_CAN::checkReceive(void)
{
    INT8U res;
    //res = mcp2515_readStatus();                                         /* RXnIF in Bit 1 and 0         */
    if ( res & MCP_STAT_RXIF_MASK )
        return CAN_MSGAVAIL;
    else 
        return CAN_NOMSG;
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            Public function, Returns error register data.
*********************************************************************************************************/
INT8U SOCKET_CAN::checkError(void)
{
    INT8U eflg ;//= mcp2515_readRegister(MCP_EFLG);

    if ( eflg & MCP_EFLG_ERRORMASK ) 
        return CAN_CTRLERROR;
    else
        return CAN_OK;
}

/*********************************************************************************************************
** Function name:           getError
** Descriptions:            Returns error register value.
*********************************************************************************************************/
INT8U SOCKET_CAN::getError(void)
{
    return 0;//mcp2515_readRegister(MCP_EFLG);
}

/*********************************************************************************************************
** Function name:           mcp2515_errorCountRX
** Descriptions:            Returns REC register value
*********************************************************************************************************/
INT8U SOCKET_CAN::errorCountRX(void)                             
{
    return 0;//mcp2515_readRegister(MCP_REC);
}

/*********************************************************************************************************
** Function name:           mcp2515_errorCountTX
** Descriptions:            Returns TEC register value
*********************************************************************************************************/
INT8U SOCKET_CAN::errorCountTX(void)                             
{
    return 0;//mcp2515_readRegister(MCP_TEC);
}

/*********************************************************************************************************
** Function name:           mcp2515_enOneShotTX
** Descriptions:            Enables one shot transmission mode
*********************************************************************************************************/
INT8U SOCKET_CAN::enOneShotTX(void)                             
{
    //mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
    //if((mcp2515_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != MODE_ONESHOT)
	//    return CAN_FAIL;
   // else
	    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           mcp2515_disOneShotTX
** Descriptions:            Disables one shot transmission mode
*********************************************************************************************************/
INT8U SOCKET_CAN::disOneShotTX(void)                             
{
   // mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
   // if((mcp2515_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != 0)
   //     return CAN_FAIL;
   // else
        return CAN_OK;
}

int16_t SOCKET_CAN::motor1ReadPos()
{
     return 100;
}
