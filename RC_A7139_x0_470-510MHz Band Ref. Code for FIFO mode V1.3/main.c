/*********************************************************************
**  Device:     A7139
**  File:       main.c
**  Target:     Winbond W77LE58
**  Tools:      ICE
**  Updated:    2017-03-17
**  Description:
**  This file is a sample code for your reference.
**
**  Copyright (C) 2017 AMICCOM Corp.
**
*********************************************************************/
#include "define.h"
#include "w77le58.h"
#include "A7139reg.h"
#include "A7139config.h"
#include "Uti.h"

/*********************************************************************
**  I/O Declaration
*********************************************************************/
#define SCS         P1_0        //SPI SCS
#define SCK         P1_1        //SPI SCK
#define SDIO        P1_2        //SPI SDIO
#define CKO         P1_3        //CKO
#define GIO1        P1_4        //GIO1
#define GIO2        P1_5        //GIO2

/*********************************************************************
**  Constant Declaration
*********************************************************************/
#define TIMEOUT     100         //100ms
#define t0hrel      1000        //1ms

/*********************************************************************
**  Global Variable Declaration
*********************************************************************/
Uint8   data    timer;
Uint8   data    TimeoutFlag;
Uint16  idata   RxCnt;
Uint32  idata   Err_ByteCnt;
Uint32  idata   Err_BitCnt;
Uint16  idata   TimerCnt0;
Uint8   data    *Uartptr;
Uint8   data    UartSendCnt;
Uint8   data    CmdBuf[11];
Uint8   idata   tmpbuf[64];
Uint8	data	fb;
bit		data	fb_ok;

const Uint8 code BitCount_Tab[16]={0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4};
const Uint8 code ID_Tab[8]={0x34,0x75,0xC5,0x8C,0xC7,0x33,0x45,0xE7};   //ID code
const Uint8 code PN9_Tab[]=
{   0xFF,0x83,0xDF,0x17,0x32,0x09,0x4E,0xD1,
    0xE7,0xCD,0x8A,0x91,0xC6,0xD5,0xC4,0xC4,
    0x40,0x21,0x18,0x4E,0x55,0x86,0xF4,0xDC,
    0x8A,0x15,0xA7,0xEC,0x92,0xDF,0x93,0x53,
    0x30,0x18,0xCA,0x34,0xBF,0xA2,0xC7,0x59,
    0x67,0x8F,0xBA,0x0D,0x6D,0xD8,0x2D,0x7D,
    0x54,0x0A,0x57,0x97,0x70,0x39,0xD2,0x7A,
    0xEA,0x24,0x33,0x85,0xED,0x9A,0x1D,0xE0
};  // This table are 64bytes PN9 pseudo random code.

/*********************************************************************
**  function Declaration
*********************************************************************/
void Timer0ISR(void);
void UART0Isr(void);
void InitTimer0(void);
void InitUART0(void);
void A7139_POR(void);
Uint8 InitRF(void);
Uint8 A7139_Config(void);
Uint8 A7139_WriteID(void);
Uint8 A7139_Cal(void);
void StrobeCMD(Uint8);
void ByteSend(Uint8);
Uint8 ByteRead(void);
void A7139_WriteReg(Uint8, Uint16);
Uint16 A7139_ReadReg(Uint8);
void A7139_WritePageA(Uint8, Uint16);
Uint16 A7139_ReadPageA(Uint8);
void A7139_WritePageB(Uint8, Uint16);
Uint16 A7139_ReadPageB(Uint8);
void A7139_WriteFIFO(void);
void RxPacket(void);
void Err_State(void);
void entry_deep_sleep_mode(void);
void wake_up_from_deep_sleep_mode(void);
void RCOSC_Cal(void);
void WOR_enable_by_preamble(void);
void WOR_enable_by_sync(void);
void WOR_enable_by_carrier(void);
void WOT_enable(void);
void TWOR_enable(void);
void RSSI_measurement(void);
void FIFO_extension_TX(void);
void FIFO_extension_RX(void);
void FIFO_extension_Infinite_TX(void);
void FIFO_extension_Infinite_RX(void);
void Auto_Resend(void);
void Auto_ACK(void);

/*********************************************************************
* main loop
*********************************************************************/
void main(void)
{
    //initsw
    PMR |= 0x01;    //set DME0

    //initHW
    P0 = 0xFF;
    P1 = 0xFF;
    P2 = 0xFF;
    P3 = 0xFF;
    P4 = 0x0F;

    InitTimer0();
    InitUART0();
    TR0=1;  //Timer0 on
    EA=1;   //enable interrupt
    
	A7139_POR();	//power on only

    while(1)
    {
        if(InitRF()) //init RF
        {
            entry_deep_sleep_mode();
            Delay1ms(2);
            wake_up_from_deep_sleep_mode();
        }
        else
        {
            break;
        }
    }

    if((P4 & 0x04)==0x04)   //if P4.2=1, master
    {
        while(1)
        {
            A7139_WriteFIFO();  //write data to TX FIFO
            StrobeCMD(CMD_TX);
            Delay10us(1);
            while(GIO2);        //wait transmit completed
            StrobeCMD(CMD_RX);
            Delay10us(1);

            timer=0;
            TimeoutFlag=0;
            while((GIO2==1)&&(TimeoutFlag==0));     //wait receive completed
            if(TimeoutFlag)
            {
                StrobeCMD(CMD_STBY);
            }
            else
            {
                RxPacket();
                Delay10ms(10);
            }
        }
    }
    else    //if P4.2=0, slave
    {
        RxCnt = 0;
        Err_ByteCnt = 0;
        Err_BitCnt = 0;

        while(1)
        {
            StrobeCMD(CMD_RX);
            Delay10us(1);
            while(GIO2);        //wait receive completed
            RxPacket();

            A7139_WriteFIFO();  //write data to TX FIFO
            StrobeCMD(CMD_TX);
            Delay10us(1);
            while(GIO2);        //wait transmit completed

            Delay10ms(9);
        }
    }
}

/************************************************************************
**  Timer0ISR
************************************************************************/
void Timer0ISR(void) interrupt 1
{
    TH0 = (65536-t0hrel)>>8;    //Reload Timer0 high byte, low byte
    TL0 = 65536-t0hrel;

    timer++;
    if(timer >= TIMEOUT)
    {
        TimeoutFlag=1;
    }

    TimerCnt0++;
    if(TimerCnt0 == 500)
    {
        TimerCnt0=0;
        CmdBuf[0]=0xF1;

        memcpy(&CmdBuf[1], &RxCnt, 2);
        memcpy(&CmdBuf[3], &Err_ByteCnt, 4);
        memcpy(&CmdBuf[7], &Err_BitCnt, 4);

        UartSendCnt=11;
        Uartptr=&CmdBuf[0];
        SBUF=CmdBuf[0];
    }
}

/************************************************************************
**  UART0ISR
************************************************************************/
void UART0Isr(void) interrupt 4 using 3
{
    if(TI==1)
    {
        TI=0;
        UartSendCnt--;
        if(UartSendCnt !=0)
        {
            Uartptr++;
            SBUF = *Uartptr;
        }
    }
}

/************************************************************************
**  init Timer0
************************************************************************/
void InitTimer0(void)
{
    TR0 = 0;
    TMOD=(TMOD & 0xF0)|0x01;    //Timer0 mode=1
    TH0 = (65536-t0hrel)>>8;    //setup Timer0 high byte, low byte
    TL0 = 65536-t0hrel;
    TF0 = 0;                    //Clear any pending Timer0 interrupts
    ET0 = 1;                    //Enable Timer0 interrupt
}

/************************************************************************
**  Init UART0
************************************************************************/
void InitUART0(void)
{
    TH1 = 0xFD;                 //BaudRate 9600;
    TL1 = 0xFD;
    SCON= 0x40;
    TMOD= (TMOD & 0x0F) | 0x20;
    REN = 1;
    TR1 = 1;
    ES  = 1;
}

/*********************************************************************
** Strobe Command
*********************************************************************/
void StrobeCMD(Uint8 cmd)
{
    Uint8 i;

    SCS=0;
    for(i=0; i<8; i++)
    {
        if(cmd & 0x80)
            SDIO = 1;
        else
            SDIO = 0;

        _nop_();
        SCK=1;
        _nop_();
        SCK=0;
        cmd<<=1;
    }
    SCS=1;
}

/************************************************************************
**  ByteSend
************************************************************************/
void ByteSend(Uint8 src)
{
    Uint8 i;

    for(i=0; i<8; i++)
    {
        if(src & 0x80)
            SDIO = 1;
        else
            SDIO = 0;

        _nop_();
        SCK=1;
        _nop_();
        SCK=0;
        src<<=1;
    }
}

/************************************************************************
**  ByteRead
************************************************************************/
Uint8 ByteRead(void)
{
    Uint8 i,tmp;

    //read data code
    SDIO=1;     //SDIO pull high
    for(i=0; i<8; i++)
    {
        if(SDIO)
            tmp = (tmp << 1) | 0x01;
        else
            tmp = tmp << 1;

        SCK=1;
        _nop_(); 
        SCK=0;
        _nop_();
    }
    return tmp;
}

/************************************************************************
**  A7139_WriteReg
************************************************************************/
void A7139_WriteReg(Uint8 address, Uint16 dataWord)
{
    Uint8 i;

    SCS=0;
    address |= CMD_Reg_W;
    for(i=0; i<8; i++)
    {
        if(address & 0x80)
            SDIO = 1;
        else
            SDIO = 0;
		
		_nop_();
        SCK=1;
        _nop_(); 
        SCK=0;
        address<<=1;
    }
    _nop_();

    //send data word
    for(i=0; i<16; i++)
    {
        if(dataWord & 0x8000)
            SDIO = 1;
        else
            SDIO = 0;

		_nop_();
        SCK=1;
        _nop_();
        SCK=0;
        dataWord<<=1;
    }
    SCS=1;
}

/************************************************************************
**  A7139_ReadReg
************************************************************************/
Uint16 A7139_ReadReg(Uint8 address)
{
    Uint8 i;
    Uint16 tmp;

    SCS=0;
    address |= CMD_Reg_R;
    for(i=0; i<8; i++)
    {
        if(address & 0x80)
            SDIO = 1;
        else
            SDIO = 0;

        _nop_(); 
        SCK=1;
        _nop_();
        SCK=0;
        address<<=1;
    }
    _nop_();
    
    //read data code
    SDIO=1;     //SDIO pull high
    for(i=0; i<16; i++)
    {
        if(SDIO)
            tmp = (tmp << 1) | 0x01;
        else
            tmp = tmp << 1;

        SCK=1;
        _nop_();
        SCK=0;
        _nop_();
    }
    SCS=1;
    return tmp;
}

/************************************************************************
**  A7139_WritePageA
************************************************************************/
void A7139_WritePageA(Uint8 address, Uint16 dataWord)
{
    Uint16 tmp;

    tmp = address;
    tmp = ((tmp << 12) | A7139Config[CRYSTAL_REG]);
    A7139_WriteReg(CRYSTAL_REG, tmp);
    A7139_WriteReg(PAGEA_REG, dataWord);
}

/************************************************************************
**  A7139_ReadPageA
************************************************************************/
Uint16 A7139_ReadPageA(Uint8 address)
{
    Uint16 tmp;

    tmp = address;
    tmp = ((tmp << 12) | A7139Config[CRYSTAL_REG]);
    A7139_WriteReg(CRYSTAL_REG, tmp);
    tmp = A7139_ReadReg(PAGEA_REG);
    return tmp;
}

/************************************************************************
**  A7139_WritePageB
************************************************************************/
void A7139_WritePageB(Uint8 address, Uint16 dataWord)
{
    Uint16 tmp;

    tmp = address;
    tmp = ((tmp << 7) | A7139Config[CRYSTAL_REG]);
    A7139_WriteReg(CRYSTAL_REG, tmp);
    A7139_WriteReg(PAGEB_REG, dataWord);
}

/************************************************************************
**  A7139_ReadPageB
************************************************************************/
Uint16 A7139_ReadPageB(Uint8 address)
{
    Uint16 tmp;

    tmp = address;
    tmp = ((tmp << 7) | A7139Config[CRYSTAL_REG]);
    A7139_WriteReg(CRYSTAL_REG, tmp);
    tmp = A7139_ReadReg(PAGEB_REG);
    return tmp;
}

/*********************************************************************
** A7139_POR
*********************************************************************/
void A7139_POR(void)
{
	//power on only
    Delay10ms(1);   			//for regulator settling time (power on only)
    
    StrobeCMD(CMD_RF_RST);  	//reset A7139 chip
    while(A7139_WriteID())		//check SPI
    {
		StrobeCMD(CMD_RF_RST);  //reset A7139 chip    	
    }
    A7139_WritePageA(PM_PAGEA, A7139Config_PageA[PM_PAGEA] | 0x1000);   //STS=1
    Delay1ms(2);
    
    entry_deep_sleep_mode();		//deep sleep
    Delay1ms(2);
    wake_up_from_deep_sleep_mode();	//wake up
    
    StrobeCMD(CMD_RF_RST);  	//reset A7139 chip
    while(A7139_WriteID())		//check SPI
    {
		StrobeCMD(CMD_RF_RST);  //reset A7139 chip    	
    }
    A7139_WritePageA(PM_PAGEA, A7139Config_PageA[PM_PAGEA] | 0x1000);   //STS=1
    Delay1ms(2);
}

/*********************************************************************
** InitRF
*********************************************************************/
Uint8 InitRF(void)
{
    //initial pin
    SCS = 1;
    SCK = 0;
    SDIO= 1;
    CKO = 1;
    GIO1= 1;
    GIO2= 1;

    Delay1ms(1);            //delay 1ms for regulator stabilized
    StrobeCMD(CMD_RF_RST);  //reset A7139 chip
    Delay1ms(1);
    
    if(A7139_Config())      //config A7139 chip
        return 1;

    Delay100us(8);          //delay 800us for crystal stabilized

    if(A7139_WriteID())     //write ID code
        return 1;

    if(A7139_Cal())         //IF and VCO Calibration
        return 1;

    return 0;
}

/*********************************************************************
** A7139_Config
*********************************************************************/
Uint8 A7139_Config(void)
{
    Uint8 i;
    Uint16 tmp;

    for(i=0; i<8; i++)
        A7139_WriteReg(i, A7139Config[i]);

    for(i=10; i<16; i++)
	{	
		if((i == 14) && (fb_ok == 1))
			A7139_WriteReg(i, A7139Config[i] | (1<<4));			//MIFS=1(Manual)
		else
			A7139_WriteReg(i, A7139Config[i]);
	}	

    for(i=0; i<16; i++)
        A7139_WritePageA(i, A7139Config_PageA[i]);

    for(i=0; i<5; i++)
        A7139_WritePageB(i, A7139Config_PageB[i]);

    //for check        
    tmp = A7139_ReadReg(SYSTEMCLOCK_REG);
    if(tmp != A7139Config[SYSTEMCLOCK_REG])
    {
        return 1;   
    }
    
    return 0;
}

/************************************************************************
**  WriteID
************************************************************************/
Uint8 A7139_WriteID(void)
{
    Uint8 i;
    Uint8 d1, d2, d3, d4;

    SCS=0;
    ByteSend(CMD_ID_W);
    for(i=0; i<4; i++)
        ByteSend(ID_Tab[i]);
    SCS=1;

    SCS=0;
    ByteSend(CMD_ID_R);
    d1=ByteRead();
    d2=ByteRead();
    d3=ByteRead();
    d4=ByteRead();
    SCS=1;
    
    if((d1!=ID_Tab[0]) || (d2!=ID_Tab[1]) || (d3!=ID_Tab[2]) || (d4!=ID_Tab[3]))
    {
        return 1;
    }
    
    return 0;
}

/*********************************************************************
** A7139_Cal
*********************************************************************/
Uint8 A7139_Cal(void)
{
    Uint8 i;
    Uint8 fb_old, fcd, fbcf;  	//IF Filter
    Uint8 vb,vbcf;          	//VCO Current
    Uint8 vcb, vccf;        	//VCO Band
    Uint16 tmp;
    bit fb_fail;
	
    StrobeCMD(CMD_STBY);

    //IF calibration procedure @STB state
	if(fb_ok == 1)
	{
		tmp = (A7139Config[CALIBRATION_REG] & 0xFFE0);
		tmp = tmp | fb | (1<<4);
		A7139_WriteReg(CALIBRATION_REG, tmp);
	}	
	else
	{	
	 fb_fail=0;
	
	 for(i=0;i<3;i++)
	 {
      A7139_WriteReg(MODE_REG, A7139Config[MODE_REG] | 0x0802);       //IF Filter & VCO Current Calibration
      do{
         tmp = A7139_ReadReg(MODE_REG);
      }while(tmp & 0x0802);
    
      //for check(IF Filter)
      tmp = A7139_ReadReg(CALIBRATION_REG);
      fb = tmp & 0x0F;
      fcd = (tmp>>11) & 0x1F;
      fbcf = (tmp>>4) & 0x01;
    
	  if((fb<4) || (fb>8))
		fb_fail = 1;
	  else
      {		  
	    if(i==0)
		 fb_old = fb;
	    else
		{	
		 if(fb != fb_old)
		 fb_fail = 1;
	    }
	  }
	  
	  if((fbcf) || (fb_fail))
      {
        return 1;
      }
	 }
	}
	 
    //for check(VCO Current)
    tmp = A7139_ReadPageA(VCB_PAGEA);
    vcb = tmp & 0x0F;
    vccf = (tmp>>4) & 0x01;
    if(vccf)
    {
        return 1;
    }
    
    
    //RSSI Calibration procedure @STB state
    A7139_WriteReg(ADC_REG, 0x4C00);                                //set ADC average=64
    A7139_WriteReg(MODE_REG, A7139Config[MODE_REG] | 0x1000);       //RSSI Calibration
    do{
        tmp = A7139_ReadReg(MODE_REG);
    }while(tmp & 0x1000);
    A7139_WriteReg(ADC_REG, A7139Config[ADC_REG]);


    //VCO calibration procedure @STB state
    for(i=0; i<3; i++)
    {
        A7139_WriteReg(PLL1_REG, Freq_Cal_Tab[i*2]);
        A7139_WriteReg(PLL2_REG, Freq_Cal_Tab[i*2+1]);
        A7139_WriteReg(MODE_REG, A7139Config[MODE_REG] | 0x0004);   //VCO Band Calibration
        do{
            tmp = A7139_ReadReg(MODE_REG);
        }while(tmp & 0x0004);
    
        //for check(VCO Band)
        tmp = A7139_ReadReg(CALIBRATION_REG);
        vb = (tmp >>5) & 0x07;
        vbcf = (tmp >>8) & 0x01;
        if(vbcf)
        {
            return 1;
        }
    }
    
	fb_ok = 1;
    return 0;
}

/*********************************************************************
** A7139_WriteFIFO
*********************************************************************/
void A7139_WriteFIFO(void)
{
    Uint8 i;

    StrobeCMD(CMD_TFR);     //TX FIFO address pointer reset

    SCS=0;
    ByteSend(CMD_FIFO_W);   //TX FIFO write command
    for(i=0; i <64; i++)
        ByteSend(PN9_Tab[i]);
    SCS=1;
}

/*********************************************************************
** RxPacket
*********************************************************************/
void RxPacket(void)
{
    Uint8 i;
    Uint8 recv;
    Uint8 tmp;

    RxCnt++;

    StrobeCMD(CMD_RFR);     //RX FIFO address pointer reset

    SCS=0;
    ByteSend(CMD_FIFO_R);   //RX FIFO read command
    for(i=0; i <64; i++)
    {
        tmpbuf[i] = ByteRead();
    }
    SCS=1;

    for(i=0; i<64; i++)
    {
        recv = tmpbuf[i];
        tmp = recv ^ PN9_Tab[i];
        if(tmp!=0)
        {
            Err_ByteCnt++;
            Err_BitCnt += (BitCount_Tab[tmp>>4] + BitCount_Tab[tmp & 0x0F]);
        }
    }
}

/*********************************************************************
** Err_State
*********************************************************************/
void Err_State(void)
{
    //ERR display
    //Error Proc...
    //...
    while(1);
}

/*********************************************************************
** entry_deep_sleep_mode
*********************************************************************/
void entry_deep_sleep_mode(void)
{
    StrobeCMD(CMD_RF_RST);              //RF reset
    A7139_WriteReg(PIN_REG, A7139Config[PIN_REG] | 0x0800);             //SCMDS=1
    A7139_WritePageA(PM_PAGEA, A7139Config_PageA[PM_PAGEA] | 0x1010);   //STS=1, QDS=1
    StrobeCMD(CMD_SLEEP);               //entry sleep mode
    Delay100us(6);                      //delay 600us for VDD_A shutdown, C load=0.1uF
    StrobeCMD(CMD_DEEP_SLEEP);          //entry deep sleep mode
    Delay100us(2);                      //delay 200us for VDD_D shutdown, C load=0.1uF
}

/*********************************************************************
** wake_up_from_deep_sleep_mode
*********************************************************************/
void wake_up_from_deep_sleep_mode(void)
{
    StrobeCMD(CMD_STBY);    //wake up
    Delay1ms(2);            //delay 2ms for VDD_D stabilized
    //InitRF();
}

/*********************************************************************
** RC Oscillator Calibration
*********************************************************************/
void RCOSC_Cal(void)
{
    Uint16 tmp;

    A7139_WritePageA(WOR2_PAGEA, A7139Config_PageA[WOR2_PAGEA] | 0x0010);       //enable RC OSC

    while(1)
    {
        A7139_WritePageA(WCAL_PAGEA, A7139Config_PageA[WCAL_PAGEA] | 0x0001);   //set ENCAL=1 to start RC OSC CAL
        do{
            tmp = A7139_ReadPageA(WCAL_PAGEA);
        }while(tmp & 0x0001);
            
        tmp = (A7139_ReadPageA(WCAL_PAGEA) & 0x03FF);       //read NUMLH[8:0]
        tmp >>= 1;  
        
        if((tmp > 183) && (tmp < 205))      //NUMLH[8:0]=194+-10 (PF8M=6.4M)
        //if((tmp > 232) && (tmp < 254))    //NUMLH[8:0]=243+-10 (PF8M=8M)
        {
            break;
        }
    }
}

/*********************************************************************
** WOR_enable_by_preamble
*********************************************************************/
void WOR_enable_by_preamble(void)
{
    StrobeCMD(CMD_STBY);
    RCOSC_Cal();        //RC Oscillator Calibration
    
    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x004D);  //GIO1=PMDO, GIO2=WTR

    //Real WOR Active Period = (WOR_AC[5:0]+1) x 244us ¡V X'TAL and Regulator Settling Time
    //Note : Be aware that X¡¦tal settling time requirement includes initial tolerance, 
    //       temperature drift, aging and crystal loading.
    A7139_WritePageA(WOR1_PAGEA, 0x8005);   //setup WOR Sleep time and Rx time
    
    A7139_WritePageA(WOR2_PAGEA, A7139Config_PageA[WOR2_PAGEA] | 0x0030);   //enable RC OSC & WOR by preamble
    A7139_WriteReg(MODE_REG, A7139Config[MODE_REG] | 0x0200);               //WORE=1 to enable WOR function
    
    while(GIO1==0);     //Stay in WOR mode until receiving preamble code(preamble detect ok)
}

/*********************************************************************
** WOR_enable_by_sync
*********************************************************************/
void WOR_enable_by_sync(void)
{
    StrobeCMD(CMD_STBY);
    RCOSC_Cal();        //RC Oscillator Calibration
    
    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0045);  //GIO1=FSYNC, GIO2=WTR

    //Real WOR Active Period = (WOR_AC[5:0]+1) x 244us ¡V X'TAL and Regulator Settling Time
    //Note : Be aware that X¡¦tal settling time requirement includes initial tolerance, 
    //       temperature drift, aging and crystal loading.
    A7139_WritePageA(WOR1_PAGEA, 0x8005);   //setup WOR Sleep time and Rx time
    
    A7139_WritePageA(WOR2_PAGEA, A7139Config_PageA[WOR2_PAGEA] | 0x0010);   //enable RC OSC & WOR by sync
    A7139_WriteReg(MODE_REG, A7139Config[MODE_REG] | 0x0200);               //WORE=1 to enable WOR function
    
    while(GIO1==0);     //Stay in WOR mode until receiving ID code(ID detect ok)
    
    /*
                        ____    ____    ____    ____    ____
    TX WTR       ______|    |__|    |__|    |__|    |__|    |__
                            ______   _______
    WOR WTR      __________|      |_|       |__________________
                                 _       ___
    WOR FSYNC    _______________| |_____|   |__________________
                                ¡õ ¡õ ¡õ       ¡õ
                                ¡õ ¡õ ¡õ       receive completed
                                ¡õ ¡õ entry RX mode
                                ¡õ disable WOR
                                sync ok 
    */
    
    A7139_WriteReg(MODE_REG, A7139Config[MODE_REG] & ~0x0200);              //WORE=0 to disable WOR function
    StrobeCMD(CMD_STBY);
    StrobeCMD(CMD_RX);
    while(GIO2==0);		//wait RX ready
    while(GIO2==1);     //wait receive completed
}

/*********************************************************************
** WOR_enable_by_carrier
*********************************************************************/
void WOR_enable_by_carrier(void)
{
    StrobeCMD(CMD_STBY);
    RCOSC_Cal();        //RC Oscillator Calibration
    
    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0049);  //GIO1=CD, GIO2=WTR

    //Real WOR Active Period = (WOR_AC[5:0]+1) x 244us ¡V X'TAL and Regulator Settling Time
    //Note : Be aware that X¡¦tal settling time requirement includes initial tolerance, 
    //       temperature drift, aging and crystal loading.
    A7139_WritePageA(WOR1_PAGEA, 0x8005);   //setup WOR Sleep time and Rx time
    
    A7139_WritePageA(WOR2_PAGEA, A7139Config_PageA[WOR2_PAGEA] | 0x0410);   //enable RC OSC & WOR by carrier
    A7139_WritePageA(RFI_PAGEA, A7139Config_PageA[RFI_PAGEA] & ~0x6000);    //select RSSI Carrier Detect
    A7139_WriteReg(ADC_REG, A7139Config[ADC_REG] | 0x8069);                 //ARSSI=1, RTH=105(-100dBm)
    A7139_WriteReg(MODE_REG, A7139Config[MODE_REG] | 0x0200);               //WORE=1 to enable WOR function
    
    while(GIO1==0);     //Stay in WOR mode until carrier signal strength is greater than the value set by RTH[7:0](carrier detect ok)
}

/*********************************************************************
** WOT_enable
*********************************************************************/
void WOT_enable(void)
{
    StrobeCMD(CMD_STBY);
    RCOSC_Cal();            //RC Oscillator Calibration
    
    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0045);  //GIO1=FSYNC, GIO2=WTR

    A7139_WritePageA(WOR1_PAGEA, 0x0005);   //setup WOT Sleep time
    
    A7139_WriteFIFO();      //write data to TX FIFO

    A7139_WriteReg(PIN_REG, A7139Config[PIN_REG] | 0x0400);     //WMODE=1 select WOT function
    A7139_WriteReg(MODE_REG, A7139Config[MODE_REG] | 0x0200);   //WORE=1 to enable WOT function
    
    while(1);
}

/*********************************************************************
** TWOR_enable
*********************************************************************/
void TWOR_enable(void)
{
    StrobeCMD(CMD_STBY);  
    RCOSC_Cal();            //RC Oscillator Calibration
    
    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0051);  //GIO1=TWOR, GIO2=WTR

    A7139_WritePageA(WOR1_PAGEA, 0x8005);   //setup TWOR SL time and AC time
    
    A7139_WritePageA(WOR2_PAGEA, A7139Config_PageA[WOR2_PAGEA] | 0x0014);       //start TWOR by WOR_AC
    //A7139_WritePageA(WOR2_PAGEA, A7139Config_PageA[WOR2_PAGEA] | 0x001C);     //start TWOR by WOR_SL
    
    StrobeCMD(CMD_SLEEP);   //entry sleep mode
    
    while(GIO1==0);         //User can use this square wave to wake up MCU or other purposes
}

/*********************************************************************
** RSSI_measurement
*********************************************************************/
void RSSI_measurement(void)
{
    Uint16 tmp;

    StrobeCMD(CMD_STBY);
    
    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0045);  //GIO1=FSYNC, GIO2=WTR
    
    A7139_WriteReg(ADC_REG, A7139Config[ADC_REG] | 0x8000);     //ARSSI=1
    
    StrobeCMD(CMD_RX);  //entry RX mode
    Delay10us(1);
    
    while(GIO1==0)      //Stay in RX mode until receiving ID code(ID detect ok)
    {
        tmp = (A7139_ReadReg(ADC_REG) & 0x00FF);    //read RSSI value(environment RSSI)
    }
    tmp = (A7139_ReadReg(ADC_REG) & 0x00FF);        //read RSSI value(wanted signal RSSI)
}

/*********************************************************************
** FIFO_extension_TX
*********************************************************************/
void FIFO_extension_TX(void)
{
    Uint8 i, n, j;

    //Set FEP=255
    A7139_WritePageA(VCB_PAGEA, A7139Config_PageA[VCB_PAGEA] & 0xC0FF);             //FEP[13:8]=0
    A7139_WritePageA(FIFO_PAGEA, (A7139Config_PageA[FIFO_PAGEA] & 0x3F00) | 0xC0FF);//FPM=[11], FEP[7:0]=255

    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0075);  //GIO1=FPF, GIO2=WTR
    A7139_WritePageA(CKO_PAGEA, (A7139Config_PageA[CKO_PAGEA] & 0xFF81) | 0x0002);  //CKO=DCK

    n=0;

    StrobeCMD(CMD_TFR);         //TX FIFO address pointer reset
    SCS=0;
    ByteSend(CMD_FIFO_W);       //write 1st TX FIFO 64bytes
    for(i=0; i<64; i++)
    {
        ByteSend(n);
        n++;
    }
    SCS=1;

    StrobeCMD(CMD_TX);          //entry TX mode

    for(j=0; j<4; j++)			//total TX length = 64 + 48*4 = 256bytes
    {
        while(GIO1==0);         //wait FPF go high
        SCS=0;
        ByteSend(CMD_FIFO_W);   //write TX FIFO 48bytes
        for(i=0; i<48; i++)
        {
            ByteSend(n);
            n++;
        }
        SCS=1;
    }
    
    while(GIO2==1); //wait transmit completed
}

/*********************************************************************
** FIFO_extension_RX
*********************************************************************/
void FIFO_extension_RX(void)
{
    Uint8 i, n, j;
    Uint8 recv;
    Uint8 tmp;

    //Set FEP=255
    A7139_WritePageA(VCB_PAGEA, A7139Config_PageA[VCB_PAGEA] & 0xC0FF);             //FEP[13:8]=0
    A7139_WritePageA(FIFO_PAGEA, (A7139Config_PageA[FIFO_PAGEA] & 0x3F00) | 0xC0FF);//FPM=[11], FEP[7:0]=255

    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0075);  //GIO1=FPF, GIO2=WTR
    A7139_WritePageA(CKO_PAGEA, (A7139Config_PageA[CKO_PAGEA] & 0xFF81) | 0x000A);  //CKO=RCK

    Err_ByteCnt=0;
    Err_BitCnt=0;
    n=0;
     
    StrobeCMD(CMD_RX);          //entry RX mode

    for(j=0; j<5 ; j++)			//total RX length = 48*5 + 16 = 256bytes
    {
        while(GIO1==0);         //wait FPF go high
        SCS=0;
        ByteSend(CMD_FIFO_R);   //read RX FIFO 48bytes
        for(i=0; i<48; i++)     
            tmpbuf[i]=ByteRead();
        SCS=1;

        //for check
        for(i=0; i<48; i++)
        {
            recv = tmpbuf[i];
            tmp = recv ^ n;
            if(tmp!=0)
            {
                Err_ByteCnt++;
                Err_BitCnt += (BitCount_Tab[tmp>>4] + BitCount_Tab[tmp & 0x0F]);
            }
            n++;
        }
    }

    while(GIO2==1);    			//wait receive completed

    SCS=0;
    ByteSend(CMD_FIFO_R);       //read RX FIFO 16bytes
    for(i=0; i<16; i++)     
        tmpbuf[i]=ByteRead();
    SCS=1;

    //for check
    for(i=0; i<16; i++)
    {
        recv = tmpbuf[i];
        tmp = recv ^ n;
        if(tmp!=0)
        {
            Err_ByteCnt++;
            Err_BitCnt += (BitCount_Tab[tmp>>4] + BitCount_Tab[tmp & 0x0F]);
        }
        n++;
    }
}

/*********************************************************************
** FIFO_extension_Infinite_TX
*********************************************************************/
void FIFO_extension_Infinite_TX(void)
{
    Uint8 i, n;
    Uint16 j;   

    //for Infinite : FEP[13:0]=remainder, FEP[13:0]<64
    A7139_WritePageA(VCB_PAGEA, A7139Config_PageA[VCB_PAGEA] & 0xC0FF);             //FEP[13:8]=0
    A7139_WritePageA(FIFO_PAGEA, (A7139Config_PageA[FIFO_PAGEA] & 0x3F00) | 0xC013);//FPM=[11], FEP[7:0]=19

    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0075);  //GIO1=FPF, GIO2=WTR
    A7139_WritePageA(CKO_PAGEA, (A7139Config_PageA[CKO_PAGEA] & 0xFF81) | 0x0002);  //CKO=DCK
    
    A7139_WriteReg(PIN_REG, A7139Config[PIN_REG] | 0x0200);     //set INFS=1, infinite length

    n=0;

    StrobeCMD(CMD_TFR);         //TX FIFO address pointer reset
    SCS=0;
    ByteSend(CMD_FIFO_W);       //write 1st TX FIFO 64bytes
    for(i=0; i<64; i++)
    {
        ByteSend(n);
        if(n==255)
            n=0;
        else
            n++;
    }
    SCS=1;

    StrobeCMD(CMD_TX);          //entry TX mode

    for(j=0; j<500; j++)
    {
        while(GIO1==0);         //wait FPF go high
        SCS=0;
        ByteSend(CMD_FIFO_W);   //write TX FIFO 48bytes
        for(i=0; i<48; i++)
        {
            ByteSend(n);
            if(n==255)
                n=0;
            else
                n++;
        }
        SCS=1;
        
        while(GIO1==1);         //wait FPF go low
        SCS=0;
        ByteSend(CMD_FIFO_W);   //write TX FIFO 16bytes
        for(i=0; i<16; i++)
        {
            ByteSend(n);
            if(n==255)
                n=0;
            else
                n++;
        }
        SCS=1;
    }

    //remainder : FEP[13:0]=19, FIFO Length=19+1=20
    while(GIO1==0);             //wait last FPF go high
    SCS=0;
    ByteSend(CMD_FIFO_W);       //write TX FIFO 20bytes
    for(i=0; i<20; i++)
    {
        ByteSend(n);
        if(n==255)
            n=0;
        else
            n++;
    }
    SCS=1;
    
    while(GIO1==1);             //wait last FPF go low
    A7139_WriteReg(PIN_REG, A7139Config[PIN_REG] & ~0x0200);    //disable INFS
    
    while(GIO2==1); //wait transmit completed
}

/*********************************************************************
** FIFO_extension_Infinite_RX
*********************************************************************/
void FIFO_extension_Infinite_RX(void)
{
    Uint8 i, n;
    Uint8 recv;
    Uint8 tmp;
    Uint16 j;

    //for Infinite : FEP[13:0]=remainder, FEP[13:0]<64
    A7139_WritePageA(VCB_PAGEA, A7139Config_PageA[VCB_PAGEA] & 0xC0FF);             //FEP[13:8]=0
    A7139_WritePageA(FIFO_PAGEA, (A7139Config_PageA[FIFO_PAGEA] & 0x3F00) | 0xC013);//FPM=[11], FEP[7:0]=19

    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0075);  //GIO1=FPF, GIO2=WTR
    A7139_WritePageA(CKO_PAGEA, (A7139Config_PageA[CKO_PAGEA] & 0xFF81) | 0x000A);  //CKO=RCK

    A7139_WriteReg(PIN_REG, A7139Config[PIN_REG] | 0x0200);     //set INFS=1, infinite length

    Err_ByteCnt=0;
    Err_BitCnt=0;
    n=0;
     
    StrobeCMD(CMD_RX);          //entry RX mode

    for(j=0; j<501 ; j++)
    {
        while(GIO1==0);         //wait FPF go high
        SCS=0;
        ByteSend(CMD_FIFO_R);   //read RX FIFO 48bytes
        for(i=0; i<48; i++)     
            tmpbuf[i]=ByteRead();
        SCS=1;
        
        while(GIO1==1);         //wait FPF go low
        if(j==500)      //last FPF go low
        {
            //set INFS=0 to terminate infinite transmission in the period of 8*(data rate clock).
            A7139_WriteReg(PIN_REG, A7139Config[PIN_REG] & ~0x0200);    //disable INFS
        }
            
        SCS=0;
        ByteSend(CMD_FIFO_R);   //read RX FIFO 16bytes
        for(i=48; i<64; i++)        
            tmpbuf[i]=ByteRead();
        SCS=1;

        //for check
        for(i=0; i<64; i++)
        {
            recv = tmpbuf[i];
            tmp = recv ^ n;
            if(tmp!=0)
            {
                Err_ByteCnt++;
                Err_BitCnt += (BitCount_Tab[tmp>>4] + BitCount_Tab[tmp & 0x0F]);
            }

            if(n==255)
                n=0;
            else
                n++;
        }
    }

    while(GIO2);    //wait receive completed

    //remainder : FEP[13:0]=19, FIFO Length=19+1=20
    SCS=0;
    ByteSend(CMD_FIFO_R);       //read RX FIFO 20bytes
    for(i=0; i<20; i++)     
        tmpbuf[i]=ByteRead();
    SCS=1;

    //for check
    for(i=0; i<20; i++)
    {
        recv = tmpbuf[i];
        tmp = recv ^ n;
        if(tmp!=0)
        {
            Err_ByteCnt++;
            Err_BitCnt += (BitCount_Tab[tmp>>4] + BitCount_Tab[tmp & 0x0F]);
        }

        if(n==255)
            n=0;
        else
            n++;
    }
}

/*********************************************************************
** Auto_Resend
*********************************************************************/
void Auto_Resend(void)
{
    StrobeCMD(CMD_STBY);

    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0071);  //GIO1=VPOAK, GIO2=WTR
    A7139_WritePageA(CKO_PAGEA, (A7139Config_PageA[CKO_PAGEA] & 0xFF81) | 0x0002);  //CKO=DCK

    A7139_WritePageB(ACK_PAGEB, 0x003F);    //enable Auto Resend, by event, fixed, ARC=15
    A7139_WritePageB(ART_PAGEB, 0x001E);    //RND=0, ARD=30
    //ACK format = Preamble + ID = 8bytes
    //data rate=10kbps, Transmission Time=(1/10k)*8*8=6.4ms
    //so set ARD=30, RX time=30*250us=7.75ms

    while(1)
    {
        A7139_WriteFIFO();
        StrobeCMD(CMD_TX);      //entry TX mode
        Delay10us(1);
        while(GIO2);            //wait transmit completed

        //check VPOAK
        if(GIO1)
        {
            //ACK ok
        }
        else
        {
            //NO ACK
        }

        StrobeCMD(CMD_STBY);    //clear VPOAK(GIO1 signal)

        Delay10ms(1);
    }
}

/*********************************************************************
** Auto_ACK
*********************************************************************/
void Auto_ACK(void)
{
    StrobeCMD(CMD_STBY);
    
    A7139_WritePageA(GIO_PAGEA, (A7139Config_PageA[GIO_PAGEA] & 0xF000) | 0x0045);  //GIO1=FSYNC, GIO2=WTR
    A7139_WritePageA(CKO_PAGEA, (A7139Config_PageA[CKO_PAGEA] & 0xFF81) | 0x000A);  //CKO=RCK

    A7139_WritePageB(ACK_PAGEB, 0x0001);    //enable Auto ACK

    while(1)
    {
        StrobeCMD(CMD_RX);      //entry RX mode
        Delay10us(1);           
        while(GIO2);            //wait receive completed

        RxPacket();
    }
}
