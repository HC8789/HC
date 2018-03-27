 /********************************************************************
*   A7139CONFIG.h
*   RF Chip-A7139 Configure Definitions
*
*   This file provides the constants associated with the
*   AMICCOM A7139 device.
*
********************************************************************/
#ifndef _A7139CONFIG_h_
#define _A7139CONFIG_h_

#define DR_10Kbps_50KIFBW       //470MHz, 10kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
//#define DR_10Kbps_100KIFBW      //470MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
//#define DR_50Kbps_50KIFBW       //470MHz, 50kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
//#define DR_100Kbps_100KIFBW     //470MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
//#define DR_150Kbps_150KIFBW     //470MHz, 150kbps (IFBW = 150KHz, Fdev = 56.25KHz), Crystal=19.2MHz
//#define DR_250Kbps_250KIFBW     //470MHz, 250kbps (IFBW = 250KHz, Fdev = 93.75KHz), Crystal=16MHz


#ifdef DR_10Kbps_50KIFBW

const Uint16 code A7139Config[]=        //470MHz, 10kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0x0823,     //SYSTEM CLOCK register,
    0x0A24,     //PLL1 register,
    0xB805,     //PLL2 register,    470.001MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D0,     //RX1 register,     IFBW=50KHz
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const Uint16 code A7139Config_PageA[]=   //470MHz, 10kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0xF606,     //TX1 register,     Fdev = 18.75kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x9B70,     //PM register,      CST=1
    0x0201,     //RTH register,
    0x400F,     //AGC1 register,    
    0x2AC0,     //AGC2 register, 
    0x0045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0825,     //CHG1 register,    480MHz
    0x0127,     //CHG2 register,    500MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const Uint16 code A7139Config_PageB[]=   //470MHz, 10kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0x0337,     //TX2 register,
    0x8200,     //IF1 register,     Enable Auto-IF, IF=100KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

const Uint16 code Freq_Cal_Tab[]=
{
    0x0A24, 0xB805, //470.001MHz
    0x0A26, 0x4805, //490.001MHz
    0x0A27, 0xD805  //510.001MHz
};

#endif


#ifdef DR_10Kbps_100KIFBW

const Uint16 code A7139Config[]=        //470MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x1221,     //SYSTEM CLOCK register,
    0x0A24,     //PLL1 register,
    0xB805,     //PLL2 register,    470.001MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D4,     //RX1 register,     IFBW=100KHz
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const Uint16 code A7139Config_PageA[]=   //470MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0xF706,     //TX1 register,     Fdev = 37.5kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x9B70,     //PM register,      CST=1
    0x0201,     //RTH register,
    0x400F,     //AGC1 register,    
    0x2AC0,     //AGC2 register, 
    0x0045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0825,     //CHG1 register,    480MHz
    0x0127,     //CHG2 register,    500MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const Uint16 code A7139Config_PageB[]=   //470MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0337,     //TX2 register,
    0x8400,     //IF1 register,     Enable Auto-IF, IF=200KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

const Uint16 code Freq_Cal_Tab[]=
{
    0x0A24, 0xB805, //470.001MHz
    0x0A26, 0x4805, //490.001MHz
    0x0A27, 0xD805  //510.001MHz
};

#endif


#ifdef DR_50Kbps_50KIFBW

const Uint16 code A7139Config[]=        //470MHz, 50kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0x0023,     //SYSTEM CLOCK register,
    0x0A24,     //PLL1 register,
    0xB805,     //PLL2 register,    470.001MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D0,     //RX1 register,     IFBW=50KHz
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const Uint16 code A7139Config_PageA[]=   //470MHz, 50kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0xF606,     //TX1 register,     Fdev = 18.75kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x1B70,     //PM register,      CST=1
    0x0201,     //RTH register,
    0x400F,     //AGC1 register,    
    0x2AC0,     //AGC2 register, 
    0x0045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0825,     //CHG1 register,    480MHz
    0x0127,     //CHG2 register,    500MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const Uint16 code A7139Config_PageB[]=   //470MHz, 50kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0x0337,     //TX2 register,
    0x8200,     //IF1 register,     Enable Auto-IF, IF=100KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

const Uint16 code Freq_Cal_Tab[]=
{
    0x0A24, 0xB805, //470.001MHz
    0x0A26, 0x4805, //490.001MHz
    0x0A27, 0xD805  //510.001MHz
};

#endif


#ifdef DR_100Kbps_100KIFBW

const Uint16 code A7139Config[]=        //470MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0021,     //SYSTEM CLOCK register,
    0x0A24,     //PLL1 register,
    0xB805,     //PLL2 register,    470.001MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D4,     //RX1 register,     IFBW=100KHz 
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const Uint16 code A7139Config_PageA[]=   //470MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0xF706,     //TX1 register,     Fdev = 37.5kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x1B70,     //PM register,
    0x0201,     //RTH register,
    0x400F,     //AGC1 register,    
    0x2AC0,     //AGC2 register, 
    0x0045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0825,     //CHG1 register,    480MHz
    0x0127,     //CHG2 register,    500MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const Uint16 code A7139Config_PageB[]=   //470MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0337,     //TX2 register,
    0x8400,     //IF1 register,     Enable Auto-IF, IF=200KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

const Uint16 code Freq_Cal_Tab[]=
{
    0x0A24, 0xB805, //470.001MHz
    0x0A26, 0x4805, //490.001MHz
    0x0A27, 0xD805  //510.001MHz
};

#endif



#ifdef DR_150Kbps_150KIFBW

const Uint16 code A7139Config[]=        //470MHz, 150kbps (IFBW = 150KHz, Fdev = 56.25KHz), Crystal=19.2MHz
{
    0x0021,     //SYSTEM CLOCK register,
    0x0A18,     //PLL1 register,
    0x7AAE,     //PLL2 register,    470.001MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D8,     //RX1 register,     IFBW=150KHz 
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const Uint16 code A7139Config_PageA[]=   //470MHz, 150kbps (IFBW = 150KHz, Fdev = 56.25KHz), Crystal=19.2MHz
{
    0xF706,     //TX1 register,     Fdev = 56.25kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x1B70,     //PM register,
    0x0201,     //RTH register,
    0x400F,     //AGC1 register,    
    0x2AC0,     //AGC2 register, 
    0x4045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0019,     //CHG1 register,    480MHz
    0x011A,     //CHG2 register,    500MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const Uint16 code A7139Config_PageB[]=   //470MHz, 150kbps (IFBW = 150KHz, Fdev = 56.25KHz), Crystal=19.2MHz
{
    0x8337,     //TX2 register,
    0x8400,     //IF1 register,     Enable Auto-IF, IF=300KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

const Uint16 code Freq_Cal_Tab[]=
{
    0x0A18, 0x7AAE, //470.001MHz
    0x0A19, 0x8558, //490.001MHz
    0x0A1A, 0x9003  //510.001MHz
};

#endif


#ifdef DR_250Kbps_250KIFBW

const Uint16 code A7139Config[]=        //470MHz, 250kbps (IFBW = 250KHz, Fdev = 93.75KHz), Crystal=16MHz
{
    0x0020,     //SYSTEM CLOCK register,
    0x0A1D,     //PLL1 register,
    0x6004,     //PLL2 register,    470.001MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18DC,     //RX1 register,     IFBW=250KHz 
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const Uint16 code A7139Config_PageA[]=   //470MHz, 250kbps (IFBW = 250KHz, Fdev = 93.75KHz), Crystal=16MHz
{
    0xF530,     //TX1 register,     Fdev = 93.75kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x1B70,     //PM register,
    0x0201,     //RTH register,
    0x400F,     //AGC1 register,    
    0x2AC0,     //AGC2 register, 
    0x8045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x001E,     //CHG1 register,    480MHz
    0x041F,     //CHG2 register,    500MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const Uint16 code A7139Config_PageB[]=   //470MHz, 250kbps (IFBW = 250KHz, Fdev = 93.75KHz), Crystal=16MHz
{
    0x0337,     //TX2 register,
    0x8800,     //IF1 register,     Enable Auto-IF, IF=500KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

const Uint16 code Freq_Cal_Tab[]=
{
    0x0A1D, 0x6004, //470.001MHz
    0x0A1E, 0xA004, //490.001MHz
    0x0A1F, 0xE004  //510.001MHz
};

#endif


#endif