//-----------------------------------------------------------------------------
//   File:      bulkloop.c
//   Contents:  Hooks required to implement USB peripheral function.
//
// $Archive: /USB/Examples/FX2LP/bulkloop/bulkloop.c $
// $Date: 3/23/05 2:55p $
// $Revision: 4 $
//
//
//-----------------------------------------------------------------------------
// Copyright 2003, Cypress Semiconductor Corporation
//-----------------------------------------------------------------------------
//
//Project Objective
//	This project illustrates the configuration of the FX2LP to accept bulk data from 
//	the host and loop it back to the host
//
//Overview
//	This project demonstrates a bulkloop operation using the FX2LP. The project 
//	illustrates the configuration of the endpoints and the interface to carry out 
//	the bulkloop operation. Four endpoints are configured to handle bulk transfer, 
//	two OUT endpoints and two IN endpoints. Data sent from the host is stored in an 
//	OUT endpoint. This data is transferred to the IN endpoint and then sent back to 
//	the host on request. 
//
//Operation:
//	The descriptor file for this project defines four endpoints to the host. 
//	Endpoint 2 and 4 are OUT endpoints. Endpoint 6 and 8 are IN endpoints. 
//	All endpoints are configured as BULK endpoints.
//
//	The function TD_Init initializes all the configuration registers. 
//	The IFCONFIG, which configures the interface, sets the FX2LP in slave FIFO mode. 
//	All the four endpoints are configured by writing into its respective EPxCFG registers. 
//
//	Once configured, the OUT endpoints are "armed" to accept data from the host. 
//	An OUT endpoint is said to be armed if it is ready to accept data from the host. 
//	Each endpoint is configured as double buffered. The OUT endpoints are armed 
//	by skipping two packets while initializing thus making them empty to receive 
//	a new packet from the host.
//
//	The function TD_Poll is where the bulkloop operation is defined. 
//
//	If data is available with endpoint 2 and if endpoint 6 is ready to accept 
//	new data, the data is transferred to endpoint 6 and it is committed by writing
//  the number of bytes to the byte count registers. Endpoint 2 is then rearmed to 
//	accept a new packet from the host
//
//	Similar operation is done on endpoint 4 and endpoint 8, where endpoint 4 is 
//	an OUT endpoint and endpoint 8 is an IN endpoint. 
//
//Code Snippets:
//
//	Descriptor:
//	    The interface descriptor defines the number of endpoints to the host. 
//		
//		The four endpoint descriptors define each endpoint to the host. Endpoint 2 and 4 
//		are defined as OUT endpoints, while endpoint 6 and 8 are defined as IN endpoints.
//		All endpoints are defined as "Bulk" endpoints. The maximum transfer length 
//		possible in a BULK transfer is 512 bytes. 
//
//		The descriptors are written for both high-speed and full-speed configurations. 
//
//	Initialization:
//		The initialization of the FX2LP, that is all components involved in the bulkloop 
//		operation, is done in the TD_Init function, which is the first function called in main().
//
//		The statement 
//		IFCONFIG |= 0x40;
//		configures the interface of the FX2LP. The FX2LP is configured in the slave FIFO mode. 
//
//		The four endpoints defined in the descriptor file have to be configured in this 
//		function. This is done by the statements below.
//
//		EP2CFG = 0xA2;
//		SYNCDELAY;                    
//		EP4CFG = 0xA0;
//		SYNCDELAY;                    
//		EP6CFG = 0xE2;
//		SYNCDELAY;                    
//		EP8CFG = 0xE0;
//
// 		The definition of each register can be found from Chapter 15 of the Technical Reference Manual (TRM)
//		in the folder Docs/Technical Reference Manual. Through the statements defined above,
//		The key characteristics of each endpoint are listed below. 
//		Endpoint 2 - OUT, Bulk, double buffered
//		Endpoint 4 - OUT, Bulk, double buffered
//		Endpoint 6 - IN, Bulk, double buffered
//		Endpoint 8 - IN, Bulk, double buffered
//
//		Writing to these registers typically takes more than 2 clock cycles needed for a MOVX instruction. 
//		Hence the SYNCDELAY, already defined, is added. The list of registers which need this delay 
//		function when writing to it is given in the TRM
//
//		The OUT endpoints, once configured, need to be armed to accept packets from the host. 
//		Since the endpoints are double buffered, we need to skip two packets to arm the endpoint. 
//		Arming is essentially freeing up the buffers and making them available to the host to receive packets.
//
//		By writing a 1 to bit7 of the byte count register the packet is skipped. 
//		EP2BCL = 0x80;                // arm EP2OUT by writing byte count w/skip.
//		SYNCDELAY;                    
//		EP2BCL = 0x80;
//		SYNCDELAY;                    
//		EP4BCL = 0x80;                // arm EP4OUT by writing byte count w/skip.
//		SYNCDELAY;                    
//		EP4BCL = 0x80; 
//		The above lines arm the 2 OUT endpoints, by skipping two packets of data making 
//		the buffers available to receive OUT data. 
//
//		AUTOPTRSETUP |= 0x01;
//		This enables the AUTO pointer used for data transfer in the TD_Poll function. 
//
//	Enumeration:
//		Every time the FX2LP receives a setup command request, an interrupt is triggered where the 
//		GotSUD flag is asserted. The Setup Command function services various set up requests from the host. 
//		The Set up Command is executed through a switch case where the entire information desired by 
//		the host is serviced. 	  
//		Summing up, the enumeration process is done by repeated calling of the function SetupCommand().
//
//	Bulk Loop Implementation:
//		The bulk loop implementation is carried out in the TD_Poll function which is called repeatedly 
//		when the device is idle.
//
//		Endpoint 2 and 4 are armed to accept data from the host. This data is transferred to endpoint 6 
//		and endpoint 8 respectively. To implement this, first, endpoint 2 is checked if it has data. 
//		This is done by reading the endpoint 2 empty bit in the endpoint status register (EP2468STAT). 
//		If endpoint 2 has data (that is sent from the host), the capability of endpoint 6 to receive 
//		the data is checked. This is done by reading the endpoint 6 Full bit in the endpoint status register. 
//		If endpoint 6 is not full, then the data is transferred.
//
//		This decision is executed by the following statements. 
//
//		if (!(EP2468STAT & bmEP2EMPTY))
//		{// check EP2 EMPTY (busy) bit in EP2468STAT (SFR), core set's this bit when 
//    	 // FIFO is empty
//     if (!(EP2468STAT & bmEP6FULL))
//     {// check EP6 FULL (busy) bit in EP2468STAT (SFR), core set's this bit 
//      // when FIFO is full
//
//		Data Transfer implementation:
//			The data pointers are initialized to the corresponding buffers.
//			The first auto-pointer is initialized to the first byte of the endpoint 2 FIFO buffer. 
//			The second auto-pointer is initialized to the first byte of the endpoint 6 FIFO buffer. 
//			
//			The number of bytes to be transferred is read from the byte count registers of Endpoint 2.
//			The registers EP2BCL, EP2BCH contain the number of bytes written into the FIFO buffer 
//			by the host. These two registers give the byte count of the data transferred to the FIFO 
//			in an OUT transaction as long as the data is not committed to the peripheral side. 
//
//			This data pointer initialization and loading of the count is done in the following statements. 
//			APTR1H = MSB( &EP2FIFOBUF );    // Initializing the first data pointer
//			APTR1L = LSB( &EP2FIFOBUF );
//
//			AUTOPTRH2 = MSB( &EP6FIFOBUF ); // Initializing the second data pointer        
//			AUTOPTRL2 = LSB( &EP6FIFOBUF );
//
//			count = (EP2BCH << 8) + EP2BCL; // The count value is loaded from the byte 
//                                			// count registers
//
//			The data transfer is carried out by the execution of the loop below. 
//
//			for( i = 0x0000; i < count; i++ )
//		    {
//			   // setup to transfer EP2OUT buffer to EP6IN buffer using AUTOPOINTER(s)
//			   EXTAUTODAT2 = EXTAUTODAT1;
//		    }
//
//			As auto pointers have been enabled, the pointers increment automatically, and the statement 
//
//			EXTAUTODAT2 = EXTAUTODAT1;
//
//			transfers data from endpoint 2 to endpoint 6. Each time the above statement is executed 
//			the auto pointer is incremented. The above statement is executed over and over to 
//			transfer each byte from endpoint 2 to 6. 
// 			Once the data is transferred, endpoint 2 has to be "re-armed" to accept a new packet from the host. 
//			Endpoint 6 has to be "committed", that is, make the FIFO buffers available to the host for reading 
//			data from the Endpoint 6.
//
//			This is accomplished by the following statements. 
//
//			EP6BCH = EP2BCH;  
//			SYNCDELAY;  
//			EP6BCL = EP2BCL;        // commit EP6IN by specifying the number of bytes the host can read from EP6
//			SYNCDELAY;                    
//			EP2BCL = 0x80;          // re (arm) EP2OUT
//
//			The same operation is carried out to implement a data loop with Endpoints 4 and 8. 
//
//----------------------------------------------------------------------------
// Code below
//----------------------------------------------------------------------------

#pragma NOIV               // Do not generate interrupt vectors

#include "fx2.h"
#include "fx2regs.h"
#include "syncdly.h"            // SYNCDELAY macro

extern BOOL GotSUD;             // Received setup data flag
extern BOOL Sleep;
extern BOOL Rwuen;
extern BOOL Selfpwr;

BYTE Configuration;             // Current configuration
BYTE AlternateSetting;          // Alternate settings
static WORD xdata LED_Count = 0;
static BYTE xdata LED_Status = 0;
void LED_Off (BYTE LED_Mask);
void LED_On (BYTE LED_Mask);

#define VR_NAKALL_ON    0xD0
#define VR_NAKALL_OFF   0xD1

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
//   The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------

void TD_Init(void)             // Called once at startup
{
   // set the CPU clock to 48MHz
   CPUCS = ((CPUCS & ~bmCLKSPD) | bmCLKSPD1) ;

   // set the slave FIFO interface to 48MHz
   IFCONFIG |= 0x40;

  // Registers which require a synchronization delay, see section 15.14
  // FIFORESET        FIFOPINPOLAR
  // INPKTEND         OUTPKTEND
  // EPxBCH:L         REVCTL
  // GPIFTCB3         GPIFTCB2
  // GPIFTCB1         GPIFTCB0
  // EPxFIFOPFH:L     EPxAUTOINLENH:L
  // EPxFIFOCFG       EPxGPIFFLGSEL
  // PINFLAGSxx       EPxFIFOIRQ
  // EPxFIFOIE        GPIFIRQ
  // GPIFIE           GPIFADRH:L
  // UDMACRCH:L       EPxGPIFTRIG
  // GPIFTRIG
  
  // Note: The pre-REVE EPxGPIFTCH/L register are affected, as well...
  //      ...these have been replaced by GPIFTC[B3:B0] registers

  // default: all endpoints have their VALID bit set
  // default: TYPE1 = 1 and TYPE0 = 0 --> BULK  
  // default: EP2 and EP4 DIR bits are 0 (OUT direction)
  // default: EP6 and EP8 DIR bits are 1 (IN direction)
  // default: EP2, EP4, EP6, and EP8 are double buffered

  // we are just using the default values, yes this is not necessary...
  EP1OUTCFG = 0xA0;
  EP1INCFG = 0xA0;
  SYNCDELAY;                    // see TRM section 15.14
  EP2CFG = 0xA2;
  SYNCDELAY;                    
  EP4CFG = 0xA0;
  SYNCDELAY;                    
  EP6CFG = 0xE2;
  SYNCDELAY;                    
  EP8CFG = 0xE0;

  // out endpoints do not come up armed
  
  // since the defaults are double buffered we must write dummy byte counts twice
  SYNCDELAY;                    
  EP2BCL = 0x80;                // arm EP2OUT by writing byte count w/skip.
  SYNCDELAY;                    
  EP2BCL = 0x80;
  SYNCDELAY;                    
  EP4BCL = 0x80;                // arm EP4OUT by writing byte count w/skip.
  SYNCDELAY;                    
  EP4BCL = 0x80;    

  // enable dual autopointer feature
  AUTOPTRSETUP |= 0x01;

}


void TD_Poll(void)              // Called repeatedly while the device is idle
{
  WORD i;
  WORD count;

  if(!(EP2468STAT & bmEP2EMPTY))
  { // check EP2 EMPTY(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is empty
     if(!(EP2468STAT & bmEP6FULL))
     {  // check EP6 FULL(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is full
        APTR1H = MSB( &EP2FIFOBUF );
        APTR1L = LSB( &EP2FIFOBUF );

        AUTOPTRH2 = MSB( &EP6FIFOBUF );
        AUTOPTRL2 = LSB( &EP6FIFOBUF );

        count = (EP2BCH << 8) + EP2BCL;

        // loop EP2OUT buffer data to EP6IN
        for( i = 0x0000; i < count; i++ )
        {
           // setup to transfer EP2OUT buffer to EP6IN buffer using AUTOPOINTER(s)
           EXTAUTODAT2 = EXTAUTODAT1;
        }
        EP6BCH = EP2BCH;  
        SYNCDELAY;  
        EP6BCL = EP2BCL;        // arm EP6IN
        SYNCDELAY;                    
        EP2BCL = 0x80;          // re(arm) EP2OUT
     }
  }

  if(!(EP2468STAT & bmEP4EMPTY))
  { // check EP4 EMPTY(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is empty
     if(!(EP2468STAT & bmEP8FULL))
     {  // check EP8 FULL(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is full
        APTR1H = MSB( &EP4FIFOBUF );
        APTR1L = LSB( &EP4FIFOBUF );

        AUTOPTRH2 = MSB( &EP8FIFOBUF );
        AUTOPTRL2 = LSB( &EP8FIFOBUF );

        count = (EP4BCH << 8) + EP4BCL;

        // loop EP4OUT buffer data to EP8IN
        for( i = 0x0000; i < count; i++ )
        {
           // setup to transfer EP4OUT buffer to EP8IN buffer using AUTOPOINTER(s)
           EXTAUTODAT2 = EXTAUTODAT1;
        }
        EP8BCH = EP4BCH;  
        SYNCDELAY;  
        EP8BCL = EP4BCL;        // arm EP8IN
        SYNCDELAY;                    
        EP4BCL = 0x80;          // re(arm) EP4OUT
     }
  }
}

BOOL TD_Suspend(void)          // Called before the device goes into suspend mode
{
   return(TRUE);
}

BOOL TD_Resume(void)          // Called after the device resumes
{
   return(TRUE);
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------

BOOL DR_GetDescriptor(void)
{
   return(TRUE);
}

BOOL DR_SetConfiguration(void)   // Called when a Set Configuration command is received
{
   Configuration = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetConfiguration(void)   // Called when a Get Configuration command is received
{
   EP0BUF[0] = Configuration;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_SetInterface(void)       // Called when a Set Interface command is received
{
   AlternateSetting = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetInterface(void)       // Called when a Set Interface command is received
{
   EP0BUF[0] = AlternateSetting;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_GetStatus(void)
{
   return(TRUE);
}

BOOL DR_ClearFeature(void)
{
   return(TRUE);
}

BOOL DR_SetFeature(void)
{
   return(TRUE);
}

BOOL DR_VendorCmnd(void)
{
  BYTE tmp;
  
  switch (SETUPDAT[1])
  {
     case VR_NAKALL_ON:
        tmp = FIFORESET;
        tmp |= bmNAKALL;      
        SYNCDELAY;                    
        FIFORESET = tmp;
        break;
     case VR_NAKALL_OFF:
        tmp = FIFORESET;
        tmp &= ~bmNAKALL;      
        SYNCDELAY;                    
        FIFORESET = tmp;
        break;
	case 0xBF:
		EP0BUF[0] = 0xBF;
		EP0BCH = 0;
 		EP0BCL = 1;
 		EP0CS |= bmHSNAK;
    break;
     default:
        return(FALSE);
  }

  return(FALSE);
}

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler
void ISR_Sudav(void) interrupt 0
{
   GotSUD = TRUE;            // Set flag
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUDAV;         // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler
void ISR_Sutok(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUTOK;         // Clear SUTOK IRQ
}

void ISR_Sof(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSOF;            // Clear SOF IRQ
}

void ISR_Ures(void) interrupt 0
{
   // whenever we get a USB reset, we should revert to full speed mode
   pConfigDscr = pFullSpeedConfigDscr;
   ((CONFIGDSCR xdata *) pConfigDscr)->type = CONFIG_DSCR;
   pOtherConfigDscr = pHighSpeedConfigDscr;
   ((CONFIGDSCR xdata *) pOtherConfigDscr)->type = OTHERSPEED_DSCR;

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmURES;         // Clear URES IRQ
}

void ISR_Susp(void) interrupt 0
{
   Sleep = TRUE;
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUSP;
}

void ISR_Highspeed(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      ((CONFIGDSCR xdata *) pConfigDscr)->type = CONFIG_DSCR;
      pOtherConfigDscr = pFullSpeedConfigDscr;
      ((CONFIGDSCR xdata *) pOtherConfigDscr)->type = OTHERSPEED_DSCR;
   }

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmHSGRANT;
}
void ISR_Ep0ack(void) interrupt 0
{
}
void ISR_Stub(void) interrupt 0
{
}
void ISR_Ep0in(void) interrupt 0
{
}
void ISR_Ep0out(void) interrupt 0
{
}
void ISR_Ep1in(void) interrupt 0
{
}
void ISR_Ep1out(void) interrupt 0
{
}
void ISR_Ep2inout(void) interrupt 0
{
}
void ISR_Ep4inout(void) interrupt 0
{
}
void ISR_Ep6inout(void) interrupt 0
{
}
void ISR_Ep8inout(void) interrupt 0
{
}
void ISR_Ibn(void) interrupt 0
{
}
void ISR_Ep0pingnak(void) interrupt 0
{
}
void ISR_Ep1pingnak(void) interrupt 0
{
}
void ISR_Ep2pingnak(void) interrupt 0
{
}
void ISR_Ep4pingnak(void) interrupt 0
{
}
void ISR_Ep6pingnak(void) interrupt 0
{
}
void ISR_Ep8pingnak(void) interrupt 0
{
}
void ISR_Errorlimit(void) interrupt 0
{
}
void ISR_Ep2piderror(void) interrupt 0
{
}
void ISR_Ep4piderror(void) interrupt 0
{
}
void ISR_Ep6piderror(void) interrupt 0
{
}
void ISR_Ep8piderror(void) interrupt 0
{
}
void ISR_Ep2pflag(void) interrupt 0
{
}
void ISR_Ep4pflag(void) interrupt 0
{
}
void ISR_Ep6pflag(void) interrupt 0
{
}
void ISR_Ep8pflag(void) interrupt 0
{
}
void ISR_Ep2eflag(void) interrupt 0
{
}
void ISR_Ep4eflag(void) interrupt 0
{
}
void ISR_Ep6eflag(void) interrupt 0
{
}
void ISR_Ep8eflag(void) interrupt 0
{
}
void ISR_Ep2fflag(void) interrupt 0
{
}
void ISR_Ep4fflag(void) interrupt 0
{
}
void ISR_Ep6fflag(void) interrupt 0
{
}
void ISR_Ep8fflag(void) interrupt 0
{
}
void ISR_GpifComplete(void) interrupt 0
{
}
void ISR_GpifWaveform(void) interrupt 0
{
}

// ...debug LEDs: accessed via movx reads only ( through CPLD )
// it may be worth noting here that the default monitor loads at 0xC000
xdata volatile const BYTE LED0_ON  _at_ 0x8800;
xdata volatile const BYTE LED0_OFF _at_ 0x8000;
xdata volatile const BYTE LED1_ON  _at_ 0x9800;
xdata volatile const BYTE LED1_OFF _at_ 0x9000;
xdata volatile const BYTE LED2_ON  _at_ 0xA800;
xdata volatile const BYTE LED2_OFF _at_ 0xA000;
xdata volatile const BYTE LED3_ON  _at_ 0xB800;
xdata volatile const BYTE LED3_OFF _at_ 0xB000;
// use this global variable when (de)asserting debug LEDs...
BYTE xdata ledX_rdvar = 0x00;
BYTE xdata LED_State = 0;
void LED_Off (BYTE LED_Mask)
{
	if (LED_Mask & bmBIT0)
	{
		ledX_rdvar = LED0_OFF;
		LED_State &= ~bmBIT0;
	}
	if (LED_Mask & bmBIT1)
	{
		ledX_rdvar = LED1_OFF;
		LED_State &= ~bmBIT1;
	}
	if (LED_Mask & bmBIT2)
	{
		ledX_rdvar = LED2_OFF;
		LED_State &= ~bmBIT2;
	}
	if (LED_Mask & bmBIT3)
	{
		ledX_rdvar = LED3_OFF;
		LED_State &= ~bmBIT3;
	}
}

void LED_On (BYTE LED_Mask)
{
	if (LED_Mask & bmBIT0)
	{
		ledX_rdvar = LED0_ON;
		LED_State |= bmBIT0;
	}
	if (LED_Mask & bmBIT1)
	{
		ledX_rdvar = LED1_ON;
		LED_State |= bmBIT1;
	}
	if (LED_Mask & bmBIT2)
	{
		ledX_rdvar = LED2_ON;
		LED_State |= bmBIT2;
	}
	if (LED_Mask & bmBIT3)
	{
		ledX_rdvar = LED3_ON;
		LED_State |= bmBIT3;
	}
}
