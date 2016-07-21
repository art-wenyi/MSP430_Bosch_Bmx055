#ifndef I2C_H_
#define I2C_H_

#include "msp430f5659.h"

#define TRUE 		0x01
#define FALSE 		0x00

// I2C Settings for master initialization
//****************************************************************************//
#define 	I2C_SOURCE_CLOCK		0x00	// 0x00 SMCLK or 0x01 ACLK
#define		I2C_SOURCE_CLOCK_FREQ  SMCLK_FREQ
#define     SMCLK_FREQ			   8000000   // 1Mhz?
// Gyro I2C clock
#define		I2C_CLOCK_FREQ			200000

// Display I2C clock
#define		I2C_CLOCK_DISPLAY_FREQ  400000

// Gyro Slave Address - 7-bit address (does not include R/nW bit)
#define I2C_WRITE_ADDR_GYRO         0x68         // 0xD0
#define I2C_READ_ADDR_GYRO          0x68         // 0xD1
#define I2C_WRITE_ADDR_MAG    		0x0C
#define I2C_WRITE_ADDR_MAG	   		0x0C
// Display Slave Address - 7-bit address (does not include R/nW bit)
#define I2C_Display_Write_ADDR		0x3C		// 0x78
#define I2C_Display_Read_ADDR		0x3C		// 0x79
//****************************************************************************//
#define UCB1RXIE         BIT0
#define UCB1TXIE         BIT1
#define UCB1RXIFG        BIT0
#define UCB1TXIFG        BIT1

#define UCBxCTL0		 UCB0CTL0		/* USCI Control Register 0 */
#define UCBxCTL1    	 UCB0CTL1		/* USCI Control Register 1 */
#define UCBxBR0     	 UCB0BR0		/* USCI Baud Rate 0 */
#define UCBxBR1     	 UCB0BR1		/* USCI Baud Rate 1 */
#define UCBxSTAT    	 UCB0STAT		/* USCI Status Register */
#define UCBxRXBUF   	 UCB0RXBUF		/* USCI Receive Buffer */
#define UCBxTXBUF   	 UCB0TXBUF		/* USCI Transmit Buffer */
#define UCBxIE 		     UCB0IE   		/* USCI Interrupt Enable Register */
#define UCBxIFG		     UCB0IFG    	/* USCI Interrupt Flags Register */
#define UCBxIV	    	 UCB0IV		    /* USCI Interrupt Vector Register */
#define UCBxRXIE         BIT0
#define UCBxTXIE         BIT1
#define UCBxRXIFG        BIT0
#define UCBxTXIFG        BIT1
#define UCBxI2CSA		 UCB0I2CSA
#define USCI_Bx_VECTOR   USCI_B0_VECTOR


// Gyro Global Functions
extern void I2C_Master_Init(unsigned char selectClockSource,
						    unsigned long clockSourceFrequency,
						    unsigned long desiredI2CClock);

extern void I2C_Set_Slave_Address(unsigned char slaveAddress);
extern void I2C_Enable(void);
extern void I2C_Disable(void);
extern void I2C_Set_Mode(unsigned short receive);
extern unsigned short I2C_Bus_Busy(void);
extern unsigned short I2C_Busy(void);
extern void I2C_Interrupt_Enable(unsigned char interruptFlags);
extern void I2C_Interrupt_Disable(unsigned char interruptFlags);
extern void I2C_Interrupt_Clear(unsigned char interruptFlags);
extern unsigned char I2C_Interrupt_Status(unsigned char mask);
extern void I2C_Write_Packet_To_Gyro(unsigned char gyroRegName,
   								   unsigned char dataLength,
								   unsigned char *writeData);
extern void I2C_Write_Packet_To_Mag(unsigned char gyroRegName,
								   unsigned char dataLength,
								   unsigned char *writeData);
extern void I2C_Read_Packet_From_Gyro(unsigned char gyroRegName,
							        unsigned char dataLength,
							        unsigned char *readData);
extern void I2C_Read_Packet_From_Mag(unsigned char gyroRegName,
							        unsigned char dataLength,
							        unsigned char *readData);


// Display Global Functions
extern void I2C_Master_Display_Init(unsigned char selectClockSource,
						    unsigned long clockSourceFrequency,
						    unsigned long desiredI2CClock);

extern void I2C_Display_Set_Slave_Address(unsigned char slaveAddress);
extern void I2C_Display_Enable(void);
extern void I2C_Display_Disable(void);
extern void I2C_Display_Set_Mode(unsigned short receive);
extern unsigned short I2C_Display_Bus_Busy(void);
extern unsigned short I2C_Display_Busy(void);
extern void I2C_Display_Interrupt_Enable(unsigned char interruptFlags);
extern void I2C_Display_Interrupt_Disable(unsigned char interruptFlags);
extern void I2C_Display_Interrupt_Clear(unsigned char interruptFlags);
extern unsigned char I2C_Display_Interrupt_Status(unsigned char mask);
extern void I2C_Write_Packet_To_Display(unsigned char displayRegName,
   								   unsigned char dataLength,
								   unsigned char writeData);

void I2C_Read_Packet(unsigned char device_address, unsigned char gyroRegName, int dataLength, unsigned char *readData);
void I2C_Write_Packet(unsigned char device_address, unsigned char gyroRegName, int dataLength, unsigned char *writedData);

#endif /*I2C_H_*/

