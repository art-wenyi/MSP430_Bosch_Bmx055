/*
 * msp430_uart.h
 * A Uart setting and using header file for msp430f5659, USCI interface
 * Created on: 2016-05-29
 *      Author: wenyi zou
 */

#ifndef MSP430_UART_H_
#define MSP430_UART_H_

#include  <msp430f5659.h>

//== register definition ====================================

#define UCAxCTL0   UCA0CTL0
#define UCAxCTL1   UCA0CTL1         // USCI control register

#define UCAxBR0    UCA0BR0
#define UCAxBR1    UCA0BR1          // USCI baudrate register

#define UCAxMCTL   UCA0MCTL         // USCI baudrate modulation register
#define UCAxSTAT   UCA0STAT         // USCI status register


#define UCAxRXBUF  UCA0RXBUF
#define UCAxTXBUF  UCA0TXBUF        // USCI receive and transmit register

#define UCAxABCTL  UCA0ABCTL        // USCI auto baudrate control
#define UCAxIRTCTL UCA0IRTCTL       // /* USCI A0 IrDA Transmit Control */
#define UCAxIRRCTL UCA0IRRCTL       ///* USCI A0 IrDA receive Control */     these three actually not used this file

#define UCAxIE     UCA0IE          // USCI interrupt enable register
#define UCAxTXIE_ON BIT1
#define UCAxTXIE_OFF ~BIT1
#define UCAxRXIE_ON BIT0
#define UCAxRXIE_OFF ~BIT0

#define UCAxIFG    UCA0IFG
#define UCAxTXIFG  BIT1
#define UCAxRXIFG  BIT0        // USCI interrupt flag

#define UCA2TXIFG  BIT1
#define UCA2RXIFG  BIT0        // USCI interrupt flag



//== below are function definition===========================

void Uart_Init(double BaudRate, char parity, char LMSBMode, int bitMode, int stopbitMode);     // Uart Initiation
double Uart_setBaudClock(double BaudRate);      // according baudrate to set the clock USCI use
void Uart_setBaudRate(double BaudRate, double Clk);      // according to the baudrate and clock, set control register
void Uart_setParity(char parity);              // set Parity bit
void Uart_setLMSBMode(char LMSBMode);           // set LSB or MSB mode
void Uart_setbitMode(int bitMode);                 // set bit mode
void Uart_setstopbitMode(int stopbitMode);         // set stop bit mode

void Uart_sendchar(char zifu);                // send char
void Uart_sendstr(char *str);                 // send string
char Uart_readchar();                        // reserve function, really don't know how to use, when to read

void Uart_enableRXINT();                      // enable uart RX interrupt
void Uart_disableRXINT();					  // disable uart RX interrupt
void Uart_enableTXINT();                      // enable uart TX interrupt
void Uart_disableTXINT();					  // disable uart TX interrupt

void UartA2_Init(double BaudRate, char parity, char LMSBMode, int bitMode, int stopbitMode);
void UartA2_setParity(char parity);              // set Parity bit
void UartA2_setLMSBMode(char LMSBMode);           // set LSB or MSB mode
void UartA2_setbitMode(int bitMode);                 // set bit mode
void UartA2_setstopbitMode(int stopbitMode);         // set stop bit mode

void UartA2_sendchar(char zifu);                // send char
void UartA2_sendstr(char *str);                 // send string
char UartA2_readchar();                        // reserve function, really don't know how to use, when to read

void UartA2_enableRXINT();                      // enable uart RX interrupt
void UartA2_disableRXINT();					  // disable uart RX interrupt
void UartA2_enableTXINT();                      // enable uart TX interrupt
void UartA2_disableTXINT();					  // disable uart TX interrupt

#endif /* MSP430_UART_H_ */
