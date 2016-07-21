/*
 * msp430_uart.h
 * A Uart setting and using C file for msp430f5659, USCI interface
 * Created on: 2016-05-29
 *      Author: wenyi zou
 */
#include  <msp430.h>
#include  <msp430f5659.h>
#include  "msp430_uart.h"
//================================================================================
void Uart_Init(double BaudRate, char parity, char LMSBMode, int bitMode, int stopbitMode)     // Uart Initiation
{
	__disable_interrupt();
	PMAPPWD = 0x02D52;                        // port mapping password
	PMAPCTL = PMAPRECFG; 					  // Allow reconfiguration during runtime
	P2MAP4 = PM_UCA0TXD;
	P2MAP5 = PM_UCA0RXD;
	PMAPPWD = 0;							  // Disable Write-Access to modify port mapping registers

	P2SEL |= BIT4 + BIT5;                     // active P2.4 to UCA0TXD and P2.5 to UCA0RXD module function
	P2DIR |= BIT4;                            // TXD need to be set as output direction
	P2DIR &= ~BIT5;                           // RXD input direction
	P2REN |= BIT5;                            // RXD need to set a pulldown resister to get input
	P2OUT &= ~BIT5;							  // (don't pull up)


	//double Clk;
	UCAxCTL1 |= UCSWRST;                  // set the USCI into reset mode
	//Clk = Uart_setBaudClock(BaudRate);        // set the Uart clock as SMCLK
	UCAxCTL1 |= UCSSEL__SMCLK;   				//SMCLK，保证速度, 1MHz频率
	//Uart_setBaudRate(BaudRate, 8000000);          // set the BaudRate
	/*UCAxBR0 = 54;                              // 8MHz 9600 (see User's Guide)  其实吧，还是查表快
	UCAxBR1 = 0;                              // 8MHz 9600          total br1*256+br0
	UCAxMCTL = UCBRS_0 + UCBRF_10 + UCOS16;   // 8MHz 9600  */
	 UCAxBR0 = 6;                              // 1MHz 9600 (see User's Guide)  其实吧，还是查表快
	UCAxBR1 = 0;                              // 1MHz 9600
	UCAxMCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // 1MHz 9600 */
	Uart_setParity(parity);              // set parity bit
	Uart_setLMSBMode(LMSBMode);           // set LSB or MSB mode
	Uart_setbitMode(bitMode);                  // set bit mode
	Uart_setstopbitMode(stopbitMode);         // set stop bit mode
	UCAxIE |= UCRXIE;                         // enable USCI_A0 RX interrupt
	UCAxCTL1 &= ~UCSWRST;                     // **Initialize USCI state machine** (this process will clear the IE bit)
	Uart_enableRXINT();                        // enable USCI_A0 RX interrupt, enable interrupt much go after UCSWRST reset

	__enable_interrupt();                     // Re-enable all interrupts
}

//================================================================================
void UartA2_Init(double BaudRate, char parity, char LMSBMode, int bitMode, int stopbitMode)     // Uart Initiation
{
	__disable_interrupt();
	 // initialize the uart port1(UCA1) on msp430f5659
	  P9SEL |= BIT2 + BIT3;                     // active P8.2 to UCA1TXD and P8.3 to UCA1RXD module function
	  P9DIR |= BIT2;                            // TXD need to be set as output direction
	  P9DIR &= ~BIT3;                           // RXD input direction
	  //P9REN |= BIT3;                            // RXD need to set a pulldown resister to get input (don't pull up)

	//double Clk;
	UCA2CTL1 |= UCSWRST;                  // set the USCI into reset mode
	//Clk = Uart_setBaudClock(BaudRate);        // set the Uart clock as SMCLK
	UCA2CTL1 |= UCSSEL__SMCLK;   				//SMCLK，保证速度, 1MHz频率
	//Uart_setBaudRate(BaudRate, 8000000);          // set the BaudRate
	/*UCA2BR0 = 54;                              // 8MHz 9600 (see User's Guide)  其实吧，还是查表快
	UCA2BR1 = 0;                              // 8MHz 9600          total br1*256+br0
	UCA2MCTL = UCBRS_0 + UCBRF_10 + UCOS16;   // 8MHz 9600  */
	UCA2BR0 = 4;                              // 8MHz 115200 (see User's Guide)  其实吧，还是查表快
	UCA2BR1 = 0;                              // 8MHz 115200
	UCA2MCTL = UCBRS_4 + UCBRF_7 + UCOS16;   // 8MHz 115200 */
	UartA2_setParity(parity);              // set parity bit
	UartA2_setLMSBMode(LMSBMode);           // set LSB or MSB mode
	UartA2_setbitMode(bitMode);                  // set bit mode
	UartA2_setstopbitMode(stopbitMode);         // set stop bit mode
	//UCA2IE |= UCRXIE;                         // enable USCI_A0 RX interrupt
	UCA2CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine** (this process will clear the IE bit)
	//UartA2_enableRXINT();                        // enable USCI_A0 RX interrupt, enable interrupt much go after UCSWRST reset

	__enable_interrupt();                     // Re-enable all interrupts
}

//================================================================================
double Uart_setBaudClock(double BaudRate)     // according baudrate to set the clock USCI use
{
	double bdclk=0;                     // return value for baudclock, convenient for the setBaudRate function
	UCAxCTL1 &= ~(UCSSEL1+UCSSEL0);       //清除之前的时钟设置 , 1 is 0x80, 0 is 0x40
	if(BaudRate<9600)                  //brclk为时钟源频率
	{
	  UCAxCTL1 |= UCSSEL0;              //ACLK，降低功耗
	  bdclk = 32768;                //波特率发生器时钟频率=ACLK(32768)
	}
	else
	{
	  UCAxCTL1 |= UCSSEL__SMCLK;   //SMCLK，保证速度
	  bdclk = 1000000;              //波特率发生器时钟频率=SMCLK(1MHz) , 设定时钟一般为1Mhz
	}
	return bdclk;
}

//================================================================================
void Uart_setBaudRate(double BaudRate, double Clk)      // according to the baudrate and clock, set control register
{
	int n = Clk / BaudRate;     //整数波特率          , see datasheet for more calculation information

	    UCAxBR1 = n >> 8;         //高8位
	    UCAxBR0 = n & 0xff;       //低8位
	    UCAxMCTL = 0;           // modulation set 0 at the beginning
	    UCAxMCTL = UCBRS2;      // set modulation, this time choose UCBRS2, it depends

	}

//================================================================================
void Uart_setParity(char parity)              // set Parity bit
{
switch(parity)
{
    case 'n':case'N': UCAxCTL0 &= ~UCPEN;               break;  //无校验
    case 'e':case'E': UCAxCTL0 |= UCPEN + UCPAR;          break;  //偶校验
    case 'o':case'O': UCAxCTL0 |= UCPEN; UCAxCTL0 &= ~UCPAR; break;  //奇校验
}
}

void UartA2_setParity(char parity)              // set Parity bit
{
switch(parity)
{
    case 'n':case'N': UCA2CTL0 &= ~UCPEN;               break;  //无校验
    case 'e':case'E': UCA2CTL0 |= UCPEN + UCPAR;          break;  //偶校验
    case 'o':case'O': UCA2CTL0 |= UCPEN; UCAxCTL0 &= ~UCPAR; break;  //奇校验
}
}
//================================================================================
void Uart_setLMSBMode(char LMSBMode)           // set LSB or MSB mode
{
switch(LMSBMode)
{
    case 'l':case'L': UCAxCTL0 &= ~UCMSB;               break;  //LSB first
    case 'm':case'M': UCAxCTL0 |= UCMSB;             break;     //MSB first
}
}

void UartA2_setLMSBMode(char LMSBMode)           // set LSB or MSB mode
{
switch(LMSBMode)
{
    case 'l':case'L': UCA2CTL0 &= ~UCMSB;               break;  //LSB first
    case 'm':case'M': UCA2CTL0 |= UCMSB;             break;     //MSB first
}
}
//================================================================================
void Uart_setbitMode(int bitMode)                  // set bit mode
{
switch(bitMode)
{
    case '8': UCAxCTL0 &= ~UC7BIT;               break;  //LSB first
    case '7': UCAxCTL0 |= UC7BIT;                break;     //MSB first
}
}

void UartA2_setbitMode(int bitMode)                  // set bit mode
{
switch(bitMode)
{
    case '8': UCA2CTL0 &= ~UC7BIT;               break;  //LSB first
    case '7': UCA2CTL0 |= UC7BIT;                break;     //MSB first
}
}
//================================================================================
void Uart_setstopbitMode(int stopbitMode)         // set stop bit mode
{
switch(stopbitMode)
{
    case '1': UCAxCTL0 &= ~UCSPB;               break;  //LSB first
    case '2': UCAxCTL0 |= UCSPB;                break;     //MSB first
}
}

void UartA2_setstopbitMode(int stopbitMode)         // set stop bit mode
{
switch(stopbitMode)
{
    case '1': UCA2CTL0 &= ~UCSPB;               break;  //LSB first
    case '2': UCA2CTL0 |= UCSPB;                break;     //MSB first
}
}

//================================================================================
void Uart_sendchar(char zifu)                // send char
{
	while((UCAxIFG & UCAxTXIFG) == 0) _NOP();              // if transmit flag still not reset, means busy, trap and wait
	UCAxTXBUF = zifu;
}

void UartA2_sendchar(char zifu)                // send char
{
	while((UCA2IFG & UCTXIFG) == 0) _NOP();              // if transmit flag still not reset, means busy, trap and wait
	UCA2TXBUF = zifu;
}
//================================================================================
void Uart_sendstr(char *str)                 // send string
{
	while(*str)
	Uart_sendchar(*str++);
}

void UartA2_sendstr(char *str)                 // send string
{
	while(*str)
	UartA2_sendchar(*str++);
}
//================================================================================
char Uart_readchar()                         // reserve function , I don't know when to use this one, instead of interrupt one
{
	while((UCAxIFG & UCAxRXIFG) == 0) _NOP();              // if transmit flag still not reset, means busy, trap and wait
	return UCAxRXBUF;
}

char UartA2_readchar()                         // reserve function , I don't know when to use this one, instead of interrupt one
{
	while((UCA2IFG & UCRXIFG) == 0) _NOP();              // if transmit flag still not reset, means busy, trap and wait
	return UCA2RXBUF;
}
//================================================================================
void Uart_enableRXINT(){
	// enable uart RX interrupt
	UCAxIE |= UCRXIE;                         // enable USCI_A0 RX interrupt, enable interrupt much go after UCSWRST reset
}

void UartA2_enableRXINT(){
	// enable uart RX interrupt
	UCA2IE |= UCRXIE;                         // enable USCI_A0 RX interrupt, enable interrupt much go after UCSWRST reset
}
//================================================================================
void Uart_disableRXINT(){
	// disable uart RX interrupt
	UCAxIE &= ~UCRXIE;
}

void UartA2_disableRXINT(){
	// disable uart RX interrupt
	UCA2IE &= ~UCRXIE;
}
//================================================================================
void Uart_enableTXINT(){
	// enable uart TX interrupt
	UCAxIE |= UCTXIE;                         // enable USCI_A0 TX interrupt, enable interrupt much go after UCSWRST reset
}

void UartA2_enableTXINT(){
	// enable uart TX interrupt
	UCA2IE |= UCTXIE;                         // enable USCI_A0 TX interrupt, enable interrupt much go after UCSWRST reset
}
//================================================================================
void Uart_disableTXINT(){
	// disable uart TX interrupt
	UCAxIE &= ~UCTXIE;
}

void UartA2_disableTXINT(){
	// disable uart TX interrupt
	UCA2IE &= ~UCTXIE;
}
