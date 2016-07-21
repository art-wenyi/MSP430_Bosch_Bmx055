#include <msp430f5659.h>
#include "i2c.h"

// Display Local Variables
unsigned char TXByteCtr;
unsigned char TxDataBuff[2];
unsigned char *PTxData;
unsigned char TXDataBuff_Count = 0;

unsigned char tx_reg_name_display;
// tx_length is how many bytes of data will be written
unsigned char tx_length_display;
// *tx_data will point to the data to be written
unsigned char tx_data_display;
// rx_reg_name is the register the MSP430 will read data from
unsigned char rx_reg_name_display;
// rx_length is how many bytes of data will be read
unsigned char rx_length_display;
// *rx_data will point to the data to be read
unsigned char *rx_data_display;
// receive flag is used for the ISR to know the MSP430 will be reading from the
// Accelerometer
unsigned char i2c_receive_flag_display;
// transmit flag is used for the ISR to know the MSP430 will be writing to the
// Accelerometer
unsigned char i2c_transmit_flag_display;
// Used to keep track how many bytes have been received
unsigned char rx_byte_ctr_display;
// Used to keep track how many bytes have been transmitted
unsigned char tx_byte_ctr_display;

// Gyro Local Variables
// tx_reg_name is the register the MSP430 will write data to
unsigned char tx_reg_name;
// tx_length is how many bytes of data will be written
unsigned char tx_length;
// *tx_data will point to the data to be written
unsigned char *tx_data;

// rx_reg_name is the register the MSP430 will read data from
unsigned char rx_reg_name;
// rx_length is how many bytes of data will be read
unsigned char rx_length;
// *rx_data will point to the data to be read
unsigned char *rx_data;

// receive flag is used for the ISR to know the MSP430 will be reading from the
// Gyro
unsigned char i2c_receive_flag;
// transmit flag is used for the ISR to know the MSP430 will be writing to the
// Gyro
unsigned char i2c_transmit_flag;

// Used to keep track how many bytes have been received
unsigned char rx_byte_ctr;
// Used to keep track how many bytes have been transmitted
unsigned char tx_byte_ctr;

///////////////////////////////////////////////////////////////////////////// Display begins/////////////////////////////////////////////////////////////////////////////
void I2C_Master_Display_Init(unsigned char selectClockSource,
				     unsigned long clockSourceFrequency,
				 	 unsigned long desiredI2CClock){
  // Set-up I2C SDA and SCL pins
	 P8SEL |= BIT5;             // use UCB1SDA, P8.5
	 P8SEL |= BIT6;				// use UCB1SCL, P8.6
	 // Disable the USCI Module, for edit settings to USCI register
	 I2C_Display_Disable();

	 UCB1CTL1 |= UCSSEL_2 + UCSWRST; 						//Select SMCLK			 											// Select ACLK
	 UCB1BR0 = (unsigned short) (clockSourceFrequency/desiredI2CClock);
	 UCB1BR1 = (unsigned short) ( (clockSourceFrequency/desiredI2CClock) >> 8 );
	 UCB1I2CSA = I2C_Display_Write_ADDR;
	 UCB1CTL0 |= UCMST + UCMODE_3 + UCSYNC;

	 I2C_Display_Enable();
	 UCB1IE |= UCTXIE;
}


void I2C_Display_Set_Slave_Address(unsigned char slaveAddress)
{
  // Set the address of the slave with which the master will communicate.
  UCB1I2CSA = slaveAddress;
}

void I2C_Display_Enable(void)
{
	UCB1CTL1 &= ~(UCSWRST);
}

void I2C_Display_Disable(void)
{
	UCB1CTL1 |= (UCSWRST);
}

void I2C_Display_Set_Mode(unsigned short receive){
	  UCB1CTL1 |= UCSWRST;
	  if (receive){
	    // Configure in receive mode
	    UCB1CTL1 &= ~(UCTR);
	  }
	  else{
	    // Configure in transmit mode
	    UCB1CTL1 |= UCTR;
	  }
}

unsigned short I2C_Display_Bus_Busy(void){
  // Return the bus busy status.
  if(UCB1STAT & UCBBUSY){
      return(0x00);
  }
  else{
      return(0x01);
  }
}

unsigned short I2C_Display_Busy(void){
  // Return the busy status.
  if((UCB1IFG & UCB1TXIFG) || (UCB1IFG & UCB1RXIFG)){
      return(0x00);
  }
  else{
      return(0x01);
  }
}

void I2C_Display_Interrupt_Enable(unsigned char interruptFlags){
  // Enable the interrupt masked bit
  UCB1IE |= interruptFlags;
}

void I2C_Display_Interrupt_Disable(unsigned char interruptFlags){
  // Disable the interrupt masked bit
  UCB1IE &= ~(interruptFlags);
}

void I2C_Display_Interrupt_Clear(unsigned char interruptFlags){
  // Clear the I2C interrupt source.
  UCB1IFG &= ~(interruptFlags);
}

unsigned char I2C_Display_Interrupt_Status(unsigned char mask){
  // Return the interrupt status of the request masked bit.
  return (UCB1IFG & mask);
}

void I2C_Write_Packet_To_Display(unsigned char displayRegName,
						    unsigned char dataLength,
						    unsigned char writeData){

	  // Assign values to global variables
	  TxDataBuff[0] = displayRegName;
	  TxDataBuff[1] = writeData;
	  TXByteCtr = 2;
	  TXDataBuff_Count = 0;

	  // Set Slave Address to [0x3C + UCTR] (Transmit Bit)
	  UCB1I2CSA = I2C_Display_Write_ADDR;
	  I2C_Display_Interrupt_Enable(UCTXIE);
	  I2C_Display_Interrupt_Enable(UCNACKIE);
	  UCB1CTL1 |= UCTR + UCTXSTT;            							// Send I2C start condition
    // this process send slave address automatically, once slave acknowledged and TXBuffer ready for new data, TXIFG set automatically
      __bis_SR_register(GIE);       									// CPU off, enable interrupts
      _no_operation();
	   while(UCB1CTL1 & UCTXSTP);          								// Ensure stop condition got sent

}
///////////////////////////////////////////////////////////////////////////// Display ends/////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////// Gyro begins /////////////////////////////////////////////////////////////////////////////
/**
* @brief <b>Function Name</b>:     : I2C_Master_Init
* @brief  <b>Description</b>: Initializes the I2C Master Block. Upon succesful
* initialization of the I2C master block, this function will have set the bus
* speed for the master, and will have enabled the I2C Master block.
* @param Input Parameters:
* <BR> unsigned char <b>selectClockSource</b> selects Clock source SMCLK (0x00)
* or ACLK (0x1)
* <BR> unsigned long <b>clockSourceFrequency</b> is the frequency of the
* selected clock source
* <BR> unsigned long <b>desiredI2CClock</b> is the desired clock rate for I2C
* communication
* @return Return Values: None
**/
void I2C_Master_Init(unsigned char selectClockSource,
				     unsigned long clockSourceFrequency,
				 	 unsigned long desiredI2CClock)
{

	P2SEL |= BIT1;             // use UCB0SDA, P2.1
	P2SEL |= BIT2;				// use UCB0SCL, P2.2
//	P2DIR &= ~BIT7;
//	P2REN = BIT7;
//	P2IES = BIT7;			// set p2.7 interrupt as high-to-low transition
//	P2IE  = BIT7;			// enable p2.7 interrupt
  // Disable the USCI Module
	I2C_Disable();
  UCBxCTL1 |= UCSWRST + UCSSEL_2;		   //Select SMCLK
  // Set the Baud rate
  UCBxBR0 = (unsigned short) (clockSourceFrequency/desiredI2CClock);
  UCBxBR1 = (unsigned short) ( (clockSourceFrequency/desiredI2CClock) >> 8 );
  /*!
  * Configure as I2C master mode.
  * UCMST = Master mode
  * UCMODE_3 = I2C mode
  * UCSYNC = Synchronous mode
  */
  UCBxCTL0 |= UCMST + UCMODE_3 + UCSYNC;

  I2C_Enable();
  UCB0IE |= UCTXIE;

}

/**
* @brief <b>Function Name</b>:     : I2C_Set_Slave_Address
* @brief  <b>Description</b>: This function will set the address that the I2C
* Master will place on the bus when initiating a transaction.
* @param Input Parameters:
* <BR> unsigned char <b>slaveAddress</b> is the address of the slave
* @return Return Values: None
**/
void I2C_Set_Slave_Address(unsigned char slaveAddress)
{
  // Set the address of the slave with which the master will communicate.
  UCBxI2CSA = slaveAddress;
}

/**
* @brief <b>Function Name</b>:     : I2C_Enable
* @brief  <b>Description</b>: Enable the I2C block
* @param Input Parameters: None
* @return Return Values: None
**/
void I2C_Enable(void)
{
  // Reset the UCSWRST bit to enable the USCI Module
  UCBxCTL1 &= ~(UCSWRST);
}

/**
* @brief <b>Function Name</b>:     : I2C_Disable
* @brief  <b>Description</b>: Disable the I2C block
* @param Input Parameters: None
* @return Return Values: None
**/
void I2C_Disable(void)
{
  // Reset the UCSWRST bit to enable the USCI Module
  UCBxCTL1 |= (UCSWRST);
}

/**
* @brief <b>Function Name</b>:     : I2C_Set_Mode
* @brief  <b>Description</b>: When the receive parameter is set to 0x01, the
* address will indicate that the I2C module is in receive mode; otherwise, the
* I2C module is in transmit mode
* @param Input Parameters:
* <BR> unsigned short <b>receive</b> can be set to a 0x01 to be in receive mode,
*  or 0x00 to be in transmit mode
* @return Return Values: None
**/
void I2C_Set_Mode(unsigned short receive)
{
  // Disable the USCI module
  UCBxCTL1 |= UCSWRST;

  if (receive)
  {
    // Configure in receive mode
    UCBxCTL1 &= ~(UCTR);
  }
  else
  {
    // Configure in transmit mode
    UCBxCTL1 |= UCTR;
  }
}

/**
* @brief <b>Function Name</b>:     : I2C_Bus_Busy
* @brief  <b>Description</b>: This function returns an indication of whether or
* not the I2C bus is busy. This function can be used in a multi-master
* enviroment to determine if another master is currently using the bus. This
* function checks the status of the bus via UCBBUSY bit in the UCBxSTAT register.
* @param Input Parameters: None
* @return Return Values:
* <BR>Returns 0x01 if the I2C master is busy; otherwise, returns 0x00.
**/
unsigned short I2C_Bus_Busy(void)
{
  // Return the bus busy status.
  if(UCBxSTAT & UCBBUSY)
  {
      return(0x00);
  }
  else
  {
      return(0x01);
  }
}

/**
* @brief <b>Function Name</b>:     : I2C_Busy
* @brief  <b>Description</b>: This function returns an indication of whether or
* not the I2C module is busy transmitting or receiving data. This function
* checks if the transmit or receive flag is set.
* @param Input Parameters: None
* @return Return Values:
* <BR>Returns 0x01 if the I2C master is busy; otherwise, returns 0x00.
**/
unsigned short I2C_Busy(void)
{
  // Return the busy status.
  if((UCBxIFG & UCBxTXIFG) || (UCBxIFG & UCBxRXIFG))
  {
      return(0x00);
  }
  else
  {
      return(0x01);
  }
}

/**
* @brief <b>Function Name</b>:     : I2C_Interrupt_Enable
* @brief  <b>Description</b>: Enables Individual I2C interrupts
* @param Input Parameters:
* <BR> unsigned char <b>interruptFlags</b> is the logical OR of any of the
* following:
*  <BR> I2C_INT_STOP - STOP condition interrupt
*  <BR> I2C_INT_START - START condition interrupt
*  <BR> I2C_INT_DATA_RX -Receive interrupt
*  <BR> I2C_INT_DATA_TX - Transmit interrupt
*  <BR> I2C_INT_NACK - Not-acknowledge interrupt
*  <BR> I2C_INT_CALIFG - Arbitration lost interrupt
* @return Return Values: None
**/
void I2C_Interrupt_Enable(unsigned char interruptFlags)
{
  // Enable the interrupt masked bit
  UCBxIE |= interruptFlags;
}

/**
* @brief <b>Function Name</b>:     : I2C_Interrupt_Disable
* @brief  <b>Description</b>: Disables Individual I2C interrupts
* @param Input Parameters:
* <BR> unsigned char <b>interruptFlags</b> is the logical OR of any of the
* following:
*  <BR> I2C_INT_STOP - STOP condition interrupt
*  <BR> I2C_INT_START - START condition interrupt
*  <BR> I2C_INT_DATA_RX -Receive interrupt
*  <BR> I2C_INT_DATA_TX - Transmit interrupt
*  <BR> I2C_INT_NACK - Not-acknowledge interrupt
*  <BR> I2C_INT_CALIFG - Arbitration lost interrupt
* @return Return Values: None
**/
void I2C_Interrupt_Disable(unsigned char interruptFlags)
{
  // Disable the interrupt masked bit
  UCBxIE &= ~(interruptFlags);
}

/**
* @brief <b>Function Name</b>:     : I2C_Interrupt_Clear
* @brief  <b>Description</b>: Clears Individual I2C interrupts flags
* @param Input Parameters:
* <BR> unsigned char <b>interruptFlags</b> is the logical OR of any of the
* following:
*  <BR> I2C_INT_STOP - STOP condition interrupt
*  <BR> I2C_INT_START - START condition interrupt
*  <BR> I2C_INT_DATA_RX -Receive interrupt
*  <BR> I2C_INT_DATA_TX - Transmit interrupt
*  <BR> I2C_INT_NACK - Not-acknowledge interrupt
*  <BR> I2C_INT_CALIFG - Arbitration lost interrupt
* @return Return Values: None
**/
void I2C_Interrupt_Clear(unsigned char interruptFlags)
{
  // Clear the I2C interrupt source.
  UCBxIFG &= ~(interruptFlags);
}

/**
* @brief <b>Function Name</b>:     : I2C_Interrupt_Status
* @brief  <b>Description</b>: Returns the interrupt status for the I2C module
* based on which flag is passed.
* @param Input Parameters:
* <BR> unsigned char <b>mask</b> is the masked interrupt flag status to be
* returned.
*  <BR> I2C_INT_STOP - STOP condition interrupt
*  <BR> I2C_INT_START - START condition interrupt
*  <BR> I2C_INT_DATA_RX -Receive interrupt
*  <BR> I2C_INT_DATA_TX - Transmit interrupt
*  <BR> I2C_INT_NACK - Not-acknowledge interrupt
*  <BR> I2C_INT_CALIFG - Arbitration lost interrupt
* @return Return Values:
* <BR>The current interrupt status, returned as 0x01 if active, otherwise 0x00.
**/
unsigned char I2C_Interrupt_Status(unsigned char mask)
{
  // Return the interrupt status of the request masked bit.
  return (UCBxIFG & mask);
}


// ===
void I2C_Read_Packet(unsigned char device_address, unsigned char gyroRegName, int dataLength, unsigned char *readData){
	// Assign values to global variables
	  rx_reg_name = gyroRegName;
	  rx_length = dataLength;
	  rx_data = readData;

	  // Reset variables for transmission
	  i2c_transmit_flag = 0;
	  i2c_receive_flag = 1;
	  tx_byte_ctr = 1;
	  rx_byte_ctr = 0;
	  UCBxI2CSA = device_address;

	  // Enable TX Interrupt to send write address, and register
	  __bic_SR_register(GIE);
	  I2C_Interrupt_Enable(UCBxTXIE);
	  I2C_Interrupt_Enable(UCNACKIE);
	  UCBxCTL1 |= UCTR + UCTXSTT;            							// Send I2C start condition
	  	__bis_SR_register(GIE);       									// CPU off, enable interrupts
	  	_no_operation();
	     while(UCBxSTAT != 0x00);          								// Ensure stop condition got sent

}


// ===
void I2C_Write_Packet(unsigned char device_address, unsigned char gyroRegName, int dataLength, unsigned char *writeData){
	  // Assign values to global variables
	  tx_reg_name = gyroRegName;
	  tx_length = dataLength;
	  tx_data = writeData;

	  // Reset variables for transmission
	  i2c_transmit_flag = 1;
	  i2c_receive_flag = 0;
	  tx_byte_ctr = 1;
	  UCBxI2CSA = device_address;

	  // Enable TX Interrupt to send write address, register and data
	  __bic_SR_register(GIE);
	  I2C_Interrupt_Enable(UCBxTXIE);
	  I2C_Interrupt_Enable(UCNACKIE);
	  UCBxCTL1 |= UCTR + UCTXSTT;            							// Send I2C start condition
		__bis_SR_register(GIE);       									// CPU off, enable interrupts
		_no_operation();
	   while(UCBxSTAT != 0x00);          								// Ensure stop condition got sent

	  // Enter Low Power Mode 0
	  //__bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
	  //__bis_SR_register(GIE);
}

///////////////////////////////////////////////////////////////////////////// Gyro ends /////////////////////////////////////////////////////////////////////////////

#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_I2C_Display_ISR(void){
  switch(__even_in_range(UCB1IV,12)){
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4: 		      						// Vector  4: NACKIFG
    _no_operation();
	  break;
  case  6: break;                           // Vector  6: STTIFG
  case  8: break;                           // Vector  8: STPIFG
  case 10: 									// Vector 10: RXIFG
  	break;
  case 12:         // Vector 12: TXIFG
  if(TXByteCtr--){
	   UCB1TXBUF = TxDataBuff[TXDataBuff_Count++];
   	   }

  else{
       UCB1CTL1 |= UCTXSTP;                  					// Send I2C stop condition
       UCB1IFG &~ UCTXIFG;
       //if(sleep_exit == FALSE)
       //__bic_SR_register_on_exit(LPM4_bits); 					 // Exit active CPU
   	   }
   break;
  default: break;
  }
}


#pragma vector = USCI_Bx_VECTOR
__interrupt void USCI_I2C_ISR(void)
{
  switch(__even_in_range(UCB0IV,12))
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4: 		                            // Vector  4: NACKIFG
/*  	__delay_cycles(4200);					// 500 usec delay upon NACK
    UCBxCTL1 |= UCTXSTT;					// Re-start condition
   // In the case of reading one byte, poll on start condition bit
   if((rx_length == 1) && (tx_byte_ctr == 3))
   {
    // Wait for start condition to be transmitted
   	 while( UCBxCTL1 & UCTXSTT );
   	 UCBxCTL1 |= UCTXSTP;               // Send I2C stop condition
   }
   else
   {
    tx_byte_ctr = 2;  // change from 1 to 2
   }		*/
	  //while( UCBxCTL1 & UCTXSTT );
	     	 UCBxCTL1 |= UCTXSTP;               // Send I2C stop condition
	     	UCBxIFG = 0;

    break;
  case  6: break;                           // Vector  6: STTIFG
  case  8: break;                           // Vector  8: STPIFG
  case 10: 									// Vector 10: RXIFG
  	if(rx_byte_ctr < rx_length-1)
  	{
  	  *rx_data++= UCBxRXBUF;				// Read receive buffer
  	  rx_byte_ctr++;
  	  if(rx_byte_ctr == rx_length-1)
  	  {
  	  	UCBxCTL1 |= UCTXSTP;                  // Send I2C stop condition (it will transmit a NACK signal before stop condition sent)
  	  }
  	}
  	else
  	{
      // Clear RX Flag
      i2c_receive_flag = 0;
  	  *rx_data = UCBxRXBUF;
      I2C_Interrupt_Disable(UCNACKIE);
  	  I2C_Interrupt_Disable(UCBxRXIE);
//  	  if(sleep_exit == FALSE)
//  	  __bic_SR_register_on_exit(LPM0_bits);  // Exit active CPU
  	}
  	break;
  case 12:                                  // Vector 12: TXIFG
   if(i2c_transmit_flag)
   {
     /*if(tx_byte_ctr == 0)
     {
       while (UCBxCTL1 & UCTXSTP);          // Ensure stop condition got sent
       UCBxCTL1 |= UCTR+UCTXSTT;            // Send I2C start condition
       tx_byte_ctr++;
     } */
     if(tx_byte_ctr == 1)
     {
	   UCBxTXBUF = tx_reg_name;			// Send Register name
	   tx_byte_ctr++;
     }
     else if(tx_length > 0)
     {
       UCBxTXBUF = *tx_data++;    		// Send data bytes
	   tx_length--;
     }
     else
     {
       UCBxCTL1 |= UCTXSTP;                  // Send I2C stop condition
  	   I2C_Interrupt_Disable(UCBxTXIE);
       I2C_Interrupt_Disable(UCNACKIE);
       i2c_transmit_flag = 0;
//       if(sleep_exit == FALSE)
//       __bic_SR_register_on_exit(LPM0_bits);  // Exit active CPU
     }
   }
   else if(i2c_receive_flag)
   {
     /* if(tx_byte_ctr == 0)
     {
       while (UCBxCTL1 & UCTXSTP);          // Ensure stop condition got sent
       UCBxCTL1 |= UCTR+UCTXSTT;            // I2C start condition, send slave address
       tx_byte_ctr++;
     } */
     if(tx_byte_ctr == 1)
     {
	   UCBxTXBUF = rx_reg_name;			// Send Register name
	   tx_byte_ctr++;
     }
     else if(tx_byte_ctr == 2)
     {
       UCBxCTL1 &= ~(UCTR);					// Enable receive bit
       UCBxCTL1 |= UCTXSTT;            		// Send I2C start condition, send slave address, reveive mode
       // In the case of reading one byte, poll on start condition bit, this defined by msp430
       if(rx_length == 1)
  	   {
        // Wait for start condition to be transmitted
  	   	 while( UCBxCTL1 & UCTXSTT );
  	   	 UCBxCTL1 |= UCTXSTP;               // Send I2C stop condition
  	   }
       I2C_Interrupt_Disable(UCBxTXIE);		// Disable TX interrupt
       I2C_Interrupt_Enable(UCBxRXIE);		// Enable RX interrupt
       tx_byte_ctr++;
     }
   }
   break;
  default: break;
  }
}



