
#include <msp430f5659.h>
#include "bosch_bmx055_msp430.h"
#include "bosch_bmx055_calibrate.h"
#include "msp430_uart.h"
//#include "msp430_math.h"
#include "math.h"
#include "main.h"
/*
 * main.c
 */

// sensor for all data from bmx055
struct Sensor sensor = {
		.gyro_reso = 1000.0/32768.0,	// dafault use 1000dps
		.accel_reso = 4.0/2048.0,		// default use 4g/s
		.mag_reso = 0.3,					// fixed 0.3 uT/count
		.convert = &Bmx_Convert_Data
};

// calibrated sensor data
struct Sensor_Calibrate sensor_calibrate = {
	.mxb = 0,
	.myb = 0,
	.mzb = 0,
	.mxs = 1,
	.mys = 1,
	.mzs = 1,
	.dt = 0
};

struct EularAngle eular_angle = {
	.roll = 0,
	.pitch = 0,
	.yaw  = 0,
	.roll_degree = 0,
	.pitch_degree = 0,
	.yaw_degree =0
};

struct KF pitchKF, rollKF;
int calibrate_flag = 0;


int main(void) {
	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	UCSCTL3 |= SELREF_2; // Set DCO FLL reference = REFO    internal 32768hz crystal
	UCSCTL4 |= SELA_2; // Set ACLK = REFO, MCLK and SMCLK use default value as DCOCLKDIV
	__bis_SR_register(SCG0);
	// Disable the FLL control loop
	UCSCTL0 = 0x0000; // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_5; // Select DCO range 16MHz operation
	UCSCTL2 = FLLD_1 + 255; // Set DCO Multiplier for 8MHz
							// (N + 1) * FLLRef = Fdco
							// (255 + 1) * 32768 = 8MHz
							// Set FLL Div = fDCOCLK/2

	P4DIR |= BIT4;                            // P1.0 output
	__bic_SR_register(SCG0);
	// Enable the FLL control loop

	// Worst-case settling time for the DCO when the DCO range bits have been
	// changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	// UG for optimization.
	// 32 x 32 x 8 MHz / 32,768 Hz ~ 262000 = MCLK cycles for DCO to settle
	__delay_cycles(262000);
	  P1DIR |= BIT2;
	  P1OUT |= BIT2;				// Feedback to button control chip, prevent reset
	// Loop until XT1,XT2 & DCO fault flag is cleared
	do {
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
		// Clear XT2,XT1,DCO fault flags
		SFRIFG1 &= ~OFIFG; // Clear fault flags
	} while (SFRIFG1 & OFIFG); // Test oscillator fault flag

	//== clock init finished, below are the main function ==
	BMX055_Init();		// sensor initiation
	UartA2_Init(115200, 'n', 'l', '8', 1);		  // init uart 2
	buttonInit();
	__enable_interrupt();
	__delay_cycles(800000);


	// setting ta1, for calculate the exec time
	TA1CTL = TASSEL_2 | ID__8 | MC_2 | TACLR;         // SMCLK, clock div by 8, continuous mode, clear TAR

	char sensor_data_disp[16];
	struct KF pitchKF, rollKF;
	pitchKF.initial = true;
	rollKF.initial = true;
	InitializeKF(&pitchKF,0.0002,0.0004,0.5);
	InitializeKF(&rollKF,0.0002,0.0004,0.5);
	trimBMX055(&sensor);		// get trim data from sensor
	//unsigned char tmp;
	while(1){
		getOrientatoin(&sensor, &sensor_calibrate, &pitchKF, &rollKF, &eular_angle);
//		float pitch_accl2,roll_accl2;
//		pitch_accl2 = sensor_calibrate.pitch_accl * RAD2DEG;
//		roll_accl2  = sensor_calibrate.roll_accl * RAD2DEG;
		// display to pc from uart
		ftos(eular_angle.roll_degree, sensor_data_disp, 1);
		UartA2_sendstr(sensor_data_disp);
		UartA2_sendstr("   ");
		ftos(eular_angle.pitch_degree, sensor_data_disp, 1);
		UartA2_sendstr(sensor_data_disp);
		UartA2_sendstr("   ");
		ftos(eular_angle.yaw_degree, sensor_data_disp, 1);
		UartA2_sendstr(sensor_data_disp);
		UartA2_sendstr("   \n");
		_nop();

		if(calibrate_flag==1){
			calibrate_flag=0;
			//calibrate(&sensor_calibrate, &sensor);
			calibrate_4axis(&sensor, &sensor_calibrate, &pitchKF, &rollKF, &eular_angle);
		}
	}

}

/* Push Button Interrupt Service Routine */
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){

	/* Executed on Push Button Interrupt */
	if (P1IFG & BIT1){
		P1IFG &= ~BIT1;												// Clear the Button Interrupt Flag
		calibrate_flag=1;
	}
}
//========================================================
//========================================================
void buttonInit(void){
	P1DIR &= ~BIT1;											// Button is an input
	P1OUT |= BIT1;                      						// Pull-up resistor
	P1REN |= BIT1;                      						// Resistor enabled
	P1DS  &= ~BIT1;
	P1IES |= BIT1;                      						// Interrupt on high-to-low transition
	P1IE |= BIT1;                       						// Interrupt enable
	P1IFG &= ~BIT1;											// Clear Interrupt Flag
}
//========================================================
// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    if(x==0)		// if only is 0
    	str[i++] = '0';
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// float to string
void ftos(float n, char *res, int afterpoint) 		// after point is the resolution after '.', code from geeksforgeeks web
{
	int ipart;
	float fpart;
	int i;
    if(n>0){
		// Extract integer part
		ipart = (int)n;
		// Extract floating part
		fpart = n - (float)ipart;
		// convert integer part to string
		i = intToStr(ipart, res, 0);
    }else{
    	// Extract integer part
    	n=-n;
		ipart = (int)(n);
		// Extract floating part
		fpart = n - (float)ipart;
		// convert integer part to string
		res[0]='-';
		i = intToStr(ipart, res+1, 0) + 1;
    }
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * powf(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
