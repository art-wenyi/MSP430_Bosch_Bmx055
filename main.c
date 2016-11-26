
#include <msp430f5659.h>
#include "bosch_bmx055_msp430.h"
#include "bosch_bmx055_calibrate.h"
#include "msp430_uart.h"
#include "math.h"

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
	.dt = 0
};

struct KF pitchKF, rollKF;

// use for convert from float to string
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftos(float n, char *res, int afterpoint);


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
	__enable_interrupt();


	// setting ta1, for calculate the exec time
	TA1CTL = TASSEL_2 | ID__8 | MC_2 | TACLR;         // SMCLK, clock div by 8, continuous mode, clear TAR

	char sensor_data_disp[16];
	float roll,pitch,yaw;
	struct KF pitchKF, rollKF;
	pitchKF.initial = true;
	rollKF.initial = true;
	//unsigned char tmp;
	while(1){
		sensor_calibrate.t_elapse =	TA1R;		// get a elapse time, for calibrate calculation
		TA1CTL |= TACLR;		// clear ta1 count
		// read data from sensor
		Gyro_Read_Data(sensor.gyro_x, sensor.gyro_y, sensor.gyro_z);
		Accel_Read_Data(sensor.accel_x, sensor.accel_y, sensor.accel_z);
		Mag_Read_Data(sensor.mag_x, sensor.mag_y, sensor.mag_z);
		// convert raw 2 byte data from float value
		sensor.convert(&sensor);
		// send raw float data to calibrate
		sensor_calibrate.gxr = -sensor.gyro_x_float;
		sensor_calibrate.gyr = -sensor.gyro_z_float;
		sensor_calibrate.gzr = -sensor.gyro_y_float;
		sensor_calibrate.ax = -sensor.accel_x_float;
		sensor_calibrate.ay = -sensor.accel_z_float;
		sensor_calibrate.az = -sensor.accel_y_float;
		sensor_calibrate.mxr = sensor.mag_y_float;
		sensor_calibrate.myr = -sensor.mag_z_float;
		sensor_calibrate.mzr = -sensor.mag_x_float;
		// acutal algorithm for calibrate

		// use quaterion filter method
//		PreProcess(&sensor_calibrate);
//		UpdateGDFilter_MARG(&sensor_calibrate);
//		roll = getRoll(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
//		pitch = getPitch(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
//		yaw = getYaw(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);

		// use simplified method
		sensor_calibrate.mx = (sensor_calibrate.mxr - MXB)/MXS;
		sensor_calibrate.my = (sensor_calibrate.myr - MYB)/MYS;
		sensor_calibrate.mz = (sensor_calibrate.mzr - MZB)/MZS;

		sensor_calibrate.gx = sensor_calibrate.gxr - GXB;
		sensor_calibrate.gy = sensor_calibrate.gyr - GYB;
		sensor_calibrate.gz = sensor_calibrate.gzr - GZB;

		sensor_calibrate.pitch_accl = -atan2(sensor_calibrate.ax,sqrt(sensor_calibrate.ay*sensor_calibrate.ay + sensor_calibrate.az*sensor_calibrate.az));
		sensor_calibrate.roll_accl = atan2(sensor_calibrate.ay, sensor_calibrate.az);

		sensor_calibrate.dt = (float)sensor_calibrate.t_elapse * 0.000001;		// convert dt to second
		UpdateKF(&pitchKF,sensor_calibrate.pitch_accl,sensor_calibrate.gy*DEG2RAD);
		RunKF(&pitchKF, sensor_calibrate.dt);
		UpdateKF(&rollKF,sensor_calibrate.roll_accl,sensor_calibrate.gx*DEG2RAD);
		RunKF(&rollKF, sensor_calibrate.dt);

		pitch = pitchKF.x0;
		roll = rollKF.x0;

		sensor_calibrate.MX = sensor_calibrate.mx*cos(pitch) + sensor_calibrate.my*sin(pitch)*sin(roll) + sensor_calibrate.mz*sin(pitch)*cos(roll);
		sensor_calibrate.MY = sensor_calibrate.my*cos(roll) - sensor_calibrate.mz*sin(roll);

		float yaw_mag = -atan2(sensor_calibrate.MY, sensor_calibrate.MX);

		yaw = yaw * 0.75 + yaw_mag*0.25;

		// display to pc from uart
		ftos(roll, sensor_data_disp, 3);
		UartA2_sendstr(sensor_data_disp);
		UartA2_sendstr("   ");
		ftos(pitch, sensor_data_disp, 3);
		UartA2_sendstr(sensor_data_disp);
		UartA2_sendstr("   ");
		ftos(yaw, sensor_data_disp, 3);
		UartA2_sendstr(sensor_data_disp);
		UartA2_sendstr("   \n");
		_nop();
	}

}


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
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
