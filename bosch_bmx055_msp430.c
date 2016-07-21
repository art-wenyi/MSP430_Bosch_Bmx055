/*
 * bosch_bmx055_msp430.c
 *
 *  Created on: Jul 8, 2016
 *      Author: art
 */

#include "bosch_bmx055_msp430.h"
#include "i2c.h"


void BMX055_Init(){

	// Set-up I2C SDA and SCL pins
	P2SEL |= BIT1;
	P2SEL |= BIT2;

	//_delay_cycles(160000);
	I2C_Master_Init(I2C_SOURCE_CLOCK,I2C_SOURCE_CLOCK_FREQ,I2C_CLOCK_FREQ);		// init the i2c communication
	BMX055_Chip_Init();
}

void BMX055_Chip_Init(){
	// start with all sensors in default mode with all registers reset
	Gyro_Write_Register(BMX055_GYRO_BGW_SOFTRESET, 0xB6);  // reset gyro, 0xB6 is the password value for reset
	__delay_cycles(160000);
   	Gyro_Write_Register(BMX055_GYRO_RANGE, G_1000DPS);  // set GYRO FS range
 	Gyro_Write_Register(BMX055_GYRO_BW, G_100Hz12Hz);     // set GYRO ODR and Bandwidth
 	Gyro_Write_Register(BMX055_GYRO_LPM1, 0x00);			// set to normal mode, no suspend
 	Accel_Write_Register(BMX055_GYRO_RATE_HBW, 0x00);              // Use filtered data, read lsb before msb to ensure data integrity

 	// start with all sensors in default mode with all registers reset
   	Accel_Write_Register(BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
   	__delay_cycles(160000);
   	Accel_Write_Register(BMX055_ACC_PMU_RANGE, A_4G); // Set accelerometer full range
   	Accel_Write_Register(BMX055_ACC_PMU_BW, ABW_63Hz);     // Set accelerometer bandwidth
   	Accel_Write_Register(BMX055_ACC_PMU_LPW, 0x00);			// set to normal mode, no suspend
   	Accel_Write_Register(BMX055_ACC_D_HBW, 0x00);              // Use filtered data

   // Configure magnetometer
	Mag_Write_Register(BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, 0x82 is the password value for soft reset
	__delay_cycles(160000);
	Mag_Write_Register(BMX055_MAG_PWR_CNTL1, 0x01); // set to sleep mode, it's needed for state transit
	__delay_cycles(160000);
	Mag_Write_Register(BMX055_MAG_PWR_CNTL2, MODR_30Hz + 0x00); // Normal mode, 30hz

}

// gyro read
void Gyro_Read_Register(unsigned char gyro_registerName, unsigned char *readData)
{
  	I2C_Read_Packet(BMX055_GYRO_ADDRESS, gyro_registerName, 1, readData);
}
// gyro write
void Gyro_Write_Register(unsigned char gyro_registerName, unsigned char writeData)
{
	I2C_Write_Packet(BMX055_GYRO_ADDRESS, gyro_registerName, 1, &writeData);
}
// accel read
void Accel_Read_Register(unsigned char gyro_registerName, unsigned char *readData)
{
  	I2C_Read_Packet(BMX055_ACC_ADDRESS, gyro_registerName, 1, readData);
}
// accel write
void Accel_Write_Register(unsigned char gyro_registerName, unsigned char writeData)
{
	I2C_Write_Packet(BMX055_ACC_ADDRESS, gyro_registerName, 1, &writeData);
}
// mag read
void Mag_Read_Register(unsigned char gyro_registerName, unsigned char *readData)
{
  	I2C_Read_Packet(BMX055_MAG_ADDRESS, gyro_registerName, 1, readData);
}
// mag write
void Mag_Write_Register(unsigned char gyro_registerName, unsigned char writeData)
{
	I2C_Write_Packet(BMX055_MAG_ADDRESS, gyro_registerName, 1, &writeData);
}


// gyro info read, since the sensor update speed is far more faster than msp430 calculate speed(5ms vs 50ms)
// it's no need to check data available or not, or use interrupt
void Gyro_Read_Data(unsigned char *x, unsigned char *y, unsigned char *z){
	Gyro_Read_Register(BMX055_GYRO_RATE_X_LSB, x);
	Gyro_Read_Register(BMX055_GYRO_RATE_X_MSB, x+1);
	Gyro_Read_Register(BMX055_GYRO_RATE_Y_LSB, y);
	Gyro_Read_Register(BMX055_GYRO_RATE_Y_MSB, y+1);
	Gyro_Read_Register(BMX055_GYRO_RATE_Z_LSB, z);
	Gyro_Read_Register(BMX055_GYRO_RATE_Z_MSB, z+1);
}

// gyro info read
void Accel_Read_Data(unsigned char *x, unsigned char *y, unsigned char *z){
	Accel_Read_Register(BMX055_ACC_D_X_LSB, x);
	Accel_Read_Register(BMX055_ACC_D_X_MSB, x+1);
	Accel_Read_Register(BMX055_ACC_D_Y_LSB, y);
	Accel_Read_Register(BMX055_ACC_D_Y_MSB, y+1);
	Accel_Read_Register(BMX055_ACC_D_Z_LSB, z);
	Accel_Read_Register(BMX055_ACC_D_Z_MSB, z+1);
}

// gyro info read
void Mag_Read_Data(unsigned char *x, unsigned char *y, unsigned char *z){
	Mag_Read_Register(BMX055_MAG_XOUT_LSB, x);
	Mag_Read_Register(BMX055_MAG_XOUT_MSB, x+1);
	Mag_Read_Register(BMX055_MAG_YOUT_LSB, y);
	Mag_Read_Register(BMX055_MAG_YOUT_MSB, y+1);
	Mag_Read_Register(BMX055_MAG_ZOUT_LSB, z);
	Mag_Read_Register(BMX055_MAG_ZOUT_MSB, z+1);
}

// convert BMX055 sensor data to float
void Bmx_Convert_Data(struct Sensor *sensor){
	// gyro convert, 16 bits
	sensor->cov_tmp = ((int)sensor->gyro_x[1] << 8) | sensor->gyro_x[0];    // [0] is high bit, [1] is low bit
	sensor->gyro_x_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	sensor->cov_tmp = ((int)sensor->gyro_y[1] << 8) | sensor->gyro_y[0];    // [0] is high bit, [1] is low bit
	sensor->gyro_y_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	sensor->cov_tmp = ((int)sensor->gyro_z[1] << 8) | sensor->gyro_z[0];    // [0] is high bit, [1] is low bit
	sensor->gyro_z_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	// accel convert, 12 bits
	sensor->cov_tmp = (((int)sensor->accel_x[1] << 8) | sensor->accel_x[0]) >> 4;    // [0] is high bit, [1] is low bit
	sensor->accel_x_float = (float)sensor->cov_tmp * sensor->accel_reso;
	sensor->cov_tmp = (((int)sensor->accel_y[1] << 8) | sensor->accel_y[0]) >> 4;    // [0] is high bit, [1] is low bit
	sensor->accel_y_float = (float)sensor->cov_tmp * sensor->accel_reso;
	sensor->cov_tmp = (((int)sensor->accel_z[1] << 8) | sensor->accel_z[0]) >> 4;    // [0] is high bit, [1] is low bit
	sensor->accel_z_float = (float)sensor->cov_tmp * sensor->accel_reso;
	//mag convert, 13 bits for xy, 15 bits for z
	sensor->cov_tmp = (((int)sensor->mag_x[1] << 8) | sensor->mag_x[0]) >> 3;    // [0] is high bit, [1] is low bit
	sensor->mag_x_float = (float)sensor->cov_tmp * sensor->mag_reso;
	sensor->cov_tmp = (((int)sensor->mag_y[1] << 8) | sensor->mag_y[0]) >> 3;    // [0] is high bit, [1] is low bit
	sensor->mag_y_float = (float)sensor->cov_tmp * sensor->mag_reso;
	sensor->cov_tmp = (((int)sensor->mag_z[1] << 8) | sensor->mag_z[0]) >> 1;    // [0] is high bit, [1] is low bit
	sensor->mag_z_float = (float)sensor->cov_tmp * sensor->mag_reso;
}
