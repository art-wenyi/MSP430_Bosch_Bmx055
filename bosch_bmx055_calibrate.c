/*
 * bosch_bmx055_calibrate.c
 *
 *  Created on: Jul 1, 2016
 *      Author: art
 */
//#include "msp430.h"
#include "bosch_bmx055_calibrate.h"
#include "bosch_bmx055_msp430.h"
//#include <msp430_math.h>
#include "math.h"


// below are simplified methods
void UpdateKF(struct KF *kf, float y, float u){
  if (kf->initial){
    kf->x0 = y;
    kf->x1 = 1.5;
  }
  kf->y = y;
  kf->u = u;
}

void InitializeKF(struct KF *kf, float w00, float w11, float v){
  kf->W00 = w00;
  kf->W11 = w11;
  kf->v = v;
}

void RunKF(struct KF *kf, float dt){
  kf->x0 = kf->x0 + (kf->u - kf->x1)*dt;
  if (kf->initial){
    kf->initial = false;
  } else {
    kf->M00 = kf->Z00 + dt*(dt*kf->Z11-kf->Z10-kf->Z01)+kf->W00;
    kf->M01 = kf->Z01 - dt*kf->Z11;
    kf->M10 = kf->Z10 - dt*kf->Z11;
    kf->M11 = kf->Z11 + kf->W11;
  }
  float tmp = kf->M00 + kf->v;
  kf->K0 = kf->M00/tmp;
  kf->K1 = kf->M10/tmp;

  kf->x0 = kf->x0 + kf->K0*(kf->y - kf->x0);
  kf->x1 = kf->x1 + kf->K1*(kf->y - kf->x0);

  if (kf->x0>PI){
    kf->x0 -= 2*PI;
  } else if (kf->x0<-PI){
    kf->x0 += 2*PI;
  }

  kf->Z00 = kf->M00*(1-kf->K0);
  kf->Z01 = kf->M01*(1-kf->K0);
  kf->Z10 = kf->M10-kf->K1*kf->M00;
  kf->Z11 = kf->M11-kf->K1*kf->M01;
}

void calibrate(struct Sensor_Calibrate *sensor_calibrate, struct Sensor *sensor){
	unsigned int count=0;
	unsigned int sample_count=6000;
	float mag_max[3] = {0,0,0};
	float mag_min[3] = {0,0,0};
	while(count<sample_count){
		count++;
		Gyro_Read_Data(sensor->gyro_x, sensor->gyro_y, sensor->gyro_z);
		Accel_Read_Data(sensor->accel_x, sensor->accel_y, sensor->accel_z);
		Mag_Read_Data(sensor->mag_x, sensor->mag_y, sensor->mag_z);
		// convert raw 2 byte data from float value
		sensor->convert(sensor);
		// send raw float data to calibrate
		sensor_calibrate->gxr = -sensor->gyro_x_float;
		sensor_calibrate->gyr = -sensor->gyro_z_float;
		sensor_calibrate->gzr = -sensor->gyro_y_float;
		sensor_calibrate->ax = -sensor->accel_x_float;
		sensor_calibrate->ay = -sensor->accel_z_float;
		sensor_calibrate->az = -sensor->accel_y_float;
		sensor_calibrate->mxr = sensor->mag_y_float;
		sensor_calibrate->myr = -sensor->mag_z_float;
		sensor_calibrate->mzr = -sensor->mag_x_float;
		if(count==1){
			mag_max[0]=sensor_calibrate->mxr;
			mag_min[0]=sensor_calibrate->mxr;
			mag_max[1]=sensor_calibrate->myr;
			mag_min[1]=sensor_calibrate->myr;
			mag_max[2]=sensor_calibrate->mzr;
			mag_min[2]=sensor_calibrate->mzr;
		}else{
			mag_max[0] = mag_max[0]>sensor_calibrate->mxr?mag_max[0]:sensor_calibrate->mxr;
			mag_min[0] = mag_min[0]<sensor_calibrate->mxr?mag_min[0]:sensor_calibrate->mxr;
			mag_max[1] = mag_max[1]>sensor_calibrate->myr?mag_max[1]:sensor_calibrate->myr;
			mag_min[1] = mag_min[1]<sensor_calibrate->myr?mag_min[1]:sensor_calibrate->myr;
			mag_max[2] = mag_max[2]>sensor_calibrate->mzr?mag_max[2]:sensor_calibrate->mzr;
			mag_min[2] = mag_min[2]<sensor_calibrate->mzr?mag_min[2]:sensor_calibrate->mzr;
		}
	}
	float base = (mag_max[0]+mag_max[1]+mag_max[2]-mag_min[0]-mag_min[1]-mag_min[2])/3;
	sensor_calibrate->mxb = (mag_max[0]+mag_min[0])/2;
	sensor_calibrate->myb = (mag_max[1]+mag_min[1])/2;
	sensor_calibrate->mzb = (mag_max[2]+mag_min[2])/2;
	sensor_calibrate->mxs = (mag_max[0]-mag_min[0])/base;
	sensor_calibrate->mys = (mag_max[1]-mag_min[1])/base;
	sensor_calibrate->mzs = (mag_max[2]-mag_min[2])/base;
}
