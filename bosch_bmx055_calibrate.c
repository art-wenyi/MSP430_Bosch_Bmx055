/*
 * bosch_bmx055_calibrate.c
 *
 *  Created on: Jul 1, 2016
 *      Author: art
 */
#include "msp430f5659.h"
#include "bosch_bmx055_calibrate.h"
#include "bosch_bmx055_msp430.h"
#include "msp430_uart.h"
#include "math.h"
#include "main.h"
#include "Display.h"

// get all info from bosch sensor, and calculate roll pitch yaw
void getOrientatoin(struct Sensor *sensor, struct Sensor_Calibrate *sensor_calibrate, struct KF *pitchKF, struct KF *rollKF, struct EularAngle *eular_angle){
	// read data from sensor
	Gyro_Read_Data(sensor->gyro_x, sensor->gyro_y, sensor->gyro_z);
	Accel_Read_Data(sensor->accel_x, sensor->accel_y, sensor->accel_z);
	Mag_Read_Data(sensor->mag_x, sensor->mag_y, sensor->mag_z);
	Temp_Read_Data(sensor->temperature);
	sensor_calibrate->t_elapse =	TA1R;		// get a elapse time, for calibrate calculation
	TA1CTL |= TACLR;		// clear ta1 count
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
	// acutal algorithm for calibrate

	// use quaterion filter method
//		PreProcess(&sensor_calibrate);
//		UpdateGDFilter_MARG(&sensor_calibrate);
//		roll = getRoll(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
//		pitch = getPitch(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
//		yaw = getYaw(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);

	// use simplified method
	sensor_calibrate->mx = (sensor_calibrate->mxr - sensor_calibrate->mxb)/sensor_calibrate->mxs;
	sensor_calibrate->my = (sensor_calibrate->myr - sensor_calibrate->myb)/sensor_calibrate->mys;
	sensor_calibrate->mz = (sensor_calibrate->mzr - sensor_calibrate->mzb)/sensor_calibrate->mzs;

	sensor_calibrate->gx = sensor_calibrate->gxr - GXB;
	sensor_calibrate->gy = sensor_calibrate->gyr - GYB;
	sensor_calibrate->gz = sensor_calibrate->gzr - GZB;

	sensor_calibrate->pitch_accl = -atan2f(sensor_calibrate->ax,sqrtf(sensor_calibrate->ay*sensor_calibrate->ay + sensor_calibrate->az*sensor_calibrate->az));
//	sensor_calibrate->roll_accl = atan2f(sensor_calibrate->ay, sensor_calibrate->az);
	sensor_calibrate->roll_accl = atan2f(sensor_calibrate->ay, sqrtf(sensor_calibrate->ax*sensor_calibrate->ax + sensor_calibrate->az*sensor_calibrate->az));

	sensor_calibrate->dt = (float)sensor_calibrate->t_elapse * 0.000001;		// convert dt to second
	UpdateKF(pitchKF, sensor_calibrate->pitch_accl, sensor_calibrate->gy*DEG2RAD);
	RunKF(pitchKF, sensor_calibrate->dt);
	UpdateKF(rollKF, sensor_calibrate->roll_accl, sensor_calibrate->gx*DEG2RAD);
	RunKF(rollKF, sensor_calibrate->dt);



	eular_angle->pitch = pitchKF->x0;
	eular_angle->roll = rollKF->x0;
	float sin_pitch = sinf(eular_angle->pitch);
	float cos_roll = cosf(eular_angle->roll);
	float sin_roll = sinf(eular_angle->roll);

	sensor_calibrate->MX = sensor_calibrate->mx*cosf(eular_angle->pitch) + sensor_calibrate->my*sin_pitch*sin_roll + sensor_calibrate->mz*sin_pitch*cos_roll;
	sensor_calibrate->MY = sensor_calibrate->my*cos_roll - sensor_calibrate->mz*sin_roll;


	float yaw_mag = -atan2f(sensor_calibrate->MY, sensor_calibrate->MX);

	sensor_calibrate->t_elapse =	TA1R;
	sensor_calibrate->dt = (float)sensor_calibrate->t_elapse * 0.000001;		// convert dt to second

	//yaw = yaw * 0.75 + yaw_mag*0.25;

	if (abs(eular_angle->yaw)>(0.5*PI) && abs(yaw_mag)>(0.5*PI)){
	  float sgn_yaw = eular_angle->yaw > 0 ? 1.0 : -1.0;
	  float sgn_yawmag = yaw_mag > 0 ? 1.0 : -1.0;
	  if (sgn_yaw != sgn_yawmag){
		  eular_angle->yaw = eular_angle->yaw + 2*PI*sgn_yawmag;
	  }
	}

	eular_angle->yaw = eular_angle->yaw * 0.75 + yaw_mag * 0.25;

	if(eular_angle->yaw>=PI){
		eular_angle->yaw = eular_angle->yaw - 2*PI;
	}else if(eular_angle->yaw<=-PI){
		eular_angle->yaw = eular_angle->yaw + 2*PI;
	}

	eular_angle->roll_degree = eular_angle->roll * RAD2DEG;
	eular_angle->pitch_degree = eular_angle->pitch * RAD2DEG;
	eular_angle->yaw_degree = eular_angle->yaw * RAD2DEG;
}

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

void calibrate_4axis(struct Sensor *sensor, struct Sensor_Calibrate *sensor_calibrate, struct KF *pitchKF, struct KF *rollKF, struct EularAngle *eular_angle){
	int calibMode = 1, calibCount = 0;
	char sensor_data_disp[16];
	float mag_1[3] = {0,0,0}, mag_2[3]={0,0,0};
	while(calibMode!=0){
		getOrientatoin(sensor, sensor_calibrate, pitchKF, rollKF, eular_angle);
//		ftos(eular_angle->roll_degree, sensor_data_disp, 1);
//		UartA2_sendstr(sensor_data_disp);
//		UartA2_sendstr("   ");
//		ftos(eular_angle->pitch_degree, sensor_data_disp, 1);
//		UartA2_sendstr(sensor_data_disp);
//		UartA2_sendstr("   ");
//		ftos(eular_angle->yaw_degree, sensor_data_disp, 1);
//		UartA2_sendstr(sensor_data_disp);
//		UartA2_sendstr("   \n");
		displayCalibration(calibMode, (int)eular_angle->roll_degree, (int)eular_angle->pitch_degree);
		switch (calibMode){		// calibrate with certain sequence, left, right, top, bottom
		  case 1:
			if (fabs(eular_angle->pitch*RAD2DEG-90.0)<=5.0 && fabs(eular_angle->roll*RAD2DEG)<=5.0){
			  if (calibCount < CALIBSAMPLENUMBER){
				mag_1[2] += sensor_calibrate->mzr;              // float
				mag_1[1] += sensor_calibrate->mxr;  //x
				calibCount++;
//				ftos((float)calibCount, sensor_data_disp, 1);
//				UartA2_sendstr(sensor_data_disp);
//				UartA2_sendstr("   \n");

			  } else {
				mag_1[2] = mag_1[2]/calibCount;
				mag_1[1] = mag_1[1]/calibCount;
				calibCount = 0;
				calibMode = 2;
//				ftos(666.6, sensor_data_disp, 1);
//				UartA2_sendstr(sensor_data_disp);
//				UartA2_sendstr("   \n");
			  }
			}
			break;
		  case 2:
			if (fabs(eular_angle->pitch*RAD2DEG+90.0)<=5.0 && fabs(eular_angle->roll*RAD2DEG)<=5.0){
			  if (calibCount < CALIBSAMPLENUMBER){
				mag_2[2] += sensor_calibrate->mzr;
				mag_2[1] += sensor_calibrate->mxr;    // x
				calibCount++;
//				ftos((float)calibCount, sensor_data_disp, 1);
//				UartA2_sendstr(sensor_data_disp);
//				UartA2_sendstr("   \n");
			  } else {
				mag_2[2] = mag_2[2]/calibCount;
				mag_2[1] = mag_2[1]/calibCount;
				calibCount = 0;
				calibMode = 3;
				ftos(777.7, sensor_data_disp, 1);
				UartA2_sendstr(sensor_data_disp);
				UartA2_sendstr("   \n");
			  }
			}
			break;

		  case 3:
			if (fabs(eular_angle->roll*RAD2DEG+90.0)<=5.0 && fabs(eular_angle->pitch*RAD2DEG)<=5.0){
			  if (calibCount < CALIBSAMPLENUMBER){
				mag_1[0] += sensor_calibrate->myr;    // y
				calibCount ++;
				ftos((float)calibCount, sensor_data_disp, 1);
				UartA2_sendstr(sensor_data_disp);
				UartA2_sendstr("   \n");
			  } else {
				mag_1[0] = mag_1[0]/calibCount;
				calibCount = 0;
				calibMode = 4;
				ftos(888.8, sensor_data_disp, 1);
				UartA2_sendstr(sensor_data_disp);
				UartA2_sendstr("   \n");
			  }
			}
			break;

		  case 4:
			if (fabs(eular_angle->roll*RAD2DEG-90.0)<=5.0 && fabs(eular_angle->pitch*RAD2DEG)<=5.0){
			  if (calibCount < CALIBSAMPLENUMBER){
				mag_2[0] += sensor_calibrate->myr;    // y
				calibCount ++;
				ftos((float)calibCount, sensor_data_disp, 1);
				UartA2_sendstr(sensor_data_disp);
				UartA2_sendstr("   \n");
			  } else {
				mag_2[0] = mag_2[0]/calibCount;
				calibCount = 0;
				calibMode = 5;
				ftos(999.9, sensor_data_disp, 1);
				UartA2_sendstr(sensor_data_disp);
				UartA2_sendstr("   \n");
			  }
			}
			break;

		  case 5:
			calibMode = 0;
			sensor_calibrate->myb = (mag_1[0]+mag_2[0])/2;
			sensor_calibrate->mxb = (mag_1[1]+mag_2[1])/2;
			sensor_calibrate->mzb = (mag_1[2]+mag_2[2])/2;
			ftos(456.7, sensor_data_disp, 1);
			UartA2_sendstr(sensor_data_disp);
			UartA2_sendstr("   \n");
			break;
		  default:
			break;
		}
	}
}

