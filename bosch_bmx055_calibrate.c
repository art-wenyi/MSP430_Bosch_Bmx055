/*
 * bosch_bmx055_calibrate.c
 *
 *  Created on: Jul 1, 2016
 *      Author: art
 */

#include "bosch_bmx055_calibrate.h"
#include "math.h"


void DLP(float *val_old, float val_new, float alpha){
  if (alpha < 0){alpha = 0;}
  if (alpha > 1){alpha = 1;}
  *val_old = *val_old*(1-alpha)+val_new*alpha;
}

void PreProcess(struct Sensor_Calibrate *sc){
  float norm;
  DLP(&sc->mx,-(sc->myr - MYB)/MYS, 0.5);   // myb, mys is user defined const, mx is dym data,
  DLP(&sc->my,(sc->mxr - MXB)/MXS , 0.5);
  DLP(&sc->mz,(sc->mzr - MZB)/MZS , 0.5);

  sc->gx = (sc->gxr-GXB)*DEG2RAD;   // gyb etc is used defined
  sc->gy = (sc->gyr-GYB)*DEG2RAD;
  sc->gz = (sc->gzr-GZB)*DEG2RAD;

  sc->ax = sc->axr; sc->ay = sc->ayr; sc->az = sc->azr;

  norm = sqrt(sc->ax*sc->ax+sc->ay*sc->ay+sc->az*sc->az);
  sc->ax /= norm;
  sc->ay /= norm;
  sc->az /= norm;

  norm = sqrt(sc->mx*sc->mx+sc->my*sc->my+sc->mz*sc->mz);
  sc->mx /= norm;
  sc->my /= norm;
  sc->mz /= norm;
}

void UpdateGDFilter_MARG(struct Sensor_Calibrate *sc){
  float wx,wy,wz;
  float f1,f2,f3,f4,f5,f6;
  float q1,q2,q3,q4;
  float q2q4,q1q3,q1q2,q3q4,q2q3,q1q4,q2q2,q3q3,q4q4;
  float j11,j12,j13,j14,j21,j22,j23,j24,j31,j32,j33,j34,j41,j42,j43,j44,
        j51,j52,j53,j54,j61,j62,j63,j64;
  float dQerr1,dQerr2,dQerr3,dQerr4;
  float dQw1,dQw2,dQw3,dQw4;
  float hx,hy,hz;
  float norm;

  sc->dt = (float)sc->t_elapse * 0.000001;

  q1 = sc->SEq1; q2 = sc->SEq2; q3 = sc->SEq3; q4 = sc->SEq4;  // global with 'SE'
  q2q4 = q2*q4;		//q2q4 = q2*q4/10000;


  q1q3 = q1*q3; q1q2 = q1*q2; q3q4 = q3*q4; q2q3 = q2*q3; q1q4 = q1*q4;
  q2q2 = q2*q2; q3q3 = q3*q3; q4q4 = q4*q4;

  f1 = 2*(q2q4-q1q3)-sc->ax;
  f2 = 2*(q1q2+q3q4)-sc->ay;
  f3 = 2*(0.5-q2q2-q3q3)-sc->az;
  f4 = 2*sc->bx*(0.5-q3q3-q4q4)+2*sc->bz*(q2q4-q1q3)-sc->mx;
  f5 = 2*sc->bx*(q2q3-q1q4)+2*sc->bz*(q1q2+q3q4)-sc->my;
  f6 = 2*sc->bx*(q1q3+q2q4)+2*sc->bz*(0.5-q2q2-q3q3)-sc->mz;

  j11 = -2.0*q3; j12 = 2.0*q4; j13 = -2.0*q1; j14 = 2.0*q2;
  j21 = 2*q2; j22 = 2*q1; j23 = 2*q4; j24 = 2*q3;
  j31 = 0; j32 = -4*q2; j33 = -4*q3; j34 = 0;
  j41 = -2*sc->bz*q3; j42 = 2*sc->bz*q4; j43 = -4*sc->bx*q3-2*sc->bz*q1; j44 = -4*sc->bx*q4+2*sc->bz*q2;
  j51 = -2*sc->bx*q4+2*sc->bz*q2; j52 = 2*sc->bx*q3+2*sc->bz*q1; j53 = 2*sc->bx*q2+2*sc->bz*q4; j54 = -2*sc->bx*q1+2*sc->bz*q3;
  j61 = 2*sc->bx*q3; j62 = 2*sc->bx*q4-4*sc->bz*q2; j63 = 2*sc->bx*q1-4*sc->bz*q3; j64 = 2*sc->bx*q2;

  dQerr1 = j11*f1 + j21*f2 + j31*f3 + j41*f4 + j51*f5 + j61*f6;
  dQerr2 = j12*f1 + j22*f2 + j32*f3 + j42*f4 + j52*f5 + j62*f6;
  dQerr3 = j13*f1 + j23*f2 + j33*f3 + j43*f4 + j53*f5 + j63*f6;
  dQerr4 = j14*f1 + j24*f2 + j34*f3 + j44*f4 + j54*f5 + j64*f6;

  norm = sqrt(dQerr1*dQerr1+dQerr2*dQerr2+dQerr3*dQerr3+dQerr4*dQerr4);

  dQerr1 /= norm;
  dQerr2 /= norm;
  dQerr3 /= norm;
  dQerr4 /= norm;

  //werr0 = 2*(q1*dQerr1+q2*dQerr2+q3*dQerr3+q4*dQerr4);
  float werrx = 2*(q1*dQerr2-q2*dQerr1-q3*dQerr4+q4*dQerr3);
  float werry = 2*(q1*dQerr3+q2*dQerr4-q3*dQerr1-q4*dQerr2);
  float werrz = 2*(q1*dQerr4-q2*dQerr3+q3*dQerr2-q4*dQerr1);

  sc->wbx += sc->zeta*werrx*sc->dt;
  sc->wby += sc->zeta*werry*sc->dt;
  sc->wbz += sc->zeta*werrz*sc->dt;

  wx = sc->gx-sc->wbx;
  wy = sc->gy-sc->wby;
  wz = sc->gz-sc->wbz;

  dQw1 = 0.5*(-q2*wx-q3*wy-q4*wz);
  dQw2 = 0.5*(q1*wx+q3*wz-q4*wy);
  dQw3 = 0.5*(q1*wy-q2*wz+q4*wx);
  dQw4 = 0.5*(q1*wz+q2*wy-q3*wx);

  q1 += (dQw1-dQerr1*sc->beta)*sc->dt;
  q2 += (dQw2-dQerr2*sc->beta)*sc->dt;
  q3 += (dQw3-dQerr3*sc->beta)*sc->dt;
  q4 += (dQw4-dQerr4*sc->beta)*sc->dt;

  norm = sqrt(q1*q1+q2*q2+q3*q3+q4*q4);
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
  q4 /= norm;

  hx = 2*sc->mx*(0.5-q3*q3-q4*q4)+2*sc->my*(q2*q3-q1*q4)+2*sc->mz*(q2*q4+q1*q3);
  hy = 2*sc->mx*(q2*q3+q1*q4)+2*sc->my*(0.5-q2*q2-q4*q4)+2*sc->mz*(q3*q4-q1*q2);
  hz = 2*sc->mx*(q2*q4-q1*q3)+2*sc->my*(q3*q4+q1*q2)+2*sc->mz*(0.5-q2*q2-q3*q3);

  sc->bx = sqrt(hx*hx+hy*hy);
  sc->bz = hz;

  sc->SEq1 = q1;
  sc->SEq2 = q2;
  sc->SEq3 = q3;
  sc->SEq4 = q4;
}

float getPitch(float q0, float q1, float q2, float q3) {
  float arg1 = 2*(q1*q3 + q0*q2);
  float arg2 = sqrt(1-pow((2*q1*q3+2*q0*q2),2));
  return -atan(arg1/arg2)*RAD2DEG;
}

float getYaw(float q0, float q1, float q2, float q3) {
	float arg1 = 2*(q1*q2-q0*q3);
	float arg2 = 2*q0*q0 - 1 + 2*q1*q1;
  return atan2(arg1,arg2)*RAD2DEG;
}

float getRoll(float q0, float q1, float q2, float q3) {
	float arg1 = 2*(q2*q3-q0*q1);
	float arg2 = 2*q0*q0 - 1 + 2*q3*q3;
  return atan2(arg1,arg2)*RAD2DEG;
}
