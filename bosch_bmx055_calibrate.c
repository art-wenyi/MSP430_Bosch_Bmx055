/*
 * bosch_bmx055_calibrate.c
 *
 *  Created on: Jul 1, 2016
 *      Author: art
 */
//#include "msp430.h"
#include "bosch_bmx055_calibrate.h"
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
