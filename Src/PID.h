/***********************************************************************************
* PID.h
*
* Implementation of PID control-loops
*
* Date			Author          Notes
* 30/01/2017	Stefan Meyre    Initial release
 *
***********************************************************************************/

#ifndef PID_H
#define PID_H

extern volatile double setRoll, setPitch, setYaw, setBeta, startUpYawReading;
extern volatile double roll, pitch, yaw;

volatile double errSumR, errSumRWatch, lastErrR, kpr, kir, kdr;
volatile double errorR, errorP, errorY;
volatile double dErrR, dErrP, dErrY;
volatile double errSumP, errSumPWatch, lastErrP, kpp, kip, kdp,ko,krpm,k1,k2,ktot;
volatile double errSumY, errSumYWatch, lastErrY, kpy, kiy, kdy;
volatile double outputP[11];
volatile double outputR[11];
volatile double outputY[11];
volatile double sampleFrequency;

void setPIDFactors(void);
void ComputePIDRoll(void);
void ComputePIDPitch(void);
void ComputePIDYaw(void);

#endif
