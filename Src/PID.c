/***********************************************************************************
 * PID.c
 *
 * Implementation of PID control-loops
 *
 * Date			Author          Notes
 * 30/01/2017	Stefan Meyre    Initial release
 *
 ***********************************************************************************/
#include "PID.h"

void setPIDFactors()
{

	krpm = 100;
	k1 = 0.7;
	k2 = 2.0;
	ktot = 1.4;
	ko = 3.0;

	kpp = kpr = 1.0;
	kdp = kdr = 1.5;
	kip = kir = 0.3;

	kpy = 5.0;
	kdy = 2.0;
	kiy = 0.3;

	sampleFrequency = 500.0;
}

void ComputePIDRoll()
{
	errorR = (-setRoll - roll);
	dErrR = errorR - lastErrR;
	errSumR = errSumR + (errorR / sampleFrequency); // 1/333Hz = 0.003s
	if (errSumR > 20)
	{
		errSumR = 20;
	}
	else if (errSumR < -20)
	{
		errSumR = -20;
	}

	outputR[0] = outputR[1];
	outputR[1] = outputR[2];
	outputR[2] = (1 * kpr * errorR) + (100 * kir * errSumR) + (100 * kdr * dErrR);
	outputR[10] = (0.6 * outputR[0] + 0.9 * outputR[1] + 1.5 * outputR[2]) / 3;
	lastErrR = errorR;
}

void ComputePIDPitch()
{
	errorP = (-setPitch - pitch);
	dErrP = errorP - lastErrP;
	errSumP = errSumP + (errorP / sampleFrequency);
	if (errSumP > 20)
	{
		errSumP = 20;
	}
	else if (errSumP < -20)
	{
		errSumP = -20;
	}

	outputP[0] = outputP[1];
	outputP[1] = outputP[2];
	outputP[2] = (1 * kpp * errorP) + (100 * kip * errSumP) + (100 * kdp * dErrP);
	outputP[10] = (0.6 * outputP[0] + 0.9 * outputP[1] + 1.5 * outputP[2]) / 3;
	lastErrP = errorP;
}

void ComputePIDYaw()
{
	errorY = (setYaw - yaw);
	dErrY = errorY - lastErrY;
	errSumY = errSumY + (errorY / sampleFrequency);
	if (errSumY > 20)
	{
		errSumY = 20;
	}
	else if (errSumY < -20)
	{
		errSumY = -20;
	}
	outputY[0] = outputY[1];
	outputY[1] = outputY[2];
	outputY[2] = (1 * kpy * errorY) + (100 * kiy * errSumY) + (100 * kdy * dErrY);
	outputY[10] = -(0.6 * outputY[0] + 0.9 * outputY[1] + 1.5 * outputY[2]) / 3;
	lastErrY = errorY;
}
