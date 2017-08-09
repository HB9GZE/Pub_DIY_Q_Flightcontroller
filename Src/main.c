/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/***********************************************************************************
 * author: Stefan Meyre / 4.April 2017 / 16.June 2017 / 31.July 2017
 *
 * This is the flight controller software of a quadrocopter.
 * It reads the sensor data (accelerator, gyro and magnetometer) and uses Madgwicks
 * quaternion algorithm to calculate the corresponding roll and pitch angles.
 * The PID loop sets the individual speed of the four BLDC motors by sending the
 * appropriate information by CAN bus to the motor controllers.
 * A gps receiver as well as a pressure transmitter are evaluated as well to allow
 * altitude setting on the fly.
 ***********************************************************************************/
#include <math.h>
#include "PID.h"
#include "MPU9250.h"
#include "BMP280.h"
#include "Parser.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define RX_BUFFER_SIZE 80
#define	UARTRECEIVEDBUFFERSIZE 8
#define	STARTUPCOUNTERSIZE 800
#define __LOADCANWITHDEFAULT \
		myTxMessage.Data[0] = '#';\
		myTxMessage.Data[1] = 'p';\
		myTxMessage.Data[2] = 'w';\
		myTxMessage.Data[3] = 'm';\
		myTxMessage.Data[4] = '2';\
		myTxMessage.Data[5] = '/';\
		hcan1.pTxMsg = &myTxMessage;\
		myTxMessage.DLC = 8;\
		myTxMessage.IDE = CAN_ID_STD;\
		myTxMessage.ExtId = 0x00;\
		myTxMessage.RTR = CAN_RTR_DATA;\

//----GPS---
uint8_t GPS_BUF[RX_BUFFER_SIZE];
uint8_t receiveBuffer[UARTRECEIVEDBUFFERSIZE];
volatile uint8_t longitude[4], latitude[4], hMSLGPS[4], groundSpeedGPS[4], headingGPS[4];
volatile uint8_t gpsDataReady;

uint8_t receiveBuffer[UARTRECEIVEDBUFFERSIZE];

volatile uint16_t averageADCRawData;
volatile uint32_t averageNewADCRawData;
volatile uint16_t averageVoltageCounter = 0;

volatile int16_t rcx, rcy, rcz, rax, ray, raz, rgx, rgy, rgz;
volatile double raxd, rayd, razd, rgxd, rgyd, rgzd, rcxd, rcyd, rczd;
volatile double raxdCal, raydCal, razdCal, rgxdCal, rgydCal, rgzdCal, rcxd, rcyd, rczd;
volatile double deltaT, alpha;

volatile double scaleconst;
volatile double mx, my, mz; //soft iron calibrated magnetometer values (calibrated with fabrication constants)
volatile double mxArray[2000], myArray[2000], mzArray[2000]; //needed for magnetometer hard iron calibration
volatile double sumRax, sumRay, sumRaz, sumRgx, sumRgy, sumRgz; //needed for accelerometer and gyro calibration
volatile double calibValRax, calibValRay, calibValRaz, calibValRgx, calibValRgy, calibValRgz;
volatile double rcxHardIronCorrection, rcyHardIronCorrection, rczHardIronCorrection;
volatile double rcNorm, rcxNorm, rcyNorm, rczNorm, startUpYawReading, yawArray[100], yawSumm, yawSummX, yawSummY;
volatile int8_t motorGotStarted, yawAverageCounter;

volatile int8_t int_lsb, int_msb;
volatile int16_t intLong;

//----variables to read PWM signal of remote Futaba control receiver----
volatile int8_t exti1_rising, exti2_rising, exti3_rising, exti4_rising;
volatile uint16_t exti1_count, exti2_count, exti3_count, exti4_count, exti5_count, exti6_count;
volatile uint16_t exti1_count_value, exti2_count_value;
volatile uint8_t gSwitch = SET, yawHold = SET;
volatile uint16_t remoteRoll, remotePitch, remoteYaw, remoteThrottle, remoteChannel5, remoteChannel6;
volatile double maxOutputLimit, scaledRemoteYaw;
volatile double trimmValueYaw, trimmValueRoll, trimmValuePitch;

GPIO_InitTypeDef GPIO_InitStruct;

uint8_t txBuffer[90];
uint8_t readBuffer[6];
uint8_t sendBuffer[3];

uint8_t headingTxBuffer[8];
uint8_t readAK8963Buffer[9];
uint8_t sendAK8963Buffer[2];

uint8_t rollTxBuffer[8];
uint8_t pitchTxBuffer[8];
uint8_t readMPU9250Buffer[22];
uint8_t sendMPU9250Buffer[2];

uint8_t readBMP280Buffer[24];
uint8_t sendBMP280Buffer[2];
int8_t readBMP280INTBuffer[24];

uint8_t dummyTxBuffer[8];

volatile int8_t dataToSendByCAN;
volatile uint8_t readyToDoTransmit = RESET;

CanTxMsgTypeDef myTxMessage;
CanRxMsgTypeDef myRxMessage;
CanTxMsgTypeDef myCANTxMessage[4];

//----Calibration----
volatile uint16_t doCalibrationSensoroutput = 1;
volatile uint16_t doCalibrationMagnetometer = 0;

volatile double setRPM, setPitch, setRoll, setYaw;
volatile uint16_t rpm_0231, rpm_0232, rpm_0233, rpm_0234;

//----Motor start and stop----
volatile uint8_t motorStart, motorStop;

//----MPU9250 and Madgwick----
volatile uint8_t readyToCalcEuler;
volatile int16_t accelCount[3]; // Stores the 16-bit signed accelerometer sensor output
volatile int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
volatile int16_t magCount[3]; // Stores the 16-bit signed magnetometer sensor output
volatile uint8_t magCalibration[3];
volatile uint8_t ST1_register, ST2_register, CTRL1_register;
volatile double roll, pitch, yaw, yawSample, aRoll, aPitch, aYaw;
volatile double fabCalConst[3];  //stores the fabrication calibration constants
volatile uint8_t inProgressFlag;
volatile uint8_t canTransmitCounter;
volatile uint8_t dataReadyfromMPU9250;
volatile uint8_t dataReadyfromMPU9250Counter;
volatile double temperature, pressure, altitude;

volatile uint16_t startUpCounter = 0;
volatile uint32_t uartState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/***********************************************************************************
 * The gps data received by the UART are stored away. We use a NEO-M8n from Ublox
 * with NEMA protocol turned off. Instead the ublock protocol UBX is used to get
 * well defined number of bytes that can be easily interpreted (no need of a NEMA parser)
 ***********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		parseFlightMonitorData();
	}

	if (huart->Instance == UART5)
	{
		latitude[0] = GPS_BUF[14];
		latitude[1] = GPS_BUF[15];
		latitude[2] = GPS_BUF[16];
		latitude[3] = GPS_BUF[17];

		longitude[0] = GPS_BUF[10];
		longitude[1] = GPS_BUF[11];
		longitude[2] = GPS_BUF[12];
		longitude[3] = GPS_BUF[13];

		hMSLGPS[0] = GPS_BUF[22];
		hMSLGPS[1] = GPS_BUF[23];
		hMSLGPS[2] = GPS_BUF[24];
		hMSLGPS[3] = GPS_BUF[25];

		groundSpeedGPS[0] = GPS_BUF[60];
		groundSpeedGPS[1] = GPS_BUF[61];
		groundSpeedGPS[2] = GPS_BUF[62];
		groundSpeedGPS[3] = GPS_BUF[63];

		headingGPS[0] = GPS_BUF[64];
		headingGPS[1] = GPS_BUF[65];
		headingGPS[2] = GPS_BUF[66];
		headingGPS[3] = GPS_BUF[67];

		gpsDataReady = SET;
	}
}

/***********************************************************************************
 * After the flight controller monitoring information have been send of to the
 * monitoring system through the UART the buzzer can be turned off
 ***********************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		if (doCalibrationSensoroutput == 0 && doCalibrationMagnetometer == 0)
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	}
}

/***********************************************************************************
 * We have received data from the I2C bus. Check from what sensor the data came
 * and store the received information away.
 ***********************************************************************************/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (dataReadyfromBMP280 == SET)
	{
		adc_T = (readBMP280Buffer[3] << 12) + (readBMP280Buffer[4] << 4) + (readBMP280Buffer[5] >> 4);
		adc_P = (readBMP280Buffer[0] << 12) + (readBMP280Buffer[1] << 4) + (readBMP280Buffer[2] >> 4);
	}
	else if (dataReadyfromMPU9250 == SET)
	{
		dataReadyfromMPU9250 = RESET;
		ray = 1 * ((int16_t) readMPU9250Buffer[0] << 8) + (int16_t) readMPU9250Buffer[1]; //MSB und LSB; we read MSB first
		rax = 1 * ((int16_t) readMPU9250Buffer[2] << 8) + (int16_t) readMPU9250Buffer[3];
		raz = 1 * ((int16_t) readMPU9250Buffer[4] << 8) + (int16_t) readMPU9250Buffer[5];
		rgx = 1 * ((int16_t) readMPU9250Buffer[8] << 8) + (int16_t) readMPU9250Buffer[9];
		rgy = -1 * ((int16_t) readMPU9250Buffer[10] << 8) + (int16_t) readMPU9250Buffer[11];
		rgz = -1 * ((int16_t) readMPU9250Buffer[12] << 8) + (int16_t) readMPU9250Buffer[13];
		rcx = ((int16_t) readMPU9250Buffer[16] << 8) + (int16_t) readMPU9250Buffer[15];
		rcy = ((int16_t) readMPU9250Buffer[18] << 8) + (int16_t) readMPU9250Buffer[17];
		rcz = ((int16_t) readMPU9250Buffer[20] << 8) + (int16_t) readMPU9250Buffer[19];

		rayd = (double) ray;
		raxd = (double) rax;
		razd = (double) raz;
		rgxd = (double) rgx;
		rgyd = (double) rgy;
		rgzd = (double) rgz;
		rcxd = (double) rcx;
		rcyd = (double) rcy;
		rczd = (double) rcz;

		ST1_register = readMPU9250Buffer[14]; //AK8963 status registers
		ST2_register = readMPU9250Buffer[21];

		//scale to proper dps value
		scaleconst = 0.005;
		rgxd = rgxd * scaleconst;
		rgyd = rgyd * scaleconst;
		rgzd = rgzd * scaleconst;

		mx = rcxd * fabCalConst[0]; //calibrate with the read factory calibration constants
		my = rcyd * fabCalConst[1];
		mz = rczd * fabCalConst[2];

		rcxd = (mx - rcxHardIronCorrection);
		rcyd = (my - rcyHardIronCorrection);
		rczd = (mz - rczHardIronCorrection); //is this needed or even correct? Try out with 0 next time to see if tilt comp works... check Kris.

		rcNorm = sqrt((rcxd * rcxd) + (rcyd * rcyd) + (rczd * rczd));
		rcxNorm = rcxd / rcNorm;
		rcyNorm = rcyd / rcNorm;
		rczNorm = rczd / rcNorm;

		readyToCalcEuler = SET;

		if (dataReadyfromMPU9250Counter == 5) //since the data sample rate of the BMP280 is only 100Hz
		{
			dataReadyfromMPU9250Counter = 0;
			readPandT(BMP280_REGISTER_STARTDATA, readBMP280Buffer);
		}
	}
}

/******************************************************************************
 *	This sends the current monitoring information to the external monitoring
 *	system. The buzzer indicates the transmission.
 ******************************************************************************/
void doTransmit(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	intLong = (uint16_t)(altitude * 100.0);
	int_lsb = (uint8_t) intLong;
	int_msb = (uint8_t)(intLong >> 8);
	txBuffer[0] = '#';
	txBuffer[1] = 'a';
	txBuffer[2] = 'l';
	txBuffer[3] = 'p';
	txBuffer[4] = '2';
	txBuffer[5] = '/';
	txBuffer[6] = int_lsb; //LSB first
	txBuffer[7] = int_msb; //MSB

	intLong = averageADCRawData;
	int_lsb = (uint8_t) intLong;
	int_msb = (uint8_t)(intLong >> 8);
	txBuffer[8] = '#';
	txBuffer[9] = 'v';
	txBuffer[10] = 'l';
	txBuffer[11] = 't';
	txBuffer[12] = '2';
	txBuffer[13] = '/';
	txBuffer[14] = int_lsb; //LSB first
	txBuffer[15] = int_msb; //MSB

	intLong = (uint16_t) yaw;
	int_lsb = (uint8_t) intLong;
	int_msb = (uint8_t)(intLong >> 8);
	txBuffer[16] = '#';
	txBuffer[17] = 'y';
	txBuffer[18] = 'a';
	txBuffer[19] = 'n';
	txBuffer[20] = '2';
	txBuffer[21] = '/';
	txBuffer[22] = int_lsb; //LSB first
	txBuffer[23] = int_msb; //MSB

	intLong = -1 * (int16_t) roll;
	int_lsb = (uint8_t) intLong;
	int_msb = (uint8_t)(intLong >> 8);
	txBuffer[24] = '#';
	txBuffer[25] = 'r';
	txBuffer[26] = 'a';
	txBuffer[27] = 'n';
	txBuffer[28] = '2';
	txBuffer[29] = '/';
	txBuffer[30] = int_lsb; //LSB first
	txBuffer[31] = int_msb; //MSB

	intLong = (int16_t) pitch;
	int_lsb = (uint8_t) intLong;
	int_msb = (uint8_t)(intLong >> 8);
	txBuffer[32] = '#';
	txBuffer[33] = 'p';
	txBuffer[34] = 'a';
	txBuffer[35] = 'n';
	txBuffer[36] = '2';
	txBuffer[37] = '/';
	txBuffer[38] = int_lsb; //LSB first
	txBuffer[39] = int_msb; //MSB

	txBuffer[40] = '#';
	txBuffer[41] = 'l';
	txBuffer[42] = 'o';
	txBuffer[43] = 'n';
	txBuffer[44] = '4';
	txBuffer[45] = '/';
	txBuffer[46] = longitude[3];
	txBuffer[47] = longitude[2];
	txBuffer[48] = longitude[1];
	txBuffer[49] = longitude[0];

	txBuffer[50] = '#';
	txBuffer[51] = 'l';
	txBuffer[52] = 'a';
	txBuffer[53] = 't';
	txBuffer[54] = '4';
	txBuffer[55] = '/';
	txBuffer[56] = latitude[3];
	txBuffer[57] = latitude[2];
	txBuffer[58] = latitude[1];
	txBuffer[59] = latitude[0];

	txBuffer[60] = '#';
	txBuffer[61] = 'a';
	txBuffer[62] = 'l';
	txBuffer[63] = 'g';
	txBuffer[64] = '4';
	txBuffer[65] = '/';
	txBuffer[66] = hMSLGPS[3];
	txBuffer[67] = hMSLGPS[2];
	txBuffer[68] = hMSLGPS[1];
	txBuffer[69] = hMSLGPS[0];

	txBuffer[70] = '#';
	txBuffer[71] = 'c';
	txBuffer[72] = 'v';
	txBuffer[73] = 'g';
	txBuffer[74] = '4';
	txBuffer[75] = '/';
	txBuffer[76] = groundSpeedGPS[3];
	txBuffer[77] = groundSpeedGPS[2];
	txBuffer[78] = groundSpeedGPS[1];
	txBuffer[79] = groundSpeedGPS[0];

	txBuffer[80] = '#';
	txBuffer[81] = 'h';
	txBuffer[82] = 'a';
	txBuffer[83] = 'g';
	txBuffer[84] = '4';
	txBuffer[85] = '/';
	txBuffer[86] = headingGPS[3];
	txBuffer[87] = headingGPS[2];
	txBuffer[88] = headingGPS[1];
	txBuffer[89] = headingGPS[0];

	while (HAL_UART_Transmit_DMA(&huart4, txBuffer, 90) != HAL_OK);
}

/******************************************************************************
 * Clears all the data variables for the PID loop calculation
 ******************************************************************************/
void clearPIDdata()
{
	errorR = lastErrR = errSumR = outputR[0] = outputR[1] = outputR[2] = outputR[3] = outputR[4] = outputR[5] = outputR[6] = outputR[7] = outputR[8] =
			outputR[9] = outputR[10] = 0;
	errorP = lastErrP = errSumP = outputP[0] = outputP[1] = outputP[2] = outputP[3] = outputP[4] = outputP[5] = outputP[6] = outputP[7] = outputP[8] =
			outputP[9] = outputP[10] = 0;
	errorY = lastErrY = errSumY = outputY[0] = outputY[1] = outputY[2] = outputY[3] = outputY[4] = outputY[5] = outputY[6] = outputY[7] = outputY[8] =
			outputY[9] = outputY[10] = 0;
}

/******************************************************************************
 * Loads the predefined horizontal calibration values of the MPU9250
 ******************************************************************************/
void setCalibrationValues()
{
	calibValRax = 23.8;
	calibValRay = -96.1;
	calibValRaz = -237.4;
	calibValRgx = -0.83;
	calibValRgy = 0.16;
	calibValRgz = 2.13;

	rcxHardIronCorrection = 38.5;
	rcyHardIronCorrection = 40.3;
	rczHardIronCorrection = -65.8;
}

/******************************************************************************
 * The accelerator and the gyro evaluate its steady state readings for compensation
 ******************************************************************************/
void calibrateMPU9250AccelerometerAndGyro()
{
	if (doCalibrationSensoroutput == 1)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		sumRax = sumRay = sumRaz = sumRgx = sumRgy = sumRgz = 0;
		errSumP = errSumPWatch = 0;
		errSumR = errSumRWatch = 0;
	}

	sumRax += raxd;
	sumRay += rayd;
	sumRaz += razd;
	sumRgx += rgxd;
	sumRgy += rgyd;
	sumRgz += rgzd;
	doCalibrationSensoroutput++;

	if (doCalibrationSensoroutput == 2001)
	{
		calibValRax = (sumRax / 2000);
		calibValRay = (sumRay / 2000);
		calibValRaz = (sumRaz / 2000 - 4096); //take g into account; without acceleration: raz should be zero
		calibValRgx = (sumRgx / 2000);
		calibValRgy = (sumRgy / 2000);
		calibValRgz = (sumRgz / 2000);
		doCalibrationSensoroutput = 0;
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	}
}

/******************************************************************************
 * Move the magnetometer in an 8 form to hard iron calibrate the magnetometer
 ******************************************************************************/
void calibrateMPU9250Magnetometer()
{
	if (doCalibrationMagnetometer == 1)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	}

	mxArray[doCalibrationMagnetometer - 1] = mx;
	myArray[doCalibrationMagnetometer - 1] = my;
	mzArray[doCalibrationMagnetometer - 1] = mz;
	doCalibrationMagnetometer++;

	if (doCalibrationMagnetometer == 2001)
	{
		double maxX, minX, maxY, minY, maxZ, minZ;

		minX = maxX = mxArray[0];
		minY = maxY = myArray[0];
		minZ = maxZ = mzArray[0];

		for (int i = 1; i < 2000; i++)
		{
			if (minX > mxArray[i])
				minX = mxArray[i];
			if (maxX < mxArray[i])
				maxX = mxArray[i];

			if (minY > myArray[i])
				minY = myArray[i];
			if (maxY < myArray[i])
				maxY = myArray[i];

			if (minZ > mzArray[i])
				minZ = mzArray[i];
			if (maxZ < mzArray[i])
				maxZ = mzArray[i];
		}

		rcxHardIronCorrection = minX + ((maxX - minX) / 2);
		rcyHardIronCorrection = minY + ((maxY - minY) / 2);
		rczHardIronCorrection = minZ + ((maxZ - minZ) / 2);

		doCalibrationMagnetometer = 0;
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	}
}

/***********************************************************************************
 * Tell the BLDC motor controllers to turn off the motors
 ***********************************************************************************/
void CAN_Motor_Stop()
{
	motorStop = RESET;
	dummyTxBuffer[0] = '#';
	dummyTxBuffer[1] = 's';
	dummyTxBuffer[2] = 't';
	dummyTxBuffer[3] = 'o';
	dummyTxBuffer[4] = '2';
	dummyTxBuffer[5] = '/';
	dummyTxBuffer[6] = 0x00;
	dummyTxBuffer[7] = 0x00;

	hcan1.pTxMsg = &myTxMessage;
	myTxMessage.DLC = 8;
	myTxMessage.StdId = 0x231;
	myTxMessage.IDE = CAN_ID_STD;
	myTxMessage.ExtId = 0x00;
	myTxMessage.RTR = CAN_RTR_DATA;
	myTxMessage.Data[0] = dummyTxBuffer[0];
	myTxMessage.Data[1] = dummyTxBuffer[1];
	myTxMessage.Data[2] = dummyTxBuffer[2];
	myTxMessage.Data[3] = dummyTxBuffer[3];
	myTxMessage.Data[4] = dummyTxBuffer[4];
	myTxMessage.Data[5] = dummyTxBuffer[5];
	myTxMessage.Data[6] = dummyTxBuffer[6];
	myTxMessage.Data[7] = dummyTxBuffer[7];

	while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);
	myTxMessage.StdId = 0x232;
	while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);
	myTxMessage.StdId = 0x233;
	while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);
	myTxMessage.StdId = 0x234;
	while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);
	__LOADCANWITHDEFAULT
	;
}

/***********************************************************************************
 * Tell the BLDC motor controllers to start the motors at minimum speed
 ***********************************************************************************/
void CAN_Motor_Start()
{
	motorStart = RESET;
	dummyTxBuffer[0] = '#';
	dummyTxBuffer[1] = 's';
	dummyTxBuffer[2] = 't';
	dummyTxBuffer[3] = 'a';
	dummyTxBuffer[4] = '2';
	dummyTxBuffer[5] = '/';
	dummyTxBuffer[6] = 0x30;
	dummyTxBuffer[7] = 0x30;

	hcan1.pTxMsg = &myTxMessage;
	myTxMessage.DLC = 8;
	myTxMessage.StdId = 0x231;
	myTxMessage.IDE = CAN_ID_STD;
	myTxMessage.ExtId = 0x00;
	myTxMessage.RTR = CAN_RTR_DATA;
	myTxMessage.Data[0] = dummyTxBuffer[0];
	myTxMessage.Data[1] = dummyTxBuffer[1];
	myTxMessage.Data[2] = dummyTxBuffer[2];
	myTxMessage.Data[3] = dummyTxBuffer[3];
	myTxMessage.Data[4] = dummyTxBuffer[4];
	myTxMessage.Data[5] = dummyTxBuffer[5];
	myTxMessage.Data[6] = dummyTxBuffer[6];
	myTxMessage.Data[7] = dummyTxBuffer[7];

	while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);
	myTxMessage.StdId = 0x232;
	while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);
	myTxMessage.StdId = 0x233;
	while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);
	myTxMessage.StdId = 0x234;
	while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);
	HAL_CAN_Transmit_IT(&hcan1);
	__LOADCANWITHDEFAULT
	;
}

/***********************************************************************************
 * Read the Futaba remote control inputs
 ***********************************************************************************/
void Read_Futaba()
{
	//check if new remote controller values are available
	if (exti1_count > 0 && exti2_count > 0 && exti3_count > 0 && exti4_count > 0 && exti5_count > 0 && exti6_count > 0)
	{
		if (exti1_count > 500 && exti1_count < 1200)
		{
			exti1_count_value = exti1_count; //only for debugging; can be deleted
			//ko = ((double) (exti1_count_value - 595) * 10) / 590; //middle position of VR potentiometer of the Futaba remote control results in a ko of 5 (min 0, max 2)
		}
		if (exti2_count > 500 && exti2_count < 1200) //exti2_count values: g_switch_down == 594; g_switch_up == 930
		{
			exti2_count_value = exti2_count; //only for debugging; can be deleted
			if (exti2_count > 900 && gSwitch == RESET && doCalibrationSensoroutput == 0)
			{
				gSwitch = SET; //represents the G switch on the Futaba remote control
				clearPIDdata();
				doCalibrationSensoroutput = 1;
			}
			if (exti2_count < 600)
			{
				gSwitch = RESET;
				clearPIDdata();
				setCalibrationValues();
			}
		}
		if (exti3_count > 500 && exti3_count < 1200)
			remoteYaw = exti3_count;
		if (exti4_count > 500 && exti4_count < 1200)
			remoteThrottle = exti4_count;
		if (exti5_count > 500 && exti5_count < 1200)
			remotePitch = exti5_count;
		if (exti6_count > 500 && exti6_count < 1200)
			remoteRoll = exti6_count;

		if (remoteThrottle < 690 && remoteYaw < 790)
		{
			clearPIDdata();
			motorStart = SET;
		}
		if (remoteThrottle < 690 && remoteYaw > 1050)
		{
			clearPIDdata();
			motorStop = SET;
		}

		//setRPM = ((double) remoteThrottle - 680.0) / 2;
		//make throttle stick less sensitive in the +/- lift off region for better maneuver control
		if (remoteThrottle <= 750)
		{
			setRPM = ((double) (remoteThrottle - 600)) * 96 / 150;
		}
		else if (remoteThrottle > 750 && remoteThrottle <= 1050)
		{
			setRPM = ((double) (remoteThrottle - 750)) * 64 / 300 + 96;
		}
		else
		{
			setRPM = ((double) (remoteThrottle - 1050)) * 96 / 150 + 160;
		}

		if (setRPM < 60.0)
		{
			setRPM = 60.0;
		}

		setPitch = 2.0 * (0.18 * ((double) remotePitch - 680.0) - 46.0) + trimmValuePitch; //multiplier of 2.0 to scale joystick movement
		if (setPitch < 1.0 && setPitch > -1.0)
			setPitch = 0;

		setRoll = -2.8 * (0.18 * ((double) remoteRoll - 680.0) - 46.0) + trimmValueRoll;
		if (setRoll < 1.0 && setRoll > -1.0)
			setRoll = 0;

		scaledRemoteYaw = 2.0 * (0.18 * ((double) remoteYaw - 680.0) - 46.0) + trimmValueYaw;
		if (scaledRemoteYaw < 10.0 && scaledRemoteYaw > -10.0)
		{
			if (yawHold == RESET)
			{
				setYaw = aYaw;
				yawHold = SET;
			}

		}
		else
		{
			setYaw = aYaw + scaledRemoteYaw;
			yawHold = RESET;
		}

		exti1_count = exti2_count = exti3_count = exti4_count = exti5_count = exti6_count = 0;
	}

}

/* USER CODE END 0 */

int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */
	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_CAN1_Init();
	MX_I2C1_Init();
	//MX_IWDG_Init();
	MX_TIM3_Init();
	//MX_UART4_Init();
	//MX_UART5_Init();
	//MX_TIM6_Init();
	//MX_TIM7_Init();
	//MX_USART6_UART_Init();

	/* Initialize interrupts */
	// MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	BMP280_Init();
	MPU9250_Init();
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

	setRPM = 60;
	setRoll = 0;
	setPitch = 0;
	setYaw = 0;
	dataToSendByCAN = RESET;
	canTransmitCounter = 0;
	motorStart = 0;
	motorStop = 0;
	maxOutputLimit = 150;
	trimmValueRoll = -6.6;
	trimmValuePitch = 4.6;
	trimmValueYaw = 0;

	MX_TIM6_Init();
	HAL_TIM_Base_Start_IT(&htim6); //start timer 6 to send periodically data to the flight controller
	MX_TIM7_Init();
	HAL_TIM_Base_Start(&htim7); //start timer 7 to count the pulse width of the remote receiver channels (exti 1-6)
	MX_NVIC_Init();
	doCalibrationSensoroutput = 0;  //start up with no calibration, use defaults
	doCalibrationMagnetometer = 0; //start up with no calibration, use defaults

	MX_UART4_Init();
	while (HAL_UART_Receive_IT(&huart4, receiveBuffer, UARTRECEIVEDBUFFERSIZE) != HAL_OK); //get ready to receive instructions from the flight monitor system (through RS232 telemetry system)

	MX_IWDG_Init();
	__HAL_IWDG_START(&hiwdg);
	__LOADCANWITHDEFAULT
	;
	clearPIDdata();
	setPIDFactors();
	setCalibrationValues();
	alpha = 0.98;  //set the default time constant of the complementary filter
	deltaT = 0.1; //set the integration time of the complementary filter
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//delay start of UART5 to read gps by about 3-4s after start up to ignore unstable first few transmits; silly work around to save development time...
		if (readyToCalcEuler && startUpCounter < STARTUPCOUNTERSIZE)
		{
			if (startUpCounter == 0)
				motorStop = SET;
			if (startUpCounter == 1)
				motorStop = RESET;
			startUpCounter++;
		}
		else if (readyToCalcEuler && startUpCounter == STARTUPCOUNTERSIZE)
		{
			MX_UART5_Init();
			while (HAL_UART_Receive_DMA(&huart5, GPS_BUF, RX_BUFFER_SIZE) != HAL_OK);
			startUpCounter = STARTUPCOUNTERSIZE + 1;
			setYaw = startUpYawReading = yaw;
		}

		Read_Futaba();

		//as soon as the sensor data is available start the flight control calculations and the CAN bus transmitting
		if (readyToCalcEuler == SET)
		{
			//refresh watchdog when new sensor data is available (every 1 / 333Hz) depending on the sample frequency of the MPU9250
			HAL_IWDG_Refresh(&hiwdg);

			raxdCal = raxd - calibValRax;
			raydCal = rayd - calibValRay;
			razdCal = razd - calibValRaz;
			rgxdCal = rgxd - calibValRgx;
			rgydCal = rgyd - calibValRgy;
			rgzdCal = rgzd - calibValRgz;

//			if (rgxdCal > -3 && rgxdCal < 3)
//				rgxdCal = 0;
//			if (rgydCal > -3 && rgydCal < 3)
//				rgydCal = 0;
//			if (rgzdCal > -3 && rgzdCal < 3)
//				rgzdCal = 0;
//			if (raxdCal > -10 && raxdCal < 10)
//				raxdCal = 0;
//			if (raydCal > -10 && raydCal < 10)
//				raydCal = 0;

			aRoll = atan2(raydCal, sqrt(raxdCal * raxdCal + razdCal * razdCal)) * 180 / M_PI;
			aPitch = -atan2(-raxdCal, sqrt(raydCal * raydCal + razdCal * razdCal)) * 180 / M_PI;

			roll = alpha * (rgydCal * deltaT + roll) + (1 - alpha) * aRoll;
			pitch = alpha * (rgxdCal * deltaT + pitch) + (1 - alpha) * aPitch;

			yawSample = atan2(rcyNorm * cos(aRoll / 180 * M_PI) - rczNorm * sin(aRoll / 180 * M_PI),
					-rcxNorm * cos(aPitch / 180 * M_PI)
							+ rcyNorm * sin(aPitch / 180 * M_PI) * sin(aRoll / 180 * M_PI - rczNorm * sin(aPitch / 180 * M_PI) * cos(aRoll / 180 * M_PI)));

			if (yawAverageCounter < 50)
			{
				yawSummX += cos(yawSample);
				yawSummY += sin(yawSample);
				yawAverageCounter++;
			}
			else
			{
				yawAverageCounter = 0;
				aYaw = atan2(yawSummY / 50, yawSummX / 50);

				aYaw = aYaw * 180 / M_PI;
				if (aYaw < 0)
				{
					aYaw += 360; 	// Ensure yaw stays between 0 and 360
				}
				aYaw += 2.26; 		//declination of Zurich is about +2.26 deg
				yawSummX = yawSummY = 0;
			}

			yaw = alpha * (rgzdCal * deltaT + yaw) + (1 - alpha) * aYaw;

			//if the sensor data calibration has to be done
			if ((doCalibrationSensoroutput > 0) && (doCalibrationSensoroutput < 2001))
			{
				calibrateMPU9250AccelerometerAndGyro();
			}
			//if the magnetometer hard iron calibration has to be done. Move the sensor over 4s in the form of a figure 8
			if ((doCalibrationMagnetometer > 0) && (doCalibrationMagnetometer < 2001))
			{
				calibrateMPU9250Magnetometer();
			}

			ComputePIDRoll();
			ComputePIDPitch();
			ComputePIDYaw();

			if (outputP[10] > maxOutputLimit)
				outputP[10] = maxOutputLimit;
			if (outputP[10] < -maxOutputLimit)
				outputP[10] = -maxOutputLimit;

			if (outputR[10] > maxOutputLimit)
				outputR[10] = maxOutputLimit;
			if (outputR[10] < -maxOutputLimit)
				outputR[10] = -maxOutputLimit;

			if (outputY[10] > maxOutputLimit)
				outputY[10] = maxOutputLimit;
			if (outputY[10] < -maxOutputLimit)
				outputY[10] = -maxOutputLimit;

			rpm_0231 = (uint16_t)(ktot * ((k1 * krpm * setRPM + k2 * ko * (-outputR[10] + outputP[10] - outputY[10]))));
			myTxMessage.Data[6] = (uint8_t) rpm_0231; 			//low byte
			myTxMessage.Data[7] = (uint8_t)(rpm_0231 >> 8); 	//high byte
			myTxMessage.StdId = 0x231;
			myCANTxMessage[0] = myTxMessage;					//what is this??
			while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);

			rpm_0234 = (uint16_t)(ktot * ((k1 * krpm * setRPM + k2 * ko * (outputR[10] + outputP[10] + outputY[10]))));
			myTxMessage.Data[6] = (uint8_t) rpm_0234; 			//low byte
			myTxMessage.Data[7] = (uint8_t)(rpm_0234 >> 8);  	//high byte
			while (hcan1.State == HAL_CAN_STATE_BUSY);
			myTxMessage.StdId = 0x234;
			while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);

			rpm_0233 = (uint16_t)(ktot * ((k1 * krpm * setRPM + k2 * ko * (-outputR[10] - outputP[10] + outputY[10]))));
			myTxMessage.Data[6] = (uint8_t) rpm_0233; 			//low byte
			myTxMessage.Data[7] = (uint8_t)(rpm_0233 >> 8); 	//high byte
			while (hcan1.State == HAL_CAN_STATE_BUSY);
			myTxMessage.StdId = 0x233;
			//HAL_Delay(2); not needed anymore
			while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);

			rpm_0232 = (uint16_t)(ktot * ((k1 * krpm * setRPM + k2 * ko * (outputR[10] - outputP[10] - outputY[10]))));
			myTxMessage.Data[6] = (uint8_t) rpm_0232; 			//low byte
			myTxMessage.Data[7] = (uint8_t)(rpm_0232 >> 8); 	//high byte
			while (hcan1.State == HAL_CAN_STATE_BUSY);
			myTxMessage.StdId = 0x232;
			while (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK);

			//read the battery voltage with ADC
			if (averageVoltageCounter < 10)
			{
				HAL_ADC_Start(&hadc1);
				if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
				{
					averageNewADCRawData += HAL_ADC_GetValue(&hadc1);
				}
				averageVoltageCounter++;
			}
			else
			{
				averageADCRawData = (uint16_t)(averageNewADCRawData / 10);
				averageNewADCRawData = 0;
				averageVoltageCounter = 0;
				HAL_ADC_Stop(&hadc1);							//why this?
			}
			//arm the gps UART for the next reading
			if (gpsDataReady == SET)
			{
				gpsDataReady = RESET;
				while (HAL_UART_Receive_DMA(&huart5, GPS_BUF, RX_BUFFER_SIZE) != HAL_OK);
			}
			//now is the right time to send the status information to the flight monitor
			if (readyToDoTransmit == SET)
			{
				doTransmit();
				readyToDoTransmit = RESET;
			}
			readyToCalcEuler = RESET;
		}

		if (motorStart == SET)
		{
			motorGotStarted = SET;
			startUpYawReading = yaw;
			CAN_Motor_Start();
		}
		if (motorStop == SET)
		{
			motorGotStarted = RESET;
			CAN_Motor_Stop();
		}

		//if pressure data is available, do all the calculations
		if (dataReadyfromBMP280 == SET)
		{
			dataReadyfromBMP280 = RESET;
			temperature = calcDoubleBMP280Temperature();
			pressure = calcDoubleBMP280Pressure();
			altitude = calcDoubleBMP280Altitude(1013.25);
		}
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
 */
static void MX_NVIC_Init(void)
{
	/* UART4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	/* UART5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(UART5_IRQn);
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* ADC_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 21;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_2TQ;
	hcan1.Init.BS1 = CAN_BS1_2TQ;
	hcan1.Init.BS2 = CAN_BS2_2TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Reload = 150;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 25;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 16800;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 500;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 136;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 10000;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 57600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

	huart5.Instance = UART5;
	huart5.Init.BaudRate = 57600;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA3 PA4 PA5
	 PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB1 PB11 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
