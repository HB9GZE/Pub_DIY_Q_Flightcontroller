/***********************************************************************************
 * Parser.c
 *
 * Read the BMP280 pressure sensor and calculate the compensated pressure and
 * altitude above sea level
 * See: http://www.bosch.com
 *
 * Date			Author          Notes
 * 30/01/2017	Stefan Meyre    Initial release
 *
 ***********************************************************************************/
#include "stm32f4xx_hal.h"
#include "Parser.h"

#define	UARTRECEIVEDBUFFERSIZE 8

extern UART_HandleTypeDef huart4;
extern volatile double sampleFrequency;
extern volatile double kpr, kir, kdr;
extern volatile double kpp, kip, kdp, ko, k1, k2, ktot;
extern volatile double setBeta;
extern volatile int16_t calibValRax, calibValRay, calibValRaz, calibValRgx, calibValRgy, calibValRgz;
extern volatile uint16_t doCalibrationSensoroutput, doCalibrationMagnetometer;
extern uint8_t dummyTxBuffer[8];
extern uint8_t receiveBuffer[UARTRECEIVEDBUFFERSIZE];
extern volatile uint8_t motorStart, motorStop;
extern volatile double setRPM, setPitch, setRoll, setYaw;
extern volatile double maxOutputLimit;
extern volatile double deltaT, alpha;
extern volatile double trimmValueYaw, trimmValuePitch, trimmValueRoll, yaw, startUpYawReading;
extern int8_t motorGotStarted;

/***********************************************************************************
 * The commands from the flight monitoring system are interpreted by parsing the
 * UART data.
 ***********************************************************************************/
void parseFlightMonitorData()
{
	rx_bytes_counter = 0;
	for (uint8_t i = 0; i < UARTRECEIVEDBUFFERSIZE; i++)
	{
		rx_bytes[i] = receiveBuffer[i];
	}
	if (rx_bytes[0] == '#')
	{
		for (uint8_t i = 0; i < UARTRECEIVEDBUFFERSIZE; i++)
		{
			rx_bytes[rx_bytes_counter] = receiveBuffer[rx_bytes_counter];
			rx_bytes_counter++;

			if (rx_bytes[rx_bytes_counter - 1] == '#')
			{
				rx_bytes_counter = 1;
				command_ready = 0;
				data_ready = 0;
			}

			if (command_ready == 1)
			{
				if ((rx_bytes[4] == '1') && (rx_bytes_counter == 7))
					data_ready = 1;
				if ((rx_bytes[4] == '2') && (rx_bytes_counter == 8))
					data_ready = 1;
			}

			if (rx_bytes[rx_bytes_counter - 1] == '/')
				command_ready = 1;

			if (data_ready == 1)
			{
				if ((rx_bytes[1] == 'p') && (rx_bytes[2] == 'w') && (rx_bytes[3] == 'm'))
				{
					setRPM = rx_bytes[6];
				}

				if ((rx_bytes[1] == 's') && (rx_bytes[2] == 't') && (rx_bytes[3] == 'a'))
				{
					motorStop = RESET;
					motorStart = SET;
				}

				if ((rx_bytes[1] == 's') && (rx_bytes[2] == 't') && (rx_bytes[3] == 'o'))
				{
					motorStart = RESET;
					motorStop = SET;
					motorGotStarted = RESET;
					setRPM = 40;
				}

				if ((rx_bytes[1] == 'c') && (rx_bytes[2] == 'a') && (rx_bytes[3] == 'l'))
				{
					clearPIDdata();
					doCalibrationSensoroutput = 1;
				}

				if ((rx_bytes[1] == 'c') && (rx_bytes[2] == 'a') && (rx_bytes[3] == 'm'))
				{
					doCalibrationMagnetometer = 1;
				}

				if ((rx_bytes[1] == 'c') && (rx_bytes[2] == 'a') && (rx_bytes[3] == 'z'))
				{
					doCalibrationSensoroutput = 0;
					setCalibrationValues();
				}

				if ((rx_bytes[1] == 'c') && (rx_bytes[2] == 'a') && (rx_bytes[3] == 'm'))
				{
					doCalibrationMagnetometer = 1;
				}

				if ((rx_bytes[1] == 'r') && (rx_bytes[2] == 'o') && (rx_bytes[3] == 'l'))
				{
					setRoll = 45 - (double) rx_bytes[6];
				}

				if ((rx_bytes[1] == 'p') && (rx_bytes[2] == 'i') && (rx_bytes[3] == 't'))
				{
					setPitch = (double) (rx_bytes[6] - 45);
				}

				if ((rx_bytes[1] == 'k') && (rx_bytes[2] == 'p') && (rx_bytes[3] == 'p'))
				{
					kpr = kpp = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 1000;
				}

				if ((rx_bytes[1] == 'k') && (rx_bytes[2] == 'd') && (rx_bytes[3] == 'p'))
				{
					kdr = kdp = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 1000;
				}

				if ((rx_bytes[1] == 'k') && (rx_bytes[2] == 'i') && (rx_bytes[3] == 'p'))
				{
					kir = kip = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 100000;
				}

				if ((rx_bytes[1] == 'k') && (rx_bytes[2] == 'o') && (rx_bytes[3] == 'p'))
				{
					ko = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 100;
				}

				if ((rx_bytes[1] == 'k') && (rx_bytes[2] == '1') && (rx_bytes[3] == '1'))
				{
					k1 = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 10000;
				}

				if ((rx_bytes[1] == 'k') && (rx_bytes[2] == '2') && (rx_bytes[3] == '2'))
				{
					k2 = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 10000;
				}

				if ((rx_bytes[1] == 'k') && (rx_bytes[2] == 't') && (rx_bytes[3] == 'o'))
				{
					ktot = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 10000;
				}

				if ((rx_bytes[1] == 's') && (rx_bytes[2] == 'f') && (rx_bytes[3] == 'm'))
				{
					sampleFrequency = (double) (rx_bytes[7] * 256 + rx_bytes[6]);
				}

				if ((rx_bytes[1] == 'r') && (rx_bytes[2] == 's') && (rx_bytes[3] == 't'))
				{
					NVIC_SystemReset();
				}

				if ((rx_bytes[1] == 'm') && (rx_bytes[2] == 'o') && (rx_bytes[3] == 'l'))
				{
					maxOutputLimit = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 100;
				}

				if ((rx_bytes[1] == 's') && (rx_bytes[2] == 'd') && (rx_bytes[3] == 't'))
				{
					deltaT = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 100000;
				}

				if ((rx_bytes[1] == 's') && (rx_bytes[2] == 'a') && (rx_bytes[3] == 'l'))
				{
					alpha = ((double) (rx_bytes[7] * 256 + rx_bytes[6])) / 10000;
				}

				if ((rx_bytes[1] == 't') && (rx_bytes[2] == 'v') && (rx_bytes[3] == 'y'))
				{
					trimmValueYaw = (double)((int16_t)(rx_bytes[7] * 256 + rx_bytes[6])) / 100;
				}

				if ((rx_bytes[1] == 't') && (rx_bytes[2] == 'v') && (rx_bytes[3] == 'r'))
				{
					trimmValueRoll =(double)((int16_t)(rx_bytes[7] * 256 + rx_bytes[6])) / 100;
				}

				if ((rx_bytes[1] == 't') && (rx_bytes[2] == 'v') && (rx_bytes[3] == 'p'))
				{
					trimmValuePitch = (double)((int16_t) (rx_bytes[7] * 256 + rx_bytes[6])) / 100;
				}

				command_ready = 0;
				data_ready = 0;
				rx_bytes_counter = 0;
			}
		}
	}
	else
	{
		command_ready = 0;
		data_ready = 0;
		rx_bytes_counter = 0;
	}
	while (HAL_UART_Receive_IT(&huart4, receiveBuffer, UARTRECEIVEDBUFFERSIZE) != HAL_OK);
}
