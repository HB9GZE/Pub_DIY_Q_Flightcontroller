/***********************************************************************************
* MPU9250.c
*
* Implementation the MPU9250
* See: http://www.invensens.com
*
* Date			Author          Notes
* 30/01/2017	Stefan Meyre    Initial release
 *
***********************************************************************************/
#include "stm32f4xx_hal.h"
#include "MPU9250.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t sendMPU9250Buffer[2];
extern uint8_t sendAK8963Buffer[2];
extern uint8_t magCalibration[3];
extern double fabCalConst[3];

void MPU9250_Init()
{
	sendMPU9250Buffer[0] = PWR_MGMT_1;
	sendMPU9250Buffer[1] = 0x80; //clear sleep mode bit(6), enable all sensors
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);
	HAL_Delay(2);

	sendMPU9250Buffer[0] = PWR_MGMT_1;
	sendMPU9250Buffer[1] = 0x01; //auto select clock source to be PLL X axis gyroscope reference if ready
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);
	HAL_Delay(2);

	sendMPU9250Buffer[0] = USER_CTRL;
	sendMPU9250Buffer[1] = 0x00; // disable master mode; attach external I2C to AK8963
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);
	HAL_Delay(2);

	sendMPU9250Buffer[0] = INT_PIN_CFG;
	sendMPU9250Buffer[1] = 0x02; //no interrupt now; active bypass
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);
	HAL_Delay(2);

		// start reading the factory cal. values
		sendAK8963Buffer[0] = AK8963_CNTL1;
		sendAK8963Buffer[1] = 0x00; //power down
		HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, sendAK8963Buffer, 2, 300);
		HAL_Delay(5);

		sendAK8963Buffer[0] = AK8963_CNTL1;
		sendAK8963Buffer[1] = 0x0F; //power up in fuse ROM read mode
		HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, sendAK8963Buffer, 2, 300);
		HAL_Delay(5);

		HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, magCalibration, 3, 300); //read the factory stored calibration values
		HAL_Delay(5);

		fabCalConst[0] = ((((double) magCalibration[0] - 128) * 0.5 / 128) + 1);
		fabCalConst[1] = ((((double) magCalibration[1] - 128) * 0.5 / 128) + 1);
		fabCalConst[2] = ((((double) magCalibration[2] - 128) * 0.5 / 128) + 1);

		sendAK8963Buffer[0] = AK8963_CNTL1;
		sendAK8963Buffer[1] = 0x00; //power down
		HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, sendAK8963Buffer, 2, 300);
		HAL_Delay(5);

		sendAK8963Buffer[0] = AK8963_CNTL1;
		sendAK8963Buffer[1] = 0x06; //power up and set to 16 bit and 100Hz update (cont. meas. mode 2)= 0x16; in 14bit resolution: 0x06;
		HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, sendAK8963Buffer, 2, 300);
		HAL_Delay(5);

	sendMPU9250Buffer[0] = I2C_MST_CTRL;
	sendMPU9250Buffer[1] = 0x4D; //turn on master I2C clock at 400kHz and Wait_for_ES
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = USER_CTRL;
	sendMPU9250Buffer[1] = 0x20; //enable master mode; isolate external I2C from AK8963
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = INT_PIN_CFG;
	sendMPU9250Buffer[1] = 0x10; //only int pulse, clear on read, no bypass
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = CONFIG;
	sendMPU9250Buffer[1] = 0x01; //disable FSYNC. sample rate at 1000Hz,gyro BW: 0x01= 184Hz,  0x05 = 10Hz
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);
	HAL_Delay(5);

	sendMPU9250Buffer[0] = SMPLRT_DIV;
	sendMPU9250Buffer[1] = 0x01; //sampling rate 0x02 = /3 equals to 333Hz rate
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = GYRO_CONFIG;
	sendMPU9250Buffer[1] = 0x00;  //set gyro sensitivity to 0x18=2000dps, 0x08=500dps, 0x10=1000dps, 0x00=250dps
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = ACCEL_CONFIG;
	sendMPU9250Buffer[1] = 0x10;  //set accelerator sensitivity to 0x10 = 8g, 0x00 = 2g
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = ACCEL_CONFIG2;
	sendMPU9250Buffer[1] = 0x03;  //0x03 = rate 1kHz and BW 44.8Hz, 0x05 = 10Hz
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = FIFO_EN;
	sendMPU9250Buffer[1] = 0x00; //no FIFO
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = I2C_SLV0_ADDR;
	sendMPU9250Buffer[1] = 0x8C; // address of magnetometer is 0x0C and set to READ
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = I2C_SLV0_REG;
	sendMPU9250Buffer[1] = 0x02; // start read data from where?
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = I2C_SLV0_CTRL;
	sendMPU9250Buffer[1] = 0x88; // reads 8 bytes. no swap, group with even end
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);

	sendMPU9250Buffer[0] = INT_ENABLE;
	sendMPU9250Buffer[1] = 0x01; //raw data ready
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, sendMPU9250Buffer, 2, 100);
}
