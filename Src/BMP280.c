/***********************************************************************************
* BMP280.c
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
#include "BMP280.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;
extern uint8_t sendBMP280Buffer[2];
extern uint8_t readBMP280Buffer[24];
extern int8_t readBMP280INTBuffer[24];
extern double temperature, pressure;
extern uint8_t dataReadyfromMPU9250Counter;

//Writes an 8 bit value over I2C
void write8(uint8_t reg, uint8_t value)
{
	sendBMP280Buffer[0] = reg;
	sendBMP280Buffer[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDRESS, sendBMP280Buffer, 2, 100);
}

//Reads an 8 bit value over I2C/SPI
uint8_t read8(uint8_t reg)
{
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, readBMP280Buffer, 1, 300); //how do we do this with interrupt
	return readBMP280Buffer[0];
}

void BMP280_Init()
{
	write8(BMP280_REGISTER_SOFTRESET, 0xB6); 	//soft reset
	write8(BMP280_REGISTER_CONFIG, 0x30); 		//no SPI, max IIR filter, 62,5ms (15,4 Hz) t_standby
	write8(BMP280_REGISTER_CONTROL, 0x57); 		//temp. oversampl 2x, pressure oversampl. 16x, normal mode
	HAL_Delay(10);

	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, BMP280_REGISTER_STARTCALIB, I2C_MEMADD_SIZE_8BIT, readBMP280Buffer, 24, 100);
	_bmp280_calib.dig_T1 = (uint16_t) ((readBMP280Buffer[1] << 8) + readBMP280Buffer[0]);
	_bmp280_calib.dig_T2 = (int16_t) ((readBMP280Buffer[3] << 8) + readBMP280Buffer[2]);
	_bmp280_calib.dig_T3 = (int16_t) ((readBMP280Buffer[5] << 8) + readBMP280Buffer[4]);
	_bmp280_calib.dig_P1 = (uint16_t) ((readBMP280Buffer[7] << 8) + readBMP280Buffer[6]);
	_bmp280_calib.dig_P2 = (int16_t) ((readBMP280Buffer[9] << 8) + readBMP280Buffer[8]);
	_bmp280_calib.dig_P3 = (int16_t) ((readBMP280Buffer[11] << 8) + readBMP280Buffer[10]);
	_bmp280_calib.dig_P4 = (int16_t) ((readBMP280Buffer[13] << 8) + readBMP280Buffer[12]);
	_bmp280_calib.dig_P5 = (int16_t) ((readBMP280Buffer[15] << 8) + readBMP280Buffer[14]);
	_bmp280_calib.dig_P6 = (int16_t) ((readBMP280Buffer[17] << 8) + readBMP280Buffer[16]);
	_bmp280_calib.dig_P7 = (int16_t) ((readBMP280Buffer[19] << 8) + readBMP280Buffer[18]);
	_bmp280_calib.dig_P8 = (int16_t) ((readBMP280Buffer[21] << 8) + readBMP280Buffer[20]);
	_bmp280_calib.dig_P9 = (int16_t) ((readBMP280Buffer[23] << 8) + readBMP280Buffer[22]);
}

//Reads P and T registers and construct the 20bit value
void readPandT(uint8_t reg, uint8_t buffer[24])
{
	dataReadyfromBMP280 = SET;
	HAL_I2C_Mem_Read_IT(&hi2c1, BMP280_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buffer, 6);
}

double calcDoubleBMP280Temperature(void)
{
	double var1, var2, T;
	var1 = (((double) adc_T) / 16384.0 - ((double) _bmp280_calib.dig_T1) / 1024.0) * ((double) _bmp280_calib.dig_T2);
	var2 = ((((double) adc_T) / 131072.0 - ((double) _bmp280_calib.dig_T1) / 8192.0) * (((double) adc_T) / 131072.0 - ((double) _bmp280_calib.dig_T1) / 8192.0))
			* ((double) _bmp280_calib.dig_T3);
	t_fine = (int32_t) (var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

double calcDoubleBMP280Pressure()
{
	double var1, var2, p;
	var1 = (t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double) _bmp280_calib.dig_P6) / 32768.0;
	var2 = var2 + (var1 * ((double) _bmp280_calib.dig_P5) * 2.0);
	var2 = (var2 / 4.0) + (((double) _bmp280_calib.dig_P4) * 65536.0);
	var1 = (((double) _bmp280_calib.dig_P3) * var1 * var1 / 524288.0 + ((double) _bmp280_calib.dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double) _bmp280_calib.dig_P1);
	if (var1 == 0.0)
		return 0;
	p = 1048576.0 - (double) adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double) _bmp280_calib.dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double) _bmp280_calib.dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double) _bmp280_calib.dig_P7)) / 16.0;
	return p / 100;
}

double calcDoubleBMP280Altitude(double seaLevelhPa)
{
	double altitude;
	//double pressure = calcBMP280Pressure(); // in Si units for Pascal
	//pressure /= 100;
	// altitude = 44330 * (1.0 - pow(pressure / (pressure/pow(1-(600/44330.0),5.255)), 0.1903));
	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 1 / 5.255));
	return altitude;
}
