/*-----------------------------------------------------------------------
     Creator		: Morris chiou
     Sensor		: BMP085  pressure & temperature sensor
     File Name		: SENSOR_BMP085.h
     Function		: SENSOR_BMP085
     Create Date	: 2017/03/21
     
     HAL Layer : SWI2C function
---------------------------------------------------------------------- */
#ifndef __BMP085_HEADER__ 
#define __BMP085_HEADER__  


//--------------------------------- Define SLAVE ADDRESS -------------------------------------
#define BMP085_SLAVE_ADDRESS     	0XEE
//--------------------------------- Define SLAVE ADDRESS -------------------------------------


#define CAL_COEFF_NUMBER         		22
#define sensor_bmp085_debug      		0       //open printf result value



//----------------------------------- Function --------------------------------------
void sensor_bmp085_cal_coeff_function(void);

CHAR8S sensor_bmp085_read_Temp_Press_function(INT32S *temperature_data,INT32S *pressure_data,FLOAT *atmosphere_data,FLOAT *altitude_data,CHAR8U oss);
//----------------------------------- Function --------------------------------------


//--------------------------------------------------------------------------------------



#endif		 //#ifndef __BMP085_HEADER__  
