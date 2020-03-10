/*-----------------------------------------------------------------------
     Creator		: Morris chiou
     Sensor		: pressure & temperature sensor
     File Name		: SENSOR_BMP085.c
     Function		: SENSOR_BMP085
     Create Date	: 2017/03/21
---------------------------------------------------------------------- */
#ifndef __BMP085_FUNCTION__
#define __BMP085_FUNCTION__

#include <stdio.h>
#include <math.h>
#include <delay.h>

#include "SENSOR_BMP085.h"
#include "SWI2C.h"

 struct{
        INT16S AC1;
        INT16S AC2;
        INT16S AC3;
        INT16S AC4;
        INT16S AC5;
        INT16S AC6;
        INT16S B1;
        INT16S B2;
        INT16S MB;
        INT16S MC;
        INT16S MD;
        }BMP085_CAL_COFF;


void sensor_bmp085_cal_coeff_function(void)
{

       CHAR8U BMP_CAL_COFF_ARRAY[CAL_COEFF_NUMBER]={0};
       INT16U count;
       CHAR8S fail_flag=0;

        for(count=0;count<CAL_COEFF_NUMBER;count++)
        {
            fail_flag = i2c_read_1_byte_data(BMP085_SLAVE_ADDRESS,(0xAA+count),&BMP_CAL_COFF_ARRAY[count]);
            if(fail_flag<=0)
                    {
                     printf("read error in CAL_COEFF_NUMBER[%d] \r\n",count);
                     break;
                    }
        }

        //calculating Calibration Coefficients
        BMP085_CAL_COFF.AC1 = (INT16S) ( (BMP_CAL_COFF_ARRAY[0]<<8) + BMP_CAL_COFF_ARRAY[1]);
        BMP085_CAL_COFF.AC2 = (INT16S) ( (BMP_CAL_COFF_ARRAY[2]<<8) + BMP_CAL_COFF_ARRAY[3]);
        BMP085_CAL_COFF.AC3 = (INT16S) ( (BMP_CAL_COFF_ARRAY[4]<<8) + BMP_CAL_COFF_ARRAY[5]);
        BMP085_CAL_COFF.AC4 = (INT16S) ( (BMP_CAL_COFF_ARRAY[6]<<8) + BMP_CAL_COFF_ARRAY[7]);
        BMP085_CAL_COFF.AC5 = (INT16S) ( (BMP_CAL_COFF_ARRAY[8]<<8) + BMP_CAL_COFF_ARRAY[9]);
        BMP085_CAL_COFF.AC6 = (INT16S) ( (BMP_CAL_COFF_ARRAY[10]<<8) + BMP_CAL_COFF_ARRAY[11]);
        BMP085_CAL_COFF.B1 =  (INT16S) ( (BMP_CAL_COFF_ARRAY[12]<<8) + BMP_CAL_COFF_ARRAY[13]);
        BMP085_CAL_COFF.B2 =  (INT16S) ( (BMP_CAL_COFF_ARRAY[14]<<8) + BMP_CAL_COFF_ARRAY[15]);
        BMP085_CAL_COFF.MB =  (INT16S) ( (BMP_CAL_COFF_ARRAY[16]<<8) + BMP_CAL_COFF_ARRAY[17]);
        BMP085_CAL_COFF.MC =  (INT16S) ( (BMP_CAL_COFF_ARRAY[18]<<8) + BMP_CAL_COFF_ARRAY[19]);
        BMP085_CAL_COFF.MD =  (INT16S) ( (BMP_CAL_COFF_ARRAY[20]<<8) + BMP_CAL_COFF_ARRAY[21]);

#if sensor_bmp085_debug

         for(count=0;count<CAL_COEFF_NUMBER;count++)
         {
            printf("a[0x%X]=0x%X  ",(0xAA+count),BMP_CAL_COFF_ARRAY[count]);
                if(count %3==0 && count!=0 )printf("\r\n");
         }
            printf("\r\n");

         printf("AC1 = 0x%X(%d)\r\n", BMP085_CAL_COFF.AC1,BMP085_CAL_COFF.AC1);
         printf("AC2 = 0x%X(%d)\r\n", BMP085_CAL_COFF.AC2,BMP085_CAL_COFF.AC2);
         printf("AC3 = 0x%X(%d)\r\n", BMP085_CAL_COFF.AC3,BMP085_CAL_COFF.AC3);
         printf("AC4 = 0x%X(%d)\r\n", BMP085_CAL_COFF.AC4,BMP085_CAL_COFF.AC4);
         printf("AC5 = 0x%X(%d)\r\n", BMP085_CAL_COFF.AC5,BMP085_CAL_COFF.AC5);
         printf("AC6 = 0x%X(%d)\r\n", BMP085_CAL_COFF.AC6,BMP085_CAL_COFF.AC6);
         printf("B1 = 0x%X(%d)\r\n", BMP085_CAL_COFF.B1,BMP085_CAL_COFF.B1);
         printf("B2 = 0x%X(%d)\r\n", BMP085_CAL_COFF.B2,BMP085_CAL_COFF.B2);
         printf("MB = 0x%X(%d)\r\n", BMP085_CAL_COFF.MB,BMP085_CAL_COFF.MB);
         printf("MC = 0x%X(%d)\r\n", BMP085_CAL_COFF.MC,BMP085_CAL_COFF.MC);
         printf("MD = 0x%X(%d)\r\n", BMP085_CAL_COFF.MD,BMP085_CAL_COFF.MD);

#endif

}



CHAR8S sensor_bmp085_read_Temp_Press_function(INT32S *temperature_data,INT32S *pressure_data,FLOAT *atmosphere_data,FLOAT *altitude_data,CHAR8U oss)

{

        CHAR8S fail_flag=0;

        INT32S X1,X2,X3,B5,REAL_TEMPUTRE;
        INT32S B3,B6,B7;
        INT32U B4;

        INT32S Temp_value;
        INT32S Press_value;

        CHAR8U high_byte_temp,low_byte_temp;
        CHAR8U high_byte_press,low_byte_press,x_low_byte_press;

        INT32S pr1,pr2,pr3;
        INT32S pr4,pr5,final_p;

        FLOAT  p_float;
        FLOAT altitude;
        FLOAT alt_temp1,alt_temp2,alt_temp3;



        // Read Calibration Coefficients & Calculating
        // I2C register address 0xAA~0xBF
        sensor_bmp085_cal_coeff_function();


        //-----------------------------------------------------------------------------------------------------------------
        // temperature measurement
        fail_flag = i2c_write_1_byte_data(BMP085_SLAVE_ADDRESS,0XF4,0X2E);
           if(fail_flag == SWI2C_STATUS_FAIL)  return -1;

        delay_ms(6);

        fail_flag = i2c_read_1_byte_data(BMP085_SLAVE_ADDRESS,0XF6,&high_byte_temp);
           if(fail_flag == SWI2C_STATUS_FAIL)  return -1;

        fail_flag = i2c_read_1_byte_data(BMP085_SLAVE_ADDRESS,0XF7,&low_byte_temp);
           if(fail_flag == SWI2C_STATUS_FAIL)  return -1;

        delay_us(30);
        //-----------------------------------------------------------------------------------------------------------------



       //-----------------------------------------------------------------------------------------------------------------
       // pressure measurement
        fail_flag = i2c_write_1_byte_data(BMP085_SLAVE_ADDRESS,0XF4, 0X34 + (CHAR8U)(oss<< 6) );
            if(fail_flag == SWI2C_STATUS_FAIL)  return -1;

       switch(oss)
        {
         case 0://datasheet max time :4.5ms
             delay_ms(6);
             break;
         case 1://datasheet max time :7.5ms
             delay_ms(9);
             break;
         case 2://datasheet max time :13.5ms
             delay_ms(15);
             break;
         case 3://datasheet max time :25.5ms
             delay_ms(27);
             break;
         default:
             return -1;  //wrong value
        }

        fail_flag = i2c_read_1_byte_data(BMP085_SLAVE_ADDRESS,0XF6,&high_byte_press);
             if(fail_flag == SWI2C_STATUS_FAIL)  return -1;

        fail_flag = i2c_read_1_byte_data(BMP085_SLAVE_ADDRESS,0XF7,&low_byte_press);
             if(fail_flag == SWI2C_STATUS_FAIL)  return -1;

        fail_flag = i2c_read_1_byte_data(BMP085_SLAVE_ADDRESS,0XF8,&x_low_byte_press);
             if(fail_flag == SWI2C_STATUS_FAIL)  return -1;
        //-----------------------------------------------------------------------------------------------------------------



        // ***************** Calculation Tempearture *****************
        Temp_value = (INT32S) ((high_byte_temp << 8) + low_byte_temp);

        X1 = ((INT32S)Temp_value - BMP085_CAL_COFF.AC6)*(BMP085_CAL_COFF.AC5)/32768;
        X2 =  (INT32S)((INT32S)BMP085_CAL_COFF.MC *2048)/(INT32S)(X1 + (INT32S)BMP085_CAL_COFF.MD);
        B5 = (INT32S)(X1 + X2);
        REAL_TEMPUTRE = (INT32S)(B5 + 8)/ 16;        // Final Temperature , ex: 153 temp  setp in 0.1 , 15.3 oC
        // ***********************************************************


        // ***************** Calculation Pressuree *****************
        pr1 = (INT32S)high_byte_press << 16;
        pr2 = (INT32S)low_byte_press << 8;
        pr3 = (INT32S)x_low_byte_press;
        pr4 = (INT32S)(8-oss);
        Press_value = (INT32S)((pr1+pr2+pr3) >> pr4);


        B6 = (INT32S)(B5 - 4000);
        pr1 = (INT32S)(B6 * B6)/4096;

        X1 = ((BMP085_CAL_COFF.B2)*pr1)/2048 ;
        X2 = (INT32S) ((INT32S)BMP085_CAL_COFF.AC2) * B6 / 2048;
        X3 = X1 + X2;
        B3 =(INT32S)(((((INT32S)BMP085_CAL_COFF.AC1*4+X3)<< (INT32S)(oss) ) + 2)/4 );
        X1 =(INT32S)(((INT32S)BMP085_CAL_COFF.AC3 * B6)/8192);
        X2 =(INT32S)(((INT32S)BMP085_CAL_COFF.B1 * ((B6*B6)/4096))/65536);
        X3 =(INT32S)(((X1 + X2) + 2)/4);
        pr4 =(INT32S)(X3 + 32768);
        pr5 =(INT32S)BMP085_CAL_COFF.AC4;
        B4 = (INT32U) ((pr4 * pr5)/32768);
        pr1 = (INT32S)(50000 >> (INT32S)(oss));
        B7 = (INT32S)((INT32U)Press_value - B3) * pr1;


        if( B7 < (0x80000000) )
            {
             p_float = ((FLOAT)B7*2/(FLOAT)B4);
            }
        else
            {
             p_float = (((FLOAT)B7*(FLOAT)B4)/2);
            }


        X1 =(INT32S) ((p_float/256) *(p_float/256));
        X1 = ((X1 *3038)/65536);
        X2 = (INT32S)((-7357 * p_float)/65536);
        p_float = (p_float + ((FLOAT)X1 +(FLOAT)X2 + 3791)/16);
        // *********************************************************



        // ***************** Calculation Atmosphere , Altitude  *****************
        alt_temp1 = ((FLOAT)p_float/101325);
        alt_temp2 = (1/5.255);
        alt_temp3 = pow(alt_temp1,alt_temp2);
        alt_temp3 = (1 - alt_temp3);
        alt_temp3 = (alt_temp3/0.0000225577);
        // **********************************************************************

#if sensor_bmp085_debug
        printf("Temp_value = %ld\r\n",Temp_value);
        printf("X1 = %ld\r\n",(INT32S)X1);
        printf("X2 = %ld\r\n",(INT32S)X2);
        printf("B5 = %lu\r\n",(INT32S)B5);
        printf("REAL_TEMPUTRE = %ld.%ld\r\n",(INT32S)REAL_TEMPUTRE/10,(INT32S)REAL_TEMPUTRE%10);
#endif


        // ***************** Final parameter  *****************
        *pressure_data = (INT32S) (p_float);                        // Final Pressure , ex: 100053 Pa = 1000.53 hpa
        *atmosphere_data = alt_temp1;                               // Final Atmosphere
        *altitude_data = alt_temp3;                                 // Final altitude
        *temperature_data = (INT32S)REAL_TEMPUTRE;                  // Final Temperature

        return 0;

}
#endif		//#ifndef __BMP085_FUNCTION__