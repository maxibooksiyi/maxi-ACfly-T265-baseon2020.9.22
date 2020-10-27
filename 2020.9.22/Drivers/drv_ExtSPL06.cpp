#include "Basic.hpp"
#include "drv_ExtSPL06.hpp"
#include "drv_ExtIIC.hpp"
#include "SensorsBackend.hpp"
#define SPLO6_ADDR 0x77


//缓冲区
Static_AXIDMABuf uint8_t tx_buf[12];
__attribute__ ((aligned (4))) Static_AXIDMABuf uint8_t rx_buf[32];	

struct SPL06_COEFFICIENTS
{
	int16_t c0;	int16_t c1;
	int32_t c00;	int32_t c10;	int16_t c01;	int16_t c11;
	int16_t c20;	int16_t c21;	
	int16_t c30;
	double KP;	double KT;
};
volatile static SPL06_COEFFICIENTS SPL06_coefficients;
static void spl06_pressure_rateset( uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
	uint8_t reg = 0;
	switch(u8SmplRate)
	{
		case 2:
			reg |= (1<<4);
			break;
		case 4:
			reg |= (2<<4);
			break;
		case 8:
			reg |= (3<<4);
			break;
		case 16:
			reg |= (4<<4);
			break;
		case 32:
			reg |= (5<<4);
			break;
		case 64:
			reg |= (6<<4);
			break;
		case 128:
			reg |= (7<<4);
			break;
		case 1:
		default:
			break;
	}
	switch(u8OverSmpl)
	{
		case 2:
			reg |= 1;
			SPL06_coefficients.KP = 1.0 / 1572864;
			break;
		case 4:
			reg |= 2;
			SPL06_coefficients.KP = 1.0 / 3670016;
			break;
		case 8:
			reg |= 3;
			SPL06_coefficients.KP = 1.0 / 7864320;
			break;
		case 16:
			SPL06_coefficients.KP = 1.0 / 253952;
			reg |= 4;
			break;
		case 32:
			SPL06_coefficients.KP = 1.0 / 516096;
			reg |= 5;
			break;
		case 64:
			SPL06_coefficients.KP = 1.0 / 1040384;
			reg |= 6;
			break;
		case 128:
			SPL06_coefficients.KP = 1.0 / 2088960;
			reg |= 7;
			break;
		case 1:
		default:
			SPL06_coefficients.KP = 1.0 / 524288;
			break;
	}
	tx_buf[0] = 0x06;
	tx_buf[1] = reg;
  ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
	os_delay(0.1);
	
	//Pressure result bit-shift,Must be set to '1' when the oversampling rate is  >8  times.
	if(u8OverSmpl > 8)
	{
		tx_buf[0] = 0x09;
		ExtIIC_SendReceiveAddr7(SPLO6_ADDR, tx_buf, 1, rx_buf, 1);

		tx_buf[0] = 0x09;
		tx_buf[1] = (1<<2)|rx_buf[0];
		ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
		os_delay(0.1);
	}
}
static void spl06_temperature_rateset( uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
		uint8_t reg = 0;
		switch(u8SmplRate)
		{
			case 2:
				reg |= (1<<4);
				break;
			case 4:
				reg |= (2<<4);
				break;
			case 8:
				reg |= (3<<4);
				break;
			case 16:
				reg |= (4<<4);
				break;
			case 32:
				reg |= (5<<4);
				break;
			case 64:
				reg |= (6<<4);
				break;
			case 128:
				reg |= (7<<4);
				break;
			case 1:
			default:
				break;
		}
		switch(u8OverSmpl)
		{
			case 2:
				reg |= 1;
				SPL06_coefficients.KT = 1.0f / 1572864;
				break;
			case 4:
				reg |= 2;
				SPL06_coefficients.KT = 1.0f / 3670016;
				break;
			case 8:
				reg |= 3;
				SPL06_coefficients.KT = 1.0f / 7864320;
				break;
			case 16:
				SPL06_coefficients.KT = 1.0f / 253952;
				reg |= 4;
				break;
			case 32:
				SPL06_coefficients.KT = 1.0f / 516096;
				reg |= 5;
				break;
			case 64:
				SPL06_coefficients.KT = 1.0f / 1040384;
				reg |= 6;
				break;
			case 128:
				SPL06_coefficients.KT = 1.0f / 2088960;
				reg |= 7;
				break;
			case 1:
			default:
				SPL06_coefficients.KT = 1.0f / 524288;
				break;
		}
		tx_buf[0] = 0x07;
		tx_buf[1] = (1<<7) | reg;
		ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
		os_delay(0.01);
		//Temperature result bit-shift,Must be set to '1' when the oversampling rate is  >8  times.
		if(u8OverSmpl > 8)
		{
			tx_buf[0] = 0x09;
			ExtIIC_SendReceiveAddr7(SPLO6_ADDR, tx_buf, 1, rx_buf, 1);
			
			tx_buf[0] = 0x09;
			tx_buf[1] = (1<<3)|rx_buf[0];
			ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
			os_delay(0.01);
		}
	}

static void ExtSPL06_Server(void* pvParameters)
{			
	//操作结果
	bool res;
	//准确周期延时
	TickType_t xLastWakeTime;
ScanExtSPL06:	
	xLastWakeTime = xTaskGetTickCount();
  while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, 0.5*configTICK_RATE_HZ );
		
		//读取ID
		tx_buf[0] = 0x0D;
		res = ExtIIC_SendReceiveAddr7( SPLO6_ADDR, tx_buf, 1, rx_buf, 1 );
		os_delay(0.1);
		if(!res || rx_buf[0]!=0x10)
			continue;
		
		//复位
		tx_buf[0] = 0x0c;
		tx_buf[1] = (1<<7) | 0b1001;
		res = ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
		os_delay(0.5);
		if(!res)
			continue;		
	
		//配置传感器
		spl06_pressure_rateset( 64 , 32 );//pressure 64 samples per sec , 32 times over sampling		
		spl06_temperature_rateset( 128 , 2 );//temperature 128 samples per sec , 2 times over sampling		
		tx_buf[0] = 0x08;
		tx_buf[1] = 0b111;  //Continuous pressure and  temperature  measur ement
		res = ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);	
    os_delay(0.1);		
		if(!res)
			continue;
		
    //读取校准系数		
		tx_buf[0] = 0x10;
		res = ExtIIC_SendReceiveAddr7( SPLO6_ADDR, tx_buf, 1, rx_buf, 18 );
		if(!res)
			continue;
		SPL06_coefficients.c0 = ( rx_buf[0] << 4 ) | ( rx_buf[1] >> 4 );
		SPL06_coefficients.c0 = ( SPL06_coefficients.c0 & 0x0800 ) ? (0xF000|SPL06_coefficients.c0) : SPL06_coefficients.c0;
		SPL06_coefficients.c1 = ( (rx_buf[1] & 0xf) << 8 ) | ( rx_buf[2] );
		SPL06_coefficients.c1 = ( SPL06_coefficients.c1 & 0x0800 ) ? (0xF000|SPL06_coefficients.c1) : SPL06_coefficients.c1;
		SPL06_coefficients.c00 = ( rx_buf[3] << 12 ) | ( rx_buf[4] << 4 ) | ( rx_buf[5] >> 4 );
		SPL06_coefficients.c00 = ( SPL06_coefficients.c00 & 0x080000 ) ? (0xFFF00000|SPL06_coefficients.c00) : SPL06_coefficients.c00;
		SPL06_coefficients.c10 = ( (rx_buf[5] & 0xf) << 16 ) | ( rx_buf[6] << 8 ) | ( rx_buf[7] >> 0 );
		SPL06_coefficients.c10 = ( SPL06_coefficients.c10 & 0x080000 ) ? (0xFFF00000|SPL06_coefficients.c10) : SPL06_coefficients.c10;
		SPL06_coefficients.c01 = ( rx_buf[8] << 8 ) | ( rx_buf[9] << 0 );
		SPL06_coefficients.c11 = ( rx_buf[10] << 8 ) | ( rx_buf[11] << 0 );
		SPL06_coefficients.c20 = ( rx_buf[12] << 8 ) | ( rx_buf[13] << 0 );
		SPL06_coefficients.c21 = ( rx_buf[14] << 8 ) | ( rx_buf[15] << 0 );
		SPL06_coefficients.c30 = ( rx_buf[16] << 8 ) | ( rx_buf[17] << 0 );		
		//注册传感器
		if(PositionSensorRegister( external_baro_sensor_index , \
														Position_Sensor_Type_GlobalPositioning , \
														Position_Sensor_DataType_s_z , \
														Position_Sensor_frame_ENU , \
														0.05 , //延时
														0 ,	//xy信任度
														50 //z信任度
														))
		{
      goto ExtSPL06Detected;		
		}		
	}
	
ExtSPL06Detected:
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, 0.033*configTICK_RATE_HZ);	
		
		int32_t buf32[2];
		
		tx_buf[0] = 0x00;
		res = ExtIIC_SendReceiveAddr7( SPLO6_ADDR, tx_buf, 1, rx_buf, 6 );
		if(!res)
		{
			PositionSensorUnRegister(external_baro_sensor_index);
			goto ScanExtSPL06;
		}
		struct __SPL06_Data
		{
			unsigned int pressure:24 ;
			unsigned int temperature:24 ;
		}__PACKED;
		__SPL06_Data* datap = (__SPL06_Data*)rx_buf;
		buf32[0] = __REV( datap->pressure ) >> 8;		
		buf32[1] = __REV( datap->temperature ) >> 8;	
		
		buf32[0] = ( buf32[0] & 0x800000 ) ? (0xFF000000|buf32[0]) : buf32[0];
		buf32[1] = ( buf32[1] & 0x800000 ) ? (0xFF000000|buf32[1]) : buf32[1];
		
		double fPsc = buf32[0] * SPL06_coefficients.KP;
		double fTsc = buf32[1] * SPL06_coefficients.KT;
		double qua2 = SPL06_coefficients.c10 + fPsc * (SPL06_coefficients.c20 + fPsc* SPL06_coefficients.c30);
		double qua3 = fTsc * fPsc * (SPL06_coefficients.c11 + fPsc * SPL06_coefficients.c21);
		
		double pressure = SPL06_coefficients.c00 + fPsc * qua2 + fTsc * SPL06_coefficients.c01 + qua3;
		double temperature = SPL06_coefficients.c0*0.5 + SPL06_coefficients.c1*fTsc;
		//更新传感器数据
		if( pressure > 0 )
		{
			vector3<double> position;
			position.z = 1.07 * 4430000 * ( 1.0 - pow( pressure / 101325.0 , 1.0 / 5.256 ) );
			PositionSensorUpdatePosition( external_baro_sensor_index , position , true , -1 );
		}
	}
}

void init_drv_ExtSPL06()
{
	xTaskCreate( ExtSPL06_Server, "ExtSPL06", 500, NULL, SysPriority_ExtSensor, NULL);
}
