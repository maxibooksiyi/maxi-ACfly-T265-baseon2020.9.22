#include "Basic.hpp"
#include "drv_Sensors.hpp"

#include "FreeRTOS.h"
#include "timers.h"
#include "SensorsBackend.hpp"
#include "AC_Math.hpp"
#include "MS_Main.hpp"
#include "TD4.hpp"
#include <limits>

/*DMA����*/
	#define DMATxStream_Enable_MINC ( DMA1_Stream0->CR |= (1<<10) )
	#define DMATxStream_Disable_MINC ( DMA1_Stream0->CR &= ~(1<<10) )
	
	#define DMARxStream_Enable_MINC ( DMA1_Stream1->CR |= (1<<10) )
	#define DMARxStream_Disable_MINC ( DMA1_Stream1->CR &= ~(1<<10) )
	
	#define DMARxStream_Enable_IRQ ( NVIC_EnableIRQ(DMA1_Stream1_IRQn) )
	#define DMARxStream_Disable_IRQ ( NVIC_DisableIRQ(DMA1_Stream1_IRQn) )
/*DMA����*/

/*������CS���Ų���*/
	#define PullUp_BMI088Gyro_CS (GPIOE->BSRR = (1<<15))
	#define PullUp_BMI088Accel_CS (GPIOB->BSRR = (1<<11))
	#define PullUp_SPL06_CS (GPIOB->BSRR = (1<<10))
	#define PullUp_AK8975_CS (GPIOA->BSRR = (1<<4))

	#define PullDown_BMI088Gyro_CS (GPIOE->BSRR = (1<<31))
	#define PullDown_BMI088Accel_CS (GPIOB->BSRR = (1<<27))
	#define PullDown_SPL06_CS (GPIOB->BSRR = (1<<26))
	#define PullDown_AK8975_CS (GPIOA->BSRR = (1<<20))
/*������CS���Ų���*/

/*�¶ȿ��Ʋ���*/
	static inline void set_TC_Out( double u )
	{
		u = constrain( u , 0.0 , 100.0 );
		u *= 0.01*TIM3->ARR;
		TIM3->CCR4 = u;
	}
	static float target_IMUTemp = 0;
	void set_TargetIMUTemperature( float temp )
	{
		if( temp > 30 )
			target_IMUTemp = temp;
		else
			target_IMUTemp = 0;
	}
/*�¶ȿ��Ʋ���*/

/*SPI����*/
	//��SPI��������
	static void SPI1_Transmit_Start( const uint8_t* TxBuf , uint16_t size );
	//��SPI�������ݲ��ȴ��������
	static void SPI1_Transmit( const uint8_t* TxBuf , uint16_t size );
	
	//��SPI��������
	static void SPI1_TransmitReceive_Start( const uint8_t* TxBuf , uint8_t* RxBuf , uint16_t size );
	//��SPI�������ݲ��ȴ��������
	static void SPI1_TransmitReceive( const uint8_t* TxBuf , uint8_t* RxBuf , uint16_t size );
/*SPI����*/

/*������״̬��*/
	
	#ifdef DCACHE_SIZE
		#define TxBufferSize DCACHE_SIZE
		#define RxBufferSize DCACHE_SIZE
		static Aligned_DMABuf uint8_t TxBuf[TxBufferSize];
		static Aligned_DMABuf uint8_t RxBuf[RxBufferSize];
	#else
		#define BufferSize 32
		static uint8_t TxBuf[BufferSize];
		static uint8_t RxBuf[BufferSize];
	#endif
	static int8_t current_read_sensor = -1;	//-1:�޴�����	
																					//0:BMI088_Gyro	
																					//1:BMI088_Accel	
																					//8:AK8975���׶�
																					//9:AK8975д�׶�
																					//16:SPL06
	static bool nd_read_BMI088_Gyro = false;
	static bool nd_read_BMI088_Accel = false;
	static bool nd_read_AK8975 = false;
	static bool nd_read_SPL06 = false;

	#define TemperatureSensors_Count 2
	static float IMU_Temperature[TemperatureSensors_Count] = {0};
/*������״̬��*/

/*BMI088����*/
	//����
	struct __BMI088_GyroData
	{
		uint8_t rsv1;
		int16_t gyro[3];	//0x02-0x07
	}__PACKED;
	
	//���ٶ�
	struct __BMI088_AccelData
	{
		uint8_t rsv1[2];
		int16_t acc[3];	//0x12-0x17
		uint32_t sensor_time:24;	//0x18-0x1a
		uint8_t rsv2[2];	//0x1b-0x1c
		uint8_t acc_int_stat;	//0x1d
		uint8_t rsv3[4];	//0x1e-0x21
		uint16_t temperature;	//0x22-0x23
	}__PACKED;
/*BMI088����*/

/*AK8975У׼����*/
	static vector3<double> AK8974_ASA;
	struct __AK8975_Data
	{
		uint8_t rsv1;
		int16_t mag[3];	//0x03-0x08
	}__PACKED;
/*AK8975У׼����*/

/*SPL06У׼����*/
	struct SPL06_COEFFICIENTS
	{
		int16_t c0;	int16_t c1;
		int32_t c00;	int32_t c10;	int16_t c01;	int16_t c11;
		int16_t c20;	int16_t c21;	
		int16_t c30;
		double KP;	double KT;
	};
	static SPL06_COEFFICIENTS SPL06_coefficients;
	struct __SPL06_Data
	{
		uint8_t rsv1;
		unsigned int pressure:24 ;
		unsigned int temperature:24 ;
	}__PACKED;
	
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
		TxBuf[0] = (0<<7) | 0x06;
		TxBuf[1] = reg;
		PullDown_SPL06_CS;
		SPI1_Transmit( TxBuf , 2 );
		PullUp_SPL06_CS;
		os_delay(0.01);
		if(u8OverSmpl > 8)
		{
			TxBuf[0] = (1<<7) | 0x09;
			PullDown_SPL06_CS;
			SPI1_TransmitReceive( TxBuf , RxBuf ,  2 );
			PullUp_SPL06_CS;
			os_delay(0.01);
	
			TxBuf[0] = (0<<7) | 0x09;
			TxBuf[1] = RxBuf[1] | (1<<2);	
			PullDown_SPL06_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_SPL06_CS;
			os_delay(0.01);
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
		TxBuf[0] = (0<<7) | 0x07;
		TxBuf[1] = (1<<7) | reg;
		PullDown_SPL06_CS;
		SPI1_Transmit( TxBuf , 2 );
		PullUp_SPL06_CS;
		os_delay(0.01);
		if(u8OverSmpl > 8)
		{
			TxBuf[0] = (1<<7) | 0x09;
			PullDown_SPL06_CS;
			SPI1_TransmitReceive( TxBuf , RxBuf ,  2 );
			PullUp_SPL06_CS;
			os_delay(0.01);
	
			TxBuf[0] = (0<<7) | 0x09;
			TxBuf[1] = RxBuf[1] | (1<<2);	
			PullDown_SPL06_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_SPL06_CS;
			os_delay(0.01);
		}
	}
/*SPL06У׼����*/

void init_drv_Sensors()
{
	/*IO��ʼ��
		SPI1_SCK:PA5
		SPI1_MOSI:PA7
		SPI1_MISO:PA6
		
		BMI088_Gyro_Cs:PE15
		BMI088_Accel_Cs:PB11
		SPL06_Cs:PB10
		AK8975_Cs:PA4
	*/
		//��GPIOʱ��
		RCC->AHB4ENR |= (1<<4) | (1<<1) | (1<<0);
		delay(1e-5);
	
		//����Moder���ù��ܣ�SPI�������ģʽ��CS��
		set_register( GPIOA->MODER , 0b10 , 10 , 2 );	//PA5
		set_register( GPIOA->MODER , 0b10 , 14 , 2 );	//PA7
		set_register( GPIOA->MODER , 0b10 , 12 , 2 );	//PA6
		set_register( GPIOE->MODER , 0b01 , 30 , 2 );	//PE15
		set_register( GPIOB->MODER , 0b01 , 22 , 2 );	//PB11
		set_register( GPIOB->MODER , 0b01 , 20 , 2 );	//PB10
		set_register( GPIOA->MODER , 0b01 , 8 , 2 );	//PA4
		
		//SCLK��MOSI��CS���������MISO��©����
		GPIOA->OTYPER |= (1<<6);
		set_register( GPIOA->PUPDR , 0b01 , 12 , 2 );	//PA6
		
		//�����ٶ�
		set_register( GPIOA->OSPEEDR , 0b01 , 10 , 2 );	//PA5
		set_register( GPIOA->OSPEEDR , 0b01 , 14 , 2 );	//PA7
		set_register( GPIOA->OSPEEDR , 0b01 , 12 , 2 );	//PA6
		set_register( GPIOE->OSPEEDR , 0b01 , 30 , 2 );	//PE15
		set_register( GPIOB->OSPEEDR , 0b01 , 22 , 2 );	//PB11
		set_register( GPIOB->OSPEEDR , 0b01 , 20 , 2 );	//PB10
		set_register( GPIOA->OSPEEDR , 0b01 , 8 , 2 );	//PA4
		
		//���ø��ù���
		set_register( GPIOA->AFR[0] , 5 , 20 , 4 );	//PA5
		set_register( GPIOA->AFR[0] , 5 , 28 , 4 );	//PA7
		set_register( GPIOA->AFR[0] , 5 , 24 , 4 );	//PA6
	/*IO��ʼ��*/
	
	/*���ȵ�·��ʼ��*/	
		//����GPIOB����ʱ��
		RCC->AHB4ENR|=(1<<1);
		os_delay(1e-2); 
		//�������Ÿ���ģʽ
		set_register( GPIOB->MODER , 0b10 , 2 , 2 );	//PB1
		//��������TIM3����
		set_register( GPIOB->AFR[0] , 2 , 4 , 4 );	//PB1
			
		//����TIM3����ʱ��
		RCC->APB1LENR|=(1<<1);
		os_delay(1e-2);

		//����PWM1ģʽ
		TIM3->CCMR2 |= (0b110<<12)|(1<<11)|(0<<8);
		
		//��λ���
		TIM3->CCR4 = 0*TIM3->ARR;
		
		//������ʱ��
		TIM3->CCER |= (1<<12);
	/*���ȵ�·��ʼ��*/
	
	/*SPI1��ʼ��*/
		//��������CS
		PullUp_BMI088Gyro_CS;
		PullUp_BMI088Accel_CS;
		PullUp_SPL06_CS;
		PullUp_AK8975_CS;
			
		//��SPI1ʱ��
		RCC->APB2ENR |= (1<<12);
		delay(1e-5);
		
		SPI1->CR1 = (1<<12);
		SPI1->CFG1 = (0b100<<28) | (1<<15) | (1<<14) | (7<<0);
		SPI1->CFG2 = (1<<31) | (1<<30) | (1<<26) | (1<<25) | (1<<24) | (1<<22);
		SPI1->IFCR = (1<<9);
		SPI1->CR1 = (1<<12);
	/*SPI1��ʼ��*/
	
	/*DMA��ʼ��*/
		//��DMA1ʱ��
		RCC->AHB1ENR |= (1<<0);
		delay(1e-5);
		
		//DMA1_Stream0 SPI1 TX
		DMA1_Stream0->PAR = (uint32_t)&SPI1->TXDR;
		DMA1_Stream0->NDTR = 2;
		DMAMUX1_Channel0->CCR = (38<<0);
		DMA1_Stream0->CR = (3<<16) | (0<<13) | (0<<10) | (0<<9) | (0b01<<6);
		DMA1_Stream0->FCR = (1<<2) | (3<<0);
		
		//DMA1_Stream1 SPI1 RX
		DMA1_Stream1->PAR = (uint32_t)&SPI1->RXDR;
		DMA1_Stream1->NDTR = 2;
		DMAMUX1_Channel1->CCR = (37<<0);
		DMA1_Stream1->CR = (3<<16) | (0<<13) | (1<<10) | (0<<9) | (0b00<<6) | (1<<4);
		DMA1_Stream1->FCR = (1<<2) | (3<<0);
		NVIC_SetPriority( DMA1_Stream1_IRQn , 3 );
	/*DMA��ʼ��*/
	
	os_delay(0.2);
	
	/*BMI088��ʼ��*/
		/*���ٶȼƳ�ʼ��*/
			//�����ٶȼƽӿ�����ΪSPI
			TxBuf[0] = (1<<7) | 0;
			PullDown_BMI088Accel_CS;
			SPI1_TransmitReceive( TxBuf , RxBuf , 3 );
			PullUp_BMI088Accel_CS;
			os_delay(0.1);
			
			//����Chip ID
			TxBuf[0] = (1<<7) | 0;
			PullDown_BMI088Accel_CS;
			SPI1_TransmitReceive( TxBuf , RxBuf , 3 );
			PullUp_BMI088Accel_CS;
			if( RxBuf[2] != 0x1e )
				while(1);
			os_delay(0.01);
			
			//��λ		
			TxBuf[0] = (0<<7) | 0x7e;
			TxBuf[1] = 0xb6;
			PullDown_BMI088Accel_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Accel_CS;
			os_delay(0.1);
			
			//�����ٶȼƽӿ�����ΪSPI
			TxBuf[0] = (1<<7) | 0;
			PullDown_BMI088Accel_CS;
			SPI1_TransmitReceive( TxBuf , RxBuf , 3 );
			PullUp_BMI088Accel_CS;
			os_delay(0.1);
			
			//�򿪼��ٶȼ�
			TxBuf[0] = (0<<7) | 0x7d;
			TxBuf[1] = 0x04;
			PullDown_BMI088Accel_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Accel_CS;
			os_delay(0.1);
			
			//���ٶȼ�ODR 800hz
			TxBuf[0] = (0<<7) | 0x40;
			TxBuf[1] = (1<<7) | (0<<4) | (0xC<<0);
			PullDown_BMI088Accel_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Accel_CS;
			os_delay(0.01);
			
			//���ٶȼ�����24g
			TxBuf[0] = (0<<7) | 0x41;
			TxBuf[1] = 0x03;
			PullDown_BMI088Accel_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Accel_CS;
			os_delay(0.01);
			
			//����Activeģʽ
			TxBuf[0] = (0<<7) | 0x7c;
			TxBuf[1] = 0x00;
			PullDown_BMI088Accel_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Accel_CS;
			os_delay(0.1);
			
			set_IMU_Accelerometer_UpdateFreq( 0 , 800 );
			IMUAccelerometerRegister( 0, "BMI088", GravityAcc / 1365.0, 0.1 );
		/*���ٶȼƳ�ʼ��*/
		
		/*���ݳ�ʼ��*/
			//����Chip ID
			TxBuf[0] = (1<<7) | 0;
			PullDown_BMI088Gyro_CS;
			SPI1_TransmitReceive( TxBuf , RxBuf , 2 );
			PullUp_BMI088Gyro_CS;
			if( RxBuf[1] != 0x0f )
				while(1);
			os_delay(0.01);
			
			//��λ		
			TxBuf[0] = (0<<7) | 0x14;
			TxBuf[1] = 0xb6;
			PullDown_BMI088Gyro_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Gyro_CS;
			os_delay(0.1);
			
			//GYRO_LPM1(0x11): normal mode
			TxBuf[0] = (0<<7) | 0x11;
			TxBuf[1] = 0x00;	
			PullDown_BMI088Gyro_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Gyro_CS;
			os_delay(0.1);
			
			//GYRO_RANGE(0x0f): 2000deg/s
			TxBuf[0] = (0<<7) | 0x0f;
			TxBuf[1] = 0x00;	
			PullDown_BMI088Gyro_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Gyro_CS;
			os_delay(0.01);
			
			//GYRO_BANDWIDTH(0x10): ODR=2000hz bandwidth=230hz
			TxBuf[0] = (0<<7) | 0x10;
			TxBuf[1] = 0x01;	
			PullDown_BMI088Gyro_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_BMI088Gyro_CS;
			os_delay(0.01);
		
			IMUGyroscopeRegister( 0, "BMI088", 0.00106526443603169529841533860381, 0.1 );
			set_IMU_Gyroscope_UpdateFreq( 0 , 2000 );
		/*���ݳ�ʼ��*/
	/*BMI088��ʼ��*/

	/*AK8975��ʼ��*/
		//����Chip ID
		TxBuf[0] = (1<<7) | 0;
		PullDown_AK8975_CS;
		SPI1_TransmitReceive( TxBuf , RxBuf , 3 );
		PullUp_AK8975_CS;
		if( RxBuf[1] != 0x48 )
			while(1);
		os_delay(0.01);
		
		//�ر�IIC�ӿ�
		TxBuf[0] = (0<<7) | 0x0f;
		TxBuf[1] = 0b00011011;	
		PullDown_AK8975_CS;
		SPI1_Transmit( TxBuf , 2 );
		PullUp_AK8975_CS;
		os_delay(0.01);
		
		/*��AK8975У׼����*/
			//����Fuseģʽ
			TxBuf[0] = (0<<7) | 0x0a;
			TxBuf[1] = 0b1111;	
			PullDown_AK8975_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_AK8975_CS;
			os_delay(0.01);
			
			//��У׼����
			TxBuf[0] = (1<<7) | 0x10;
			PullDown_AK8975_CS;
			SPI1_TransmitReceive( TxBuf , RxBuf , 4 );
			PullUp_AK8975_CS;
			AK8974_ASA.x = (RxBuf[1] - 128)*0.5/128 + 1;
			AK8974_ASA.y = (RxBuf[2] - 128)*0.5/128 + 1;
			AK8974_ASA.z = (RxBuf[3] - 128)*0.5/128 + 1;
			os_delay(0.01);
			
			//����Powerdownģʽ
			TxBuf[0] = (0<<7) | 0x0a;
			TxBuf[1] = 0b0000;	
			PullDown_AK8975_CS;
			SPI1_Transmit( TxBuf , 2 );
			PullUp_AK8975_CS;
			os_delay(0.01);
			
			IMUMagnetometerRegister( Internal_Magnetometer_Index, "AK8975", 0.003, 0.1 );
		/*��AK8975У׼����*/
	/*AK8975��ʼ��*/

	/*SPL06��ʼ��*/
		//��λ
		TxBuf[0] = (0<<7) | 0x0c;
		TxBuf[1] = (1<<7) | (0b1001);
		PullDown_SPL06_CS;
		SPI1_Transmit( TxBuf , 2 );
		PullUp_SPL06_CS;
		os_delay(0.1);

		//���ô�����
		spl06_pressure_rateset( 64 , 32 );//pressure 64 samples per sec , 32 times over sampling		
		spl06_temperature_rateset( 128 , 2 );//temperature 128 samples per sec , 2 times over sampling
		TxBuf[0] = (0<<7) | 0x08;
		TxBuf[1] = 0b111;
		PullDown_SPL06_CS;
		SPI1_Transmit( TxBuf , 2 );
		PullUp_SPL06_CS;
		os_delay(0.1);
		
		TxBuf[0] = (1<<7) | 0x10;
		PullDown_SPL06_CS;
		SPI1_TransmitReceive( TxBuf , RxBuf , 19 );
		PullUp_SPL06_CS;
		SPL06_coefficients.c0 = ( RxBuf[1] << 4 ) | ( RxBuf[2] >> 4 );
		SPL06_coefficients.c0 = ( SPL06_coefficients.c0 & 0x0800 ) ? (0xF000|SPL06_coefficients.c0) : SPL06_coefficients.c0;
		SPL06_coefficients.c1 = ( (RxBuf[2] & 0xf) << 8 ) | ( RxBuf[3] );
		SPL06_coefficients.c1 = ( SPL06_coefficients.c1 & 0x0800 ) ? (0xF000|SPL06_coefficients.c1) : SPL06_coefficients.c1;
		SPL06_coefficients.c00 = ( RxBuf[4] << 12 ) | ( RxBuf[5] << 4 ) | ( RxBuf[6] >> 4 );
		SPL06_coefficients.c00 = ( SPL06_coefficients.c00 & 0x080000 ) ? (0xFFF00000|SPL06_coefficients.c00) : SPL06_coefficients.c00;
		SPL06_coefficients.c10 = ( (RxBuf[6] & 0xf) << 16 ) | ( RxBuf[7] << 8 ) | ( RxBuf[8] >> 0 );
		SPL06_coefficients.c10 = ( SPL06_coefficients.c10 & 0x080000 ) ? (0xFFF00000|SPL06_coefficients.c10) : SPL06_coefficients.c10;
		SPL06_coefficients.c01 = ( RxBuf[9] << 8 ) | ( RxBuf[10] << 0 );
		SPL06_coefficients.c11 = ( RxBuf[11] << 8 ) | ( RxBuf[12] << 0 );
		SPL06_coefficients.c20 = ( RxBuf[13] << 8 ) | ( RxBuf[14] << 0 );
		SPL06_coefficients.c21 = ( RxBuf[15] << 8 ) | ( RxBuf[16] << 0 );
		SPL06_coefficients.c30 = ( RxBuf[17] << 8 ) | ( RxBuf[18] << 0 );
		//ע�ᴫ����
		PositionSensorRegister( internal_baro_sensor_index , \
														Position_Sensor_Type_GlobalPositioning , \
														Position_Sensor_DataType_s_z , \
														Position_Sensor_frame_ENU , \
														0.05 , //��ʱ
														0 ,	//xy���ζ�
														50 //z���ζ�
														);
	/*SPL06��ʼ��*/
		
	/*��TIM16 17������������ʱ��*/
		RCC->APB2ENR |= (1<<18) | (1<<17);
		os_delay(1e-2);
		//��Ƶ��10Mhz
		TIM16->PSC = TIM17->PSC = ( APB2TIMERCLK / 10000000 ) - 1;
		//TIM16 BMI088����2000hz�ж�
		TIM16->ARR = 10e6 / 2000 - 1;
		//TIM17 BMI088���ٶ�800hz�ж�
		TIM17->ARR = 10e6 / 800 - 1;
		//�򿪶�ʱ���ж�
		TIM16->DIER = 1<<0;
		TIM17->DIER = 1<<0;
		//��DMA�ж�
		DMARxStream_Enable_IRQ;
		//�򿪶�ʱ��
		TIM16->CR1 = 1<<0;
		TIM17->CR1 = 1<<0;
		//������ʱ�жϣ���ʱ����DMA�ж����ȼ�������ͬ��
		NVIC_SetPriority( TIM16_IRQn , 3 );
		NVIC_EnableIRQ( TIM16_IRQn );
		NVIC_SetPriority( TIM17_IRQn , 3 );
		NVIC_EnableIRQ( TIM17_IRQn );
	/*��TIM16 17������������ʱ��*/
}
extern float debug_test[30];
//BMI088��������ж�
extern "C" void TIM16_IRQHandler()
{
	TIM16->SR = 0;
	
	//�ж��Ƿ��AK8975
	static uint8_t AK8975_counter = 0;
	if( ++AK8975_counter >= 20 )
	{
		AK8975_counter = 0;
		nd_read_AK8975 = true;
	}
	
	//�ж��Ƿ��SPL06
	static uint8_t SPL06_counter = 0;
	if( ++SPL06_counter >= 66 )
	{
		SPL06_counter = 0;
		nd_read_SPL06 = true;
	}
	
	if( current_read_sensor == -1 )
	{
		//SPI���߿���
		//���ж�����
		PullDown_BMI088Gyro_CS;
		current_read_sensor = 0;
		TxBuf[0] = (1<<7) | 0x02;	
		SPI1_TransmitReceive_Start( TxBuf , RxBuf , sizeof(__BMI088_GyroData) );
	}
	else
		//SPI�������ڶ�
		nd_read_BMI088_Gyro = true;
	
	/*�����¶�*/
		//�¶��˲�
		static TD4_Lite temp_filter;
		double h = IMU_Gyroscope_UpdateT[0];
		temp_filter.track4( IMU_Temperature[1], h, 7,7,7,7 );
		if( target_IMUTemp > 25 )
		{
			
			//�¶����
			float temperature_error = target_IMUTemp - temp_filter.x1;
			float target_TempRate = constrain( temperature_error*0.1f, 0.5f );
			//�¶��ٶ����
			float TRateErr = target_TempRate - temp_filter.get_x2();
			static double TC_I = 0.0;
			if( IMU_Temperature[0]>target_IMUTemp+6 || IMU_Temperature[1]>target_IMUTemp+6 )
			{	//�д������¶ȹ���ֹͣ����
				set_TC_Out(0);
			}
			else
			{
				TC_I += 20.0*h* constrain( TRateErr, 0.5f );
				TC_I = constrain( TC_I, 0.0, 100.0 );
				set_TC_Out( constrain( 300.0*TRateErr + TC_I, 0.0, 100.0 ) );
			}			
		}
		else
			set_TC_Out(0);
	/*�����¶�*/
}
//BMI088���ٶ�����ж�
extern "C" void TIM17_IRQHandler()
{
	TIM17->SR = 0;

	if( current_read_sensor == -1 )
	{
		//SPI���߿���
		//���ж�����
		PullDown_BMI088Accel_CS;
		current_read_sensor = 1;
		TxBuf[0] = (1<<7) | 0x12;
		SPI1_TransmitReceive_Start( TxBuf , RxBuf , sizeof(__BMI088_AccelData) );
	}
	else
		//SPI�������ڶ�
		nd_read_BMI088_Accel = true;
}


static __BMI088_GyroData BMI088_GyroDataBuffer;
static void BMI088GyroTCB( void *pvParameter1, uint32_t ulParameter2 )
{
	uint8_t initial_counter;
	vector3<int32_t> data;
	volatile __BMI088_GyroData* datap = &BMI088_GyroDataBuffer;
	
	DMARxStream_Disable_IRQ;
	data.set_vector( datap->gyro[0] , datap->gyro[1] , datap->gyro[2] );
	DMARxStream_Enable_IRQ;
	
	//����Ƿ�����		
	bool data_error = false;
	if( data.x == std::numeric_limits<int16_t>::min() || data.y == std::numeric_limits<int16_t>::min() || data.z == std::numeric_limits<int16_t>::min() || \
			data.x == std::numeric_limits<int16_t>::max() || data.y == std::numeric_limits<int16_t>::max() || data.z == std::numeric_limits<int16_t>::max() )
		data_error = true;
	//���´���������
	IMUGyroscopeUpdate( 0, data, data_error, 0.01 );		
	MS_Notify_IMUGyroUpdate(0);
}

static __BMI088_AccelData BMI088_AccelDataBuffer;
static void BMI088AccelTCB( void *pvParameter1, uint32_t ulParameter2 )
{	
	uint8_t initial_counter;
	vector3<int32_t> data;
	int16_t temperature;
	volatile __BMI088_AccelData* datap = &BMI088_AccelDataBuffer;
	
	DMARxStream_Disable_IRQ;
	data.set_vector( datap->acc[0] , datap->acc[1] , datap->acc[2] );
	temperature = (((uint8_t*)&datap->temperature)[0] << 3) | (((uint8_t*)&datap->temperature)[1] >> 5);
	DMARxStream_Enable_IRQ;
	
	//�����¶�	
	if( temperature > 1023 )
		temperature -= 2048;
	IMU_Temperature[0] = temperature*0.125 + 23;
	//����Ƿ�����
	bool data_error = false;
	if( data.x == std::numeric_limits<int16_t>::min() || data.y == std::numeric_limits<int16_t>::min() || data.z == std::numeric_limits<int16_t>::min() || \
			data.x == std::numeric_limits<int16_t>::max() || data.y == std::numeric_limits<int16_t>::max() || data.z == std::numeric_limits<int16_t>::max() )
		data_error = true;
	//���´���������
	IMUAccelerometerUpdate( 0, data, data_error, 0.01 );
	MS_Notify_IMUAceelUpdate(0);
}

static __AK8975_Data AK8975_DataBuffer;
static void AK8975TCB( void *pvParameter1, uint32_t ulParameter2 )
{
	uint8_t initial_counter;
	vector3<int32_t> data;
	volatile __AK8975_Data* datap = &AK8975_DataBuffer;
	
	DMARxStream_Disable_IRQ;
	data.set_vector( datap->mag[1] , datap->mag[0] , -datap->mag[2] );
	DMARxStream_Enable_IRQ;
	
	//���´���������
	IMUMagnetometerUpdate( Internal_Magnetometer_Index, data, false, 0.01 );
}

static __SPL06_Data SPL06_DataBuffer;
static void SPL06TCB( void *pvParameter1, uint32_t ulParameter2 )
{
	uint8_t initial_counter;
	int32_t buf32[2];
	volatile __SPL06_Data* datap = &SPL06_DataBuffer;
	
	DMARxStream_Disable_IRQ;
	buf32[0] = __REV( datap->pressure ) >> 8;		
	buf32[1] = __REV( datap->temperature ) >> 8;		
	DMARxStream_Enable_IRQ;
	
	buf32[0] = ( buf32[0] & 0x800000 ) ? (0xFF000000|buf32[0]) : buf32[0];
	buf32[1] = ( buf32[1] & 0x800000 ) ? (0xFF000000|buf32[1]) : buf32[1];
	
	double fPsc = buf32[0] * SPL06_coefficients.KP;
	double fTsc = buf32[1] * SPL06_coefficients.KT;
	double qua2 = SPL06_coefficients.c10 + fPsc * (SPL06_coefficients.c20 + fPsc* SPL06_coefficients.c30);
	double qua3 = fTsc * fPsc * (SPL06_coefficients.c11 + fPsc * SPL06_coefficients.c21);
	
	double pressure = SPL06_coefficients.c00 + fPsc * qua2 + fTsc * SPL06_coefficients.c01 + qua3;
	double temperature = SPL06_coefficients.c0*0.5 + SPL06_coefficients.c1*fTsc;
	IMU_Temperature[1] = temperature;
	//���´���������
	if( pressure > 0 )
	{
		vector3<double> position;
		position.z = 1.07 * 4430000 * ( 1.0 - pow( pressure / 101325.0 , 1.0 / 5.256 ) );
		PositionSensorUpdatePosition( internal_baro_sensor_index , position , true , -1 );
	}
}

extern "C" void DMA1_Stream1_IRQHandler()
{
	DMA1->LIFCR = (1<<11) | (1<<10)  | (1<<9)  | (1<<8)  | (1<<6);
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//��������
	switch( current_read_sensor )
	{
		case 0:	//BMI088 Gyro
		{
			PullUp_BMI088Gyro_CS;
			BMI088_GyroDataBuffer = *(__BMI088_GyroData*)RxBuf;
			xTimerPendFunctionCallFromISR( BMI088GyroTCB, 0, 0, &xHigherPriorityTaskWoken );
			break;
		}
		case 1:	//BMI088 Accel	
		{
			PullUp_BMI088Accel_CS;
			BMI088_AccelDataBuffer = *(__BMI088_AccelData*)RxBuf;
			xTimerPendFunctionCallFromISR( BMI088AccelTCB, 0, 0, &xHigherPriorityTaskWoken );				
			break;
		}
		
		case 8:	//AK8975������
		{
			PullUp_AK8975_CS;
			AK8975_DataBuffer = *(__AK8975_Data* )RxBuf;				
			xTimerPendFunctionCallFromISR( AK8975TCB, 0, 0, &xHigherPriorityTaskWoken );				
			
			//���Ͳ���ָ��
			TxBuf[0] = (0<<7) | 0x0a;
			TxBuf[1] = 0b0001;	
			PullDown_AK8975_CS;
			current_read_sensor = 9;
			SPI1_Transmit_Start( TxBuf , 2 );
			goto exit_tcirq;
			break;
		}
		case 9:	//AK8975���Ͳ�������
		{
			PullUp_AK8975_CS;
			break;
		}
		
		case 16:	//SPL06������
		{
			PullUp_SPL06_CS;
			SPL06_DataBuffer = *(__SPL06_Data*)RxBuf;
			xTimerPendFunctionCallFromISR( SPL06TCB, 0, 0, &xHigherPriorityTaskWoken );
			break;
		}
	}
	
	//��ʣ�µĴ�����
	if( nd_read_BMI088_Gyro )
	{
		PullDown_BMI088Gyro_CS;
		nd_read_BMI088_Gyro = false;
		current_read_sensor = 0;
		TxBuf[0] = (1<<7) | 0x02;	
		SPI1_TransmitReceive_Start( TxBuf , RxBuf , sizeof(__BMI088_GyroData) );
	}
	else if( nd_read_BMI088_Accel )
	{
		PullDown_BMI088Accel_CS;
		nd_read_BMI088_Accel = false;
		current_read_sensor = 1;
		TxBuf[0] = (1<<7) | 0x12;
		SPI1_TransmitReceive_Start( TxBuf , RxBuf , sizeof(__BMI088_AccelData) );
	}
	else if( nd_read_AK8975 )
	{
		PullDown_AK8975_CS;
		nd_read_AK8975 = false;
		current_read_sensor = 8;
		TxBuf[0] = (1<<7) | 0x03;
		SPI1_TransmitReceive_Start( TxBuf , RxBuf , sizeof(__AK8975_Data) );
	}
	else if( nd_read_SPL06 )
	{
		PullDown_SPL06_CS;
		nd_read_SPL06 = false;
		current_read_sensor = 16;
		TxBuf[0] = (1<<7) | 0x00;
		SPI1_TransmitReceive_Start( TxBuf , RxBuf , sizeof(__SPL06_Data) );
	}
	else
	{
		current_read_sensor = -1;
	}
exit_tcirq:
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*SPI����*/
	static void SPI1_Transmit_Start( const uint8_t* TxBuf , uint16_t size )
	{
		//�ر�DMAͨ��
		DMA1_Stream0->CR &= ~(1<<0);
		DMA1_Stream1->CR &= ~(1<<0);
		
		//��Tx MINC���ر�Rx MINC
		DMATxStream_Enable_MINC;
		DMARxStream_Disable_MINC;
		
		//���D-Cache
		#ifdef DCACHE_SIZE
			SCB_CleanDCache_by_Addr((uint32_t*)TxBuf, size);
		#endif
		
		//�ر�SPI����
		SPI1->CR1 &= ~(1<<0);
		//�ر�DMA Request
		SPI1->CFG1 &= ~( (1<<15) | (1<<14) );
		//���DMA״̬
		DMA1->LIFCR = (1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0);
		DMA1->LIFCR = (1<<11) | (1<<10)  | (1<<9)  | (1<<8)  | (1<<6);
		//����DMA�洢����ַ
		DMA1_Stream0->M0AR = (uint32_t)TxBuf;
		DMA1_Stream1->M0AR = (uint32_t)RxBuf;
		//���SPI״̬
		SPI1->IFCR = (1<<9) | (1<<4);
		//����DMA��������
		DMA1_Stream0->NDTR = DMA1_Stream1->NDTR = SPI1->CR2 = size;
		//ʹ��Rx DMA
		DMA1_Stream1->CR |= (1<<0);
		//��Rx DMA Request
		SPI1->CFG1 |= (1<<14);
		//ʹ��Tx DMA
		DMA1_Stream0->CR |= (1<<0);
		//��Tx DMA Request
		SPI1->CFG1 |= (1<<15);
		//��SPI���迪ʼ����
		SPI1->CR1 |= (1<<0);
		SPI1->CR1 |= (1<<9);
	}
	static void SPI1_Transmit( const uint8_t* TxBuf , uint16_t size )
	{
		SPI1_Transmit_Start( TxBuf , size );
		//�ȴ�DMA�������
		while( ( DMA1->LISR & (1<<11) ) == 0 );
	}

	static void SPI1_TransmitReceive_Start( const uint8_t* TxBuf , uint8_t* RxBuf , uint16_t size )
	{
		//�ر�DMAͨ��
		DMA1_Stream0->CR &= ~(1<<0);
		DMA1_Stream1->CR &= ~(1<<0);
		
		//�ر�Tx MINC����Rx MINC
		DMATxStream_Disable_MINC;
		DMARxStream_Enable_MINC;
		
		//���D-Cache
		#ifdef DCACHE_SIZE		
			SCB_CleanDCache_by_Addr((uint32_t*)TxBuf, 1);
			SCB_InvalidateDCache_by_Addr((uint32_t*)RxBuf, size);
		#endif
		
		//�ر�SPI����
		SPI1->CR1 &= ~(1<<0);
		//�ر�DMA Request
		SPI1->CFG1 &= ~( (1<<15) | (1<<14) );
		//���DMA״̬
		DMA1->LIFCR = (1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0);
		DMA1->LIFCR = (1<<11) | (1<<10)  | (1<<9)  | (1<<8)  | (1<<6);
		//����DMA�洢����ַ
		DMA1_Stream0->M0AR = (uint32_t)TxBuf;
		DMA1_Stream1->M0AR = (uint32_t)RxBuf;
		//���SPI״̬
		SPI1->IFCR = (1<<9) | (1<<4);
		//����DMA��������
		DMA1_Stream0->NDTR = DMA1_Stream1->NDTR = SPI1->CR2 = size;
		//ʹ��Rx DMA
		DMA1_Stream1->CR |= (1<<0);
		//��Rx DMA Request
		SPI1->CFG1 |= (1<<14);
		//ʹ��Tx DMA
		DMA1_Stream0->CR |= (1<<0);
		//��Tx DMA Request
		SPI1->CFG1 |= (1<<15);
		
		//��SPI���迪ʼ����
		SPI1->CR1 |= (1<<0);
		SPI1->CR1 |= (1<<9);
	}
	static void SPI1_TransmitReceive( const uint8_t* TxBuf , uint8_t* RxBuf , uint16_t size )
	{	
		SPI1_TransmitReceive_Start( TxBuf , RxBuf , size );
		//�ȴ�DMA�������
		while( ( DMA1->LISR & (1<<11) ) == 0 ){}		
	}
/*SPI����*/
