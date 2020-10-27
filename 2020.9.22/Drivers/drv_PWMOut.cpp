#include "drv_PwmOut.hpp"
#include "Basic.hpp"
#include "stm32h7xx.h"
#include "TimeBase.hpp"

//舵机通道频率
#define AUX_FREQ 50
//主电机个数
static uint8_t MainMotorCount = 8;
//获取Aux通道个数
uint8_t get_AuxChannelCount()
{
	return 8 - MainMotorCount;
}

void set_MainMotorCount( uint8_t count )
{
	//电机数必须是偶数
	if( count & 1 )
		++count;
	if( MainMotorCount != count )
	{
		MainMotorCount = count;
		
		if( count >= 2 )
			TIM4->ARR = 1e6 / 400;
		else
			TIM4->ARR = 1e6 / AUX_FREQ;
		
		if( count >= 4 )
			TIM8->ARR = 1e6 / 400;
		else
			TIM8->ARR = 1e6 / AUX_FREQ;
		
		if( count >= 6 )
			TIM3->ARR = 1e6 / 400;
		else
			TIM3->ARR = 1e6 / AUX_FREQ;
		
		if( count >= 8 )
			TIM15->ARR = 1e6 / 400;
		else
			TIM15->ARR = 1e6 / AUX_FREQ;
	}
}

void MainMotor_PWM_Out( double out[8] )
{
	if( MainMotorCount >= 1 )
		TIM4->CCR3 = out[0]*10 + 1000;
	if( MainMotorCount >= 2 )
		TIM4->CCR4 = out[1]*10 + 1000;
	if( MainMotorCount >= 3 )
		TIM8->CCR1 = out[2]*10 + 1000;
	if( MainMotorCount >= 4 )
		TIM8->CCR2 = out[3]*10 + 1000;
	if( MainMotorCount >= 5 )
		TIM3->CCR1 = out[4]*10 + 1000;
	if( MainMotorCount >= 6 )
		TIM3->CCR2 = out[5]*10 + 1000;
	if( MainMotorCount >= 7 )
		TIM15->CCR1 = out[6]*10 + 1000;
	if( MainMotorCount >= 8 )
		TIM15->CCR2 = out[7]*10 + 1000;
}

void Aux_PWM_Out( double out, uint8_t ind )
{
	if( ind==7 && MainMotorCount+8<=8 )
		TIM4->CCR3 = out*10 + 1000;
	else if( ind==6 && MainMotorCount+7<=8 )
		TIM4->CCR4 = out*10 + 1000;
	else if( ind==5 && MainMotorCount+6<=8 )
		TIM8->CCR1 = out*10 + 1000;
	else if( ind==4 && MainMotorCount+5<=8 )
		TIM8->CCR2 = out*10 + 1000;
	else if( ind==3 && MainMotorCount+4<=8 )
		TIM3->CCR1 = out*10 + 1000;
	else if( ind==2 && MainMotorCount+3<=8 )
		TIM3->CCR2 = out*10 + 1000;
	else if( ind==1 && MainMotorCount+2<=8 )
		TIM15->CCR1 = out*10 + 1000;
	else if( ind==0 && MainMotorCount+1<=8 )
		TIM15->CCR2 = out*10 + 1000;
}

void PWM_DisableAll()
{
  TIM15->CCR2=0;
	TIM15->CCR1=0;
  TIM3->CCR2 =0;
  TIM3->CCR1 =0;
  TIM8->CCR2 =0;
	TIM8->CCR1 =0;
	TIM4->CCR4 =0;
	TIM4->CCR3 =0;
}
void PWM_PullDownAll()
{
  TIM15->CCR2=1000;
	TIM15->CCR1=1000;
  TIM3->CCR2 =1000;
  TIM3->CCR1 =1000;
  TIM8->CCR2 =1000;
	TIM8->CCR1 =1000;
	TIM4->CCR4 =1000;
	TIM4->CCR3 =1000;
}
void MainMotor_PullDownAll()
{
	if( MainMotorCount >= 1 )
		TIM4->CCR3 = 1000;
	if( MainMotorCount >= 2 )
		TIM4->CCR4 = 1000;
	if( MainMotorCount >= 3 )
		TIM8->CCR1 = 1000;
	if( MainMotorCount >= 4 )
		TIM8->CCR2 = 1000;
	if( MainMotorCount >= 5 )
		TIM3->CCR1 = 1000;
	if( MainMotorCount >= 6 )
		TIM3->CCR2 = 1000;
	if( MainMotorCount >= 7 )
		TIM15->CCR1 = 1000;
	if( MainMotorCount >= 8 )
		TIM15->CCR2 = 1000;
}
void PWM_PullUpAll()
{
	TIM15->CCR2=2000;
	TIM15->CCR1=2000;
  TIM3->CCR2 =2000;
  TIM3->CCR1 =2000;
  TIM8->CCR2 =2000;
	TIM8->CCR1 =2000;
	TIM4->CCR4 =2000;
	TIM4->CCR3 =2000;
}


/*
   PWM1(TIM15_CH2) -- PE6
	 PWM2(TIM15_CH1) -- PE5
	 PWM3( TIM3_CH2) -- PB5
	 PWM4( TIM3_CH1) -- PB4
   PWM5( TIM8_CH2) -- PC7
	 PWM6( TIM8_CH1) -- PC6
	 PWM7( TIM4_CH4) -- PD15
	 PWM8( TIM4_CH3) -- PD14
*/
void init_drv_PWMOut()
{
	/*配置GPIO*/  
	
	//使能GPIO时钟
	RCC->AHB4ENR |= (1<<4)|(1<<1)|(1<<2)|(1<<3);
	os_delay(0.01);
  //复用模式
	set_register( GPIOB->MODER, 0b10, 4*2, 2 );	//PB4
	set_register( GPIOB->MODER, 0b10, 5*2, 2 );	//PB5
	set_register( GPIOC->MODER, 0b10, 6*2, 2 );	//PC6
	set_register( GPIOC->MODER, 0b10, 7*2, 2 );	//PC7
	set_register( GPIOD->MODER, 0b10, 14*2, 2 );	//PD14
	set_register( GPIOD->MODER, 0b10, 15*2, 2 );	//PD15
	set_register( GPIOE->MODER, 0b10, 5*2, 2 );	//PE5
	set_register( GPIOE->MODER, 0b10, 6*2, 2 );	//PE6
	
	//开漏输出
	set_register( GPIOB->OTYPER, 0, 4, 1 );	//PB4
	set_register( GPIOB->OTYPER, 0, 5, 1 );	//PB5
	set_register( GPIOC->OTYPER, 0, 6, 1 );	//PC6
	set_register( GPIOC->OTYPER, 0, 7, 1 );	//PC7
	set_register( GPIOD->OTYPER, 0, 14, 1 );	//PD14
	set_register( GPIOD->OTYPER, 0, 15, 1 );	//PD15
	set_register( GPIOE->OTYPER, 0, 5, 1 );	//PE5
	set_register( GPIOE->OTYPER, 0, 6, 1 );	//PE6
	
	//上拉
	set_register( GPIOB->PUPDR, 0b01, 4*2, 2 );	//PB4
	set_register( GPIOB->PUPDR, 0b01, 5*2, 2 );	//PB5
	set_register( GPIOC->PUPDR, 0b01, 6*2, 2 );	//PC6
	set_register( GPIOC->PUPDR, 0b01, 7*2, 2 );	//PC7
	set_register( GPIOD->PUPDR, 0b01, 14*2, 2 );	//PD14
	set_register( GPIOD->PUPDR, 0b01, 15*2, 2 );	//PD15
	set_register( GPIOE->PUPDR, 0b01, 5*2, 2 );	//PE5
	set_register( GPIOE->PUPDR, 0b01, 6*2, 2 );	//PE6
	
	//速度
	set_register( GPIOB->OSPEEDR, 0b01, 4*2, 2 );	//PB4
	set_register( GPIOB->OSPEEDR, 0b01, 5*2, 2 );	//PB5
	set_register( GPIOC->OSPEEDR, 0b01, 6*2, 2 );	//PC6
	set_register( GPIOC->OSPEEDR, 0b01, 7*2, 2 );	//PC7
	set_register( GPIOD->OSPEEDR, 0b01, 14*2, 2 );	//PD14
	set_register( GPIOD->OSPEEDR, 0b01, 15*2, 2 );	//PD15
	set_register( GPIOE->OSPEEDR, 0b01, 5*2, 2 );	//PE5
	set_register( GPIOE->OSPEEDR, 0b01, 6*2, 2 );	//PE6
	
	//复用功能配置
	set_register( GPIOB->AFR[0], 2, 4*4, 4 );	//PB4
	set_register( GPIOB->AFR[0], 2, 5*4, 4 );	//PB5
	set_register( GPIOC->AFR[0], 3, 6*4, 4 );	//PC6
	set_register( GPIOC->AFR[0], 3, 7*4, 4 );	//PC7
	set_register( GPIOD->AFR[1], 2, 14*4-32, 4 );	//PD14
	set_register( GPIOD->AFR[1], 2, 15*4-32, 4 );	//PD15
	set_register( GPIOE->AFR[0], 4, 5*4, 4 );	//PE5
	set_register( GPIOE->AFR[0], 4, 6*4, 4 );	//PE6
	
	
	/*配置TIM*/  	
  //使能TIM时钟
	RCC->APB1LENR|=(1<<2)|(1<<1);
  RCC->APB2ENR|=(1<<16)|(1<<1);
	os_delay(0.01);
	
	PWM_DisableAll();
	
	TIM3->PSC  =(APB1TIMERCLK / 1e6) - 1;
	TIM3->ARR  = 1e6 / 400;
  TIM3->CCMR1 = (0b110<<12)|(1<<11)|(0b110<<4)|(1<<3);
  TIM3->CCER =(1<<4)|(1<<0);
  TIM3->EGR = (1<<0);
  TIM3->CR1 = (1<<7)|(1<<0);

	TIM4->PSC = (APB1TIMERCLK / 1e6) - 1;
	TIM4->ARR = 1e6 / 400;
	TIM4->CCMR2 = (0b110<<12)|(1<<11)|(0b110<<4)|(1<<3);
	TIM4->CCER =(1<<12) | (1<<8);
	TIM4->EGR |=(1<<0);
	TIM4->CR1 = (1<<7)|(1<<0);
	

  TIM8->PSC = (APB2TIMERCLK / 1e6)-1;
  TIM8->ARR = 1e6 / 400;
  TIM8->CCMR1 = (0b110<<4)|(0b110<<12)|(1<<3)|(1<<11);                                       
	TIM8->CCER = (1<<4)|(1<<0);
	TIM8->BDTR = (1<<15);
  TIM8->EGR = (1<<0);                    
  TIM8->CR1 = (1<<7)|(1<<0);          

  TIM15->PSC=(APB2TIMERCLK / 1e6)-1;
  TIM15->ARR = 1e6 / 400;
  TIM15->CCMR1 = (0b110<<4)|(0b110<<12)|(1<<3)|(1<<11);                                       
	TIM15->CCER = (1<<4)|(1<<0);
	TIM15->BDTR = (1<<15);
  TIM15->EGR = (1<<0);                    
  TIM15->CR1 = (1<<7)|(1<<0);   

   
}