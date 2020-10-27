#pragma once

#include "AC_Math.hpp"

/*二阶ButterWorth带阻*/
	class Filter_Butter2_BS
	{
		private:
			bool available;	
		
			double k_1[4];
		
			double in_1[2];
			double out_1[2];
		public:
			//滤波器可用状态
			inline bool is_available()
			{
				return this->available;
			}
			inline void set_inavailable()
			{
				this->available = false;
			}
			
			//设置滤波频率
			inline bool set_cutoff_frequency( double sample_freq , double notch_freq, double bandwidth )
			{
				if( sample_freq>2.2*notch_freq && sample_freq>2.2*bandwidth )
				{
					this->available = true;
					
					double alpha = tan(Pi * bandwidth / sample_freq);
					double beta = -cos(2.0 * Pi * notch_freq / sample_freq);
					double a0_inv = 1.0 / (alpha + 1.0);
						
					this->k_1[0] = a0_inv;
					this->k_1[1] = 2.0 * beta * a0_inv;
					this->k_1[2] = (1.0 - alpha) * a0_inv;
					this->k_1[3] = 2.0 * beta;
				}
				else
					this->available =false;
				
				return this->available;
			}
			inline void set_cutoff_frequency_from( const Filter_Butter2_BS& filter )
			{
				this->k_1[0] = filter.k_1[0];	this->k_1[1] = filter.k_1[1];	this->k_1[2] = filter.k_1[2];	this->k_1[3] = filter.k_1[3];
				this->available = filter.available;		
			}
			
			//设置初始值
			inline void reset( double initial_value )
			{
				this->in_1[0] = this->in_1[1] = initial_value;
				this->out_1[0] = this->out_1[1] = initial_value;
			}
			
			//构造函数
			Filter_Butter2_BS()
			{
				this->available = false;
				this->reset(0);
			}
			Filter_Butter2_BS( double sample_freq , double notch_freq, double bandwidth )
			{
				set_cutoff_frequency( sample_freq, notch_freq, bandwidth );
				this->reset(0);
			}
			
			//滤波
			inline double get_result(){return this->out_1[0];}
			inline double run( double newdata )
			{
				if( this->available )
				{
					double out_1_2 = this->out_1[1];	this->out_1[1] = this->out_1[0];
					this->out_1[0] = this->k_1[0]*( newdata + this->k_1[3]*this->in_1[0] + this->in_1[1] ) - this->k_1[1]*this->out_1[1] - this->k_1[2]*out_1_2;			
					this->in_1[1] = this->in_1[0];	this->in_1[0] = newdata;
				}
				else
					reset( newdata );
				return this->out_1[0];
			}
			
			//添加偏移
			inline double add_offset( double offset )
			{
				this->in_1[0] += offset;	this->in_1[1] += offset;
				this->out_1[0] += offset;	this->out_1[1] += offset;				
				return this->out_1[0];
			}
	};
/*二阶ButterWorth带阻*/
