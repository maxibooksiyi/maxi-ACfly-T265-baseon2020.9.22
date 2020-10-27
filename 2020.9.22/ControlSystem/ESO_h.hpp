#pragma once

//高度无参ESO
//朱文杰 20180102
//请勿用于商业用途
//！！抄袭必究！！

#include <stdbool.h>
#include "AC_Math.hpp"
#include "Filters_LP.hpp"

class ESO_h
{
	private:
		double beta;
		
		double Hz;
		double h;
		
		double T;
		double invT;
		double z_inertia;
		double z1;
		double z2;
		double u;
		Filter_Butter2_LP zin_filter;
		Filter_Butter2_LP gAcc_filter;
	
		//输出->加速度增益
		double b;		
		//当前输出力（加速度）
		double force;

		bool err_sign;
		double err_continues_time;
	
	public:		
		inline void init( double T, double beta, double Hz )
		{			
			this->T = T;
			this->invT = 1.0f / T;
			this->beta = beta;
			this->z1 = this->u = 0;
			this->z2 =1.0f;
			
			this->Hz = Hz;
			this->h = 1.0 / Hz;
			zin_filter.set_cutoff_frequency( Hz, beta );
			gAcc_filter.set_cutoff_frequency( Hz, beta );
		}
		ESO_h()
		{
			
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->u - this->z_inertia );
			this->force = this->b * this->z_inertia;
		}
		
		inline double get_u(){ return this->u; }
		inline double get_T(){ return this->T; }
		inline double get_b(){ return this->b; }
		inline double get_force(){ return this->force; }
		inline double get_hover_throttle(){ return this->z2; }
		
		double run( double acc );
};

