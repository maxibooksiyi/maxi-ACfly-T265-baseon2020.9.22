#pragma once

#include <limits>
#include "arm_math.h"
#include "math_common_tables.hpp"

/*
	������ѧ��
	20190909�����Ľ�
*/

/*
	��������
*/
	#define Pi 3.1415926535897932384626433832795
	#define Pi_f 3.1415926535897932384626433832795f
	
	#define GravityAcc 980.665	//�������ٶ�(cm/s^2)
	#define rEarth 637139300	//����뾶(cm)
/*��������*/

/*ENU��Bodyheading����ϵת��*/
	//ENU����ת��ΪBodyHeading��xΪ��ͷ���������ƽ�У���yΪ�����ͷ�󷽣������ƽ�У���zΪ�Ϸ���
	#define ENU2BodyHeading_x( enu_x , enu_y , Yaw_sin , Yaw_cos ) ( enu_x*Yaw_cos + enu_y*Yaw_sin )
	#define ENU2BodyHeading_y( enu_x , enu_y , Yaw_sin , Yaw_cos ) ( -enu_x*Yaw_sin + enu_y*Yaw_cos )
	//BoduHeadingת��ΪENU����
	#define BodyHeading2ENU_x( body_x , body_y , Yaw_sin , Yaw_cos ) ( body_x*Yaw_cos - body_y*Yaw_sin )
	#define BodyHeading2ENU_y( body_x , body_y , Yaw_sin , Yaw_cos ) ( body_x*Yaw_sin + body_y*Yaw_cos )
/*ENU��Bodyheading����ϵת��*/

/*�ж����������Ƿ����
	v_1:������1
	v_2:������2
*/
	inline bool is_equal(float v_1, float v_2)
	{
		return fabsf(v_1 - v_2) < std::numeric_limits<float>::epsilon();
	}
	inline bool is_equal(double v_1, double v_2)
	{
		return fabs(v_1 - v_2) < std::numeric_limits<double>::epsilon();
	}
/*�ж����������Ƿ����*/
	
/*�жϸ������Ƿ�Ϊ0
	fVall:������
*/
	inline bool is_zero(float fVal1) 
	{
		return fabsf(fVal1) < std::numeric_limits<float>::epsilon() ? true : false;
	}
	inline bool is_zero(double fVal1) 
	{
		return fabs(fVal1) < std::numeric_limits<double>::epsilon() ? true : false;
	}
/*�жϸ������Ƿ�Ϊ0*/

/*������ȡ��
	num��������
	divider������
*/
	inline double Mod( double number, double divider )
	{
		if( is_zero(divider) )
			return 0;
		if( divider < 0 )
			divider = -divider;
		double fraction = number / divider;
		fraction = fraction - (int32_t)fraction;
		return divider*fraction;
	}
	inline float Mod( float number, float divider )
	{
		if( is_zero(divider) )
			return 0;
		if( divider < 0 )
			divider = -divider;
		float fraction = number / divider;
		fraction = fraction - (int32_t)fraction;
		return divider*fraction;
	}
/*������ȡ��*/
	
/*�������Ǻ���*/
	inline double fast_sin(double theta)
	{
		double fract, tin;                             /* Temporary variables for input, output */
		uint16_t indexS, indexC;                         /* Index variable */
		double f1, f2, d1, d2;                        /* Two nearest output values */
		double findex, Dn, Df, temp;

		/* input x is in degrees */
		/* Scale the input, divide input by 360, for cosine add 0.25 (pi/2) to read sine table */
		tin = theta * 0.15915494309189533576888376337251;

		if (tin < 0.0f)
		{
			tin = -tin;
		}

		tin = tin - (uint32_t)tin;

		/* Calculation of index of the table */
		findex = (double)512 * tin;
		indexS = (uint16_t)(((uint16_t)findex) & 0x1ff);
		indexC = (uint16_t)((indexS + (512 / 4)) & 0x1ff);

		/* fractional value calculation */
		fract = findex - (double)indexS;

		/* Read two nearest values of input value from the cos & sin tables */
		f1 = sinTable_f64[indexS + 0];
		f2 = sinTable_f64[indexS + 1];
		d1 = sinTable_f64[indexC + 0];
		d2 = sinTable_f64[indexC + 1];

		Dn = 0.0122718463030; // delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE
		Df = f2 - f1; // delta between the values of the functions
		temp = Dn * (d1 + d2) - 2 * Df;
		temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
		temp = fract * temp + d1 * Dn;

		/* Calculation of sine value */
		if (theta < 0.0)
			return -(fract * temp + f1);
		else
			return fract * temp +f1;
	}
	inline double fast_cos(double theta)
	{
		double fract, tin;                             /* Temporary variables for input, output */
		uint16_t indexS, indexC;                         /* Index variable */
		double f1, f2, d1, d2;                        /* Two nearest output values */
		double findex, Dn, Df, temp;

		/* input x is in degrees */
		/* Scale the input, divide input by 360, for cosine add 0.25 (pi/2) to read sine table */
		tin = theta * 0.15915494309189533576888376337251;

		if (tin < 0.0f)
		{
			tin = -tin;
		}

		tin = tin - (uint32_t)tin;

		/* Calculation of index of the table */
		findex = (double)512 * tin;
		indexS = (uint16_t)(((uint16_t)findex) & 0x1ff);
		indexC = (uint16_t)((indexS + (512 / 4)) & 0x1ff);

		/* fractional value calculation */
		fract = findex - (double)indexS;

		/* Read two nearest values of input value from the cos & sin tables */
		f1 = sinTable_f64[indexC + 0];
		f2 = sinTable_f64[indexC + 1];
		d1 = -sinTable_f64[indexS + 0];
		d2 = -sinTable_f64[indexS + 1];

		//return (1.0 - fract) * f1 + fract * f2;

		Dn = 0.0122718463030; // delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE
		Df = f2 - f1;          // delta between the values of the functions

		temp = Dn * (d1 + d2) - 2 * Df;
		temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
		temp = fract * temp + d1 * Dn;

		/* Calculation of cosine value */
		return fract * temp + f1;
	}
	inline void fast_sin_cos( double theta, double* sin_value, double* cos_value )
	{
		double fract, tin;                             /* Temporary variables for input, output */
		uint16_t indexS, indexC;                         /* Index variable */
		double f1, f2, d1, d2;                        /* Two nearest output values */
		double findex, Dn, Df, temp;

		/* input x is in degrees */
		/* Scale the input, divide input by 360, for cosine add 0.25 (pi/2) to read sine table */
		tin = theta * 0.15915494309189533576888376337251;

		if (tin < 0.0f)
		{
			tin = -tin;
		}

		tin = tin - (uint32_t)tin;

		/* Calculation of index of the table */
		findex = (double)512 * tin;
		indexS = (uint16_t)(((uint16_t)findex) & 0x1ff);
		indexC = (uint16_t)((indexS + (512 / 4)) & 0x1ff);

		/* fractional value calculation */
		fract = findex - (double)indexS;

		/* Read two nearest values of input value from the cos & sin tables */
		f1 = sinTable_f64[indexC + 0];
		f2 = sinTable_f64[indexC + 1];
		d1 = -sinTable_f64[indexS + 0];
		d2 = -sinTable_f64[indexS + 1];

		//return (1.0 - fract) * f1 + fract * f2;

		Dn = 0.0122718463030; // delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE
		Df = f2 - f1;          // delta between the values of the functions

		temp = Dn * (d1 + d2) - 2 * Df;
		temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
		temp = fract * temp + d1 * Dn;

		/* Calculation of cosine value */
		*cos_value = fract * temp + f1;
		
		/* Read two nearest values of input value from the cos & sin tables */
		f1 = sinTable_f64[indexS + 0];
		f2 = sinTable_f64[indexS + 1];
		d1 = sinTable_f64[indexC + 0];
		d2 = sinTable_f64[indexC + 1];

		Df = f2 - f1; // delta between the values of the functions
		temp = Dn * (d1 + d2) - 2 * Df;
		temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
		temp = fract * temp + d1 * Dn;

		/* Calculation of sine value */
		if (theta < 0.0)
			*sin_value =  -(fract * temp + f1);
		else
			*sin_value =  fract * temp +f1;
	}
/*�������Ǻ���*/
	
/*ƽ������
	x:��
*/
	template<class T>inline T sq( T data ) 
	{ 
		return data*data;
	}
/*��ȫ��������*/
	
/*��ȫ��������
	x:������
*/
	inline float safe_sqrt(float x)
	{
		if( x < 0 )
			return 0;
		return sqrtf(x);
	}
	inline double safe_sqrt(double x)
	{
		if( x < 0 )
			return 0;
		return sqrt(x);
	}
/*��ȫ��������*/

/*����exp����*/
	inline double fast_expd( double x )
	{
		double d;
    *((int*)(&d) + 0) = 0;
    *((int*)(&d) + 1) = (int)(1512775 * x + 1072632447);
    
    double x_ind = x * 1.4426950408889634073599246810019;
    if( x_ind > 0 )
        x_ind -= (int)x_ind;
    else
        x_ind -= (int)x_ind - 1;
    x_ind *= 693;
    int x_ind_int = round( x_ind );
    d *= fast_expd_table[x_ind_int];
            
    return d;
	}
/*����exp����*/
	
/*�ж����Ƿ��ڷ�Χ��
	x:��
*/
	//�ж��Ƿ���min-max��Χ��
	#define in_range( x , min , max ) ( (x<=max) && (x>=min) )
	//�ж��Ƿ���-range��+range��Χ��
	#define in_symmetry_range( x , range ) ( ( x >= -range ) && ( x <= range ) )
	//�ж��Ƿ���mid-range��mid+range��Χ��
	#define in_symmetry_range_mid( x , mid , range ) ( ( x >= mid - range ) && ( x <= mid + range ) )
/*�ж����Ƿ��ڷ�Χ��*/
	
/*�Ƴ�����
	x:��Ҫ�Ƴ���������
*/
	//�Ƴ�-band��+band����
	template<class T>inline T remove_deadband( T data , T band ) 
	{ 
		if( data > band ) 
			return data-band;
		else if( data < -band ) 
			return data+band;	
		else 
			return 0;
	}
/*�Ƴ�����*/

/*�޷�
	x:��Ҫ���޷�����
*/
	//min-max��Χ�޷�
	template<class T> inline T constrain( const T x , const T min , const T max )
	{
		if( x < min )
			return min;
		else if( x > max )
			return max;
		else return x;
	}
	
	//-max��+max��Χ�޷�
	template<class T> inline T constrain( const T x , const T max )
	{
		if( x < -max )
			return -max;
		else if( x > max )
			return max;
		else return x;
	}
/*�޷�*/
	
/*���ȽǶȵ�λת��
	x:��Ҫ��ת������
*/
	inline float rad2degree( float x ) { return x*57.295779513f; }
	inline double rad2degree( double x ) { return x*57.295779513082320876798154814105; }
	inline float degree2rad( float x ) { return x*0.0174532925f; }
	inline double degree2rad( double x ) { return x*0.01745329251994329576923690768489; }
/*���ȽǶȵ�λת��*/

/*Ascii���ж�ת��
	x:Ascii��
*/
	//�Ƿ��д��ĸ
	#define is_capital_letter(x) ( x>=65 && x<=90 )
	//�Ƿ�Сд��ĸ
	#define is_lowercase_letter(x) ( x>=97 && x<=122 )
	//�Ƿ�����
	#define is_number(x) ( x>=48 && x<=57 )
	//Ascii����תʵ������
	#define Ascii2num(x) (x-48)
/*Ascii���ж�ת��*/
	
/*���ź���
	x:��
*/
	template<class T> inline T sign( T x )
	{
		if( x > 0 ) return 1;
		else if( x < 0 ) return -1;
		else return 0;
	}
/*���ź���*/
	
/*�����޷�
	x y z:����Ԫ��
	length:�޷�����
*/
	/*��ά����*/
		inline void constrain_vector( float& x , float& y , float length )
		{
			if( length <= 0.0f ) return;
			float vec_length = safe_sqrt( x*x + y*y );
			if( vec_length > length )
			{
				vec_length = length / vec_length;
				x *= vec_length;
				y *= vec_length;
			}
		}
		inline void constrain_vector( double& x , double& y , double length )
		{
			if( length <= 0.0 ) return;
			double vec_length = safe_sqrt( x*x + y*y );
			if( vec_length > length )
			{
				vec_length = length / vec_length;
				x *= vec_length;
				y *= vec_length;
			}
		}
	/*��ά����*/
		
	/*��ά����*/
		inline void constrain_vector( float& x , float& y , float& z , float length )
		{
			if( length <= 0.0f ) return;
			float vec_length = safe_sqrt( x*x + y*y + z*z );
			if( vec_length > length )
			{
				vec_length = length / vec_length;
				x *= vec_length;
				y *= vec_length;
				z *= vec_length;
			}
		}
		inline void constrain_vector( double& x , double& y , double& z , double length )
		{
			if( length <= 0.0 ) return;
			float vec_length = safe_sqrt( x*x + y*y + z*z );
			if( vec_length > length )
			{
				vec_length = length / vec_length;
				x *= vec_length;
				y *= vec_length;
				z *= vec_length;
			}
		}
	/*��ά����*/
/*�����޷�*/
	
/*nxn����ȡ��
	a:nxn����
*/	
	inline bool Matrix_Inverse(float a[], unsigned char n)  
	{  
			int *is,*js,i,j,k,l,u,v;  
			float d,p;  
			is = new int[n];  
			js = new int[n];  
			for (k=0; k<=n-1; k++)  
			{  
					d=0.0f;  
					for (i=k; i<=n-1; ++i)  
					for (j=k; j<=n-1; ++j)  
					{  
							l=i*n+j; p=fabsf(a[l]);  
							if (p>d) { d=p; is[k]=i; js[k]=j;}  
					}  
					if (is_zero(d))  
					{  
						delete[] is;
						delete[] js; 
							return(false);  
					}  
					if (is[k]!=k)  
							for (j=0; j<=n-1; ++j)  
							{  
									u=k*n+j; v=is[k]*n+j;  
									p=a[u]; a[u]=a[v]; a[v]=p;  
							}  
					if (js[k]!=k)  
							for (i=0; i<=n-1; ++i)  
							{  
									u=i*n+k; v=i*n+js[k];  
									p=a[u]; a[u]=a[v]; a[v]=p;  
							}  
					l=k*n+k;  
					a[l]=1.0f/a[l];  
					for (j=0; j<=n-1; ++j)  
							if (j!=k)  
							{ u=k*n+j; a[u]=a[u]*a[l];}  
					for (i=0; i<=n-1; ++i)  
							if (i!=k)  
									for (j=0; j<=n-1; ++j)  
					if (j!=k)  
					{  
							u=i*n+j;  
							a[u] -= a[i*n+k]*a[k*n+j];  
					}  
					for (i=0; i<=n-1; ++i)  
							if (i!=k)  
							{  
									u=i*n+k;  
									a[u] = -a[u]*a[l];  
							}  
			}  
			for (k=n-1; k>=0; --k)  
			{  
					if (js[k]!=k)  
					for (j=0; j<=n-1; ++j)  
					{  
							u=k*n+j; v=js[k]*n+j;  
					p=a[u]; a[u]=a[v]; a[v]=p;  
					}  
					if (is[k]!=k)  
					for (i=0; i<=n-1; ++i)  
					{   
							u=i*n+k; v=i*n+is[k];  
							p=a[u]; a[u]=a[v]; a[v]=p;  
					}  
			}  
			delete[] is;
			delete[] js;   
			return(true);  
	}
	
	inline bool Matrix_Inverse(double a[], unsigned char n)  
	{  
			int *is,*js,i,j,k,l,u,v;  
			double d,p;  
			is = new int[n];  
			js = new int[n];  
			for (k=0; k<=n-1; k++)  
			{  
					d=0.0f;  
					for (i=k; i<=n-1; ++i)  
					for (j=k; j<=n-1; ++j)  
					{  
							l=i*n+j; p=fabs(a[l]);  
							if (p>d) { d=p; is[k]=i; js[k]=j;}  
					}  
					if (is_zero(d))  
					{  
						delete[] is;
						delete[] js; 
							return(false);  
					}  
					if (is[k]!=k)  
							for (j=0; j<=n-1; ++j)  
							{  
									u=k*n+j; v=is[k]*n+j;  
									p=a[u]; a[u]=a[v]; a[v]=p;  
							}  
					if (js[k]!=k)  
							for (i=0; i<=n-1; ++i)  
							{  
									u=i*n+k; v=i*n+js[k];  
									p=a[u]; a[u]=a[v]; a[v]=p;  
							}  
					l=k*n+k;  
					a[l]=1.0f/a[l];  
					for (j=0; j<=n-1; ++j)  
							if (j!=k)  
							{ u=k*n+j; a[u]=a[u]*a[l];}  
					for (i=0; i<=n-1; ++i)  
							if (i!=k)  
									for (j=0; j<=n-1; ++j)  
					if (j!=k)  
					{  
							u=i*n+j;  
							a[u] -= a[i*n+k]*a[k*n+j];  
					}  
					for (i=0; i<=n-1; ++i)  
							if (i!=k)  
							{  
									u=i*n+k;  
									a[u] = -a[u]*a[l];  
							}  
			}  
			for (k=n-1; k>=0; --k)  
			{  
					if (js[k]!=k)  
					for (j=0; j<=n-1; ++j)  
					{  
							u=k*n+j; v=js[k]*n+j;  
					p=a[u]; a[u]=a[v]; a[v]=p;  
					}  
					if (is[k]!=k)  
					for (i=0; i<=n-1; ++i)  
					{   
							u=i*n+k; v=i*n+is[k];  
							p=a[u]; a[u]=a[v]; a[v]=p;  
					}  
			}  
			delete[] is;
			delete[] js;   
			return(true);  
	}
/*nxn����ȡ��*/