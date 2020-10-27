#pragma once

#include "AC_Math.hpp"

/*
	��ά������
	20190909�����Ľ�
*/

template<typename T>
class vector3
{
	private:
		
	public:
		T x , y , z;
	
		/*���캯��
			x y z��������ά����
		*/
			inline vector3(){this->x = this->y = this->z = 0;}
			inline vector3( T x , T y , T z )
			{
				this->x = x;
				this->y = y;
				this->z = z;
			}
			inline void set_vector( const T x , const T y , const T z )
			{
				this->x = x;
				this->y = y;
				this->z = z;
			}
		/*���캯��*/
		
		//��������
		inline void zero()
		{
			x=0;	y=0;	z=0;
		}
		
		//�Ⱥ����������
		inline vector3<T>& operator =( const vector3<T>& b )
		{
			x=b.x;	y=b.y;	z=b.z;
			return *this;
		}
		
		//�±����������
		inline T& operator[](int i)
		{
			if( i == 0 ) return x;
			else if( i == 1 ) return y;
			else return z;
		}
		
		/*�����������������������������
			����*���������
			����%���������
			����&������Ԫ�ظ������
		*/
			inline vector3<T> operator +( const vector3<T>& b ) const
			{
				return vector3<T>( x+b.x , y+b.y , z+b.z );
			}
			inline vector3<T> operator -( const vector3<T>& b ) const
			{
				return vector3<T>( x-b.x , y-b.y , z-b.z );
			}
			inline vector3<T> operator *( const T b ) const
			{
				return vector3<T>( x*b , y*b , z*b );
			}
			inline vector3<T> operator /( const T b ) const
			{
				return vector3<T>( x/b , y/b , z/b );
			}
			inline vector3<T> operator -(void) const
			{
				return vector3<T>( -x , -y , -z );
			}
			
			inline vector3<T> operator +=( const vector3<T>& b )
			{
				x += b.x;
				y += b.y;
				z += b.z;
				return *this;
			}
			
			inline vector3<T> operator -=( const vector3<T>& b )
			{
				this->x -= b.x;
				this->y -= b.y;
				this->z -= b.z;
				return *this;
			}
			inline vector3<T> operator *=( T b )
			{
				this->x *= b;
				this->y *= b;
				this->z *= b;
				return *this;
			}
			
			//���
			inline T operator *( const vector3<T>& b )const
			{
				return this->x*b.x + this->y*b.y + this->z*b.z;
			}		
			//���
			inline vector3<T> operator %( const vector3<T>& b )const
			{
				vector3<T> result;
				result.x = this->y * b.z - this->z *b.y;
				result.y = this->z * b.x - this->x *b.z;
				result.z = this->x * b.y - this->y *b.x;
				return result;
			}
			//Ԫ�ظ������
			inline vector3<T> operator &( const vector3<T>& b )const
			{
				vector3<T> result;
				result.x = this->x * b.x;
				result.y = this->y * b.y;
				result.z = this->z * b.z;
				return result;
			}
			
			inline bool operator ==(const vector3<T> &v) const
			{
					return x==v.x && y==v.y && z==v.z;
			}
			
			inline bool operator !=(const vector3<T> &v) const
			{
					return x!=v.x || y!=v.y || z!=v.z;
			}
		/*�����������������������������*/
		
		//��������ƽ��
		inline T get_square() const
		{
			return this->x*this->x + this->y*this->y + this->z*this->z;
		}
			
		inline void constrain( const T max_length );
		inline void normalize();
		
		/*������λ�����н�����
			vec_a , vec_b:��λ����a��b
		*/
			static inline vector3<float> get_included_angle_from_unit_vector( vector3<float> vec_a , vector3<float> vec_b )
			{
				vector3<float> angle_vec = vec_a % vec_b;
				float angle_sin = safe_sqrt( angle_vec.get_square() );
				float angle_cosin = vec_a * vec_b;
				if( is_zero( angle_sin ) )
				{
					if( angle_cosin > 0.0f )	//angle is zero
						return vector3<float>( 0.0f , 0.0f , 0.0f );
					else
					{	//angle is 180 degree
						vector3<float> rvec;
						rvec.x = 0;
						rvec.y = vec_a.z;
						rvec.z = -vec_a.y;
						
						rvec.normalize();
						rvec *= Pi_f;
					}
				}
				else
				{
					float angle;
					if( angle_sin > 1.0f )
						angle = 0.5f * Pi_f;
					else
						angle = asinf( angle_sin );
					if( angle_cosin < 0.0f )
						angle = Pi_f - angle;
					return angle_vec * angle / angle_sin;
				}
			}
			static inline vector3<double> get_included_angle_from_unit_vector( vector3<double> vec_a , vector3<double> vec_b )
			{
				vector3<double> angle_vec = vec_a % vec_b;
				double angle_sin = safe_sqrt( angle_vec.get_square() );
				double angle_cosin = vec_a * vec_b;
				if( is_zero( angle_sin ) )
				{
					if( angle_cosin > 0.0 )	//angle is zero
						return vector3<double>( 0.0 , 0.0 , 0.0 );
					else
					{	//angle is 180 degree
						vector3<double> rvec;
						rvec.x = 0;
						rvec.y = vec_a.z;
						rvec.z = -vec_a.y;
						
						rvec.normalize();
						rvec *= Pi;
						return rvec;
					}
				}
				else
				{
					double angle;
					if( angle_sin > 1.0 )
						angle = 0.5 * Pi;
					else
						angle = asin( angle_sin );
					if( angle_cosin < 0.0 )
						angle = Pi - angle;
					return angle_vec * angle / angle_sin;
				}
			}
		/*������λ�����н�����*/
};

/*���������Ⱥ���������*/
	template<>
	inline bool vector3<float>::operator ==(const vector3<float> &v) const
	{
			return (is_equal(x,v.x) && is_equal(y,v.y) && is_equal(z,v.z));
	}
	template<>
	inline bool vector3<double>::operator ==(const vector3<double> &v) const
	{
			return (is_equal(x,v.x) && is_equal(y,v.y) && is_equal(z,v.z));
	}
	
	template<>
	inline bool vector3<float>::operator !=(const vector3<float> &v) const
	{
			return (!is_equal(x,v.x) || !is_equal(y,v.y) || !is_equal(z,v.z));
	}
	template<>
	inline bool vector3<double>::operator !=(const vector3<double> &v) const
	{
			return (!is_equal(x,v.x) || !is_equal(y,v.y) || !is_equal(z,v.z));
	}
/*���������Ⱥ���������*/

/*���������޷�����*/
	template<>
	inline void vector3<float>::constrain ( float max_length )
	{
		float length = get_square();
		float sq_max_length = max_length * max_length;
		if( length > sq_max_length )
		{
			float scale = sqrtf( sq_max_length / length );
			(*this) *= scale;
		}	
	}	
	template<>
	inline void vector3<double>::constrain ( double max_length )
	{
		double length = get_square();
		double sq_max_length = max_length * max_length;
		if( length > sq_max_length )
		{
			double scale = sqrt( sq_max_length / length );
			(*this) *= scale;
		}	
	}
/*���������޷�����*/
	
/*����������һ������*/
	template<>
	inline void vector3<float>::normalize()
	{
		float length = safe_sqrt( get_square() );
		if( is_zero( length ) )
			return;
		
		length = 1.0f / length;
		*this *= length;
	}
	template<>
	inline void vector3<double>::normalize()
	{
		double length = safe_sqrt( get_square() );
		if( is_zero( length ) )
			return;
		
		length = 1.0 / length;
		*this *= length;
	}
/*����������һ������*/