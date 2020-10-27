#pragma once

#include "AC_Math.hpp"

/*
	��ά������
	20190909�����Ľ�
*/

template<typename T>
class vector2
{
	private:
		
	public:
		T x , y;
	
		/*���캯��
			x y��������ά����
		*/
			inline vector2(){this->x = this->y = 0;}
			inline vector2( T x , T y )
			{
				this->x = x;
				this->y = y;
			}
			inline void set_vector( const T x , const T y )
			{
				this->x = x;
				this->y = y;
			}
		/*���캯��*/
		
		//��������
		inline void zero()
		{
			x=0;	y=0;
		}
		
		//�Ⱥ����������
		inline vector2<T>& operator =( const vector2<T>& b )
		{
			x=b.x;	y=b.y;
			return *this;
		}
		
		//�±����������
		inline T& operator[](int i)
		{
			if( i == 0 ) return x;
			else return y;
		}
		
		/*�����������������������������
			����*���������
			����%���������
			����&������Ԫ�ظ������
		*/
			inline vector2<T> operator +( const vector2<T>& b ) const
			{
				return vector2<T>( x+b.x , y+b.y );
			}
			inline vector2<T> operator -( const vector2<T>& b ) const
			{
				return vector2<T>( x-b.x , y-b.y );
			}
			inline vector2<T> operator *( const T b ) const
			{
				return vector2<T>( x*b , y*b );
			}
			inline vector2<T> operator /( const T b ) const
			{
				return vector2<T>( x/b , y/b );
			}
			inline vector2<T> operator -(void) const
			{
				return vector2<T>( -x , -y );
			}
			
			inline vector2<T> operator +=( const vector2<T>& b )
			{
				x += b.x;
				y += b.y;
				return *this;
			}
			
			inline vector2<T> operator -=( const vector2<T>& b )
			{
				this->x -= b.x;
				this->y -= b.y;
				return *this;
			}
			inline vector2<T> operator *=( T b )
			{
				this->x *= b;
				this->y *= b;
				return *this;
			}
			
			//���
			inline T operator *( const vector2<T>& b )const
			{
				return this->x*b.x + this->y*b.y;
			}		
			//���
			inline vector2<T> operator %( const vector2<T>& b )const
			{
				vector2<T> result;
				result.x = this->y * b.x - this->x *b.y;
				result.y = this->x * b.y - this->y *b.x;
				return result;
			}
			//Ԫ�ظ������
			inline vector2<T> operator &( const vector2<T>& b )const
			{
				vector2<T> result;
				result.x = this->x * b.x;
				result.y = this->y * b.y;
				return result;
			}
			
			inline bool operator ==(const vector2<T> &v) const
			{
				return x==v.x && y==v.y;
			}
			
			inline bool operator !=(const vector2<T> &v) const
			{
				return x!=v.x || y!=v.y;
			}
		/*�����������������������������*/
		
		//��������ƽ��
		inline T get_square() const
		{
			return this->x*this->x + this->y*this->y;
		}
			
		inline void constrain( const T max_length );
		inline void normalize();
};

/*���������Ⱥ���������*/
	template<>
	inline bool vector2<float>::operator ==(const vector2<float> &v) const
	{
			return (is_equal(x,v.x) && is_equal(y,v.y));
	}
	template<>
	inline bool vector2<double>::operator ==(const vector2<double> &v) const
	{
			return (is_equal(x,v.x) && is_equal(y,v.y));
	}
	
	template<>
	inline bool vector2<float>::operator !=(const vector2<float> &v) const
	{
			return (!is_equal(x,v.x) || !is_equal(y,v.y));
	}
	template<>
	inline bool vector2<double>::operator !=(const vector2<double> &v) const
	{
			return (!is_equal(x,v.x) || !is_equal(y,v.y));
	}
/*���������Ⱥ���������*/

/*���������޷�����*/
	template<>
	inline void vector2<float>::constrain ( float max_length )
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
	inline void vector2<double>::constrain ( double max_length )
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
	inline void vector2<float>::normalize()
	{
		float length = safe_sqrt( get_square() );
		if( is_zero( length ) )
			return;
		
		length = 1.0f / length;
		*this *= length;
	}
	template<>
	inline void vector2<double>::normalize()
	{
		double length = safe_sqrt( get_square() );
		if( is_zero( length ) )
			return;
		
		length = 1.0 / length;
		*this *= length;
	}
/*����������һ������*/