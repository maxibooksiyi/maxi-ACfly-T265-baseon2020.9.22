#pragma once

#include "TimeBase.hpp"
#include "stm32h743xx.h"

#include <string.h>
#include "Basic.h"

/*
	�ɱȽ϶���������
	�������ƴ洢������
*/
class SName
{
	private:
		char name[16];
	
	public:
		SName(){ memset( this->name , 0 , 16 ); }
		SName( const char* name )
		{
			uint8_t i;
			for( i = 0 ; i < 16 ; ++i )
			{
				this->name[i] = name[i];
				if( name[i] == 0 )
					break;				
			}
			memset( &this->name[i] , 0 , 16-i );
		}
		
		//��ȡ�ַ�����17�ֽڳ��ȣ�
		void get_CharStr( char* str )
		{
			str[16] = 0;
			memcpy( str , name , 16 );
		}
		
		inline bool operator ==(const SName &n2) const
		{
			for( uint8_t i = 0; i < 16; ++i )
			{
				if( this->name[i] != n2.name[i] )
					return false;
				if( this->name[i] == 0 )
					return true;
			}
			return true;
		}
		
		inline bool operator !=(const SName &n2) const
		{
			for( uint8_t i = 0; i < 16; ++i )
			{
				if( this->name[i] != n2.name[i] )
					return true;
				if( this->name[i] == 0 )
					return false;
			}
			return false;
		}
		
		//�Ƚ��������������
		bool operator<(const SName& name) const
		{
			for( uint8_t i = 0; i < 16; ++i )
			{
				if( this->name[i] < name.name[i] )
					return true;
				else if( this->name[i] > name.name[i] )
					return false;
				if( this->name[i] == 0 )
					return false;
			}
			return false;
		}
		
		//����ƴ��
		SName operator+(const SName &n)
    {
			uint8_t i;
			SName result;
			for( i = 0 ; i < 16 ; ++i )
			{
				if( name[i] == 0 )
					break;		
				result.name[i] = this->name[i];		
			}
			uint8_t ni = i;
      for(  ; i < 16 ; ++i )
			{
				result.name[i] = n.name[i-ni];
				if( n.name[i-ni] == 0 )
					break;	
			}
			return result;
    }
};

//��ʼ�����ָʾ
//��ʼ����ɲ��ܽ��г�ʼ������
bool getInitializationCompleted();
void setInitializationCompleted();
void LockInitializationStatus();
void UnLockInitializationStatus();

/*
	���üĴ���
	reg:�Ĵ���ָ��
	value:Ҫ����ָ��λ�е�ֵ
	offset:Ҫ�����λ��ƫ��
	value_length:Ҫ�����λ��
*/
inline void set_register( volatile unsigned int& reg , const unsigned char value , const unsigned char offset , const unsigned char value_length )
{
	unsigned char offset_end_bit = offset + value_length;
	for( unsigned char i = offset ; i < offset_end_bit ; ++ i )
		reg &= ~(1<<i);
	
	reg |= ( value << offset );
}

void init_Basic();