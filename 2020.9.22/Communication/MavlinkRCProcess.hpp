#pragma once

#include "mavlink.h"

//Mavlink��Ϣ��������
extern void (*const Mavlink_RC_Process[])( uint8_t Port_index , const mavlink_message_t* msg_sd );
//Mavlink��Ϣ����������
extern const uint16_t Mavlink_RC_Process_Count;