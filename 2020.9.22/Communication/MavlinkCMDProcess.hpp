#pragma once

#include "mavlink.h"

//Mavlink���������
extern void (*const Mavlink_CMD_Process[])( uint8_t port_index , const mavlink_message_t* msg_rd );
//Mavlink�����������
extern const uint16_t Mavlink_CMD_Process_Count;