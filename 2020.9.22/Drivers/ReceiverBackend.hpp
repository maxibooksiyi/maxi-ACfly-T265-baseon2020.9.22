#pragma once

#include "Basic.hpp"
#include <stdbool.h>
#include "Receiver.hpp"

bool ReceiverRegister( SName name );

//���½��ջ�����
bool ReceiverUpdate( SName name, bool connected, float raw_data[16], uint8_t channel_count, double TIMEOUT );