/* host_check 桩：基地单电机接口，不含 HAL/CAN */
#ifndef __DART_MOTOR_STUB_H
#define __DART_MOTOR_STUB_H
#include "dart_config.h"

typedef enum {
    DART_MOTOR_BASE = 0,
    DART_MOTOR_NUM,
} dart_motor_id_e;

bool Dart_Motor_Is_Online(dart_motor_id_e id);
int32_t Dart_Motor_Get_Encoder_Sum(dart_motor_id_e id);
float Dart_Motor_Get_Speed_Rpm(dart_motor_id_e id);
void Dart_Motor_Set_Output(dart_motor_id_e id, float out_raw);

#endif
