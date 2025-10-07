#ifndef MOTOR_HPP
#define MOTOR_HPP

// #include "sp_middleware/motor/rm_motor/rm_motor.hpp"  // 实际路径
#include "motor/rm_motor/rm_motor.hpp"
#include <cstdint>   // 实际路径





constexpr float P_MIN = -3.141593f * 4;
constexpr float P_MAX = 3.141593f * 4;
constexpr float V_MIN = -30.0f;
constexpr float V_MAX = 30.0f;
constexpr float T_MIN = -10.0f;
constexpr float T_MAX = 10.0f;
// 在motor.hpp中添加底盘参数
constexpr float WHEEL_DIAMETER = 0.154f;     // 麦轮直径0.154m
constexpr float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0f;
constexpr float TRACK_WIDTH = 0.370f;        // 横向间距0.370m
constexpr float WHEEL_BASE = 0.330f;         // 纵向间距0.330m
constexpr float GEAR_RATIO = 14.9f;          // 减速比14.9

//inline sp::LK_Motor lk_motor_x(2, 0.06);
inline sp::RM_Motor rm_motor_1(1, sp::RM_Motors::M3508);
inline sp::RM_Motor rm_motor_2(2, sp::RM_Motors::M3508);
inline sp::RM_Motor rm_motor_3(3, sp::RM_Motors::M3508);
inline sp::RM_Motor rm_motor_4(4, sp::RM_Motors::M3508);
//inline sp::DM_Motor dm_motor_1(0x171, 0x181, P_MAX, V_MAX, T_MAX);
#pragma once

#include <cstdint>

// 功率传感器相关函数声明
void parse_power_sensor_data(uint8_t* data, float* motor_powers);
bool is_power_sensor_frame(uint32_t can_id);
bool is_power_data_valid(float power);

// 功率传感器配置参数
constexpr uint32_t POWER_SENSOR_BASE_ID = 0x500;
constexpr float POWER_SENSOR_SCALE = 0.1f;
constexpr uint32_t SENSOR_TIMEOUT_MS = 100;






typedef struct
{
	float absolute_angle_set;	// 绝对角度的目标值，rad
	float absolute_speed_set;	// 绝对速度的目标值，rad/s
	float given_torque;			// 电机给定的力矩，Nm
} GimbalData;


#endif  // MOTOR_HPP