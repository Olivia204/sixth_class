#ifndef MOTOR_HPP
#define MOTOR_HPP


#include "applications/motor/motor.hpp"


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


// 电机使能
inline bool motor_enable = false;       // 电机使能标志
inline uint16_t motor_enable_freq = 0;  // 电机使能计数器

typedef struct
{
	float absolute_angle_set;	// 绝对角度的目标值，rad
	float absolute_speed_set;	// 绝对速度的目标值，rad/s
	float given_torque;			// 电机给定的力矩，Nm
} GimbalData;


#endif  // MOTOR_HPP