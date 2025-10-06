#include "cmsis_os.h"
#include "can/can.hpp"
#include "motor/motor.hpp"
#include "uart/uart.hpp"
#include "tools/pid/pid.hpp"

// 运动学参数常量定义
constexpr float WHEEL_DIAMETER = 0.154f;
constexpr float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0f;
constexpr float TRACK_WIDTH = 0.370f;
constexpr float WHEEL_BASE = 0.330f;
constexpr float GEAR_RATIO = 14.9f;
constexpr float MAX_SPEED = 30.0f;

// RM3508建议扭矩限幅
constexpr float RM3508_MAX_TORQUE = 3.0f;
constexpr float RM3508_MAX_CURRENT = 10.0f;

// 全局变量
MultiMotorData motor_data;
sp::DBusSwitchMode last_sw_l = remote.sw_l;
inline bool motor_enable = false;
inline uint16_t motor_enable_freq = 0;
float filtered_actual_speeds[4] = {0};    // 滤波后的实际速度（rad/s）
constexpr float FILTER_ALPHA = 0.3f;      // 滤波系数

// 电机对象定义
sp::RM_Motor rm_motor_2(1, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor rm_motor_1(2, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor rm_motor_3(3, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor rm_motor_4(4, sp::RM_Motors::M3508, 14.9f);

// PID控制器配置
sp::PID motor_pid_angle[4] = {
    {0.001f, 0.12f, 0.002f, 0.15f, RM3508_MAX_TORQUE, 1.5f, 1.0f, true, true},
    {0.001f, 0.12f, 0.002f, 0.15f, RM3508_MAX_TORQUE, 1.5f, 1.0f, true, true},
    {0.001f, 0.12f, 0.002f, 0.15f, RM3508_MAX_TORQUE, 1.5f, 1.0f, true, true},
    {0.001f, 0.12f, 0.002f, 0.15f, RM3508_MAX_TORQUE, 1.5f, 1.0f, true, true}
};

sp::PID motor_pid_speed[4] = {
    {0.001f, 0.06f, 0.002f, 0.1f, RM3508_MAX_TORQUE, 1.5f, 0.3f, false, true},
    {0.001f, 0.06f, 0.002f, 0.1f, RM3508_MAX_TORQUE, 1.5f, 0.3f, false, true},
    {0.001f, 0.06f, 0.002f, 0.1f, RM3508_MAX_TORQUE, 1.5f, 0.3f, false, true},
    {0.001f, 0.06f, 0.002f, 0.1f, RM3508_MAX_TORQUE, 1.5f, 0.3f, false, true}
};

// 函数声明
void enable_all_motors();
void send_rm_motor(uint8_t motor_id);
void mecanum_kinematics(float vx, float vy, float omega, float wheel_speeds[4]);
void mecanum_kinematics(float vx, float vy, float omega, float wheel_speeds[4]) {
    // 标准化输入
    vx = fminf(fmaxf(vx, -1.0f), 1.0f) * MAX_SPEED;
    vy = fminf(fmaxf(vy, -1.0f), 1.0f) * MAX_SPEED;
    omega = fminf(fmaxf(omega, -1.0f), 1.0f) * MAX_SPEED;
    
    // 修正后的运动学模型
    float L = TRACK_WIDTH / 2.0f;    // 轮距的一半
    float W = WHEEL_BASE / 2.0f;     // 轴距的一半
    float R = sqrt(L * L + W * W);   // 旋转半径
    
    // 计算各轮转速（rad/s）
    wheel_speeds[0] = ( vx - vy - omega * R) / WHEEL_RADIUS;  // 左前轮
    wheel_speeds[1] = ( vx + vy + omega * R) / WHEEL_RADIUS;  // 右前轮
    wheel_speeds[2] = ( vx - vy + omega * R) / WHEEL_RADIUS;  // 左后轮
    wheel_speeds[3] = ( vx + vy - omega * R) / WHEEL_RADIUS;  // 右后轮
    
    // 考虑减速比
    for (int i = 0; i < 4; i++) {
        wheel_speeds[i] *= GEAR_RATIO;
    }
}
extern "C" void control_task()
{
    remote.request();
    can1.config();
    can1.start();
    can2.config();
    can2.start();

    // 初始化电机控制数据
    sp::RM_Motor* motors[] = {&rm_motor_1, &rm_motor_2, &rm_motor_3, &rm_motor_4};
    for (int i = 0; i < 4; i++) {
        motor_data.absolute_angle_set[i] = motors[i]->angle;
        motor_data.absolute_speed_set[i] = 0.0f;
        motor_data.given_torque[i] = 0.0f;
        filtered_actual_speeds[i] = 0.0f;
    }
// 在control_task函数中添加
float motor_compensation[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // 电机补偿系数

// 在发送电机指令前添加补偿
for (int i = 0; i < 4; i++) {
    motor_data.given_torque[i] *= motor_compensation[i];
}

    while (true) {
               // 获取遥控器输入
        float vx = remote.ch3 * 1.0f;
        float vy = remote.ch2 * 1.0f;
        float omega = remote.ch0 * 1.0f;
        
        // 运动学计算
        float target_speeds[4];
        mecanum_kinematics(vx, vy, omega, target_speeds);

        // 速度滤波与单位转换
        for (int i = 0; i < 4; i++) {
            // 电机转速 → 轮子转速（rad/s）
            float actual_speed = motors[i]->speed / GEAR_RATIO;
            filtered_actual_speeds[i] = FILTER_ALPHA * actual_speed + 
                                     (1 - FILTER_ALPHA) * filtered_actual_speeds[i];
        }

        // 电机使能控制
        if (motor_enable_freq == 5000) motor_enable = true;
        motor_enable_freq = (motor_enable_freq + 1) % 15000;

        if (motor_enable) {
            enable_all_motors();
            motor_enable = false;
            continue;
        }

                
        // 速度环控制
        if (remote.sw_r == sp::DBusSwitchMode::MID) {
            for (int i = 0; i < 4; i++) {
                motor_pid_speed[i].calc(target_speeds[i], filtered_actual_speeds[i]);
                motor_data.given_torque[i] = motor_pid_speed[i].out * motor_compensation[i];
                
                // 扭矩限幅保护
                motor_data.given_torque[i] = fminf(fmaxf(
                    motor_data.given_torque[i], -RM3508_MAX_TORQUE), RM3508_MAX_TORQUE);
            }
        } else {
            // 急停
            for (int i = 0; i < 4; i++) {
                motor_data.given_torque[i] = 0.0f;
            }
        }

        // 发送电机指令
        for (int i = 0; i < 4; i++) {
            motors[i]->cmd(motor_data.given_torque[i]);
            send_rm_motor(i);
        }

        // 调试输出
        if (osKernelGetTickCount() % 100 == 0) {
            printf("Target: %.2f, %.2f, %.2f, %.2f\\n", 
                   target_speeds[0], target_speeds[1], target_speeds[2], target_speeds[3]);
            printf("Actual: %.2f, %.2f, %.2f, %.2f\\n", 
                   filtered_actual_speeds[0], filtered_actual_speeds[1], 
                   filtered_actual_speeds[2], filtered_actual_speeds[3]);
        }

        last_sw_l = remote.sw_l;
        osDelay(1);
    }
}
void enable_all_motors() {
    uint8_t tx_data[8];
    sp::RM_Motor* motors[] = {&rm_motor_1, &rm_motor_2, &rm_motor_3, &rm_motor_4};
    
    for (int i = 0; i < 4; i++) {
        motors[i]->write_enable(tx_data);
        can1.send(motors[i]->tx_id, tx_data);
        osDelay(1);
    }
}

void send_rm_motor(uint8_t motor_id) {
    uint8_t tx_data[8];
    sp::RM_Motor* motors[] = {&rm_motor_1, &rm_motor_2, &rm_motor_3, &rm_motor_4};
    
    if (motor_id < 4) {
        motors[motor_id]->write(tx_data);
        can1.send(motors[motor_id]->tx_id, tx_data);
    }
}

void mecanum_kinematics(float vx, float vy, float omega, float wheel_speeds[4]) {
    // 计算各轮转速（rad/s）
    wheel_speeds[0] = (vx - vy - omega*(TRACK_WIDTH+WHEEL_BASE)/2.0f) / WHEEL_RADIUS;
    wheel_speeds[1] = (vx + vy + omega*(TRACK_WIDTH+WHEEL_BASE)/2.0f) / WHEEL_RADIUS;
    wheel_speeds[2] = (vx - vy + omega*(TRACK_WIDTH+WHEEL_BASE)/2.0f) / WHEEL_RADIUS;
    wheel_speeds[3] = (vx + vy - omega*(TRACK_WIDTH+WHEEL_BASE)/2.0f) / WHEEL_RADIUS;
    
    // 考虑减速比（电机转速 = 轮子转速 * GEAR_RATIO）
    for (int i = 0; i < 4; i++) {
        wheel_speeds[i] *= GEAR_RATIO;
    }
}
