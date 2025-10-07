#include "cmsis_os.h"
#include "can/can.hpp"
#include "motor/motor.hpp"
#include "uart/uart.hpp"
#include "tools/pid/pid.hpp"
#include <cstdint>  
#include "motor/rm_motor/rm_motor.hpp"


constexpr float MAX_SPEED = 30.0f;
// 新增功率控制参数（基于M3508技术文档）
constexpr float MAX_CHASSIS_POWER = 60.0f;        // 底盘最大允许功率
constexpr float POWER_BUFFER = 4.0f;             // 功率缓冲值
constexpr float K1 = 2.0f;                       // 转矩损耗系数
constexpr float K2 = 0.01f;                      // 速度损耗系数  
constexpr float K3 = 5.0f;                       // 静态待机损耗
constexpr float RM3508_STALL_TORQUE = 4.5f;      // 堵转扭矩（来自文档表格）
constexpr float RM3508_STALL_CURRENT = 2.5f;     // 堵转电流（来自文档表格）
constexpr float TORQUE_CONSTANT = 0.3f;          // 转矩常数 0.3 N·m/A（来自文档）
// 在文件顶部添加函数声明（约第20行附近）

float calculate_power_consumption(float torques[4], float speeds[4]);





// 方案2：基于运动学精确计算
constexpr float MAX_OMEGA = MAX_SPEED / sqrt(
    (TRACK_WIDTH/2)*(TRACK_WIDTH/2) + 
    (WHEEL_BASE/2)*(WHEEL_BASE/2)
);




// 新增功率控制函数
// 修正功率计算函数（基于文档2理论）


// RM3508建议扭矩限幅
constexpr float RM3508_MAX_TORQUE = 3.0f;
constexpr float RM3508_MAX_CURRENT = 10.0f;

// 全局变量

sp::DBusSwitchMode last_sw_l = remote.sw_l;

inline uint16_t motor_enable_freq = 0;
float filtered_actual_speeds[4] = {0};    // 滤波后的实际速度（rad/s）
constexpr float FILTER_ALPHA = 0.3f;      // 滤波系数
// 在全局变量区域添加（约第80行附近）




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
float calculate_power_consumption(float torques[4],  float speeds[4]) 
{
    float torque_sum_square = 0.0f;
    float speed_sum_square = 0.0f;
    float total_expected_power = 0.0f;
    
    for (int i = 0; i < 4; i++) {
        torque_sum_square += torques[i] * torques[i];
        speed_sum_square += speeds[i] * speeds[i];
    }
    total_expected_power=K1 * torque_sum_square + K2 * speed_sum_square + K3;
    return total_expected_power;
}

static float calculate_power_scale_factor(float target_torques[4], float actual_speeds[4]) {
    float available_power = MAX_CHASSIS_POWER - POWER_BUFFER;
    float current_power = calculate_power_consumption(target_torques, actual_speeds);
    
    if (current_power <= available_power) {
        return 1.0f;
    }
    
    // 计算缩放系数k的平方
    float torque_sum_square = 0.0f;
    for (int i = 0; i < 4; i++) {
        torque_sum_square += target_torques[i] * target_torques[i];
    }
    
    if (torque_sum_square < 0.001f) return 0.0f;
    
    float k_square = (available_power - K2 * (actual_speeds[0]*actual_speeds[0] + 
                   actual_speeds[1]*actual_speeds[1] + actual_speeds[2]*actual_speeds[2] + 
                   actual_speeds[3]*actual_speeds[3]) - K3) / (K1 * torque_sum_square);
    
    return (k_square > 0) ? sqrtf(k_square) : 0.0f;
    
    }  
// ==================== 修正后的运动学函数 ====================
void mecanum_kinematics(float vx, float vy, float omega, float wheel_speeds[4]) {
    // 1. 输入验证和安全检查
    if (TRACK_WIDTH <= 0 || WHEEL_BASE <= 0 || WHEEL_RADIUS <= 0) {
        for (int i = 0; i < 4; i++) wheel_speeds[i] = 0.0f;
        return;
    }
    
    // 2. 输入标准化（-1.0 到 1.0 范围）
    vx = fminf(fmaxf(vx, -1.0f), 1.0f) * MAX_SPEED;
    vy = fminf(fmaxf(vy, -1.0f), 1.0f) * MAX_SPEED;
    omega = fminf(fmaxf(omega, -1.0f), 1.0f) * MAX_OMEGA;
    
    // 3. 运动学参数计算
    const float L = TRACK_WIDTH / 2.0f;    // 轮距的一半
    const float W = WHEEL_BASE / 2.0f;     // 轴距的一半
    const float R = sqrt(L * L + W * W);   // 旋转半径
    
    // 4. 标准麦轮运动学公式（修正版）
    wheel_speeds[0] = (vx - vy - omega * R) / WHEEL_RADIUS;  // 左前轮
    wheel_speeds[1] = (vx + vy + omega * R) / WHEEL_RADIUS;  // 右前轮
    wheel_speeds[2] = (vx + vy - omega * R) / WHEEL_RADIUS;  // 左后轮（修正）
    wheel_speeds[3] = (vx - vy + omega * R) / WHEEL_RADIUS;  // 右后轮（修正）
    
    // 5. 应用减速比
    for (int i = 0; i < 4; i++) {
        wheel_speeds[i] *= GEAR_RATIO;
    }
    
    // 6. 速度限幅保护
    constexpr float MAX_WHEEL_SPEED = 30.0f * GEAR_RATIO / WHEEL_RADIUS;
    for (int i = 0; i < 4; i++) {
        wheel_speeds[i] = fminf(fmaxf(wheel_speeds[i], -MAX_WHEEL_SPEED), MAX_WHEEL_SPEED);
    }
}


extern "C" void control_task()
{
   
    struct MotorData {
        float given_torque[4];
        float absolute_angle_set[4];
        float absolute_speed_set[4];
    } motor_data;  // 定义motor_data变量


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
       
     
        float vx = 0.0f, vy = 0.0f, omega = 0.0f;
        
        // 验证遥控器连接状态
        if (remote.is_alive(100)) {
            // 通道映射（根据实际遥控器配置调整）
            vx = remote.ch_rh * MAX_SPEED;    // 通常ch2为右水平摇杆
            vy = remote.ch_lh * MAX_SPEED;    // 通常ch3为左水平摇杆  
            omega = remote.ch_lv * MAX_OMEGA; // 通常ch0为左垂直摇杆
            
            // 死区处理（内联实现）
            constexpr float DEADZONE = 0.05f;
            if (fabsf(vx) < DEADZONE) vx = 0.0f;
            if (fabsf(vy) < DEADZONE) vy = 0.0f;
            if (fabsf(omega) < DEADZONE) omega = 0.0f;
            
            // 限制输出范围
            vx = fminf(fmaxf(vx, -MAX_SPEED), MAX_SPEED);
            vy = fminf(fmaxf(vy, -MAX_SPEED), MAX_SPEED);
            omega = fminf(fmaxf(omega, -MAX_OMEGA), MAX_OMEGA);
       
        
      




    
        
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

       
                
        // 速度环控制
        if (remote.sw_r == sp::DBusSwitchMode::MID) {
        float target_torques[4] = {0};
    
    // 计算原始目标转矩
        for (int i = 0; i < 4; i++) {
            motor_pid_speed[i].calc(target_speeds[i], filtered_actual_speeds[i]);
            target_torques[i] = motor_pid_speed[i].out * motor_compensation[i];
        
        // 单电机扭矩限幅保护
            target_torques[i] = fminf(fmaxf(target_torques[i], -RM3508_MAX_TORQUE), RM3508_MAX_TORQUE);
        }
    
    // 应用功率限制
    float power_scale = calculate_power_scale_factor(target_torques, filtered_actual_speeds);
    if (power_scale < 0.1f) power_scale = 0.0f;
   
    // 应用缩放后的转矩
        for (int i = 0; i < 4; i++) {
            motor_data.given_torque[i] = target_torques[i] * power_scale;
        }
    

   

    
        
    } else {
    // 急停模式
        for (int i = 0; i < 4; i++) {
            motor_data.given_torque[i] = 0.0f;
            vx = vy = omega = 0.0f;
            }


        // 发送电机指令
        for (int i = 0; i < 4; i++) {
            motors[i]->cmd(motor_data.given_torque[i]);
            send_rm_motor(i);
        }
       }
     
        

        last_sw_l = remote.sw_l;
        osDelay(1);
    }
    
}
}
/**
 * @brief 计算预期功率消耗
 * @param torques 各电机目标转矩数组（N·m）
 * @param speeds 各电机实际转速数组（rad/s）
 * @return 预期总功率（W）
 */




void enable_all_motors() {
    uint8_t tx_data[8];
    sp::RM_Motor* motors[] = {&rm_motor_1, &rm_motor_2, &rm_motor_3, &rm_motor_4};
    
    for (int i = 0; i < 4; i++) {
        // If there is a specific enable method, call it here. Otherwise, just send a command or remove this step.
        // Example: motors[i]->enable(); // Uncomment and implement if such a method exists.
        can1.send(motors[i]->tx_id);
        osDelay(1);
    }
}

void send_rm_motor(uint8_t motor_id) {
    uint8_t tx_data[8];
    sp::RM_Motor* motors[] = {&rm_motor_1, &rm_motor_2, &rm_motor_3, &rm_motor_4};
    
    if (motor_id < 4) {
        motors[motor_id]->write(tx_data);
        can1.send(motors[motor_id]->tx_id);
    }
}

