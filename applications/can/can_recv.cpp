#include "cmsis_os.h"
#include "can.hpp"
#include "motor/motor.hpp"
#include <cstdint> 
#include "motor/rm_motor/rm_motor.hpp"
#include <cmath> 

// 全局功率变量
float motor_powers[4] = {0};              // 各电机实时功率
float total_chassis_power = 0.0f;  
float current_power = 0.0f;        // 底盘总功率
float motor_efficiencies[4] = {0.68f, 0.68f, 0.68f, 0.68f}; // 电机效率系数
constexpr float V_bus = 24.0f;  // M3508额定电压
// 假设的电机转矩常数（需要根据实际电机型号调整）
constexpr float Kt_M3508 = 0.3f;  // N·m/A，M3508的转矩常数

void can1_recv(uint32_t stamp_ms)
{
    can1.recv();
    sp::RM_Motor* motors[] = {&rm_motor_1, &rm_motor_2, &rm_motor_3, &rm_motor_4};
    
    for (int i = 0; i < 4; i++) {
        if (can1.rx_id == motors[i]->rx_id) {
            motors[i]->read(can1.rx_data, stamp_ms);
            
            // 从扭矩反推电流: I = τ / Kt
            float estimated_current = motors[i]->torque / Kt_M3508;
            
            // 使用电流计算功率（假设电压24V）
            constexpr float V_bus = 24.0f;
            motor_powers[i] = V_bus * fabsf(estimated_current) * motor_efficiencies[i];
            
            break;
        }
    }
     // 计算总功率
    current_power = motor_powers[0] + motor_powers[1] +
                         motor_powers[2] + motor_powers[3];
    
    // if (can1.rx_id == rm_motor_x.rx_id)
    //     rm_motor_x.read(can1.rx_data, stamp_ms);

    // if (can1.rx_id == lk_motor_x.rx_id)
    //     lk_motor_x.read(can1.rx_data);
}

void can2_recv(uint32_t stamp_ms)
{
    can2.recv();
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
    auto stamp_ms = osKernelSysTick();

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (hcan == &hcan1)
            can1_recv(stamp_ms);
        else if (hcan == &hcan2)
            can2_recv(stamp_ms);
    }
}
