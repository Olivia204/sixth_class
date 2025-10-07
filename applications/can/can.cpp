#include "cmsis_os.h"
#include "can.hpp"
#include "motor/motor.hpp"
#include "uart/uart.hpp"
#include "tools/pid/pid.hpp"
#include <cstdint>
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include <cmath>
#include "motor/rm_motor/rm_motor.hpp"

void enable_rm_motor() {
    uint8_t tx_data[8] = {0x01}; // 使能指令
    // 快速连续发送（间隔由CAN硬件自动处理）
    can1.send(rm_motor_1.tx_id);
    osDelay(0.00001f); // 极短延时

    can1.send(rm_motor_2.tx_id);
    osDelay(0.00001f); // 极短延时
    can1.send(rm_motor_3.tx_id);
    osDelay(0.00001f); // 极短延时
    can1.send(rm_motor_4.tx_id);
    osDelay(0.00001f); // 极短延时
}



void enable_rm_motors() {
    // 所有电机发送零扭矩（等效使能）
    rm_motor_1.cmd(0.0f);
    rm_motor_2.cmd(0.0f);
    rm_motor_3.cmd(0.0f);
    rm_motor_4.cmd(0.0f);
}

