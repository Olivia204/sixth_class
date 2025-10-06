#include "cmsis_os.h"
#include "can.hpp"
#include "motor/motor.hpp"

void can1_recv(uint32_t stamp_ms)
{
    can1.recv();
    if (can1.rx_id == rm_motor_1.rx_id)
        rm_motor_1.read(can1.rx_data, stamp_ms);
    if (can1.rx_id == rm_motor_2.rx_id)
        rm_motor_2.read(can1.rx_data, stamp_ms);
    if (can1.rx_id == rm_motor_3.rx_id)
        rm_motor_3.read(can1.rx_data, stamp_ms);
    if (can1.rx_id == rm_motor_4.rx_id)
        rm_motor_4.read(can1.rx_data, stamp_ms);
   // if (can1.rx_id == rm_motor_x.rx_id)
       // rm_motor_x.read(can1.rx_data, stamp_ms);

   // if (can1.rx_id == lk_motor_x.rx_id)
      //  lk_motor_x.read(can1.rx_data);
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