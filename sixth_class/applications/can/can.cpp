#include "cmsis_os.h"
#include "can.hpp"
#include "motor/motor.hpp"

void enable_all_motors() {
    sp::RM_Motor* motors[] = {&rm_motor_1, &rm_motor_2, &rm_motor_3, &rm_motor_4};    
    for (int i = 0; i < 4; i++) {
        motors[i]->write_clear_error(tx_data);
        can1.send(motors[i]->tx_id, tx_data);
        osDelay(1);
        motors[i]->write_enable(tx_data);
        can1.send(motors[i]->tx_id, tx_data);
        osDelay(1);
    }
}
void enable_rm_motor() {
    uint8_t tx_data[8];
    rm_motor_1.write_enable(tx_data);
    can1.send(rm_motor_1.tx_id, tx_data);
    osDelay(1);
    rm_motor_2.write_enable(tx_data);
    can1.send(rm_motor_2.tx_id, tx_data);
    osDelay(1);
    rm_motor_3.write_enable(tx_data);
    can1.send(rm_motor_3.tx_id, tx_data);
    osDelay(1);
    rm_motor_4.write_enable(tx_data);
    can1.send(rm_motor_4.tx_id, tx_data);
    osDelay(1);
}
void send_rm_motor() {
    unit_8_t tx_data[8];
	rm_motor_1.write(can1.tx_data);
    can1.send(rm_motor_1.tx_id);
    rm_motor_2.write(can1.tx_data);
    can1.send(rm_motor_2.tx_id);
    rm_motor_3.write(can1.tx_data);
    can1.send(rm_motor_3.tx_id);
    rm_motor_4.write(can1.tx_data);
    can1.send(rm_motor_4.tx_id);

}

//void send_rm_motor() {
  //  rm_motor_x.write(can1.tx_data);
    //can1.send(rm_motor_x.tx_id);
//}

//void send_lk_motor() {
  //  lk_motor_x.write(can1.tx_data);
    //can1.send(lk_motor_x.tx_id);
//}