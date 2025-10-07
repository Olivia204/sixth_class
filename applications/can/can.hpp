#ifndef CAN_HPP
#define CAN_HPP

#include "io/can/can.hpp"

extern float current_power;
extern float total_chassis_power;

inline sp::CAN can1(&hcan1);
inline sp::CAN can2(&hcan2);

void enable_rm_motor();

void send_rm_motor();
//void send_rm_motor();
//void send_lk_motor();

#endif  // CAN_HPP