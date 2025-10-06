#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "can/can.hpp"
#include "motor/motor.hpp"
#include "uart/uart.hpp"

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
  while (true) {
    plotter.plot(
      rm_motor_1.speed,
      target_wheel_speeds[0],          // 电机1目标速度
      rm_motor_2.speed,
      rm_motor_3.speed,
      rm_motor_4.speed,
     
    
      //lk_motor_x.angle,
      //lk_motor_x.speed,
      //lk_motor_x.torque,

      // rm_motor_x.angle,
      // rm_motor_x.speed,
      // rm_motor_x.torque,

      
      
    );

    osDelay(10);  // 100Hz
  }
}
