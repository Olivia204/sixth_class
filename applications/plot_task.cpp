#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "can/can.hpp"
#include "motor/motor.hpp"
#include "uart/uart.hpp"
#include <cstdint>  
#include "motor/rm_motor/rm_motor.hpp"

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
    // 等待初始化完成
    osDelay(1000);
    
    
    
    while (true) {
        // 确保所有变量都已正确更新
        plotter.plot(
           
            rm_motor_1.speed,
            rm_motor_1.angle
        );
        
        osDelay(10);  // 100Hz更新率
    }
}
