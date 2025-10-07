#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

constexpr uint8_t LED_POSITION_COUNT = 3;  // 改为3个LED位置
constexpr uint16_t FLOW_SPEED_MS = 150;    // 适当降低流水速度

// 定义3个位置的颜色（红、绿、蓝循环）
const uint32_t POSITION_COLORS[LED_POSITION_COUNT] = {
    0xFF0000,  // 位置1：红色
    0x00FF00,  // 位置2：绿色
    0x0000FF   // 位置3：蓝色
};

extern "C" void led_task()
{
    led.start();

    uint8_t current_position = 0;
    
    while (1) {
        // 熄灭所有LED
        led.set(0, 0, 0);
        
        // 点亮当前位置LED
        switch (current_position) {
            case 0: 
                led.set(255, 0, 0);  // 红色
                break;
            case 1:
                led.set(0, 255, 0);  // 绿色
                break;
            case 2:
                led.set(0, 0, 255);  // 蓝色
                break;
        }
        
        // 移动到下一个位置
        current_position = (current_position + 1) % LED_POSITION_COUNT;
        
        osDelay(FLOW_SPEED_MS);
    }
}
