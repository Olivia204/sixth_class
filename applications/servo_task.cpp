/**
 * @file servo_task.cpp
 * @brief 舵机控制任务，实现上电后1秒内从0度到180度的转动
 */

#include "cmsis_os.h"
#include "main.h"
#include "io/pwm/pwm_driver.h"
#include "utils/timer.h"

/**
 * @class ServoController
 * @brief 舵机控制类，封装PWM控制逻辑
 */
class ServoController {
public:
  /**
   * @brief 构造函数
   * @param pwm_tim PWM定时器句柄
   * @param pwm_channel PWM通道
   */
  ServoController(TIM_HandleTypeDef* pwm_tim, uint32_t pwm_channel)
    : pwm_tim_(pwm_tim), pwm_channel_(pwm_channel) {}

  /**
   * @brief 初始化舵机控制器
   * @return 初始化是否成功
   */
  bool initialize() {
    if (pwm_tim_ == nullptr) {
      return false;
    }
    // 启动PWM输出
    HAL_TIM_PWM_Start(pwm_tim_, pwm_channel_);
    // 初始位置设为0度
    set_angle(45.0f);
    return true;
  }

  /**
   * @brief 设置舵机角度
   * @param angle_deg 目标角度(0-180度)
   */
  void set_angle(float angle_deg) {
    // 限制角度范围
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;
    
    // 将角度转换为PWM占空比
    // 舵机标准: 0.5ms (0°) to 2.5ms (180°) @ 50Hz (20ms周期)
    constexpr float min_pulse_ms = 0.5f;
    constexpr float max_pulse_ms = 2.5f;
    constexpr float period_ms = 20.0f;
    
    float pulse_width_ms = min_pulse_ms + (angle_deg / 180.0f) * (max_pulse_ms - min_pulse_ms);
    float duty_cycle = (pulse_width_ms / period_ms) * 100.0f;
    
    // 设置PWM输出
    pwm_set_duty_cycle(pwm_tim_, pwm_channel_, duty_cycle);
    
    current_angle_deg_ = angle_deg;
  }

  /**
   * @brief 获取当前角度
   * @return 当前角度(度)
   */
  float get_current_angle() const {
    return current_angle_deg_;
  }

private:
  TIM_HandleTypeDef* pwm_tim_;      // PWM定时器
  uint32_t pwm_channel_;            // PWM通道
  float current_angle_deg_{0.0f};   // 当前角度
};

// 全局舵机控制器对象
ServoController g_servo(&htim2, TIM_CHANNEL_1);

/**
 * @brief 舵机控制任务
 * @param argument FreeRTOS任务参数
 */
extern "C" void servo_task(void* argument) {
  // 初始化舵机控制器
  if (!g_servo.initialize()) {
    // 初始化失败，挂起任务
    osThreadSuspend(nullptr);
    return;
  }

  // 上电后等待系统稳定
  osDelay(100);
  
  constexpr float target_angle_deg = 180.0f;  // 目标角度180度
  constexpr uint32_t move_duration_ms = 1000; // 转动时间1秒
  constexpr uint32_t update_interval_ms = 10;  // 更新间隔10ms (100Hz)
  
  constexpr uint32_t total_steps = move_duration_ms / update_interval_ms;
  constexpr float angle_step = target_angle_deg / total_steps;
  
  uint32_t step_count = 0;
  
  // 主控制循环
  while (true) {
    if (step_count < total_steps) {
      // 计算当前目标角度（线性插值）
      float target_angle = step_count * angle_step;
      
      // 设置舵机角度
      g_servo.set_angle(target_angle);
      
      step_count++;
    } else {
      // 到达目标角度，保持180度位置
      g_servo.set_angle(target_angle_deg);
    }
    
    // 固定频率更新
    osDelay(update_interval_ms);
  }
}
