/**
 * @file buzzer_task.cpp
 * @brief 蜂鸣器控制任务实现文件
 * @author YuZixuan
 * @date 2024-09-28
 */
#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// 蜂鸣器硬件配置：使用TIM4通道3，84MHz时钟
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 音调持续时间
#define QUARTER_NOTE_MS    200   // 四分音符时长(ms)
#define EIGHTH_NOTE_MS     100   // 八分音符时长(ms)

void playNote(uint32_t freq, float duty, uint32_t duration) {
    buzzer.set(freq, duty);
    buzzer.start();
    osDelay(duration);
    buzzer.stop();
    osDelay(20); // 音符间短暂间隔
}

void playVehicleIdSong() {
    // 车辆识别歌曲 - 欢快的启动旋律
    playNote(262, 0.2, QUARTER_NOTE_MS);  // C4
    playNote(330, 0.2, QUARTER_NOTE_MS);  // E4
    playNote(392, 0.2, QUARTER_NOTE_MS);  // G4
    playNote(523, 0.3, QUARTER_NOTE_MS*2);// C5 (二分音符)
    osDelay(QUARTER_NOTE_MS);
}

void playErrorSong1() {
    // 错误1歌曲 - 下降的悲伤旋律
    playNote(392, 0.15, EIGHTH_NOTE_MS);  // G4
    playNote(349, 0.15, EIGHTH_NOTE_MS);  // F4
    playNote(330, 0.15, EIGHTH_NOTE_MS);  // E4
    playNote(294, 0.15, EIGHTH_NOTE_MS);  // D4
    playNote(262, 0.2, QUARTER_NOTE_MS);  // C4
    osDelay(QUARTER_NOTE_MS);
}

void playErrorSong2() {
    // 错误2歌曲 - 急促的警告旋律
    for(int i=0; i<3; i++) {
        playNote(784, 0.25, EIGHTH_NOTE_MS/2);  // G5 (十六分音符)
        playNote(659, 0.25, EIGHTH_NOTE_MS/2);  // E5
    }
    playNote(523, 0.3, QUARTER_NOTE_MS);  // C5
    playNote(392, 0.3, QUARTER_NOTE_MS);  // G4
    osDelay(QUARTER_NOTE_MS);
}

void playSuccessSong() {
    // 成功歌曲 - 上升的欢快旋律
    playNote(330, 0.2, EIGHTH_NOTE_MS);  // E4
    playNote(392, 0.2, EIGHTH_NOTE_MS);  // G4
    playNote(523, 0.25, QUARTER_NOTE_MS);// C5
    playNote(659, 0.3, QUARTER_NOTE_MS); // E5
    osDelay(QUARTER_NOTE_MS);
}

extern "C" void buzzer_task()
{
    // 上电后播放车辆识别歌曲
    playVehicleIdSong();
    
    // 示例：演示各种歌曲（实际使用时根据需要调用）
    osDelay(1000);
    playErrorSong1();  // 出现错误1
    osDelay(1000);
    playErrorSong2();  // 出现错误2
    osDelay(1000);
    playSuccessSong(); // 操作成功

    while (true) {
        // 主任务循环
        osDelay(100);
    }
}
