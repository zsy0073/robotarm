/******************************************************
* FileName:      TempSensorDisplay.h
* Date:          2025/05/08
* Description:   温度传感器显示功能的头文件，用于MLX90614和OLED显示屏
*****************************************************/

#ifndef TEMP_SENSOR_DISPLAY_H
#define TEMP_SENSOR_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 温度传感器和显示屏引脚定义
#define TEMP_SDA_PIN           13    // MLX90614和OLED的SDA引脚
#define TEMP_SCL_PIN           14    // MLX90614和OLED的SCL引脚
#define OLED_ADDRESS           0x3C  // OLED的I2C地址
#define MLX90614_ADDRESS       0x5A  // MLX90614的I2C地址

// 任务优先级与栈大小
#define TEMP_DISPLAY_PRIORITY  (1)   // 温度显示任务优先级
#define TEMP_DISPLAY_STACK_SIZE (4096) // 温度显示任务栈大小

// 全局任务句柄声明
extern TaskHandle_t tempDisplayTaskHandle;

// 函数声明
void initTempSensorDisplay();
void tempDisplayTask(void *pvParameters);
bool getTempSensorStatus();
float getObjectTemperature();
float getAmbientTemperature();

#endif // TEMP_SENSOR_DISPLAY_H