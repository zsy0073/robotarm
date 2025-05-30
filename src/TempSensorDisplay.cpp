/******************************************************
* FileName:      TempSensorDisplay.cpp
* Date:          2025/05/08
* Description:   温度传感器显示功能的实现，用于MLX90614和OLED显示屏
*****************************************************/

#include "TempSensorDisplay.h"
#include <Adafruit_MLX90614.h>  // MLX90614温度传感器库
#include <Adafruit_SSD1306.h>   // SSD1306 OLED显示库

// 全局变量
TaskHandle_t tempDisplayTaskHandle = NULL;
static Adafruit_MLX90614 *mlx = NULL;
static Adafruit_SSD1306 *display = NULL;
static bool sensorInitialized = false;
static float currentObjectTemp = 0.0f;
static float currentAmbientTemp = 0.0f;

// OLED显示屏尺寸定义
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// 温度显示任务 - 负责从MLX90614读取温度并在OLED上显示
void tempDisplayTask(void *pvParameters) {
    Serial.println("温度显示任务启动");
    
    // 创建OLED显示对象
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
    
    // 创建MLX90614温度传感器对象
    mlx = new Adafruit_MLX90614();
    
    // 初始化I2C总线，使用指定引脚
    Wire.begin(TEMP_SDA_PIN, TEMP_SCL_PIN);
    
    // 初始化温度传感器
    if (!mlx->begin()) {
        Serial.println("无法找到MLX90614传感器。请检查接线。");
        sensorInitialized = false;
        // 即使找不到传感器，也继续尝试显示
    } else {
        sensorInitialized = true;
        Serial.println("MLX90614温度传感器初始化成功");
    }
    
    // 初始化OLED显示屏
    if(!display->begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("无法找到SSD1306 OLED显示屏。请检查接线。");
        vTaskDelete(NULL); // 如果无法初始化显示屏，则结束任务
        return;
    } else {
        Serial.println("SSD1306 OLED显示屏初始化成功");
    }
    
    // 显示器初始化设置
    display->setTextSize(1);                // 设置字体大小
    display->setTextColor(SSD1306_WHITE);   // 设置文字颜色
    display->clearDisplay();                // 清除显示缓存
    
    // 显示启动信息    display->setCursor(0, 0);
    display->println("ROBOT ARM TEMP MONITOR");
    display->println("Initializing...");
    display->display();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    for (;;) {
        // 读取温度数据
        if (sensorInitialized) {
            currentObjectTemp = mlx->readObjectTempC();      // 目标物体温度(摄氏度)
            currentAmbientTemp = mlx->readAmbientTempC();    // 环境温度(摄氏度)
        } else {
            // 如果传感器未初始化成功，尝试重新初始化
            if (mlx->begin()) {
                sensorInitialized = true;
                Serial.println("MLX90614温度传感器初始化成功");
            }
        }
        
        // 清除显示缓存
        display->clearDisplay();
        
        // 设置显示位置
        display->setCursor(0, 0);
        
        // 显示标题
        display->println("ROBOT ARM TEMP MONITOR");
        display->println("--------------------");
        
        // 显示传感器状态
        if (!sensorInitialized) {
            display->println("Sensor: NOT CONNECTED");
            display->println("Check MLX90614 wiring");
        } else {
            // 显示环境温度
            display->print("Ambient: ");
            display->print(currentAmbientTemp, 1); // 保留1位小数
            display->println(" C");
            
            // 显示物体温度
            display->print("Object: ");
            display->print(currentObjectTemp, 1);  // 保留1位小数
            display->println(" C");
            
            // 显示温度状态
            display->println();
            if (currentObjectTemp > 40) {
                display->println("WARNING: TOO HOT!");
            } else if (currentObjectTemp < 10) {
                display->println("WARNING: TOO COLD!");
            } else {
                display->println("Status: NORMAL");
            }
        }
        
        // 显示时间戳
        display->println();
        display->print("Runtime: ");
        display->print(millis() / 1000);
        display->println("s");
        
        // 发送数据到显示缓存
        display->display();
        
        // 每秒更新一次温度
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 获取温度传感器状态
bool getTempSensorStatus() {
    return sensorInitialized;
}

// 获取当前物体温度
float getObjectTemperature() {
    return currentObjectTemp;
}

// 获取当前环境温度
float getAmbientTemperature() {
    return currentAmbientTemp;
}