/******************************************************
* FileName:      WebServerController.h
* Date:          2025/05/05
* Description:   机械臂Web服务器控制器的头文件
*****************************************************/

#ifndef WEB_SERVER_CONTROLLER_H
#define WEB_SERVER_CONTROLLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>  // 添加AsyncJson头文件，提供AsyncCallbackJsonWebHandler支持
#include <SPIFFS.h>     // 添加SPIFFS头文件，用于存储预设位置
#include "RobotArmFreeRTOS.h"
#include "RobotArmWebPage.h"  // 引入网页HTML文件
#include "RobotArmController.h"  // 添加对机械臂控制器的引用

// WiFi配置
#define WIFI_SSID "RobotArm"
#define WIFI_PASSWORD "12345678"
#define WIFI_CHANNEL 1
#define MAX_CLIENTS 4

// Web服务器端口
#define WEB_SERVER_PORT 80

// 预设位置相关定义
#define MAX_PRESETS 5
#define PRESET_FILE "/presets.json"

// Web服务器任务函数
void webServerTask(void *pvParameters);

// Web服务器控制器类
class WebServerController {
public:
    // 构造函数
    WebServerController();
    
    // 初始化函数
    bool init();
    
    // 启动Web服务器
    void start();
    
    // 处理API请求
    void setupRoutes();
    
    // 设置机械臂控制器
    void setRobotArmController(RobotArmController* controller);
    
    // 检查机械臂控制器是否已连接
    static bool isArmControllerConnected();

private:
    // Web服务器实例
    AsyncWebServer *webServer;
    
    // 机械臂控制器实例指针
    static RobotArmController* armController;
    
    // 迭代控制相关变量
    static bool isIterationEnabled;      // 是否启用迭代
    static uint8_t iterationCount;       // 当前迭代次数
    static uint8_t maxIterationCount;    // 最大迭代次数
    
    // 处理主页请求的函数
    static void handleRoot(AsyncWebServerRequest *request);
    
    // 处理API请求的函数
    static void handleGetServos(AsyncWebServerRequest *request);
    static void handleHomePosition(AsyncWebServerRequest *request);
    static void handleUnloadServos(AsyncWebServerRequest *request);
    
    // 迭代控制相关函数
    static void handleStartIteration(AsyncWebServerRequest *request);
    static void handleContinueIteration(AsyncWebServerRequest *request);
    static void handleStopIteration(AsyncWebServerRequest *request);
    static bool continueIteration();     // 进行下一次迭代，返回是否仍在迭代中
    static void resetIteration();        // 重置迭代状态
    
    // 预设位置相关函数
    static void handleSavePreset(AsyncWebServerRequest *request);
    static void handleLoadPreset(AsyncWebServerRequest *request);
    static void handleGetPresets(AsyncWebServerRequest *request);
    
    // 预设位置文件操作
    static bool initPresetStorage();
    static bool savePresetToStorage(const char* name, uint16_t positions[]);
    static bool loadPresetFromStorage(const char* name, uint16_t positions[]);
    static void getPresetsList(JsonArray& presetsArray);
    
    // 辅助函数 - 用于约束舵机位置
    static uint16_t constrainServoPosition(uint8_t servoId, uint16_t position);
    
    // 辅助函数 - 发送舵机控制命令（仅用于掉电所有舵机）
    static void sendServoCommand(ServoCommandType type);
    
    // 辅助函数 - 发送舵机组控制命令
    static void sendServoGroupCommand(ServoCommandType type, uint8_t servoCount, LobotServo* servos, uint16_t time = 0);
    
    // 辅助函数 - 获取当前舵机位置
    static uint16_t getCurrentServoPosition(uint8_t servoId);
    
    // 辅助函数 - 移动到初始位置
    static void moveToHomePosition();
};

// HTML页面声明已移至RobotArmWebPage.h

#endif // WEB_SERVER_CONTROLLER_H