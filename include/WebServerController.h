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
#include "RobotArmFreeRTOS.h"
#include "RobotArmWebPage.h"  // 引入网页HTML文件

// WiFi配置
#define WIFI_SSID "RobotArm"
#define WIFI_PASSWORD "12345678"
#define WIFI_CHANNEL 1
#define MAX_CLIENTS 4

// Web服务器端口
#define WEB_SERVER_PORT 80

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

private:
    // Web服务器实例
    AsyncWebServer *webServer;
    
    // 处理主页请求的函数
    static void handleRoot(AsyncWebServerRequest *request);
    
    // 处理API请求的函数
    static void handleGetServos(AsyncWebServerRequest *request);
    static void handleHomePosition(AsyncWebServerRequest *request);
    static void handleUnloadServos(AsyncWebServerRequest *request);
};

// HTML页面声明已移至RobotArmWebPage.h

#endif // WEB_SERVER_CONTROLLER_H