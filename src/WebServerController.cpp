/******************************************************
* FileName:      WebServerController.cpp
* Date:          2025/05/05
* Description:   机械臂Web服务器控制器的实现文件
*****************************************************/

#include "WebServerController.h"
#include "RobotArmWebPage.h"

// 全局Web服务器实例
AsyncWebServer webServer(WEB_SERVER_PORT);

// Web服务器控制器类实现
WebServerController::WebServerController() {
    webServer = &::webServer;
}

// 初始化函数
bool WebServerController::init() {
    // 配置WiFi - 接入点模式
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, 0, MAX_CLIENTS);
    
    Serial.println("WiFi接入点已启动");
    Serial.print("SSID: ");
    Serial.println(WIFI_SSID);
    Serial.print("密码: ");
    Serial.println(WIFI_PASSWORD);
    Serial.print("IP地址: ");
    Serial.println(WiFi.softAPIP());
    
    return true;
}

// 启动Web服务器
void WebServerController::start() {
    // 设置路由
    setupRoutes();
    
    // 启动Web服务器
    webServer->begin();
    Serial.println("Web服务器已启动");
}

// 设置处理路由
void WebServerController::setupRoutes() {
    // 路由 - 主页
    webServer->on("/", HTTP_GET, handleRoot);
    
    // API - 获取所有舵机位置
    webServer->on("/api/servos", HTTP_GET, handleGetServos);

    // API - 回到初始位置
    webServer->on("/api/home", HTTP_POST, handleHomePosition);

    // API - 掉电所有舵机
    webServer->on("/api/unload", HTTP_POST, handleUnloadServos);

    // API - 设置单个舵机位置
    AsyncCallbackJsonWebHandler* setServoHandler = new AsyncCallbackJsonWebHandler("/api/servo", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        if (jsonObj.containsKey("id") && jsonObj.containsKey("position") && jsonObj.containsKey("time")) {
            int servoId = jsonObj["id"];
            int position = jsonObj["position"];
            int time = jsonObj["time"];
            
            // 约束位置在有效范围内
            position = constrainServoPosition(servoId, position);
            
            // 发送舵机控制命令
            sendServoCommand(MOVE_SINGLE_SERVO, servoId, position, time);
            
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Missing required parameters");
        }
    });
    webServer->addHandler(setServoHandler);
}

// 处理主页请求
void WebServerController::handleRoot(AsyncWebServerRequest *request) {
    // HTML页面内容已移至RobotArmWebPage.h
    request->send_P(200, "text/html", index_html);
}

// 处理获取所有舵机位置请求
void WebServerController::handleGetServos(AsyncWebServerRequest *request) {
    String response = "{\"positions\":[";
    for (int i = 0; i < SERVO_COUNT; i++) {
        response += String(getCurrentServoPosition(i + 1));
        if (i < SERVO_COUNT - 1) {
            response += ",";
        }
    }
    response += "]}";
    request->send(200, "application/json", response);
}

// 处理回到初始位置请求
void WebServerController::handleHomePosition(AsyncWebServerRequest *request) {
    moveToHomePosition();
    request->send(200, "text/plain", "OK");
}

// 处理掉电所有舵机请求
void WebServerController::handleUnloadServos(AsyncWebServerRequest *request) {
    sendServoCommand(UNLOAD_SERVOS);
    request->send(200, "text/plain", "OK");
}

// Web服务器任务实现
void webServerTask(void *pvParameters) {
    Serial.println("Web服务器任务启动...");
    
    // 创建并初始化Web服务器控制器
    WebServerController webServerController;
    webServerController.init();
    webServerController.start();
    
    // 任务保持运行
    for (;;) {
        // Web服务器在后台运行，任务只需阻塞
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}