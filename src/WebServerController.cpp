/******************************************************
* FileName:      WebServerController.cpp
* Date:          2025/05/05
* Description:   机械臂Web服务器控制器的实现文件
*****************************************************/

#include "WebServerController.h"
#include "RobotArmWebPage.h"
#include "RobotArmConfig.h"  // 添加RobotArmConfig.h头文件引用

// 全局Web服务器实例
AsyncWebServer webServer(WEB_SERVER_PORT);

// 定义静态成员变量
RobotArmController* WebServerController::armController = nullptr;
bool WebServerController::isIterationEnabled = false;
uint8_t WebServerController::iterationCount = 0;
uint8_t WebServerController::maxIterationCount = 5;

// 检查机械臂控制器是否已连接
bool WebServerController::isArmControllerConnected() {
    return armController != nullptr;
}

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

// 设置机械臂控制器
void WebServerController::setRobotArmController(RobotArmController* controller) {
    armController = controller;
    if (controller != nullptr) {
        Serial.println("机械臂控制器已成功连接到Web服务器控制器");
    } else {
        Serial.println("错误：尝试设置空的机械臂控制器");
    }
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

    // API - 设置舵机组位置
    AsyncCallbackJsonWebHandler* setServoHandler = new AsyncCallbackJsonWebHandler("/api/servo", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        if (jsonObj.containsKey("servos") && jsonObj.containsKey("time")) {
            JsonArray servosArray = jsonObj["servos"].as<JsonArray>();
            int time = jsonObj["time"];
            
            // 调试信息
            Serial.print("收到API请求：舵机组命令，时间=");
            Serial.println(time);
            
            // 准备舵机组命令
            uint8_t servoCount = min((int)servosArray.size(), (int)SERVO_COUNT);
            
            LobotServo servos[SERVO_COUNT];
            
            for (uint8_t i = 0; i < servoCount; i++) {
                JsonObject servoObj = servosArray[i];
                if (servoObj.containsKey("id") && servoObj.containsKey("position")) {
                    uint8_t id = servoObj["id"];
                    uint16_t position = servoObj["position"];
                    
                    // 约束位置在有效范围内
                    position = constrainServoPosition(id, position);
                    
                    servos[i].ID = id;
                    servos[i].Position = position;
                    
                    Serial.print("舵机ID=");
                    Serial.print(id);
                    Serial.print("，目标位置=");
                    Serial.println(position);
                }
            }
            
            // 发送舵机组控制命令
            sendServoGroupCommand(MOVE_ALL_SERVOS, servoCount, servos, time);
            
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Missing required parameters");
        }
    });
    webServer->addHandler(setServoHandler);

    // 添加调试API - 检查控制器连接状态
    webServer->on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (WebServerController::isArmControllerConnected()) {
            request->send(200, "text/plain", "控制器已连接");
        } else {
            request->send(503, "text/plain", "控制器未连接");
        }
    });
    
    // API - 预设位置相关路由
    webServer->on("/api/presets", HTTP_GET, handleGetPresets);
    
    // API - 保存预设位置
    AsyncCallbackJsonWebHandler* savePresetHandler = new AsyncCallbackJsonWebHandler("/api/preset/save", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        if (jsonObj.containsKey("name") && jsonObj.containsKey("positions")) {
            String name = jsonObj["name"].as<String>();
            JsonArray positionsArray = jsonObj["positions"].as<JsonArray>();
            
            uint16_t positions[SERVO_COUNT];
            for (int i = 0; i < SERVO_COUNT && i < positionsArray.size(); i++) {
                positions[i] = positionsArray[i].as<uint16_t>();
            }
            
            bool success = savePresetToStorage(name.c_str(), positions);
            if (success) {
                request->send(200, "text/plain", "Preset saved");
            } else {
                request->send(500, "text/plain", "Failed to save preset");
            }
        } else {
            request->send(400, "text/plain", "Missing required parameters");
        }
    });
    webServer->addHandler(savePresetHandler);
    
    // API - 加载预设位置
    AsyncCallbackJsonWebHandler* loadPresetHandler = new AsyncCallbackJsonWebHandler("/api/preset/load", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        if (jsonObj.containsKey("name")) {
            String name = jsonObj["name"].as<String>();
            uint16_t positions[SERVO_COUNT];
            
            bool success = loadPresetFromStorage(name.c_str(), positions);
            if (success) {
                // 使用加载的位置控制机械臂
                LobotServo servos[SERVO_COUNT];
                for (uint8_t i = 0; i < SERVO_COUNT; i++) {
                    servos[i].ID = i + 1;
                    servos[i].Position = positions[i];
                }
                sendServoGroupCommand(MOVE_ALL_SERVOS, SERVO_COUNT, servos, 1000); // 1秒移动到位
                request->send(200, "text/plain", "Preset loaded");
            } else {
                request->send(404, "text/plain", "Preset not found");
            }
        } else {
            request->send(400, "text/plain", "Missing preset name");
        }
    });
    webServer->addHandler(loadPresetHandler);
    
    // 添加录制和回放相关API
    
    // API - 开始录制
    AsyncCallbackJsonWebHandler* startRecordingHandler = new AsyncCallbackJsonWebHandler("/api/record/start", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        String fileName = "/record_default.json"; // 默认文件名
        
        if (jsonObj.containsKey("fileName")) {
            fileName = jsonObj["fileName"].as<String>();
        }
        
        bool success = startRecording(fileName.c_str());
        if (success) {
            request->send(200, "text/plain", "Recording started");
        } else {
            request->send(500, "text/plain", "Failed to start recording");
        }
    });
    webServer->addHandler(startRecordingHandler);
    
    // API - 停止录制
    webServer->on("/api/record/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
        bool success = stopRecording();
        
        DynamicJsonDocument doc(128);
        doc["success"] = success;
        doc["frameCount"] = getRecordFrameCount();
        
        String response;
        serializeJson(doc, response);
        
        request->send(success ? 200 : 500, "application/json", response);
    });
    
    // API - 获取录制文件列表
    webServer->on("/api/record/files", HTTP_GET, [](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(4096);
        JsonArray filesArray = doc.createNestedArray("files");
        
        listRecordFiles(filesArray);
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    // API - 播放录制文件
    AsyncCallbackJsonWebHandler* playRecordingHandler = new AsyncCallbackJsonWebHandler("/api/record/play", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        
        if (jsonObj.containsKey("fileName")) {
            String fileName = jsonObj["fileName"].as<String>();
            bool success = startPlayback(fileName.c_str());
            
            if (success) {
                request->send(200, "text/plain", "Playback started");
            } else {
                request->send(500, "text/plain", "Failed to start playback");
            }
        } else {
            request->send(400, "text/plain", "Missing fileName parameter");
        }
    });
    webServer->addHandler(playRecordingHandler);
    
    // API - 停止回放
    webServer->on("/api/record/stop-play", HTTP_POST, [](AsyncWebServerRequest *request) {
        stopPlayback();
        request->send(200, "text/plain", "Playback stopped");
    });
    
    // API - 删除录制文件
    AsyncCallbackJsonWebHandler* deleteRecordingHandler = new AsyncCallbackJsonWebHandler("/api/record/delete", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        
        if (jsonObj.containsKey("fileName")) {
            String fileName = jsonObj["fileName"].as<String>();
            bool success = deleteRecordFile(fileName.c_str());
            
            if (success) {
                request->send(200, "text/plain", "Recording file deleted");
            } else {
                request->send(500, "text/plain", "Failed to delete recording file");
            }
        } else {
            request->send(400, "text/plain", "Missing fileName parameter");
        }
    });
    webServer->addHandler(deleteRecordingHandler);
    
    // API - 获取录制器状态
    webServer->on("/api/record/status", HTTP_GET, handleGetRecorderStatus);

    // API - 获取录制的命令
    webServer->on("/api/record/commands", HTTP_GET, handleGetRecordedCommands);

    // API - 删除预设
    AsyncCallbackJsonWebHandler* deletePresetHandler = new AsyncCallbackJsonWebHandler("/api/preset/delete", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        if (jsonObj.containsKey("id")) {
            String presetId = jsonObj["id"].as<String>();
            bool success = deletePreset(presetId.c_str());
            
            if (success) {
                request->send(200, "text/plain", "Preset deleted");
            } else {
                request->send(404, "text/plain", "Preset not found");
            }
        } else {
            request->send(400, "text/plain", "Missing preset id");
        }
    });
    webServer->addHandler(deletePresetHandler);
}

// 处理主页请求
void WebServerController::handleRoot(AsyncWebServerRequest *request) {
    // HTML页面内容已移至RobotArmWebPage.h
    request->send_P(200, "text/html; charset=UTF-8", index_html);
}

// 处理获取所有舵机位置请求
void WebServerController::handleGetServos(AsyncWebServerRequest *request) {
    String response = "{\"positions\":[";
    
    if (armController != nullptr) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            response += String(getCurrentServoPosition(i + 1));
            if (i < SERVO_COUNT - 1) {
                response += ",";
            }
        }
    } else {
        // 控制器未连接，返回默认值
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (i == 0) response += "500";
            else if (i == 1) response += "500";
            else if (i == 2) response += "500";
            else if (i == 3) response += "500";
            else if (i == 4) response += "500";
            else if (i == 5) response += "500";
            else if (i == 6) response += "500";
            else if (i == 7) response += "0";
            
            if (i < SERVO_COUNT - 1) {
                response += ",";
            }
        }
        Serial.println("警告：机械臂控制器未连接，返回默认位置值");
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

// 处理获取预设列表请求
void WebServerController::handleGetPresets(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    JsonArray presetsArray = doc.createNestedArray("presets");
    
    getPresetsList(presetsArray);
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// 约束舵机位置在有效范围内
uint16_t WebServerController::constrainServoPosition(uint8_t servoId, uint16_t position) {
    // 确保位置在0-1000范围内
    uint16_t constrainedPosition = constrain(position, 0, 1000);
    
    if (armController != nullptr) {
        // 如果控制器已连接，可以使用控制器的约束逻辑
        // 这里简单返回约束后的值
        return constrainedPosition;
    } else {
        Serial.println("警告：机械臂控制器未连接，使用默认约束逻辑");
        return constrainedPosition;
    }
}

// 发送舵机控制命令 - 处理UNLOAD_SERVOS类型命令
void WebServerController::sendServoCommand(ServoCommandType type) {
    if (WebServerController::armController != nullptr) {
        // 处理掉电指令
        if (type == UNLOAD_SERVOS) {
            Serial.println("发送掉电指令：所有舵机掉电");
            // 使用FreeRTOS队列发送命令
            ServoCommand_t command;
            command.type = UNLOAD_SERVOS;
            
            // 通过队列发送掉电命令
            extern QueueHandle_t servoCommandQueue;
            if (servoCommandQueue != NULL) {
                xQueueSend(servoCommandQueue, &command, portMAX_DELAY);
                Serial.println("掉电命令已发送到队列");
            } else {
                Serial.println("错误：命令队列不可用");
            }
        } 
    } else {
        Serial.println("错误：机械臂控制器未连接，无法发送舵机控制命令");
    }
}

// 发送舵机组控制命令
void WebServerController::sendServoGroupCommand(ServoCommandType type, uint8_t servoCount, LobotServo* servos, uint16_t time) {
    if (WebServerController::armController != nullptr) {
        // 根据命令类型执行不同操作
        switch (type) {
            case MOVE_ALL_SERVOS:
                // 使用外部函数通过FreeRTOS队列发送命令，确保与PS2控制器使用同样的执行路径
                moveAllServos(servos, servoCount, time);
                
                // 与PS2控制器输出格式保持一致
                Serial.print("舵机组命令：[");
                // 输出角度值数组
                for (int i = 0; i < SERVO_COUNT; i++) {
                    // 查找当前舵机ID是否在命令中
                    bool found = false;
                    uint16_t position = (armController != nullptr) ? armController->getCurrentPositions()[i] : 500;
                    for (uint8_t j = 0; j < servoCount; j++) {
                        if (servos[j].ID == i + 1) {
                            position = servos[j].Position;
                            found = true;
                            break;
                        }
                    }
                    Serial.print(PULSE_TO_DEGREE(position), 1); // 保留1位小数
                    if (i < SERVO_COUNT - 1) Serial.print("、");
                }
                Serial.print("][");
                // 输出脉冲值数组
                for (int i = 0; i < SERVO_COUNT; i++) {
                    // 查找当前舵机ID是否在命令中
                    bool found = false;
                    uint16_t position = (armController != nullptr) ? armController->getCurrentPositions()[i] : 500;
                    for (uint8_t j = 0; j < servoCount; j++) {
                        if (servos[j].ID == i + 1) {
                            position = servos[j].Position;
                            found = true;
                            break;
                        }
                    }
                    Serial.print(position);
                    if (i < SERVO_COUNT - 1) Serial.print("、");
                }
                Serial.println("]");
                break;
                
            default:
                Serial.println("未知的舵机命令类型");
                break;
        }
    } else {
        Serial.println("错误：机械臂控制器未连接，无法发送舵机组控制命令");
    }
}

// 获取当前舵机位置
uint16_t WebServerController::getCurrentServoPosition(uint8_t servoId) {
    if (armController != nullptr) {
        // 索引从0开始，而servoId从1开始
        if (servoId >= 1 && servoId <= SERVO_COUNT) {
            // 使用公共方法获取当前位置数组
            uint16_t position = armController->getCurrentPositions()[servoId - 1];
            return position;
        } else {
            return 500; // 返回默认中间位置
        }
    } else {
        Serial.print("警告：机械臂控制器未连接，返回舵机");
        Serial.print(servoId);
        Serial.println("的默认位置值");
        
        // 返回默认值
        if (servoId == 8) { // 夹爪
            return 0;
        } else {
            return 500; // 其他舵机中间位置
        }
    }
}

// 移动到初始位置
void WebServerController::moveToHomePosition() {
    if (armController != nullptr) {
        Serial.println("发送回到初始位置命令");
        
        // 创建回到初始位置命令 - 同时直接发送舵机组控制指令
        // 创建舵机控制数组，使用配置文件中定义的初始位置值
        LobotServo servos[SERVO_COUNT];
        servos[0].ID = BASE_SERVO_ID;
        servos[0].Position = HOME_BASE;
        
        servos[1].ID = SHOULDER_PITCH_ID;
        servos[1].Position = HOME_SHOULDER;
        
        servos[2].ID = SHOULDER_ROLL_ID;
        servos[2].Position = HOME_SHOULDER;
        
        servos[3].ID = ELBOW_SERVO_ID;
        servos[3].Position = HOME_ELBOW;
        
        servos[4].ID = WRIST_PITCH_ID;
        servos[4].Position = HOME_WRIST_PITCH;
        
        servos[5].ID = WRIST_ROLL_ID;
        servos[5].Position = HOME_WRIST_ROLL;
        
        servos[6].ID = WRIST_YAW_ID;
        servos[6].Position = HOME_WRIST_YAW;
        
        servos[7].ID = GRIPPER_SERVO_ID;
        servos[7].Position = HOME_GRIPPER;
        
        // 发送舵机组控制命令 - 直接移动所有舵机到初始位置
        sendServoGroupCommand(MOVE_ALL_SERVOS, SERVO_COUNT, servos, 1000);
        
        // 同时发送队列命令，确保所有处理路径都能接收到
        ServoCommand_t command;
        command.type = MOVE_TO_HOME;
        command.time = 1000; // 1秒内完成移动
        
        // 通过队列发送命令
        extern QueueHandle_t servoCommandQueue;
        if (servoCommandQueue != NULL) {
            // 先清空队列中的所有命令
            ServoCommand_t dummyCommand;
            while (xQueueReceive(servoCommandQueue, &dummyCommand, 0) == pdTRUE) {
                // 清空队列
            }
            // 然后将复位命令放在队列最前面
            xQueueSendToFront(servoCommandQueue, &command, portMAX_DELAY);
            Serial.println("初始位置命令已发送到队列最前面，将立即执行");
        } else {
            Serial.println("错误：命令队列不可用");
        }
    } else {
        Serial.println("错误：机械臂控制器未连接，无法回到初始位置");
    }
}

// 初始化预设存储
bool WebServerController::initPresetStorage() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS挂载失败");
        return false;
    }
    
    // 检查预设文件是否存在，如果不存在则创建一个空的预设文件
    if (!SPIFFS.exists(PRESET_FILE)) {
        File file = SPIFFS.open(PRESET_FILE, FILE_WRITE);
        if (!file) {
            Serial.println("创建预设文件失败");
            return false;
        }
        
        // 创建一个空的JSON数组
        file.println("[]");
        file.close();
    }
    
    return true;
}

// 保存预设到存储
bool WebServerController::savePresetToStorage(const char* name, uint16_t positions[]) {
    // 确保SPIFFS已初始化
    if (!SPIFFS.exists(PRESET_FILE)) {
        if (!initPresetStorage()) {
            return false;
        }
    }
    
    // 读取现有预设
    File file = SPIFFS.open(PRESET_FILE, FILE_READ);
    if (!file) {
        Serial.println("打开预设文件失败");
        return false;
    }
    
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.println("解析预设JSON失败");
        return false;
    }
    
    JsonArray presets = doc.as<JsonArray>();
    
    // 检查是否已存在同名预设
    bool found = false;
    for (JsonObject preset : presets) {
        if (preset["name"] == name) {
            // 更新现有预设
            JsonArray positionsArray = preset["positions"];
            positionsArray.clear();
            for (int i = 0; i < SERVO_COUNT; i++) {
                positionsArray.add(positions[i]);
            }
            found = true;
            break;
        }
    }
    
    // 如果不存在，创建新的预设
    if (!found) {
        JsonObject newPreset = presets.createNestedObject();
        newPreset["name"] = name;
        JsonArray positionsArray = newPreset.createNestedArray("positions");
        for (int i = 0; i < SERVO_COUNT; i++) {
            positionsArray.add(positions[i]);
        }
    }
    
    // 将更新后的预设写回文件
    file = SPIFFS.open(PRESET_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("打开预设文件失败");
        return false;
    }
    
    serializeJson(doc, file);
    file.close();
    
    Serial.print("预设 '");
    Serial.print(name);
    Serial.println("' 已保存");
    return true;
}

// 从存储加载预设
bool WebServerController::loadPresetFromStorage(const char* name, uint16_t positions[]) {
    // 确保SPIFFS已初始化
    if (!SPIFFS.exists(PRESET_FILE)) {
        if (!initPresetStorage()) {
            return false;
        }
    }
    
    // 读取预设文件
    File file = SPIFFS.open(PRESET_FILE, FILE_READ);
    if (!file) {
        Serial.println("打开预设文件失败");
        return false;
    }
    
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.println("解析预设JSON失败");
        return false;
    }
    
    // 查找预设
    JsonArray presets = doc.as<JsonArray>();
    for (JsonObject preset : presets) {
        if (preset["name"] == name) {
            JsonArray positionsArray = preset["positions"];
            for (int i = 0; i < SERVO_COUNT && i < positionsArray.size(); i++) {
                positions[i] = positionsArray[i].as<uint16_t>();
            }
            
            Serial.print("预设 '");
            Serial.print(name);
            Serial.println("' 已加载");
            return true;
        }
    }
    
    Serial.print("找不到预设 '");
    Serial.print(name);
    Serial.println("'");
    return false;
}

// 获取预设列表
void WebServerController::getPresetsList(JsonArray& presetsArray) {
    // 确保SPIFFS已初始化
    if (!SPIFFS.exists(PRESET_FILE)) {
        if (!initPresetStorage()) {
            return;
        }
    }
    
    // 读取预设文件
    File file = SPIFFS.open(PRESET_FILE, FILE_READ);
    if (!file) {
        Serial.println("打开预设文件失败");
        return;
    }
    
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.println("解析预设JSON失败");
        return;
    }
    
    // 返回名称和ID列表
    JsonArray filePresets = doc.as<JsonArray>();
    int index = 0;
    for (JsonObject preset : filePresets) {
        String name = preset["name"].as<String>();
        
        // 创建预设对象，包含ID和名称
        JsonObject presetObj = presetsArray.createNestedObject();
        presetObj["id"] = name;  // 使用name作为ID
        presetObj["name"] = name;
        
        index++;
    }
}

// 删除预设
bool WebServerController::deletePreset(const char* presetId) {
    // 确保SPIFFS已初始化
    if (!SPIFFS.exists(PRESET_FILE)) {
        if (!initPresetStorage()) {
            return false;
        }
    }
    
    // 读取现有预设
    File file = SPIFFS.open(PRESET_FILE, FILE_READ);
    if (!file) {
        Serial.println("打开预设文件失败");
        return false;
    }
    
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.println("解析预设JSON失败");
        return false;
    }
    
    JsonArray presets = doc.as<JsonArray>();
    bool found = false;
    
    // 创建新数组存储保留的预设
    DynamicJsonDocument newDoc(4096);
    JsonArray newPresets = newDoc.createNestedArray();
    
    // 遍历所有预设，复制非目标预设
    for (JsonObject preset : presets) {
        if (preset["name"] != presetId) {
            JsonObject newPreset = newPresets.createNestedObject();
            newPreset["name"] = preset["name"];
            
            JsonArray oldPositions = preset["positions"];
            JsonArray newPositions = newPreset.createNestedArray("positions");
            
            for (JsonVariant pos : oldPositions) {
                newPositions.add(pos.as<uint16_t>());
            }
        } else {
            found = true;
            Serial.print("找到要删除的预设 '");
            Serial.print(presetId);
            Serial.println("'");
        }
    }
    
    if (!found) {
        Serial.print("未找到要删除的预设 '");
        Serial.print(presetId);
        Serial.println("'");
        return false;
    }
    
    // 将新的预设列表写回文件
    file = SPIFFS.open(PRESET_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("打开预设文件写入失败");
        return false;
    }
    
    serializeJson(newPresets, file);
    file.close();
    
    Serial.print("预设 '");
    Serial.print(presetId);
    Serial.println("' 已成功删除");
    return true;
}

// 获取录制器状态
void WebServerController::handleGetRecorderStatus(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(256);
    
    // 获取录制状态
    if (isRecording()) {
        doc["state"] = "recording";
        doc["frameCount"] = getRecordFrameCount();
    } else if (isPlaying()) {
        doc["state"] = "playing";
    } else {
        doc["state"] = "idle";
    }
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// 获取录制的命令
void WebServerController::handleGetRecordedCommands(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    
    // 创建命令数组
    JsonArray commandsArray = doc.createNestedArray("commands");
    
    // 添加最新记录的舵机命令
    // 这里我们只获取最近的5条记录，避免数据量过大
    extern ServoCommandRecorder servoRecorder;
    
    // 获取当前录制状态
    if (isRecording()) {
        doc["state"] = "recording";
        doc["frameCount"] = getRecordFrameCount();
        
        // TODO: 在ServoCommandRecorder类中添加获取最近命令的方法
        // 暂时使用简单的状态信息
        JsonObject cmdObj = commandsArray.createNestedObject();
        cmdObj["time"] = millis();
        cmdObj["type"] = "info";
        cmdObj["text"] = "已记录 " + String(getRecordFrameCount()) + " 帧命令";
    } else if (isPlaying()) {
        doc["state"] = "playing";
        
        JsonObject cmdObj = commandsArray.createNestedObject();
        cmdObj["time"] = millis();
        cmdObj["type"] = "info";
        cmdObj["text"] = "正在播放录制的动作组";
    } else {
        doc["state"] = "idle";
        
        JsonObject cmdObj = commandsArray.createNestedObject();
        cmdObj["time"] = millis();
        cmdObj["type"] = "info";
        cmdObj["text"] = "等待录制命令...";
    }
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// Web服务器任务实现
void webServerTask(void *pvParameters) {
    Serial.println("Web服务器任务启动...");
    
    // 等待系统稳定和网络初始化
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 创建并初始化Web服务器控制器
    WebServerController webServerController;
    webServerController.init();
    
    // 尝试获取机械臂控制器实例
    // 使用外部定义的全局机械臂控制器实例
    extern RobotArmController armController;
    webServerController.setRobotArmController(&armController);
    
    // 启动Web服务器
    webServerController.start();
    
    // 任务保持运行
    for (;;) {
        // Web服务器在后台运行，任务只需阻塞
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}