/******************************************************
* FileName:      RobotArmFreeRTOS.cpp
* Date:          2025/05/04
* Description:   机械臂项目的FreeRTOS任务实现
*****************************************************/

#include "RobotArmFreeRTOS.h"
#include "RobotArmController.h"
#include "ServoCommandRecorder.h" // 添加舵机命令录制器头文件

// 函数前向声明
void processPS2Input();

// 全局任务句柄
TaskHandle_t servoControlTaskHandle = NULL;
TaskHandle_t inputProcessingTaskHandle = NULL;
TaskHandle_t statusMonitorTaskHandle = NULL;
TaskHandle_t webServerTaskHandle = NULL; // 添加Web服务器任务句柄
TaskHandle_t playbackTaskHandle = NULL; // 回放任务句柄

// 全局队列句柄
QueueHandle_t servoCommandQueue = NULL;

// 舵机控制器实例
LobotServoController servoController;

// 机械臂控制器实例 - 负责主要控制逻辑
RobotArmController armController;

// 初始化FreeRTOS任务和资源
bool initRobotArmTasks() {
    bool initSuccess = true;
    
    // 创建命令队列 - 队列长度为10，每个项目大小为ServoCommand_t的大小
    servoCommandQueue = xQueueCreate(10, sizeof(ServoCommand_t));
    if (servoCommandQueue == NULL) {
        return false;
    }
    
    // 初始化舵机控制器
    servoController = LobotServoController(Serial2);
    // 使用指定的引脚配置Serial2
    Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX, SERVO_SERIAL_TX);
    delay(500); // 等待舵机控制器初始化完成
    
    // 优先直接移动舵机到初始位置，确保立即执行
    LobotServo servos[8];
    
    // 设置各舵机初始位置
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
    
    // 同时移动所有舵机到初始位置
    servoController.moveServos(servos, 8, SLOW_MOVE_TIME);
    
    // 输出初始化舵机位置信息
    Serial.print("舵机组命令：[");
    // 输出角度值数组
    for (int i = 0; i < 8; i++) {
        Serial.print(PULSE_TO_DEGREE(servos[i].Position), 1); // 保留1位小数
        if (i < 7) Serial.print("、");
    }
    Serial.print("][");
    // 输出脉冲值数组
    for (int i = 0; i < 8; i++) {
        Serial.print(servos[i].Position);
        if (i < 7) Serial.print("、");
    }
    Serial.println("]");
    
    // 使用RobotArmController初始化PS2控制器，不输出调试信息
    int error = armController.initializePS2(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN);
    if (error != 0 && error != 2 && error != 3) {
        initSuccess = false;
    }
    
    // 创建舵机控制任务
    xTaskCreatePinnedToCore(
        servoControlTask,               // 任务函数
        "ServoControlTask",             // 任务名称
        SERVO_CONTROL_STACK_SIZE,       // 堆栈大小
        NULL,                           // 任务参数
        SERVO_CONTROL_PRIORITY,         // 任务优先级
        &servoControlTaskHandle,        // 任务句柄指针
        1                               // 在Core 1上运行
    );
    
    // 创建输入处理任务
    xTaskCreatePinnedToCore(
        inputProcessingTask,            // 任务函数
        "InputProcessingTask",          // 任务名称
        INPUT_PROCESSING_STACK_SIZE,    // 堆栈大小
        NULL,                           // 任务参数
        INPUT_PROCESSING_PRIORITY,      // 任务优先级
        &inputProcessingTaskHandle,     // 任务句柄指针
        0                               // 在Core 0上运行
    );
    
    // 创建状态监控任务
    xTaskCreatePinnedToCore(
        statusMonitorTask,              // 任务函数
        "StatusMonitorTask",            // 任务名称
        STATUS_MONITOR_STACK_SIZE,      // 堆栈大小
        NULL,                           // 任务参数
        STATUS_MONITOR_PRIORITY,        // 任务优先级
        &statusMonitorTaskHandle,       // 任务句柄指针
        0                               // 在Core 0上运行
    );
    
    // 创建Web服务器任务
    xTaskCreatePinnedToCore(
        webServerTask,                  // 任务函数
        "WebServerTask",                // 任务名称
        WEB_SERVER_STACK_SIZE,          // 堆栈大小
        NULL,                           // 任务参数
        WEB_SERVER_PRIORITY,            // 任务优先级
        &webServerTaskHandle,           // 任务句柄指针
        0                               // 在Core 0上运行
    );
    
    return initSuccess;
}

// 舵机控制任务 - 处理舵机命令队列
void servoControlTask(void *pvParameters) {
    ServoCommand_t command;
    
    for (;;) {
        // 等待队列中的命令
        if (xQueueReceive(servoCommandQueue, &command, portMAX_DELAY) == pdTRUE) {
            // 处理收到的命令
            switch (command.type) {
                case MOVE_SINGLE_SERVO:                  
                    servoController.moveServo(command.servoID, command.position, command.time);
                    // 更新RobotArmController中的位置
                    armController.setServoPosition(command.servoID, command.position);
                    
                    // 如果处于录制状态，记录该命令
                    if (servoRecorder.getState() == STATE_RECORDING) {
                        LobotServo servo;
                        servo.ID = command.servoID;
                        servo.Position = command.position;
                        servoRecorder.recordCommand(MOVE_SINGLE_SERVO, 1, &servo, command.time);
                    }
                    break;
                    
                case MOVE_ALL_SERVOS:
                    servoController.moveServos(command.servos, command.servoCount, command.time);
                    // 更新RobotArmController中的位置
                    for (int i = 0; i < command.servoCount; i++) {
                        if (command.servos[i].ID >= 1 && command.servos[i].ID <= SERVO_COUNT) {
                            armController.setServoPosition(command.servos[i].ID, command.servos[i].Position);
                        }
                    }
                    
                    // 如果处于录制状态，记录该命令
                    if (servoRecorder.getState() == STATE_RECORDING) {
                        servoRecorder.recordCommand(MOVE_ALL_SERVOS, command.servoCount, command.servos, command.time);
                    }
                    break;
                    
                case MOVE_TO_HOME:
                    // 委托RobotArmController移动到初始位置
                    armController.moveToInitialPosition();
                    
                    // 如果处于录制状态，记录该命令
                    if (servoRecorder.getState() == STATE_RECORDING) {
                        LobotServo servos[SERVO_COUNT];
                        
                        // 设置各舵机初始位置
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
                        
                        servoRecorder.recordCommand(MOVE_ALL_SERVOS, SERVO_COUNT, servos, SLOW_MOVE_TIME);
                    }
                    break;
                    
                case UNLOAD_SERVOS:
                    // 创建舵机ID数组
                    {
                        uint8_t servoIds[SERVO_COUNT];
                        for (int i = 0; i < SERVO_COUNT; i++) {
                            servoIds[i] = i + 1;
                        }
                        servoController.setServoUnload(SERVO_COUNT, servoIds[0], servoIds[1], servoIds[2], 
                                                     servoIds[3], servoIds[4], servoIds[5], servoIds[6], 
                                                     servoIds[7]);
                        
                        // 如果处于录制状态，记录舵机掉电命令
                        if (servoRecorder.getState() == STATE_RECORDING) {
                            // 舵机掉电不需要具体舵机数据
                            servoRecorder.recordCommand(UNLOAD_SERVOS, 0, NULL, 0);
                        }
                    }
                    break;
            }
        }
        
        // 短暂让出CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// 输入处理任务 - 处理PS2控制器输入
void inputProcessingTask(void *pvParameters) {
    // 移除任务启动提示
    
    // 记录上次更新时间
    unsigned long lastUpdateTime = 0;
    
    for (;;) {
        unsigned long currentTime = millis();
        
        // 每20ms读取一次控制器
        if (currentTime - lastUpdateTime >= 20) {
            lastUpdateTime = currentTime;
            
            // 处理PS2输入
            processPS2Input();
        }
        
        // 让出CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// 处理PS2控制器输入 - 使用RobotArmController处理逻辑
void processPS2Input() {
    // 使用RobotArmController处理PS2输入并生成舵机命令
    ServoCommandType type = MOVE_SINGLE_SERVO; // 设置默认值避免未初始化
    uint8_t servoCount = 0;
    LobotServo servos[SERVO_COUNT];
    uint16_t time = 0;
    
    // 使用try-catch结构保护运动学模式崩溃
    bool hasCommand = false;
    
    // 让RobotArmController处理输入并生成命令
    try {
        hasCommand = armController.processPS2AndGenerateCommands(type, servoCount, servos, time);
    } catch (...) {
        // 如果出现任何异常，回到安全模式
        Serial.println("警告：处理PS2输入时发生错误，重置为关节角度模式");
        
        // 强制切换回关节角度模式
        if(armController.getControlMode() != JOINT_ANGLE_MODE) {
            armController.setControlMode(JOINT_ANGLE_MODE);
        }
        
        // 不生成命令
        hasCommand = false;
    }
    
    // 有命令生成则发送到命令队列
    if (hasCommand) {
        ServoCommand_t command;
        command.type = type;
        command.time = time;
        
        // 根据命令类型设置不同的参数
        switch(type) {
            case MOVE_ALL_SERVOS:
                command.servoCount = servoCount;
                for (int i = 0; i < servoCount; i++) {
                    command.servos[i] = servos[i];
                }
                break;
                
            case MOVE_SINGLE_SERVO:
                command.servoID = servos[0].ID;
                command.position = servos[0].Position;
                break;
                
            case MOVE_TO_HOME:
                // 复位功能特殊处理：创建一个包含所有舵机的命令
                command.servoCount = 8;  // 所有8个舵机
                command.time = SLOW_MOVE_TIME;  // 使用较慢的移动速度
                
                // 设置各舵机初始位置
                command.servos[0].ID = BASE_SERVO_ID;
                command.servos[0].Position = HOME_BASE;
                
                command.servos[1].ID = SHOULDER_PITCH_ID;
                command.servos[1].Position = HOME_SHOULDER;
                
                command.servos[2].ID = SHOULDER_ROLL_ID;
                command.servos[2].Position = HOME_SHOULDER;
                
                command.servos[3].ID = ELBOW_SERVO_ID;
                command.servos[3].Position = HOME_ELBOW;
                
                command.servos[4].ID = WRIST_PITCH_ID;
                command.servos[4].Position = HOME_WRIST_PITCH;
                
                command.servos[5].ID = WRIST_ROLL_ID;
                command.servos[5].Position = HOME_WRIST_ROLL;
                
                command.servos[6].ID = WRIST_YAW_ID;
                command.servos[6].Position = HOME_WRIST_YAW;
                
                command.servos[7].ID = GRIPPER_SERVO_ID;
                command.servos[7].Position = HOME_GRIPPER;
                
                // 更改命令类型为直接移动所有舵机
                command.type = MOVE_ALL_SERVOS;
                
                Serial.println("发送复位指令: 所有舵机直接移动到初始位置");
                break;
                
            // UNLOAD_SERVOS不需要额外参数
            default:
                break;
        }
        
        // 发送命令到队列，对复位命令使用最高优先级
        if (type == MOVE_TO_HOME) {
            // 先清空队列中的所有命令
            ServoCommand_t dummyCommand;
            while (xQueueReceive(servoCommandQueue, &dummyCommand, 0) == pdTRUE) {
                // 清空队列
            }
            
            // 然后发送复位命令
            xQueueSendToFront(servoCommandQueue, &command, portMAX_DELAY);
        } else {
            // 正常发送其他命令
            xQueueSend(servoCommandQueue, &command, portMAX_DELAY);
        }
    }
    
    // 添加安全检查 - 确保ESP32不会崩溃
    static unsigned long lastWatchdogReset = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastWatchdogReset > 500) {
        // 每500ms执行一次看门狗喂食
        delay(1); // 短暂放弃CPU控制权，让其他任务运行
        lastWatchdogReset = currentTime;
    }
}

// 状态监控任务 - 基础系统状态监控
void statusMonitorTask(void *pvParameters) {
    // 移除任务启动提示
    
    for (;;) {
        // 此任务保留为框架，但移除了电池电压监控功能
        // 可在此处添加其他必要的系统状态监控
        
        // 让出CPU
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒检查一次状态
    }
}

// 获取当前舵机位置 - 供WebServerController使用
uint16_t getCurrentServoPosition(uint8_t servoID) {
    // 使用RobotArmController获取最新位置
    return armController.getServoPosition(servoID);
}

// 发送舵机命令到队列
void sendServoCommand(ServoCommandType type, uint8_t servoID, 
                     uint16_t position, uint16_t time) {
    ServoCommand_t command;
    
    command.type = type;
    command.servoID = servoID;
    command.position = position;
    command.time = time;
    
    // 发送命令到队列
    xQueueSend(servoCommandQueue, &command, portMAX_DELAY);
}

// 移动到初始位置
void moveToHomePosition() {
    ServoCommand_t command;
    
    command.type = MOVE_TO_HOME;
    command.time = DEFAULT_MOVE_TIME;
    
    // 发送命令到队列
    xQueueSend(servoCommandQueue, &command, portMAX_DELAY);
}

// 移动所有舵机
void moveAllServos(LobotServo servos[], uint8_t count, uint16_t time) {
    ServoCommand_t command;
    
    command.type = MOVE_ALL_SERVOS;
    command.servoCount = count;
    command.time = time;
    
    // 复制舵机数据
    for (int i = 0; i < count && i < SERVO_COUNT; i++) {
        command.servos[i] = servos[i];
    }
    
    // 发送命令到队列
    xQueueSend(servoCommandQueue, &command, portMAX_DELAY);
}

// 约束舵机角度在有效范围内 - 使用RobotArmController
uint16_t constrainServoPosition(uint8_t servoID, uint16_t position) {
    // 委托给RobotArmController进行位置约束
    return armController.constrainServoAngle(servoID, position);
}

// 开始录制
bool startRecording(const char* fileName) {
    // 检查是否已经在录制或回放中
    if (servoRecorder.getState() != STATE_IDLE) {
        Serial.println("错误：录制器当前不处于空闲状态，无法开始录制");
        return false;
    }
    
    // 开始录制
    bool success = servoRecorder.startRecording(fileName);
    
    if (success) {
        Serial.print("开始录制舵机命令，录制文件：");
        Serial.println(fileName);
    }
    
    return success;
}

// 停止录制
bool stopRecording() {
    // 检查是否在录制状态
    if (servoRecorder.getState() != STATE_RECORDING) {
        Serial.println("错误：录制器当前不处于录制状态，无法停止录制");
        return false;
    }
    
    // 停止录制
    bool success = servoRecorder.stopRecording();
    
    if (success) {
        Serial.print("停止录制，共录制 ");
        Serial.print(servoRecorder.getFrameCount());
        Serial.println(" 帧命令");
    }
    
    return success;
}

// 开始回放
bool startPlayback(const char* fileName) {
    // 检查是否已经在录制或回放中
    if (servoRecorder.getState() != STATE_IDLE) {
        Serial.println("错误：录制器当前不处于空闲状态，无法开始回放");
        return false;
    }
    
    // 开始回放
    bool success = servoRecorder.startPlayback(fileName);
    
    if (success) {
        Serial.print("开始回放舵机动作，文件：");
        Serial.println(fileName);
        
        // 创建一个回放任务
        xTaskCreatePinnedToCore(
            playbackTask,                // 任务函数
            "PlaybackTask",              // 任务名称
            2048,                        // 堆栈大小
            NULL,                        // 任务参数
            2,                           // 任务优先级（与输入处理相同）
            &playbackTaskHandle,         // 任务句柄指针
            0                            // 在Core 0上运行
        );
    }
    
    return success;
}

// 停止回放
void stopPlayback() {
    // 检查是否在回放状态
    if (servoRecorder.getState() != STATE_PLAYING) {
        return;
    }
    
    // 停止回放
    servoRecorder.stopPlayback();
    Serial.println("停止回放");
    
    // 如果回放任务存在，停止它
    if (playbackTaskHandle != NULL) {
        vTaskDelete(playbackTaskHandle);
        playbackTaskHandle = NULL;
    }
}

// 检查是否在录制中
bool isRecording() {
    return servoRecorder.getState() == STATE_RECORDING;
}

// 检查是否在回放中
bool isPlaying() {
    return servoRecorder.getState() == STATE_PLAYING;
}

// 获取已录制的帧数
uint16_t getRecordFrameCount() {
    return servoRecorder.getFrameCount();
}

// 列出所有录制文件
void listRecordFiles(JsonArray& filesArray) {
    servoRecorder.listRecordFiles(filesArray);
}

// 删除录制文件
bool deleteRecordFile(const char* fileName) {
    return servoRecorder.deleteRecordFile(fileName);
}

// 回放任务 - 负责按时间回放记录的命令
void playbackTask(void* pvParameters) {
    Serial.println("舵机命令回放任务启动");
    
    ServoCommand_t command;
    
    while (servoRecorder.getState() == STATE_PLAYING) {
        // 获取下一帧命令
        if (servoRecorder.playNextFrame(command)) {
            // 发送命令到队列
            xQueueSend(servoCommandQueue, &command, portMAX_DELAY);
        }
        
        // 短暂让出CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    Serial.println("舵机命令回放任务结束");
    
    // 删除自身
    playbackTaskHandle = NULL;
    vTaskDelete(NULL);
}