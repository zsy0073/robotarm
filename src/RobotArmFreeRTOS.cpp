/******************************************************
* FileName:      RobotArmFreeRTOS.cpp
* Date:          2025/05/04
* Description:   机械臂项目的FreeRTOS任务实现
*****************************************************/

#include "RobotArmFreeRTOS.h"

// 函数前向声明
void processPS2Input();

// 全局任务句柄
TaskHandle_t servoControlTaskHandle = NULL;
TaskHandle_t inputProcessingTaskHandle = NULL;
TaskHandle_t statusMonitorTaskHandle = NULL;
TaskHandle_t webServerTaskHandle = NULL; // 添加Web服务器任务句柄

// 全局队列句柄
QueueHandle_t servoCommandQueue = NULL;

// 舵机控制器实例
LobotServoController servoController;

// PS2控制器实例
PS2X ps2x;

// 当前舵机位置
uint16_t currentServoPositions[SERVO_COUNT] = {
    HOME_BASE,         // 底座舵机初始位置
    HOME_SHOULDER,     // 肩部俯仰舵机初始位置
    HOME_ELBOW,        // 肘部舵机初始位置
    HOME_WRIST_PITCH,  // 腕部俯仰舵机初始位置
    HOME_WRIST_ROLL,   // 腕部滚转舵机初始位置
    HOME_WRIST_YAW,    // 腕部偏航舵机初始位置
    HOME_GRIPPER       // 夹爪舵机初始位置
};

// 舵机方向配置
const int servoDirections[SERVO_COUNT] = {
    BASE_SERVO_DIR,        // 底座方向
    SHOULDER_PITCH_DIR,    // 肩部俯仰方向
    SHOULDER_ROLL_DIR,     // 肩部滚转方向
    ELBOW_SERVO_DIR,       // 肘部方向
    WRIST_PITCH_DIR,       // 腕部俯仰方向
    WRIST_ROLL_DIR,        // 腕部滚转方向
    WRIST_YAW_DIR,         // 腕部偏航方向
    GRIPPER_SERVO_DIR      // 夹爪方向
};

// 是否使用自定义限位
bool useCustomLimits = USE_CUSTOM_LIMITS;

// 初始化FreeRTOS任务和资源
bool initRobotArmTasks() {
    bool initSuccess = true;
    
    // 创建命令队列 - 队列长度为10，每个项目大小为ServoCommand_t的大小
    servoCommandQueue = xQueueCreate(10, sizeof(ServoCommand_t));
    if (servoCommandQueue == NULL) {
        Serial.println("创建命令队列失败");
        return false;
    }
    
    // 初始化PS2控制器
    int error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, true, true);
    if (error != 0) {
        Serial.print("PS2手柄初始化失败，错误代码：");
        Serial.println(error);
        initSuccess = false;
    } else {
        Serial.println("PS2手柄初始化成功");
    }
    
    // 初始化舵机控制器
    servoController = LobotServoController(Serial2);
    Serial2.begin(115200);
    delay(500); // 等待舵机控制器初始化
    
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
    
    // 如果初始化成功，发送移动到初始位置的命令
    if (initSuccess) {
        moveToHomePosition();
    }
    
    return initSuccess;
}

// 舵机控制任务 - 处理舵机命令队列
void servoControlTask(void *pvParameters) {
    ServoCommand_t command;
    
    Serial.println("舵机控制任务已启动");
    
    for (;;) {
        // 等待队列中的命令
        if (xQueueReceive(servoCommandQueue, &command, portMAX_DELAY) == pdTRUE) {
            // 根据命令类型执行操作
            switch (command.type) {
                case MOVE_SINGLE_SERVO:
                    servoController.moveServo(command.servoID, command.position, command.time);
                    // 更新当前位置
                    if (command.servoID >= 1 && command.servoID <= SERVO_COUNT) {
                        currentServoPositions[command.servoID - 1] = command.position;
                    }
                    break;
                    
                case MOVE_ALL_SERVOS:
                    servoController.moveServos(command.servos, command.servoCount, command.time);
                    // 更新所有当前位置
                    for (int i = 0; i < command.servoCount; i++) {
                        if (command.servos[i].ID >= 1 && command.servos[i].ID <= SERVO_COUNT) {
                            currentServoPositions[command.servos[i].ID - 1] = command.servos[i].Position;
                        }
                    }
                    break;
                    
                case MOVE_TO_HOME:
                    // 移动到初始位置
                    {
                        LobotServo servos[SERVO_COUNT];
                        for (int i = 0; i < SERVO_COUNT; i++) {
                            servos[i].ID = i + 1;
                            
                            // 根据舵机ID设置初始位置
                            switch (i + 1) {
                                case BASE_SERVO_ID:
                                    servos[i].Position = HOME_BASE;
                                    currentServoPositions[i] = HOME_BASE;
                                    break;
                                case SHOULDER_PITCH_ID:
                                    servos[i].Position = HOME_SHOULDER;
                                    currentServoPositions[i] = HOME_SHOULDER;
                                    break;
                                case SHOULDER_ROLL_ID:
                                    servos[i].Position = HOME_SHOULDER;
                                    currentServoPositions[i] = HOME_SHOULDER;
                                    break;
                                case ELBOW_SERVO_ID:
                                    servos[i].Position = HOME_ELBOW;
                                    currentServoPositions[i] = HOME_ELBOW;
                                    break;
                                case WRIST_PITCH_ID:
                                    servos[i].Position = HOME_WRIST_PITCH;
                                    currentServoPositions[i] = HOME_WRIST_PITCH;
                                    break;
                                case WRIST_ROLL_ID:
                                    servos[i].Position = HOME_WRIST_ROLL;
                                    currentServoPositions[i] = HOME_WRIST_ROLL;
                                    break;
                                case WRIST_YAW_ID:
                                    servos[i].Position = HOME_WRIST_YAW;
                                    currentServoPositions[i] = HOME_WRIST_YAW;
                                    break;
                                case GRIPPER_SERVO_ID:
                                    servos[i].Position = HOME_GRIPPER;
                                    currentServoPositions[i] = HOME_GRIPPER;
                                    break;
                            }
                        }
                        servoController.moveServos(servos, SERVO_COUNT, command.time);
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
    Serial.println("输入处理任务已启动");
    
    // 记录上次更新时间
    unsigned long lastUpdateTime = 0;
    
    for (;;) {
        unsigned long currentTime = millis();
        
        // 每20ms读取一次控制器
        if (currentTime - lastUpdateTime >= 20) {
            lastUpdateTime = currentTime;
            
            ps2x.read_gamepad();
            
            // 处理PS2输入
            processPS2Input();
        }
        
        // 让出CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// 处理PS2控制器输入
void processPS2Input() {
    // 摇杆死区设置
    const int STICK_DEADZONE = 30;  
    
    // 第一个模拟摇杆 - 控制底座旋转和肩部俯仰
    int lx = ps2x.Analog(PSS_LX) - 128;  // 范围-127到127
    int ly = ps2x.Analog(PSS_LY) - 128;  // 范围-127到127
    
    // 第二个模拟摇杆 - 控制肘部和腕部俯仰
    int rx = ps2x.Analog(PSS_RX) - 128;  // 范围-127到127
    int ry = ps2x.Analog(PSS_RY) - 128;  // 范围-127到127
    
    // 死区检查
    if (abs(lx) < STICK_DEADZONE) lx = 0;
    if (abs(ly) < STICK_DEADZONE) ly = 0;
    if (abs(rx) < STICK_DEADZONE) rx = 0;
    if (abs(ry) < STICK_DEADZONE) ry = 0;
    
    // 计算舵机移动增量
    int baseIncrement = map(lx, -127, 127, -5, 5);
    int shoulderIncrement = map(ly, -127, 127, -5, 5) * (-1);  // 反转方向
    int elbowIncrement = map(ry, -127, 127, -5, 5) * (-1);     // 反转方向
    int wristIncrement = map(rx, -127, 127, -5, 5);
    
    // 按钮控制 - 夹爪控制
    if (ps2x.ButtonPressed(PSB_R1)) {
        // 打开夹爪
        uint16_t position = constrainServoPosition(GRIPPER_SERVO_ID, 
                                                 currentServoPositions[GRIPPER_SERVO_ID - 1] + 50);
        sendServoCommand(MOVE_SINGLE_SERVO, GRIPPER_SERVO_ID, position, 500);
    }
    
    if (ps2x.ButtonPressed(PSB_L1)) {
        // 关闭夹爪
        uint16_t position = constrainServoPosition(GRIPPER_SERVO_ID, 
                                                 currentServoPositions[GRIPPER_SERVO_ID - 1] - 50);
        sendServoCommand(MOVE_SINGLE_SERVO, GRIPPER_SERVO_ID, position, 500);
    }
    
    // 如果任何摇杆有移动，更新相应舵机位置
    if (baseIncrement != 0) {
        uint16_t position = constrainServoPosition(BASE_SERVO_ID, 
                                                currentServoPositions[BASE_SERVO_ID - 1] + 
                                                baseIncrement * servoDirections[BASE_SERVO_ID - 1]);
        sendServoCommand(MOVE_SINGLE_SERVO, BASE_SERVO_ID, position, 20);
    }
    
    if (shoulderIncrement != 0) {
        uint16_t position = constrainServoPosition(SHOULDER_PITCH_ID, 
                                                currentServoPositions[SHOULDER_PITCH_ID - 1] + 
                                                shoulderIncrement * servoDirections[SHOULDER_PITCH_ID - 1]);
        sendServoCommand(MOVE_SINGLE_SERVO, SHOULDER_PITCH_ID, position, 20);
    }
    
    if (elbowIncrement != 0) {
        uint16_t position = constrainServoPosition(ELBOW_SERVO_ID, 
                                                currentServoPositions[ELBOW_SERVO_ID - 1] + 
                                                elbowIncrement * servoDirections[ELBOW_SERVO_ID - 1]);
        sendServoCommand(MOVE_SINGLE_SERVO, ELBOW_SERVO_ID, position, 20);
    }
    
    if (wristIncrement != 0) {
        uint16_t position = constrainServoPosition(WRIST_PITCH_ID, 
                                                currentServoPositions[WRIST_PITCH_ID - 1] + 
                                                wristIncrement * servoDirections[WRIST_PITCH_ID - 1]);
        sendServoCommand(MOVE_SINGLE_SERVO, WRIST_PITCH_ID, position, 20);
    }
    
    // 功能按键
    if (ps2x.ButtonPressed(PSB_START)) {
        // 回到初始位置
        moveToHomePosition();
    }
    
    if (ps2x.ButtonPressed(PSB_SELECT)) {
        // 掉电所有舵机
        sendServoCommand(UNLOAD_SERVOS);
    }
    
    // 其他按钮的功能可以根据需要添加
}

// 状态监控任务 - 基础系统状态监控
void statusMonitorTask(void *pvParameters) {
    Serial.println("状态监控任务已启动");
    
    for (;;) {
        // 此任务保留为框架，但移除了电池电压监控功能
        // 可在此处添加其他必要的系统状态监控
        
        // 让出CPU
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒检查一次状态
    }
}

// 获取当前舵机位置 - 供WebServerController使用
uint16_t getCurrentServoPosition(uint8_t servoID) {
    if (servoID >= 1 && servoID <= SERVO_COUNT) {
        return currentServoPositions[servoID - 1];
    }
    return 0;
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

// 约束舵机角度在有效范围内
uint16_t constrainServoPosition(uint8_t servoID, uint16_t position) {
    uint16_t minPos, maxPos;
    
    // 根据是否使用自定义限位来确定限位值
    if (useCustomLimits) {
        switch (servoID) {
            case BASE_SERVO_ID:
                minPos = CUSTOM_BASE_MIN;
                maxPos = CUSTOM_BASE_MAX;
                break;
            case SHOULDER_PITCH_ID:
            case SHOULDER_ROLL_ID:
                minPos = CUSTOM_SHOULDER_MIN;
                maxPos = CUSTOM_SHOULDER_MAX;
                break;
            case ELBOW_SERVO_ID:
                minPos = CUSTOM_ELBOW_MIN;
                maxPos = CUSTOM_ELBOW_MAX;
                break;
            case WRIST_PITCH_ID:
                minPos = CUSTOM_WRIST_PITCH_MIN;
                maxPos = CUSTOM_WRIST_PITCH_MAX;
                break;
            case WRIST_ROLL_ID:
                minPos = CUSTOM_WRIST_ROLL_MIN;
                maxPos = CUSTOM_WRIST_ROLL_MAX;
                break;
            case WRIST_YAW_ID:
                minPos = CUSTOM_WRIST_YAW_MIN;
                maxPos = CUSTOM_WRIST_YAW_MAX;
                break;
            case GRIPPER_SERVO_ID:
                minPos = CUSTOM_GRIPPER_MIN;
                maxPos = CUSTOM_GRIPPER_MAX;
                break;
            default:
                minPos = SERVO_MIN_PULSE;
                maxPos = SERVO_MAX_PULSE;
        }
    } else {
        // 使用默认限位
        switch (servoID) {
            case BASE_SERVO_ID:
                minPos = BASE_MIN_ANGLE;
                maxPos = BASE_MAX_ANGLE;
                break;
            case SHOULDER_PITCH_ID:
            case SHOULDER_ROLL_ID:
                minPos = SHOULDER_MIN_ANGLE;
                maxPos = SHOULDER_MAX_ANGLE;
                break;
            case ELBOW_SERVO_ID:
                minPos = ELBOW_MIN_ANGLE;
                maxPos = ELBOW_MAX_ANGLE;
                break;
            case WRIST_PITCH_ID:
                minPos = WRIST_PITCH_MIN;
                maxPos = WRIST_PITCH_MAX;
                break;
            case WRIST_ROLL_ID:
                minPos = WRIST_ROLL_MIN;
                maxPos = WRIST_ROLL_MAX;
                break;
            case WRIST_YAW_ID:
                minPos = WRIST_YAW_MIN;
                maxPos = WRIST_YAW_MAX;
                break;
            case GRIPPER_SERVO_ID:
                minPos = GRIPPER_MIN;
                maxPos = GRIPPER_MAX;
                break;
            default:
                minPos = SERVO_MIN_PULSE;
                maxPos = SERVO_MAX_PULSE;
        }
    }
    
    // 约束在范围内
    return constrain(position, minPos, maxPos);
}