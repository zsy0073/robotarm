/******************************************************
* FileName:      RobotArmController.cpp
* Date:          2025/05/04
* Description:   机械臂控制器类的实现文件
*****************************************************/

#include "RobotArmController.h"

// 构造函数
RobotArmController::RobotArmController() : 
    controller(Serial),
    useCustomLimits(USE_CUSTOM_LIMITS),
    lastUpdateTime(0)
{
    // 初始化当前舵机位置数组
    currentPosition[0] = HOME_BASE;
    currentPosition[1] = HOME_SHOULDER;
    currentPosition[2] = HOME_SHOULDER;
    currentPosition[3] = HOME_ELBOW;
    currentPosition[4] = HOME_WRIST_PITCH;
    currentPosition[5] = HOME_WRIST_ROLL;
    currentPosition[6] = HOME_WRIST_YAW;
    currentPosition[7] = HOME_GRIPPER;
    
    // 初始化舵机速度控制数组
    for (int i = 0; i < 8; i++) {
        servoSpeedControl[i] = 0;
    }
}

// 初始化函数
bool RobotArmController::init() {
    Serial.println("机械臂控制程序启动 - 240度舵机正反转控制版本");
    
    // 初始化PS2手柄
    int ps2Error = initPS2Controller();
    
    // 如果PS2手柄初始化成功，则初始化机械臂
    if (ps2Error == 0) {
        delay(1000); // 等待一小段时间确保稳定
        initRobotArm();
        lastUpdateTime = millis();
        return true;
    }
    
    return false;
}

// 主循环函数 - 在Arduino的loop函数中调用
void RobotArmController::update() {
    // 更新舵机方向
    updateServoDirections();
    
    // 根据方向更新舵机位置
    moveServos();
}

// 初始化PS2手柄
int RobotArmController::initPS2Controller() {
    int error = 0;
    
    // 配置手柄，需要设置手柄的各个引脚以及是否开启震动和压感功能
    error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, false, false);
    
    switch (error) {
        case 0:
            Serial.println("PS2手柄初始化成功");
            break;
        case 1:
            Serial.println("错误：未找到手柄，请检查连线");
            break;
        case 2:
            Serial.println("错误：手柄未进入Analog模式，请按Analog按钮");
            break;
        case 3:
            Serial.println("错误：手柄拒绝进入Pressures模式");
            break;
        default:
            Serial.println("未知错误");
            break;
    }
    
    return error;
}

// 初始化机械臂位置
void RobotArmController::initRobotArm() {
    // 创建舵机数组
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
    controller.moveServos(servos, 8, SLOW_MOVE_TIME);
    
    // 更新当前位置
    for (int i = 0; i < 8; i++) {
        currentPosition[i] = servos[i].Position;
        servoSpeedControl[i] = 0; // 停止所有舵机
    }
    
    Serial.println("机械臂已初始化到初始位置");
    // 打印当前位置的角度值，便于调试
    Serial.println("当前舵机角度(度):");
    for (int i = 0; i < 8; i++) {
        Serial.print("舵机 ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(PULSE_TO_DEGREE(currentPosition[i]));
        Serial.println("度");
    }
}

// 限制舵机角度在安全范围内
uint16_t RobotArmController::constrainServoAngle(uint8_t servoID, uint16_t pulseValue) {
    // 根据是否启用自定义限位选择不同的限位值
    if (useCustomLimits) {
        // 使用自定义限位
        switch (servoID) {
            case BASE_SERVO_ID:
                return constrain(pulseValue, CUSTOM_BASE_MIN, CUSTOM_BASE_MAX);
            case SHOULDER_PITCH_ID:
                return constrain(pulseValue, CUSTOM_SHOULDER_MIN, CUSTOM_SHOULDER_MAX);
            case SHOULDER_ROLL_ID:
                return constrain(pulseValue, CUSTOM_SHOULDER_MIN, CUSTOM_SHOULDER_MAX);
            case ELBOW_SERVO_ID:
                return constrain(pulseValue, CUSTOM_ELBOW_MIN, CUSTOM_ELBOW_MAX);
            case WRIST_PITCH_ID:
                return constrain(pulseValue, CUSTOM_WRIST_PITCH_MIN, CUSTOM_WRIST_PITCH_MAX);
            case WRIST_ROLL_ID:
                return constrain(pulseValue, CUSTOM_WRIST_ROLL_MIN, CUSTOM_WRIST_ROLL_MAX);
            case WRIST_YAW_ID:
                return constrain(pulseValue, CUSTOM_WRIST_YAW_MIN, CUSTOM_WRIST_YAW_MAX);
            case GRIPPER_SERVO_ID:
                return constrain(pulseValue, CUSTOM_GRIPPER_MIN, CUSTOM_GRIPPER_MAX);
            default:
                return pulseValue;
        }
    } else {
        // 使用默认限位
        switch (servoID) {
            case BASE_SERVO_ID:
                return constrain(pulseValue, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
            case SHOULDER_PITCH_ID:
                return constrain(pulseValue, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
            case SHOULDER_ROLL_ID:
                return constrain(pulseValue, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
            case ELBOW_SERVO_ID:
                return constrain(pulseValue, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
            case WRIST_PITCH_ID:
                return constrain(pulseValue, WRIST_PITCH_MIN, WRIST_PITCH_MAX);
            case WRIST_ROLL_ID:
                return constrain(pulseValue, WRIST_ROLL_MIN, WRIST_ROLL_MAX);
            case WRIST_YAW_ID:
                return constrain(pulseValue, WRIST_YAW_MIN, WRIST_YAW_MAX);
            case GRIPPER_SERVO_ID:
                return constrain(pulseValue, GRIPPER_MIN, GRIPPER_MAX);
            default:
                return pulseValue;
        }
    }
}

// 根据摇杆值计算舵机转动方向
int RobotArmController::calculateServoSpeed(int stickValue, int servoIndex) {
    const int deadZone = 20;
    
    // 转换为-128到127的范围
    stickValue -= 128;
    
    // 在死区内返回0(停止)
    if (abs(stickValue) < deadZone) {
        return 0;
    }
    
    // 根据舵机方向设置和摇杆值计算实际转动速度
    int direction = (stickValue > 0) ? 1 : -1;
    // 应用舵机正反转设置
    return direction * servoDirections[servoIndex] * SERVO_PULSE_SPEED;
}

// 根据手柄输入更新舵机方向
void RobotArmController::updateServoDirections() {
    // 读取PS2手柄状态
    ps2x.read_gamepad();
    
    // 读取摇杆和按钮状态
    int leftStickX = ps2x.Analog(PSS_LX);
    int leftStickY = ps2x.Analog(PSS_LY);
    int rightStickX = ps2x.Analog(PSS_RX);
    int rightStickY = ps2x.Analog(PSS_RY);
    
    // 读取方向键状态
    bool padUp = ps2x.Button(PSB_PAD_UP);
    bool padDown = ps2x.Button(PSB_PAD_DOWN);
    bool padLeft = ps2x.Button(PSB_PAD_LEFT);
    bool padRight = ps2x.Button(PSB_PAD_RIGHT);
    
    // 读取肩部按键状态
    bool l1Pressed = ps2x.Button(PSB_L1);
    bool r1Pressed = ps2x.Button(PSB_R1);
    bool l2Pressed = ps2x.Button(PSB_L2);
    bool r2Pressed = ps2x.Button(PSB_R2);
    
    // 根据摇杆值计算舵机转动速度
    servoSpeedControl[0] = calculateServoSpeed(leftStickX, 0);   // 左摇杆X轴控制底座旋转
    servoSpeedControl[1] = calculateServoSpeed(leftStickY, 1);   // 左摇杆Y轴控制肩部俯仰
    servoSpeedControl[3] = calculateServoSpeed(rightStickY, 3);  // 右摇杆Y轴控制肘部
    servoSpeedControl[5] = calculateServoSpeed(rightStickX, 5);  // 右摇杆X轴控制腕部滚转

    // 根据肩部按键计算肩部滚转速度
    if (l1Pressed) servoSpeedControl[2] = -SERVO_PULSE_SPEED * servoDirections[2];
    else if (r1Pressed) servoSpeedControl[2] = SERVO_PULSE_SPEED * servoDirections[2];
    else servoSpeedControl[2] = 0;
    
    // 根据方向键计算腕部俯仰和偏航速度
    if (padUp) servoSpeedControl[4] = -SERVO_PULSE_SPEED * servoDirections[4];
    else if (padDown) servoSpeedControl[4] = SERVO_PULSE_SPEED * servoDirections[4];
    else servoSpeedControl[4] = 0;
    
    if (padLeft) servoSpeedControl[6] = -SERVO_PULSE_SPEED * servoDirections[6];
    else if (padRight) servoSpeedControl[6] = SERVO_PULSE_SPEED * servoDirections[6];
    else servoSpeedControl[6] = 0;
    
    // 根据L2/R2计算夹爪开合速度
    if (l2Pressed) servoSpeedControl[7] = -SERVO_PULSE_SPEED * servoDirections[7];
    else if (r2Pressed) servoSpeedControl[7] = SERVO_PULSE_SPEED * servoDirections[7];
    else servoSpeedControl[7] = 0;
    
    // 按下三角形按钮，机械臂回到初始位置
    if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
        Serial.println("回到初始位置");
        initRobotArm();
    }
    
    // 按下方形按钮，夹爪完全打开
    if (ps2x.ButtonPressed(PSB_SQUARE)) {
        Serial.println("夹爪完全打开");
        LobotServo servos[1];
        servos[0].ID = GRIPPER_SERVO_ID;
        servos[0].Position = useCustomLimits ? CUSTOM_GRIPPER_MAX : GRIPPER_MAX;
        controller.moveServos(servos, 1, DEFAULT_MOVE_TIME);
        currentPosition[7] = servos[0].Position;
        
        Serial.print("夹爪角度: ");
        Serial.print(PULSE_TO_DEGREE(currentPosition[7]));
        Serial.println("度");
    }
    
    // 按下圆形按钮，夹爪完全闭合
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
        Serial.println("夹爪完全闭合");
        LobotServo servos[1];
        servos[0].ID = GRIPPER_SERVO_ID;
        servos[0].Position = useCustomLimits ? CUSTOM_GRIPPER_MIN : GRIPPER_MIN;
        controller.moveServos(servos, 1, DEFAULT_MOVE_TIME);
        currentPosition[7] = servos[0].Position;
        
        Serial.print("夹爪角度: ");
        Serial.print(PULSE_TO_DEGREE(currentPosition[7]));
        Serial.println("度");
    }
    
    // 按下SELECT+L3组合键切换限位设置
    if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_L3)) {
        useCustomLimits = !useCustomLimits;
        Serial.print("切换限位设置: ");
        if (useCustomLimits) {
            Serial.println("使用自定义限位");
            Serial.println("自定义限位范围(度):");
            Serial.print("底座: ");
            Serial.print(CUSTOM_BASE_MIN_DEG);
            Serial.print("-");
            Serial.println(CUSTOM_BASE_MAX_DEG);
            
            Serial.print("肩部: ");
            Serial.print(CUSTOM_SHOULDER_MIN_DEG);
            Serial.print("-");
            Serial.println(CUSTOM_SHOULDER_MAX_DEG);
            
            Serial.print("肘部: ");
            Serial.print(CUSTOM_ELBOW_MIN_DEG);
            Serial.print("-");
            Serial.println(CUSTOM_ELBOW_MAX_DEG);
            
            Serial.print("夹爪: ");
            Serial.print(CUSTOM_GRIPPER_MIN_DEG);
            Serial.print("-");
            Serial.println(CUSTOM_GRIPPER_MAX_DEG);
        } else {
            Serial.println("使用默认限位");
            Serial.println("默认限位范围(度):");
            Serial.print("底座: ");
            Serial.print(BASE_MIN_DEG);
            Serial.print("-");
            Serial.println(BASE_MAX_DEG);
            
            Serial.print("肩部: ");
            Serial.print(SHOULDER_MIN_DEG);
            Serial.print("-");
            Serial.println(SHOULDER_MAX_DEG);
            
            Serial.print("肘部: ");
            Serial.print(ELBOW_MIN_DEG);
            Serial.print("-");
            Serial.println(ELBOW_MAX_DEG);
            
            Serial.print("夹爪: ");
            Serial.print(GRIPPER_MIN_DEG);
            Serial.print("-");
            Serial.println(GRIPPER_MAX_DEG);
        }
    }
    
    // 按下START按钮，查询当前电池电压和打印当前舵机角度
    if (ps2x.ButtonPressed(PSB_START)) {
        controller.getBatteryVoltage();
        Serial.print("电池电压: ");
        Serial.print(controller.batteryVoltage / 100.0);
        Serial.println("V");
        
        // 打印当前限位模式
        Serial.print("当前限位模式: ");
        Serial.println(useCustomLimits ? "自定义限位" : "默认限位");
        
        // 打印所有舵机的当前角度 (度)
        Serial.println("当前舵机角度(度):");
        for (int i = 0; i < 8; i++) {
            Serial.print("舵机 ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(PULSE_TO_DEGREE(currentPosition[i]));
            Serial.println("度");
        }
    }
}

// 根据速度更新舵机位置
void RobotArmController::moveServos() {
    unsigned long currentTime = millis();
    
    // 每隔一定时间更新舵机位置
    if (currentTime - lastUpdateTime >= SERVO_UPDATE_INTERVAL) {
        lastUpdateTime = currentTime;
        
        bool needToMove = false;
        LobotServo servos[8];
        int servoCount = 0;
        
        // 根据速度更新每个舵机的位置
        for (int i = 0; i < 8; i++) {
            if (servoSpeedControl[i] != 0) {
                // 计算新位置
                uint16_t newPosition = currentPosition[i] + servoSpeedControl[i];
                
                // 限制在安全范围内
                uint8_t servoID = i + 1;  // 舵机ID从1开始
                newPosition = constrainServoAngle(servoID, newPosition);
                
                // 如果到达了限位，停止该舵机
                if (newPosition == currentPosition[i]) {
                    servoSpeedControl[i] = 0;
                    continue;
                }
                
                // 更新当前位置
                currentPosition[i] = newPosition;
                
                // 添加到要移动的舵机列表
                servos[servoCount].ID = servoID;
                servos[servoCount].Position = newPosition;
                servoCount++;
                needToMove = true;
            }
        }
        
        // 输出所有八个舵机的当前状态
        Serial.print("舵机状态:");
        for (int i = 0; i < 8; i++) {
            if (i > 0) Serial.print(",");
            Serial.print("#");
            Serial.print(i + 1);  // 舵机ID
            Serial.print(":");
            Serial.print(currentPosition[i]);  // 舵机当前位置(脉冲值)
            Serial.print("(");
            Serial.print(PULSE_TO_DEGREE(currentPosition[i]), 1);  // 舵机当前角度，保留1位小数
            Serial.print("°)");
        }
        Serial.println();
        
        // 如果有舵机需要移动，发送命令
        if (needToMove && servoCount > 0) {
            // 发送舵机组控制命令
            controller.moveServos(servos, servoCount, SERVO_UPDATE_INTERVAL);
        }
    }
}