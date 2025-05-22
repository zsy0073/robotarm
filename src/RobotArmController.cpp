/******************************************************
* FileName:      RobotArmController.cpp
* Date:          5/4/2025
* Description:   机械臂控制器类的实现文件
*****************************************************/

#include "RobotArmController.h"
#include "ServoCommandRecorder.h"
#include <math.h>
#include "SafetyFeatures.h" // 添加安全功能头文件

// 构造函数
RobotArmController::RobotArmController() : 
    controller(Serial),
    useCustomLimits(USE_CUSTOM_LIMITS),
    lastUpdateTime(0),
    controlMode(JOINT_ANGLE_MODE)  // 默认使用关节角度控制模式
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
    
    // 初始化末端位姿
    for (int i = 0; i < 6; i++) {
        endEffectorPose[i] = 0.0f;
    }
    
    // 初始化关节角度
    for (int i = 0; i < ARM_DOF; i++) {
        currentJointAngles[i] = 0.0f;
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

// 初始化PS2控制器接口，供FreeRTOS模块使用
int RobotArmController::initializePS2(uint8_t clkPin, uint8_t cmdPin, uint8_t selPin, uint8_t datPin) {
    // 配置手柄，使用传入的引脚参数，关闭震动和压感功能
    int error = ps2x.config_gamepad(clkPin, cmdPin, selPin, datPin, false, false);
    
    // 输出初始化状态
    switch (error) {
        case 0:
            Serial.println("PS2手柄初始化成功 (RobotArmController)");
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

// 处理PS2输入并生成舵机命令
bool RobotArmController::processPS2AndGenerateCommands(ServoCommandType& type, uint8_t& servoCount, 
                                                     LobotServo servos[], uint16_t& time) {
    // 读取PS2手柄状态 - 使用false参数禁用震动
    ps2x.read_gamepad(false, false);
    
    // 检测Select按钮按下，用于切换控制模式
    bool selectPressed = ps2x.ButtonPressed(PSB_SELECT);
    if (selectPressed) {
        // 切换控制模式
        ArmControlMode newMode = (controlMode == JOINT_ANGLE_MODE) ? 
                                KINEMATICS_MODE : JOINT_ANGLE_MODE;
        setControlMode(newMode);
        
        // 输出当前模式
        Serial.print("控制模式已切换为: ");
        Serial.println((newMode == JOINT_ANGLE_MODE) ? "关节角度模式" : "运动学模式");
        
        return false; // 不生成舵机命令
    }
    
    // 检测三角键是否按下，用于机械臂复位
    bool trianglePressed = ps2x.ButtonPressed(PSB_TRIANGLE);
    
    // 检测其他图形键的按下状态
    bool crossPressed = ps2x.ButtonPressed(PSB_CROSS);     // 十字键，用于开始录制
    bool circlePressed = ps2x.ButtonPressed(PSB_CIRCLE);   // 圆形键，用于停止录制
    bool squarePressed = ps2x.ButtonPressed(PSB_SQUARE);   // 方块键，用于播放录制的动作
    
    // 检查录制和回放功能的按键
    if (crossPressed) {
        // 十字键被按下，开始录制
        Serial.println("开始录制动作组");
        // 定义录制文件名为当前时间戳
        char fileName[32];
        sprintf(fileName, "/record_%lu.json", millis());
        servoRecorder.startRecording(fileName);
        return false; // 不生成实际的舵机命令
    }
    
    if (circlePressed) {
        // 圆形键被按下，停止录制
        Serial.println("停止录制动作组");
        servoRecorder.stopRecording();
        return false; // 不生成实际的舵机命令
    }
    
    if (squarePressed) {
        // 方块键被按下，播放最后录制的动作组
        Serial.println("播放动作组");
        // 获取记录文件列表，并播放最后一个文件
        DynamicJsonDocument doc(1024);
        JsonArray filesArray = doc.createNestedArray("files");
        servoRecorder.listRecordFiles(filesArray);
        
        if (filesArray.size() > 0) {
            const char* lastFile = filesArray[filesArray.size() - 1];
            servoRecorder.startPlayback(lastFile);
        } else {
            Serial.println("没有找到录制文件");
        }
        return false; // 不生成实际的舵机命令
    }
    
    if (trianglePressed) {
        // 如果三角键被按下，触发复位功能并立即执行
        type = MOVE_TO_HOME;
        time = SLOW_MOVE_TIME; // 使用较慢的移动速度进行复位
        
        // 立即复位到初始位置，更新当前状态
        moveToInitialPosition();
        
        // 如果在运动学模式，还需要同步关节角度和更新末端位姿
        if (controlMode == KINEMATICS_MODE) {
            syncServosToJointAngles();
            updateEndEffectorPose();
        }
        
        // 输出复位命令信息
        Serial.println("舵机组命令：复位到初始位置");
        return true;
    }
    
    // 基于当前控制模式处理输入
    if (controlMode == KINEMATICS_MODE) {
        // 运动学模式下的控制逻辑
        return processKinematicsMode(type, servoCount, servos, time);
    } else {
        // 关节角度模式下的控制逻辑
        return processJointAngleMode(type, servoCount, servos, time);
    }
}

// 处理关节角度控制模式下的PS2输入
bool RobotArmController::processJointAngleMode(ServoCommandType& type, uint8_t& servoCount, 
                                             LobotServo servos[], uint16_t& time) {
    // 摇杆死区设置
    const int STICK_DEADZONE = 15;  
    
    // 读取摇杆值
    int lx = ps2x.Analog(PSS_LX) - 128;  // 范围-127到127
    int ly = ps2x.Analog(PSS_LY) - 128;  // 范围-127到127
    int rx = ps2x.Analog(PSS_RX) - 128;  // 范围-127到127
    int ry = ps2x.Analog(PSS_RY) - 128;  // 范围-127到127
    
    // 死区检查
    if (abs(lx) < STICK_DEADZONE) lx = 0;
    if (abs(ly) < STICK_DEADZONE) ly = 0;
    if (abs(rx) < STICK_DEADZONE) rx = 0;
    if (abs(ry) < STICK_DEADZONE) ry = 0;
    
    // 使用恒定的舵机移动增量
    const int FIXED_INCREMENT = 5; // 固定增量值
    const int SERVO_PULSE_DELTA = DEGREE_TO_PULSE(2.4); // 与SERVO_PULSE_SPEED相同的增量
    
    // 根据摇杆方向确定移动增量
    int baseIncrement = (lx == 0) ? 0 : ((lx > 0) ? FIXED_INCREMENT : -FIXED_INCREMENT);
    int shoulderIncrement = (ly == 0) ? 0 : ((ly > 0) ? -FIXED_INCREMENT : FIXED_INCREMENT); // 反转方向
    int elbowIncrement = (ry == 0) ? 0 : ((ry > 0) ? -FIXED_INCREMENT : FIXED_INCREMENT);    // 反转方向
    int wristIncrement = (rx == 0) ? 0 : ((rx > 0) ? FIXED_INCREMENT : -FIXED_INCREMENT);
    
    // 读取方向键和L2/R2状态
    bool padUp = ps2x.Button(PSB_PAD_UP);
    bool padDown = ps2x.Button(PSB_PAD_DOWN);
    bool padLeft = ps2x.Button(PSB_PAD_LEFT);
    bool padRight = ps2x.Button(PSB_PAD_RIGHT);
    bool l1Pressed = ps2x.Button(PSB_L1);
    bool l2Pressed = ps2x.Button(PSB_L2);
    bool r1Pressed = ps2x.Button(PSB_R1);
    bool r2Pressed = ps2x.Button(PSB_R2);
    
    // 处理所有控制输入，包括摇杆、方向键和按钮
    servoCount = 0;
    bool hasMovement = false;
    
    // 1. 左摇杆控制1、2号舵机
    if (lx != 0) {
        uint16_t position = constrainServoAngle(BASE_SERVO_ID, 
                                            currentPosition[BASE_SERVO_ID - 1] + 
                                            baseIncrement * servoDirections[BASE_SERVO_ID - 1]);
        servos[servoCount].ID = BASE_SERVO_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    
    if (ly != 0) {
        uint16_t position = constrainServoAngle(SHOULDER_PITCH_ID, 
                                            currentPosition[SHOULDER_PITCH_ID - 1] + 
                                            shoulderIncrement * servoDirections[SHOULDER_PITCH_ID - 1]);
        servos[servoCount].ID = SHOULDER_PITCH_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    
    // 2. 右摇杆控制3、4号舵机
    if (rx != 0) {
        uint16_t position = constrainServoAngle(SHOULDER_ROLL_ID, 
                                            currentPosition[SHOULDER_ROLL_ID - 1] + 
                                            wristIncrement * servoDirections[SHOULDER_ROLL_ID - 1]);
        servos[servoCount].ID = SHOULDER_ROLL_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    
    if (ry != 0) {
        uint16_t position = constrainServoAngle(ELBOW_SERVO_ID, 
                                            currentPosition[ELBOW_SERVO_ID - 1] + 
                                            elbowIncrement * servoDirections[ELBOW_SERVO_ID - 1]);
        servos[servoCount].ID = ELBOW_SERVO_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    
    // 3. 方向键控制5、6号舵机
    if (padUp) {
        uint16_t position = constrainServoAngle(WRIST_PITCH_ID, 
                                            currentPosition[WRIST_PITCH_ID - 1] - 
                                            SERVO_PULSE_DELTA * servoDirections[WRIST_PITCH_ID - 1]);
        servos[servoCount].ID = WRIST_PITCH_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    else if (padDown) {
        uint16_t position = constrainServoAngle(WRIST_PITCH_ID, 
                                            currentPosition[WRIST_PITCH_ID - 1] + 
                                            SERVO_PULSE_DELTA * servoDirections[WRIST_PITCH_ID - 1]);
        servos[servoCount].ID = WRIST_PITCH_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    
    if (padLeft) {
        uint16_t position = constrainServoAngle(WRIST_ROLL_ID, 
                                            currentPosition[WRIST_ROLL_ID - 1] - 
                                            SERVO_PULSE_DELTA * servoDirections[WRIST_ROLL_ID - 1]);
        servos[servoCount].ID = WRIST_ROLL_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    else if (padRight) {
        uint16_t position = constrainServoAngle(WRIST_ROLL_ID, 
                                            currentPosition[WRIST_ROLL_ID - 1] + 
                                            SERVO_PULSE_DELTA * servoDirections[WRIST_ROLL_ID - 1]);
        servos[servoCount].ID = WRIST_ROLL_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    
    // 4. L1/L2控制7号舵机
    if (l1Pressed) {
        uint16_t position = constrainServoAngle(WRIST_YAW_ID, 
                                            currentPosition[WRIST_YAW_ID - 1] - 
                                            SERVO_PULSE_DELTA * servoDirections[WRIST_YAW_ID - 1]);
        servos[servoCount].ID = WRIST_YAW_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    else if (l2Pressed) {
        uint16_t position = constrainServoAngle(WRIST_YAW_ID, 
                                            currentPosition[WRIST_YAW_ID - 1] + 
                                            SERVO_PULSE_DELTA * servoDirections[WRIST_YAW_ID - 1]);
        servos[servoCount].ID = WRIST_YAW_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    
    // 5. R1/R2控制8号舵机
    if (r1Pressed) {
        uint16_t position = constrainServoAngle(GRIPPER_SERVO_ID, 
                                            currentPosition[GRIPPER_SERVO_ID - 1] + 
                                            SERVO_PULSE_DELTA * servoDirections[GRIPPER_SERVO_ID - 1]);
        servos[servoCount].ID = GRIPPER_SERVO_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    else if (r2Pressed) {
        uint16_t position = constrainServoAngle(GRIPPER_SERVO_ID, 
                                            currentPosition[GRIPPER_SERVO_ID - 1] - 
                                            SERVO_PULSE_DELTA * servoDirections[GRIPPER_SERVO_ID - 1]);
        servos[servoCount].ID = GRIPPER_SERVO_ID;
        servos[servoCount].Position = position;
        servoCount++;
        hasMovement = true;
    }
    
    if (hasMovement && servoCount > 0) {
        time = 20; // 快速响应
        type = MOVE_ALL_SERVOS;
        
        // 命令解析后输出舵机组命令信息
        Serial.print("舵机组命令：[");
        // 输出角度值数组
        for (int i = 0; i < 8; i++) {
            Serial.print(PULSE_TO_DEGREE(currentPosition[i]), 1); // 保留1位小数
            if (i < 7) Serial.print("、");
        }
        Serial.print("][");
        // 输出脉冲值数组
        for (int i = 0; i < 8; i++) {
            Serial.print(currentPosition[i]);
            if (i < 7) Serial.print("、");
        }
        Serial.println("]");
        
        return true;
    }
    
    return false; // 没有生成新的命令
}

// 处理运动学控制模式下的PS2输入
bool RobotArmController::processKinematicsMode(ServoCommandType& type, uint8_t& servoCount, 
                                             LobotServo servos[], uint16_t& time) {
    // 摇杆死区设置
    const int STICK_DEADZONE = 15;  
    
    // 读取摇杆值
    int lx = ps2x.Analog(PSS_LX) - 128;  // 范围-127到127
    int ly = ps2x.Analog(PSS_LY) - 128;  // 范围-127到127
    int rx = ps2x.Analog(PSS_RX) - 128;  // 范围-127到127
    int ry = ps2x.Analog(PSS_RY) - 128;  // 范围-127到127
    
    // 死区检查
    if (abs(lx) < STICK_DEADZONE) lx = 0;
    if (abs(ly) < STICK_DEADZONE) ly = 0;
    if (abs(rx) < STICK_DEADZONE) rx = 0;
    if (abs(ry) < STICK_DEADZONE) ry = 0;
    
    // 运动学控制的增量系数 (米/弧度)
    const float POS_INCREMENT = 0.005f; // 位置增量，每次移动5mm
    const float ANG_INCREMENT = 0.05f; // 角度增量，每次约3度
    
    // 读取方向键和L2/R2状态
    bool padUp = ps2x.Button(PSB_PAD_UP);
    bool padDown = ps2x.Button(PSB_PAD_DOWN);
    bool padLeft = ps2x.Button(PSB_PAD_LEFT);
    bool padRight = ps2x.Button(PSB_PAD_RIGHT);
    bool l1Pressed = ps2x.Button(PSB_L1);
    bool l2Pressed = ps2x.Button(PSB_L2);
    bool r1Pressed = ps2x.Button(PSB_R1);
    bool r2Pressed = ps2x.Button(PSB_R2);
    
    // 位置增量值
    float dx = 0.0f, dy = 0.0f, dz = 0.0f;
    float droll = 0.0f, dpitch = 0.0f, dyaw = 0.0f;
    bool hasMovement = false;
    
    // 1. 左摇杆控制末端XY位置
    if (lx != 0) {
        dx = (lx > 0) ? POS_INCREMENT : -POS_INCREMENT;
        hasMovement = true;
    }
    
    if (ly != 0) {
        dy = (ly > 0) ? -POS_INCREMENT : POS_INCREMENT; // 反转Y轴方向
        hasMovement = true;
    }
    
    // 2. 上下方向键控制Z轴高度
    if (padUp) {
        dz = POS_INCREMENT;
        hasMovement = true;
    }
    else if (padDown) {
        dz = -POS_INCREMENT;
        hasMovement = true;
    }
    
    // 3. 右摇杆控制末端Roll和Pitch角度
    if (rx != 0) {
        droll = (rx > 0) ? ANG_INCREMENT : -ANG_INCREMENT;
        hasMovement = true;
    }
    
    if (ry != 0) {
        dpitch = (ry > 0) ? -ANG_INCREMENT : ANG_INCREMENT; // 反转俯仰方向
        hasMovement = true;
    }
    
    // 4. 左右方向键控制末端Yaw角度
    if (padLeft) {
        dyaw = ANG_INCREMENT;
        hasMovement = true;
    }
    else if (padRight) {
        dyaw = -ANG_INCREMENT;
        hasMovement = true;
    }
    
    // 5. R1/R2控制夹爪
    if (r1Pressed) {
        // 夹爪控制保持原样，不使用运动学
        uint16_t position = constrainServoAngle(GRIPPER_SERVO_ID, 
                                            currentPosition[GRIPPER_SERVO_ID - 1] + 
                                            5 * servoDirections[GRIPPER_SERVO_ID - 1]);
        servos[0].ID = GRIPPER_SERVO_ID;
        servos[0].Position = position;
        servoCount = 1;
        time = 20;
        type = MOVE_SINGLE_SERVO;
        
        // 更新当前夹爪位置
        currentPosition[GRIPPER_SERVO_ID - 1] = position;
        
        return true;
    }
    else if (r2Pressed) {
        // 夹爪控制保持原样，不使用运动学
        uint16_t position = constrainServoAngle(GRIPPER_SERVO_ID, 
                                            currentPosition[GRIPPER_SERVO_ID - 1] - 
                                            5 * servoDirections[GRIPPER_SERVO_ID - 1]);
        servos[0].ID = GRIPPER_SERVO_ID;
        servos[0].Position = position;
        servoCount = 1;
        time = 20;
        type = MOVE_SINGLE_SERVO;
        
        // 更新当前夹爪位置
        currentPosition[GRIPPER_SERVO_ID - 1] = position;
        
        return true;
    }
    
    // 如果有移动，则应用增量
    if (hasMovement) {
        // 执行增量运动学控制
        bool success = moveEndEffectorIncremental(dx, dy, dz, droll, dpitch, dyaw);
        
        if (success) {
            // 准备舵机命令
            for (int i = 0; i < ARM_DOF; i++) {
                servos[i].ID = i + 1;
                servos[i].Position = currentPosition[i];
            }
            servoCount = ARM_DOF;
            time = 20; // 快速响应
            type = MOVE_ALL_SERVOS;
            
            Serial.println("运动学控制：末端增量移动成功");
            // 输出逆解迭代次数和最终误差
            Serial.print("逆解迭代次数: "); 
            Serial.println(kinematics.getLastIterCount());
            Serial.print("最终误差: "); 
            Serial.println(kinematics.getLastError(), 6);
            return true;
        } else {
            Serial.println("运动学控制：末端增量移动失败，可能超出工作空间范围");
            // 失败时也输出逆解迭代次数和最终误差
            Serial.print("逆解迭代次数: "); 
            Serial.println(kinematics.getLastIterCount());
            Serial.print("最终误差: "); 
            Serial.println(kinematics.getLastError(), 6);
            return false;
        }
    }
    
    return false; // 没有生成新的命令
}

// 获取当前舵机位置
uint16_t RobotArmController::getServoPosition(uint8_t servoID) {
    if (servoID >= 1 && servoID <= 8) {
        return currentPosition[servoID - 1];
    }
    return 0;
}

// 设置舵机位置
void RobotArmController::setServoPosition(uint8_t servoID, uint16_t position) {
    if (servoID >= 1 && servoID <= 8) {
        currentPosition[servoID - 1] = position;
    }
}

// 移动到初始位置
void RobotArmController::moveToInitialPosition() {
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
    
    // 外部实际控制由FreeRTOS队列处理
    // 仅更新内部状态
    for (int i = 0; i < 8; i++) {
        currentPosition[i] = servos[i].Position;
        servoSpeedControl[i] = 0; // 停止所有舵机
    }
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
    
    // 使用新的前缀输出舵机位置
    Serial.print("舵机组命令：[");
    // 输出角度值数组
    for (int i = 0; i < 8; i++) {
        Serial.print(PULSE_TO_DEGREE(currentPosition[i]), 1); // 保留1位小数
        if (i < 7) Serial.print("、");
    }
    Serial.print("][");
    // 输出脉冲值数组
    for (int i = 0; i < 8; i++) {
        Serial.print(currentPosition[i]);
        if (i < 7) Serial.print("、");
    }
    Serial.println("]");
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

// 所有舵机掉电
void RobotArmController::unloadAllServos() {
    // 使用正确的方法掉电所有舵机 - 一次掉电一个舵机
    controller.setServoUnload(1, BASE_SERVO_ID);
    controller.setServoUnload(1, SHOULDER_PITCH_ID);
    controller.setServoUnload(1, SHOULDER_ROLL_ID);
    controller.setServoUnload(1, ELBOW_SERVO_ID);
    controller.setServoUnload(1, WRIST_PITCH_ID);
    controller.setServoUnload(1, WRIST_ROLL_ID);
    controller.setServoUnload(1, WRIST_YAW_ID);
    controller.setServoUnload(1, GRIPPER_SERVO_ID);
    
    Serial.println("已执行所有舵机掉电操作");
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
    
    // 确定方向但使用恒定速度
    int direction = (stickValue > 0) ? 1 : -1;
    
    // 应用舵机方向设置并返回恒定速度
    return direction * servoDirections[servoIndex] * SERVO_PULSE_SPEED;
}

// 根据手柄输入更新舵机方向
void RobotArmController::updateServoDirections() {
    // 读取PS2手柄状态 - 禁用震动功能
    ps2x.read_gamepad(false, false);
    
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
    
    // 读取按钮状态
    bool l1Pressed = ps2x.Button(PSB_L1);
    bool r1Pressed = ps2x.Button(PSB_R1);
    bool l2Pressed = ps2x.Button(PSB_L2);
    bool r2Pressed = ps2x.Button(PSB_R2);
    
    // 新的控制方案
    // 1. 左摇杆控制1、2号舵机
    servoSpeedControl[0] = calculateServoSpeed(leftStickX, 0);   // 左摇杆X轴控制1号舵机（底座旋转）
    servoSpeedControl[1] = calculateServoSpeed(leftStickY, 1);   // 左摇杆Y轴控制2号舵机（肩部俯仰）
    
    // 2. 右摇杆控制3、4号舵机
    servoSpeedControl[2] = calculateServoSpeed(rightStickX, 2);  // 右摇杆X轴控制3号舵机（肩部滚转）
    servoSpeedControl[3] = calculateServoSpeed(rightStickY, 3);  // 右摇杆Y轴控制4号舵机（肘部）
    
    // 3. 方向键控制5、6号舵机
    if (padUp) {
        servoSpeedControl[4] = -SERVO_PULSE_SPEED * servoDirections[4];     // 上方向键控制5号舵机（腕部俯仰）向上
    } 
    else if (padDown) {
        servoSpeedControl[4] = SERVO_PULSE_SPEED * servoDirections[4]; // 下方向键控制5号舵机（腕部俯仰）向下
    }
    else {
        servoSpeedControl[4] = 0;
    }
    
    if (padLeft) {
        servoSpeedControl[5] = -SERVO_PULSE_SPEED * servoDirections[5];    // 左方向键控制6号舵机（腕部滚转）向左
    }
    else if (padRight) {
        servoSpeedControl[5] = SERVO_PULSE_SPEED * servoDirections[5]; // 右方向键控制6号舵机（腕部滚转）向右
    }
    else {
        servoSpeedControl[5] = 0;
    }
    
    // 4. L1/L2控制7号舵机（腕部偏航）
    if (l1Pressed) {
        servoSpeedControl[6] = -SERVO_PULSE_SPEED * servoDirections[6];
    }
    else if (l2Pressed) {
        servoSpeedControl[6] = SERVO_PULSE_SPEED * servoDirections[6];
    }
    else {
        servoSpeedControl[6] = 0;
    }
    
    // 5. R1/R2控制8号舵机（夹爪）
    if (r1Pressed) {
        servoSpeedControl[7] = SERVO_PULSE_SPEED * servoDirections[7];    // R1开夹爪
    }
    else if (r2Pressed) {
        servoSpeedControl[7] = -SERVO_PULSE_SPEED * servoDirections[7]; // R2闭夹爪
    }
    else {
        servoSpeedControl[7] = 0;
    }
    
    // SELECT和START按钮的功能已被清空
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
        
        // 如果有舵机需要移动，发送命令
        if (needToMove && servoCount > 0) {
            // 发送舵机组控制命令
            controller.moveServos(servos, servoCount, SERVO_UPDATE_INTERVAL);
        }
    }
}

// 切换控制模式
void RobotArmController::setControlMode(ArmControlMode mode) {
    // 切换控制模式前先同步数据
    if (controlMode != mode) {
        if (mode == KINEMATICS_MODE) {
            // 从关节角度模式切换到运动学模式
            Serial.println("正在初始化运动学模式...");
            
            // 首先移动到一个安全的非奇异姿态
            LobotServo servos[8];
            
            // 设置一个安全的非奇异姿态 - 机械臂处于"准备"位置
            // 底座居中，肩部略抬，肘部弯曲，腕部自然伸展
            servos[0].ID = BASE_SERVO_ID;
            servos[0].Position = HOME_BASE;  // 底座居中 (120°)
            
            servos[1].ID = SHOULDER_PITCH_ID;
            servos[1].Position = DEGREE_TO_PULSE(80);  // 肩部抬起 (80°)
            
            servos[2].ID = SHOULDER_ROLL_ID;
            servos[2].Position = HOME_SHOULDER;  // 肩部滚转居中 (120°)
            
            servos[3].ID = ELBOW_SERVO_ID;
            servos[3].Position = DEGREE_TO_PULSE(100);  // 肘部微弯 (100°)
            
            servos[4].ID = WRIST_PITCH_ID;
            servos[4].Position = DEGREE_TO_PULSE(110);  // 腕部略下倾 (110°)
            
            servos[5].ID = WRIST_ROLL_ID;
            servos[5].Position = HOME_WRIST_ROLL;  // 腕部滚转居中 (120°)
            
            servos[6].ID = WRIST_YAW_ID;
            servos[6].Position = HOME_WRIST_YAW;  // 腕部偏航居中 (120°)
            
            servos[7].ID = GRIPPER_SERVO_ID;
            servos[7].Position = HOME_GRIPPER;  // 夹爪保持初始状态
            
            // 移动到安全姿态
            controller.moveServos(servos, 8, 1000);
            delay(1000);  // 等待移动完成
            
            // 更新当前舵机位置
            for (int i = 0; i < 8; i++) {
                currentPosition[i] = servos[i].Position;
            }
            
            // 然后初始化关节角度数组
            for (int i = 0; i < ARM_DOF && i < 8; i++) {
                if (i < 7) {  // 确保在有效范围内
                    uint8_t servoID = i + 1;
                    currentJointAngles[i] = pulseToJointAngle(servoID, currentPosition[i]);
                }
            }
            
            // 执行正运动学计算初始末端位姿
            float T[4][4];
            bool fkSuccess = kinematics.forwardKinematics(currentJointAngles, T);
            
            if (fkSuccess) {
                // 从变换矩阵中提取位置和欧拉角
                float position[3];
                float euler[3];
                kinematics.matrixToEuler(T, position, euler);
                
                // 更新末端位姿
                endEffectorPose[0] = position[0];  // X
                endEffectorPose[1] = position[1];  // Y
                endEffectorPose[2] = position[2];  // Z
                endEffectorPose[3] = euler[2];     // Roll
                endEffectorPose[4] = euler[1];     // Pitch
                endEffectorPose[5] = euler[0];     // Yaw
                
                Serial.println("末端位姿已成功计算:");
                Serial.print("位置: [");
                Serial.print(position[0], 3); Serial.print(", ");
                Serial.print(position[1], 3); Serial.print(", ");
                Serial.print(position[2], 3); Serial.println("]");
                Serial.print("姿态(rad): [");
                Serial.print(euler[2], 3); Serial.print(", ");
                Serial.print(euler[1], 3); Serial.print(", ");
                Serial.print(euler[0], 3); Serial.println("]");
            } else {
                // 如果正运动学计算失败，使用默认值
                Serial.println("正运动学计算失败，使用默认末端位姿");
                endEffectorPose[0] = 0.20f;  // X = 20cm
                endEffectorPose[1] = 0.0f;   // Y = 0
                endEffectorPose[2] = 0.25f;  // Z = 25cm
                endEffectorPose[3] = 0.0f;   // Roll = 0
                endEffectorPose[4] = 0.0f;   // Pitch = 0
                endEffectorPose[5] = 0.0f;   // Yaw = 0
            }
            
            Serial.println("已切换到运动学控制模式");
        } else {
            // 从运动学模式切换到关节角度模式
            // 无需额外处理，直接切换即可
            Serial.println("已切换到关节角度控制模式");
        }
        
        controlMode = mode;
    }
}

// 运动学模式 - 设置末端执行器位姿
bool RobotArmController::setEndEffectorPose(float x, float y, float z, float roll, float pitch, float yaw) {
    if (controlMode != KINEMATICS_MODE) {
        setControlMode(KINEMATICS_MODE);
    }
    
    // 保存目标位姿
    float targetPose[6] = {x, y, z, roll, pitch, yaw};
    
    // 创建目标位姿矩阵
    float T_target[4][4];
    kinematics.eulerToMatrix(x, y, z, roll, pitch, yaw, T_target);
    
    // 使用当前关节角度作为初始解
    float q_result[ARM_DOF];
    
    // 尝试求解逆运动学
    bool success = kinematics.inverseKinematics(T_target, currentJointAngles, q_result);
    
    if (success) {
        // 更新关节角度
        for (int i = 0; i < ARM_DOF; i++) {
            currentJointAngles[i] = q_result[i];
        }
        
        // 更新末端位姿
        for (int i = 0; i < 6; i++) {
            endEffectorPose[i] = targetPose[i];
        }
        
        // 将关节角度转换为舵机脉冲值并应用
        syncJointAnglesToServos();
        
        // 输出结果
        Serial.println("末端位姿设置成功");
        Serial.print("位置: [");
        Serial.print(x, 3); Serial.print(", ");
        Serial.print(y, 3); Serial.print(", ");
        Serial.print(z, 3); Serial.println("]");
        Serial.print("姿态(rad): [");
        Serial.print(roll, 3); Serial.print(", ");
        Serial.print(pitch, 3); Serial.print(", ");
        Serial.print(yaw, 3); Serial.println("]");
        
        return true;
    } else {
        Serial.println("末端位姿设置失败：逆运动学求解错误");
        Serial.print("逆解迭代次数: "); Serial.println(kinematics.getLastIterCount());
        Serial.print("最终误差: "); Serial.println(kinematics.getLastError(), 6);
        return false;
    }
}

// 运动学模式 - 获取当前末端执行器位姿
bool RobotArmController::getEndEffectorPose(float& x, float& y, float& z, float& roll, float& pitch, float& yaw) {
    if (controlMode != KINEMATICS_MODE) {
        // 如果不在运动学模式，先切换模式
        setControlMode(KINEMATICS_MODE);
    }
    
    // 返回当前末端位姿
    x = endEffectorPose[0];
    y = endEffectorPose[1];
    z = endEffectorPose[2];
    roll = endEffectorPose[3];
    pitch = endEffectorPose[4];
    yaw = endEffectorPose[5];
    
    return true;
}

// 运动学模式 - 增量式移动末端执行器
bool RobotArmController::moveEndEffectorIncremental(float dx, float dy, float dz, float droll, float dpitch, float dyaw) {
    if (controlMode != KINEMATICS_MODE) {
        setControlMode(KINEMATICS_MODE);
    }
    
    // 获取当前位姿
    float x = endEffectorPose[0];
    float y = endEffectorPose[1];
    float z = endEffectorPose[2];
    float roll = endEffectorPose[3];
    float pitch = endEffectorPose[4];
    float yaw = endEffectorPose[5];
    
    // 应用增量
    x += dx;
    y += dy;
    z += dz;
    roll += droll;
    pitch += dpitch;
    yaw += dyaw;
    
    // 设置新的位姿
    return setEndEffectorPose(x, y, z, roll, pitch, yaw);
}

// 运动学模式 - 舵机脉冲值转换为关节角度(弧度)
float RobotArmController::pulseToJointAngle(uint8_t servoID, uint16_t pulse) {
    // 舵机脉冲值范围: 0-1000 对应 0-240度
    float degreeAngle = PULSE_TO_DEGREE(pulse);
    
    // 不同关节可能有不同的零位和旋转方向
    switch(servoID) {
        case BASE_SERVO_ID: // 底座旋转关节
            // 将0-240度映射到-PI到+PI (假设中间位置120度对应0弧度)
            return (degreeAngle - 120.0f) * PI / 180.0f;
            
        case SHOULDER_PITCH_ID: // 肩部俯仰关节
            // 将0-240度映射到适当的范围
            return (120.0f - degreeAngle) * PI / 180.0f; // 反转方向
            
        case SHOULDER_ROLL_ID: // 肩部滚转关节
            return (degreeAngle - 120.0f) * PI / 180.0f;
            
        case ELBOW_SERVO_ID: // 肘部关节
            return (120.0f - degreeAngle) * PI / 180.0f; // 反转方向
            
        case WRIST_PITCH_ID: // 腕部俯仰关节
            return (120.0f - degreeAngle) * PI / 180.0f; // 反转方向
            
        case WRIST_ROLL_ID: // 腕部滚转关节
            return (degreeAngle - 120.0f) * PI / 180.0f;
            
        case WRIST_YAW_ID: // 腕部偏航关节
            return (degreeAngle - 120.0f) * PI / 180.0f;
            
        default: // 夹爪或其他关节
            return (degreeAngle - 120.0f) * PI / 180.0f;
    }
}

// 运动学模式 - 关节角度(弧度)转换为舵机脉冲值
uint16_t RobotArmController::jointAngleToPulse(uint8_t servoID, float angle) {
    float degreeAngle;
    
    // 将弧度转换为角度，并考虑不同关节的零位和方向
    switch(servoID) {
        case BASE_SERVO_ID: // 底座旋转关节
            // 将-PI到+PI映射回0-240度
            degreeAngle = angle * 180.0f / PI + 120.0f;
            break;
            
        case SHOULDER_PITCH_ID: // 肩部俯仰关节
            degreeAngle = 120.0f - angle * 180.0f / PI; // 反转方向
            break;
            
        case SHOULDER_ROLL_ID: // 肩部滚转关节
            degreeAngle = angle * 180.0f / PI + 120.0f;
            break;
            
        case ELBOW_SERVO_ID: // 肘部关节
            degreeAngle = 120.0f - angle * 180.0f / PI; // 反转方向
            break;
            
        case WRIST_PITCH_ID: // 腕部俯仰关节
            degreeAngle = 120.0f - angle * 180.0f / PI; // 反转方向
            break;
            
        case WRIST_ROLL_ID: // 腕部滚转关节
            degreeAngle = angle * 180.0f / PI + 120.0f;
            break;
            
        case WRIST_YAW_ID: // 腕部偏航关节
            degreeAngle = angle * 180.0f / PI + 120.0f;
            break;
            
        default: // 夹爪或其他关节
            degreeAngle = angle * 180.0f / PI + 120.0f;
            break;
    }
    
    // 限制角度范围在0-240度内
    degreeAngle = constrain(degreeAngle, 0.0f, 240.0f);
    
    // 转换为脉冲值
    uint16_t pulse = DEGREE_TO_PULSE(degreeAngle);
    
    // 确保脉冲值在有效范围内
    return constrain(pulse, 0, 1000);
}

// 同步关节角度和舵机位置
void RobotArmController::syncJointAnglesToServos() {
    LobotServo servos[ARM_DOF];
    
    // 将关节角度转换为舵机脉冲值
    for (int i = 0; i < ARM_DOF; i++) {
        uint8_t servoID = i + 1;
        uint16_t pulse = jointAngleToPulse(servoID, currentJointAngles[i]);
        
        // 设置舵机位置
        currentPosition[i] = pulse;
        servos[i].ID = servoID;
        servos[i].Position = pulse;
    }
    
    // 移动所有舵机
    controller.moveServos(servos, ARM_DOF, 100);
    
    Serial.println("已同步关节角度到舵机位置");
}

void RobotArmController::syncServosToJointAngles() {
    // 将当前舵机脉冲值转换为关节角度
    for (int i = 0; i < ARM_DOF; i++) {
        uint8_t servoID = i + 1;
        currentJointAngles[i] = pulseToJointAngle(servoID, currentPosition[i]);
    }
    
    Serial.println("已同步舵机位置到关节角度");
}

// 更新末端执行器位姿
void RobotArmController::updateEndEffectorPose() {
    // 使用正运动学计算末端位姿
    float T[4][4];
    if (kinematics.forwardKinematics(currentJointAngles, T)) {
        // 提取位置和欧拉角
        float position[3];
        float euler[3];
        kinematics.matrixToEuler(T, position, euler);
        
        // 更新末端位姿
        endEffectorPose[0] = position[0];
        endEffectorPose[1] = position[1];
        endEffectorPose[2] = position[2];
        endEffectorPose[3] = euler[2]; // roll
        endEffectorPose[4] = euler[1]; // pitch
        endEffectorPose[5] = euler[0]; // yaw
        
        Serial.println("末端位姿已更新");
        Serial.print("位置: [");
        Serial.print(position[0], 3); Serial.print(", ");
        Serial.print(position[1], 3); Serial.print(", ");
        Serial.print(position[2], 3); Serial.println("]");
        Serial.print("姿态(rad): [");
        Serial.print(euler[2], 3); Serial.print(", ");
        Serial.print(euler[1], 3); Serial.print(", ");        Serial.print(euler[0], 3); Serial.println("]");
    } else {
        Serial.println("末端位姿更新失败：正运动学计算错误");
    }
}

// 检查舵机是否正在移动
bool RobotArmController::isServoMoving(uint8_t servoID) {
    // 检查servoID是否有效，超出范围的ID视为不在移动
    if (servoID < 1 || servoID > SERVO_COUNT) {
        return false;
    }
    
    // 检查速度控制值是否非零 - 表示舵机正在移动
    if (servoSpeedControl[servoID - 1] != 0) {
        return true;
    }
    
    // 如果使用了舵机控制器提供的运动状态检测，也可以在这里添加
    // 例如: return controller.isServoMoving(servoID);
    
    // 当前简单实现：任何非零的速度控制值都表示舵机在移动
    return false;
}

// 检查所有舵机是否都停止移动
bool RobotArmController::allServosIdle() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servoSpeedControl[i] != 0) {
            return false; // 只要有一个舵机在移动，就返回false
        }
    }
    return true; // 所有舵机都不在移动
}