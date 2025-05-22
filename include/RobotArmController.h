/******************************************************
* FileName:      RobotArmController.h
* Date:          2025/05/04
* Description:   机械臂控制器类的头文件
*****************************************************/

#ifndef ROBOT_ARM_CONTROLLER_H
#define ROBOT_ARM_CONTROLLER_H

#include <Arduino.h>
#include "PS2X_lib.h"
#include "LobotServoController.h"
#include "RobotArmConfig.h"
#include "RobotArmCommon.h" // 使用共享头文件获取ServoCommandType等类型定义
#include "ServoCommandRecorder.h" // 添加舵机命令录制器头文件
#include "RobotKinematics.h" // 添加运动学模块头文件

// 机械臂控制模式
enum ArmControlMode {
    JOINT_ANGLE_MODE,   // 关节角度控制模式（默认）
    KINEMATICS_MODE     // 运动学控制模式
};

class RobotArmController {
public:
    // 构造函数
    RobotArmController();
    
    // 初始化函数
    bool init();
    
    // 主循环函数 - 在Arduino的loop函数中调用
    void update();
    
    // 新增接口函数，支持与FreeRTOS集成
    
    // 初始化PS2控制器，返回错误码
    int initializePS2(uint8_t clkPin, uint8_t cmdPin, uint8_t selPin, uint8_t datPin);
    
    // 处理PS2输入并生成命令
    bool processPS2AndGenerateCommands(ServoCommandType& type, uint8_t& servoCount, 
                                      LobotServo servos[], uint16_t& time);
    
    // 获取当前舵机位置
    uint16_t getServoPosition(uint8_t servoID);
    
    // 设置舵机位置（供外部调用）
    void setServoPosition(uint8_t servoID, uint16_t position);
    
    // 移动到初始位置
    void moveToInitialPosition();
    
    // 约束舵机角度在有效范围内
    uint16_t constrainServoAngle(uint8_t servoID, uint16_t pulseValue);
    
    // 掉电所有舵机（供外部调用）
    void unloadAllServos();
    
    // 访问当前位置数组（使其对友元类可见）
    uint16_t* getCurrentPositions() { return currentPosition; }
    
    // 获取舵机控制器对象的引用
    LobotServoController& getServoController() { return controller; }
    
    // 移动舵机组
    void moveServos();

    // 检查舵机是否正在移动
    bool isServoMoving(uint8_t servoID);
    
    // 检查所有舵机是否都停止移动
    bool allServosIdle();
    
    // 运动学模式相关方法
    // 切换控制模式
    void setControlMode(ArmControlMode mode);
    
    // 获取当前控制模式
    ArmControlMode getControlMode() const { return controlMode; }
    
    // 运动学模式 - 设置末端执行器位姿
    bool setEndEffectorPose(float x, float y, float z, float roll, float pitch, float yaw);
    
    // 运动学模式 - 获取当前末端执行器位姿
    bool getEndEffectorPose(float& x, float& y, float& z, float& roll, float& pitch, float& yaw);
    
    // 运动学模式 - 增量式移动末端执行器
    bool moveEndEffectorIncremental(float dx, float dy, float dz, float droll, float dpitch, float dyaw);
    
    // 运动学模式 - 舵机脉冲值转换为关节角度(弧度)
    float pulseToJointAngle(uint8_t servoID, uint16_t pulse);
    
    // 运动学模式 - 关节角度(弧度)转换为舵机脉冲值
    uint16_t jointAngleToPulse(uint8_t servoID, float angle);
    
    // 处理不同控制模式的输入
    bool processJointAngleMode(ServoCommandType& type, uint8_t& servoCount, 
                             LobotServo servos[], uint16_t& time);
    bool processKinematicsMode(ServoCommandType& type, uint8_t& servoCount, 
                             LobotServo servos[], uint16_t& time);
    
private:
    // PS2控制器对象
    PS2X ps2x;
    
    // 舵机控制器对象
    LobotServoController controller;
    
    // 是否使用自定义限位
    bool useCustomLimits;
    
    // 当前舵机位置 (脉冲值)
    uint16_t currentPosition[8];
    
    // 舵机方向设置数组
    const int servoDirections[8] = {
      BASE_SERVO_DIR,        // 底座方向
      SHOULDER_PITCH_DIR,    // 肩部俯仰方向
      SHOULDER_ROLL_DIR,     // 肩部滚转方向
      ELBOW_SERVO_DIR,       // 肘部方向
      WRIST_PITCH_DIR,       // 腕部俯仰方向
      WRIST_ROLL_DIR,        // 腕部滚转方向
      WRIST_YAW_DIR,         // 腕部偏航方向
      GRIPPER_SERVO_DIR      // 夹爪方向
    };
    
    // 舵机转动速度 (每次更新时的脉冲增量) - 所有舵机使用相同的固定速度
    // 该值相当于约2.4度/更新(240度舵机下)
    const int SERVO_PULSE_SPEED = DEGREE_TO_PULSE(2.4);
    
    // 舵机更新时间间隔(ms)
    const unsigned long SERVO_UPDATE_INTERVAL = 20;
    
    // 舵机转动速度指示
    int servoSpeedControl[8];
    
    // 舵机更新时间间隔相关
    unsigned long lastUpdateTime;
    
    // 辅助功能函数
    int initPS2Controller();
    void initRobotArm();
    int calculateServoSpeed(int stickValue, int servoIndex);
    void updateServoDirections();
    
    // 运动学模块相关
    RobotKinematics kinematics;       // 运动学计算对象
    ArmControlMode controlMode;       // 当前控制模式
    float currentJointAngles[ARM_DOF]; // 当前关节角度(弧度)
    float endEffectorPose[6];         // 当前末端位姿[x,y,z,roll,pitch,yaw]
    
    // 同步关节角度和舵机位置
    void syncJointAnglesToServos();
    void syncServosToJointAngles();
    
    // 更新末端执行器位姿
    void updateEndEffectorPose();
};

#endif // ROBOT_ARM_CONTROLLER_H