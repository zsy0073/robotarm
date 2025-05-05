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

class RobotArmController {
public:
    // 构造函数
    RobotArmController();
    
    // 初始化函数
    bool init();
    
    // 主循环函数 - 在Arduino的loop函数中调用
    void update();
    
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
    uint16_t constrainServoAngle(uint8_t servoID, uint16_t pulseValue);
    int calculateServoSpeed(int stickValue, int servoIndex);
    void updateServoDirections();
    void moveServos();
};

#endif // ROBOT_ARM_CONTROLLER_H