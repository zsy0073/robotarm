/******************************************************
* FileName:      RobotArmConfig.h
* Date:          2025/05/04
* Description:   机械臂项目的配置文件，包含可随时修改的自定义声明
*****************************************************/

#ifndef ROBOT_ARM_CONFIG_H
#define ROBOT_ARM_CONFIG_H

// 舵机配置
#define SERVO_COUNT 8  // 机械臂使用的舵机数量(更新为8个舵机)

// 舵机ID定义（七轴SRS结构机械臂 + 夹爪）  
#define BASE_SERVO_ID         1   // 底座旋转舵机ID  
#define SHOULDER_PITCH_ID     2   // 肩部俯仰舵机ID  
#define SHOULDER_ROLL_ID      3   // 肩部滚转舵机ID（冗余轴）  
#define ELBOW_SERVO_ID        4   // 肘部舵机ID  
#define WRIST_PITCH_ID        5   // 腕部俯仰舵机ID  
#define WRIST_ROLL_ID         6   // 腕部滚转舵机ID（冗余轴）  
#define WRIST_YAW_ID          7   // 腕部偏航舵机ID（第7轴）  
#define GRIPPER_SERVO_ID      8   // 夹爪舵机ID

// 舵机正反转设置（1表示正转，-1表示反转）
#define BASE_SERVO_DIR        1   // 底座舵机方向
#define SHOULDER_PITCH_DIR   -1   // 肩部俯仰舵机方向（反转）
#define SHOULDER_ROLL_DIR     1   // 肩部滚转舵机方向
#define ELBOW_SERVO_DIR      -1   // 肘部舵机方向（反转）
#define WRIST_PITCH_DIR       1   // 腕部俯仰舵机方向
#define WRIST_ROLL_DIR        1   // 腕部滚转舵机方向
#define WRIST_YAW_DIR         1   // 腕部偏航舵机方向
#define GRIPPER_SERVO_DIR     1   // 夹爪舵机方向

// 舵机角度换算关系（240度舵机）
// Lobot舵机控制器使用的范围是0-1000，对应实际舵机的0-240度
#define SERVO_MIN_PULSE        0    // 舵机最小脉冲位置（0度）
#define SERVO_MAX_PULSE     1000    // 舵机最大脉冲位置（240度）
#define DEGREE_TO_PULSE(deg) ((deg) * 1000 / 240)  // 角度转脉冲值
#define PULSE_TO_DEGREE(pul) ((pul) * 240 / 1000)  // 脉冲值转角度

// 舵机角度限制 (基于240度舵机实际角度，单位:度)
#define BASE_MIN_DEG          0     // 底座最小角度 (0度)
#define BASE_MAX_DEG        240     // 底座最大角度 (240度)
#define SHOULDER_MIN_DEG     48     // 肩部最小角度 (约48度)
#define SHOULDER_MAX_DEG    192     // 肩部最大角度 (约192度)
#define ELBOW_MIN_DEG        48     // 肘部最小角度 (约48度)
#define ELBOW_MAX_DEG       192     // 肘部最大角度 (约192度)
#define WRIST_PITCH_MIN_DEG  48     // 腕部俯仰最小角度 (约48度)
#define WRIST_PITCH_MAX_DEG 192     // 腕部俯仰最大角度 (约192度)
#define WRIST_ROLL_MIN_DEG    0     // 腕部滚转最小角度 (0度)
#define WRIST_ROLL_MAX_DEG  240     // 腕部滚转最大角度 (240度)
#define WRIST_YAW_MIN_DEG    48     // 腕部偏航最小角度 (约48度)
#define WRIST_YAW_MAX_DEG   192     // 腕部偏航最大角度 (约192度)
#define GRIPPER_MIN_DEG       0     // 夹爪最小角度 (0度，闭合)
#define GRIPPER_MAX_DEG     120     // 夹爪最大角度 (120度，完全打开)

// 舵机角度限制 (脉冲值，与上面的度数对应)
#define BASE_MIN_ANGLE      DEGREE_TO_PULSE(BASE_MIN_DEG)
#define BASE_MAX_ANGLE      DEGREE_TO_PULSE(BASE_MAX_DEG)
#define SHOULDER_MIN_ANGLE  DEGREE_TO_PULSE(SHOULDER_MIN_DEG)
#define SHOULDER_MAX_ANGLE  DEGREE_TO_PULSE(SHOULDER_MAX_DEG)
#define ELBOW_MIN_ANGLE     DEGREE_TO_PULSE(ELBOW_MIN_DEG)
#define ELBOW_MAX_ANGLE     DEGREE_TO_PULSE(ELBOW_MAX_DEG)
#define WRIST_PITCH_MIN     DEGREE_TO_PULSE(WRIST_PITCH_MIN_DEG)
#define WRIST_PITCH_MAX     DEGREE_TO_PULSE(WRIST_PITCH_MAX_DEG)
#define WRIST_ROLL_MIN      DEGREE_TO_PULSE(WRIST_ROLL_MIN_DEG)
#define WRIST_ROLL_MAX      DEGREE_TO_PULSE(WRIST_ROLL_MAX_DEG)
#define WRIST_YAW_MIN       DEGREE_TO_PULSE(WRIST_YAW_MIN_DEG)
#define WRIST_YAW_MAX       DEGREE_TO_PULSE(WRIST_YAW_MAX_DEG)
#define GRIPPER_MIN         DEGREE_TO_PULSE(GRIPPER_MIN_DEG)
#define GRIPPER_MAX         DEGREE_TO_PULSE(GRIPPER_MAX_DEG)

// 自定义限位设置 - 是否启用
#define USE_CUSTOM_LIMITS true    // 设置为true启用自定义限位，false使用默认限位

// 舵机自定义限位角度 (可根据实际使用情况调整，基于240度舵机实际角度，单位:度)
// 这些是用户可根据实际情况自定义的限位，用于不同场景下的安全限制
#define CUSTOM_BASE_MIN_DEG          30     // 底座最小角度
#define CUSTOM_BASE_MAX_DEG          210    // 底座最大角度
#define CUSTOM_SHOULDER_MIN_DEG      60     // 肩部最小角度
#define CUSTOM_SHOULDER_MAX_DEG      180    // 肩部最大角度
#define CUSTOM_ELBOW_MIN_DEG         60     // 肘部最小角度
#define CUSTOM_ELBOW_MAX_DEG         180    // 肘部最大角度
#define CUSTOM_WRIST_PITCH_MIN_DEG   60     // 腕部俯仰最小角度
#define CUSTOM_WRIST_PITCH_MAX_DEG   180    // 腕部俯仰最大角度
#define CUSTOM_WRIST_ROLL_MIN_DEG    30     // 腕部滚转最小角度
#define CUSTOM_WRIST_ROLL_MAX_DEG    210    // 腕部滚转最大角度
#define CUSTOM_WRIST_YAW_MIN_DEG     60     // 腕部偏航最小角度
#define CUSTOM_WRIST_YAW_MAX_DEG     180    // 腕部偏航最大角度
#define CUSTOM_GRIPPER_MIN_DEG       0      // 夹爪最小角度
#define CUSTOM_GRIPPER_MAX_DEG       90     // 夹爪最大角度 (调整为更小的开合范围)

// 自定义限位脉冲值 (由角度转换得到)
#define CUSTOM_BASE_MIN          DEGREE_TO_PULSE(CUSTOM_BASE_MIN_DEG)
#define CUSTOM_BASE_MAX          DEGREE_TO_PULSE(CUSTOM_BASE_MAX_DEG)
#define CUSTOM_SHOULDER_MIN      DEGREE_TO_PULSE(CUSTOM_SHOULDER_MIN_DEG)
#define CUSTOM_SHOULDER_MAX      DEGREE_TO_PULSE(CUSTOM_SHOULDER_MAX_DEG)
#define CUSTOM_ELBOW_MIN         DEGREE_TO_PULSE(CUSTOM_ELBOW_MIN_DEG)
#define CUSTOM_ELBOW_MAX         DEGREE_TO_PULSE(CUSTOM_ELBOW_MAX_DEG)
#define CUSTOM_WRIST_PITCH_MIN   DEGREE_TO_PULSE(CUSTOM_WRIST_PITCH_MIN_DEG)
#define CUSTOM_WRIST_PITCH_MAX   DEGREE_TO_PULSE(CUSTOM_WRIST_PITCH_MAX_DEG)
#define CUSTOM_WRIST_ROLL_MIN    DEGREE_TO_PULSE(CUSTOM_WRIST_ROLL_MIN_DEG)
#define CUSTOM_WRIST_ROLL_MAX    DEGREE_TO_PULSE(CUSTOM_WRIST_ROLL_MAX_DEG)
#define CUSTOM_WRIST_YAW_MIN     DEGREE_TO_PULSE(CUSTOM_WRIST_YAW_MIN_DEG)
#define CUSTOM_WRIST_YAW_MAX     DEGREE_TO_PULSE(CUSTOM_WRIST_YAW_MAX_DEG)
#define CUSTOM_GRIPPER_MIN       DEGREE_TO_PULSE(CUSTOM_GRIPPER_MIN_DEG)
#define CUSTOM_GRIPPER_MAX       DEGREE_TO_PULSE(CUSTOM_GRIPPER_MAX_DEG)

// 舵机运动参数
#define DEFAULT_MOVE_TIME   1000  // 默认舵机运动时间(毫秒)
#define FAST_MOVE_TIME      500   // 快速运动时间(毫秒)
#define SLOW_MOVE_TIME      2000  // 慢速运动时间(毫秒)


// ESP32S3与舵机控制板通信引脚
#define SERVO_SERIAL_TX     16    // ESP32S3连接舵机控制板的TX引脚
#define SERVO_SERIAL_RX     17    // ESP32S3连接舵机控制板的RX引脚

// PS2手柄控制参数
#define PS2_DAT_PIN         12    // PS2手柄数据引脚
#define PS2_CMD_PIN         11    // PS2手柄命令引脚
#define PS2_SEL_PIN         10    // PS2手柄选择引脚
#define PS2_CLK_PIN         9    // PS2手柄时钟引脚

// 串口通信参数
#define SERIAL_BAUDRATE     115200

// 预设位置 (中间位置，大约120度，对应脉冲值500)
#define HOME_BASE           DEGREE_TO_PULSE(120)   // 初始位置 - 底座
#define HOME_SHOULDER       DEGREE_TO_PULSE(120)   // 初始位置 - 肩部
#define HOME_ELBOW          DEGREE_TO_PULSE(120)   // 初始位置 - 肘部
#define HOME_WRIST_PITCH    DEGREE_TO_PULSE(120)   // 初始位置 - 腕部俯仰
#define HOME_WRIST_ROLL     DEGREE_TO_PULSE(120)   // 初始位置 - 腕部旋转
#define HOME_WRIST_YAW      DEGREE_TO_PULSE(120)   // 初始位置 - 腕部偏航
#define HOME_GRIPPER        DEGREE_TO_PULSE(0)     // 初始位置 - 夹爪(闭合)

#endif // ROBOT_ARM_CONFIG_H