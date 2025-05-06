/******************************************************
* FileName:      ServoCommandRecorder.h
* Date:          2025/05/06
* Description:   舵机指令录制器，用于记录带时间戳的舵机指令
*****************************************************/

#ifndef SERVO_COMMAND_RECORDER_H
#define SERVO_COMMAND_RECORDER_H

#include <Arduino.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "RobotArmCommon.h"

// 默认录制文件路径
#define DEFAULT_RECORD_FILE "/servo_records.json"

// 最大录制帧数
#define MAX_RECORD_FRAMES 1000

// 录制状态枚举
enum RecorderState {
    STATE_IDLE,         // 空闲状态，未录制
    STATE_RECORDING,    // 正在录制
    STATE_PLAYING       // 正在回放
};

// 单个录制帧结构体
struct RecordFrame {
    unsigned long timestamp;  // 时间戳（ms）
    ServoCommandType type;    // 命令类型
    uint8_t servoCount;       // 舵机数量
    LobotServo servos[SERVO_COUNT];  // 舵机位置信息
    uint16_t time;            // 执行时间
};

class ServoCommandRecorder {
private:
    // 录制帧数组
    RecordFrame frames[MAX_RECORD_FRAMES];
    
    // 当前录制的帧索引
    uint16_t frameIndex;
    
    // 起始时间
    unsigned long startTime;
    
    // 当前录制状态
    RecorderState state;
    
    // 记录文件名
    String recordFileName;
    
    // 保存记录到文件
    bool saveRecordToFile();
    
    // 从文件加载记录
    bool loadRecordFromFile(const char* fileName);

    // 初始化SPIFFS
    bool initSPIFFS();

public:
    // 构造函数
    ServoCommandRecorder();
    
    // 开始录制
    bool startRecording(const char* fileName = DEFAULT_RECORD_FILE);
    
    // 停止录制
    bool stopRecording();
    
    // 记录一个舵机命令
    bool recordCommand(ServoCommandType type, uint8_t servoCount, LobotServo* servos, uint16_t time);
    
    // 开始回放
    bool startPlayback(const char* fileName = DEFAULT_RECORD_FILE);
    
    // 停止回放
    void stopPlayback();
    
    // 回放下一帧（供FreeRTOS任务调用）
    bool playNextFrame(ServoCommand_t& command);
    
    // 获取当前状态
    RecorderState getState() { return state; }
    
    // 获取已录制帧数
    uint16_t getFrameCount() { return frameIndex; }
    
    // 列出所有录制文件
    void listRecordFiles(JsonArray& filesArray);
    
    // 删除录制文件
    bool deleteRecordFile(const char* fileName);
};

// 外部声明，供其他模块使用
extern ServoCommandRecorder servoRecorder;

#endif // SERVO_COMMAND_RECORDER_H