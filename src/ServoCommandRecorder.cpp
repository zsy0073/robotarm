/******************************************************
* FileName:      ServoCommandRecorder.cpp
* Date:          2025/05/06
* Description:   舵机指令录制器实现文件
*****************************************************/

#include "ServoCommandRecorder.h"

// 声明全局实例，供其他模块使用
ServoCommandRecorder servoRecorder;

// 构造函数
ServoCommandRecorder::ServoCommandRecorder() : 
    frameIndex(0), 
    startTime(0), 
    state(STATE_IDLE), 
    recordFileName(DEFAULT_RECORD_FILE) {
    // 初始化SPIFFS
    initSPIFFS();
}

// 初始化SPIFFS
bool ServoCommandRecorder::initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("错误：SPIFFS挂载失败");
        return false;
    }
    return true;
}

// 开始录制
bool ServoCommandRecorder::startRecording(const char* fileName) {
    // 检查当前状态
    if (state != STATE_IDLE) {
        Serial.println("错误：录制器不处于空闲状态，无法开始录制");
        return false;
    }
    
    // 初始化录制参数
    frameIndex = 0;
    startTime = millis();
    recordFileName = fileName;
    state = STATE_RECORDING;
    
    Serial.print("开始录制舵机指令，录制文件：");
    Serial.println(recordFileName);
    
    return true;
}

// 停止录制
bool ServoCommandRecorder::stopRecording() {
    // 检查当前状态
    if (state != STATE_RECORDING) {
        Serial.println("错误：录制器未处于录制状态，无法停止录制");
        return false;
    }
    
    Serial.print("停止录制，共录制 ");
    Serial.print(frameIndex);
    Serial.println(" 帧");
    
    // 更改状态
    state = STATE_IDLE;
    
    // 保存录制数据到文件
    return saveRecordToFile();
}

// 记录一个舵机命令
bool ServoCommandRecorder::recordCommand(ServoCommandType type, uint8_t servoCount, LobotServo* servos, uint16_t time) {
    // 检查当前状态
    if (state != STATE_RECORDING) {
        // 如果不是在录制状态，不记录
        return false;
    }
    
    // 检查是否超出最大帧数
    if (frameIndex >= MAX_RECORD_FRAMES) {
        Serial.println("警告：已达最大录制帧数，停止录制");
        stopRecording();
        return false;
    }
    
    // 记录当前帧
    RecordFrame& frame = frames[frameIndex];
    frame.timestamp = millis() - startTime;
    frame.type = type;
    frame.servoCount = servoCount;
    frame.time = time;
    
    // 复制舵机数据
    for (uint8_t i = 0; i < servoCount && i < SERVO_COUNT; i++) {
        frame.servos[i] = servos[i];
    }
    
    // 调试信息
    Serial.print("记录第 ");
    Serial.print(frameIndex + 1);
    Serial.print(" 帧，时间戳 ");
    Serial.print(frame.timestamp);
    Serial.print(" ms，舵机数量 ");
    Serial.println(frame.servoCount);
    
    // 帧索引加1
    frameIndex++;
    
    return true;
}

// 保存记录到文件
bool ServoCommandRecorder::saveRecordToFile() {
    // 检查SPIFFS挂载状态
    if (!SPIFFS.begin(true)) {
        Serial.println("错误：SPIFFS挂载失败，无法保存录制文件");
        return false;
    }
    
    // 打开文件准备写入
    File file = SPIFFS.open(recordFileName, FILE_WRITE);
    if (!file) {
        Serial.print("错误：无法打开文件 ");
        Serial.println(recordFileName);
        return false;
    }
    
    // 创建JSON文档
    DynamicJsonDocument doc(64 * 1024); // 64KB应该足够存储大多数录制数据
    
    // 添加录制信息
    doc["frameCount"] = frameIndex;
    doc["totalTime"] = frames[frameIndex - 1].timestamp; // 最后一帧的时间戳就是总录制时间
    
    // 创建帧数组
    JsonArray framesArray = doc.createNestedArray("frames");
    
    // 添加每一帧的数据
    for (uint16_t i = 0; i < frameIndex; i++) {
        JsonObject frameObj = framesArray.createNestedObject();
        
        // 帧基本信息
        frameObj["timestamp"] = frames[i].timestamp;
        frameObj["type"] = (int)frames[i].type;
        frameObj["time"] = frames[i].time;
        
        // 舵机数据
        JsonArray servosArray = frameObj.createNestedArray("servos");
        for (uint8_t j = 0; j < frames[i].servoCount; j++) {
            JsonObject servoObj = servosArray.createNestedObject();
            servoObj["id"] = frames[i].servos[j].ID;
            servoObj["position"] = frames[i].servos[j].Position;
        }
    }
    
    // 序列化JSON并写入文件
    if (serializeJson(doc, file) == 0) {
        Serial.println("错误：写入JSON数据到文件失败");
        file.close();
        return false;
    }
    
    // 关闭文件
    file.close();
    
    Serial.print("成功保存录制数据到文件：");
    Serial.println(recordFileName);
    
    return true;
}

// 从文件加载记录
bool ServoCommandRecorder::loadRecordFromFile(const char* fileName) {
    // 检查SPIFFS挂载状态
    if (!SPIFFS.begin(true)) {
        Serial.println("错误：SPIFFS挂载失败，无法加载录制文件");
        return false;
    }
    
    // 检查文件是否存在
    if (!SPIFFS.exists(fileName)) {
        Serial.print("错误：录制文件不存在：");
        Serial.println(fileName);
        return false;
    }
    
    // 打开文件准备读取
    File file = SPIFFS.open(fileName, FILE_READ);
    if (!file) {
        Serial.print("错误：无法打开文件 ");
        Serial.println(fileName);
        return false;
    }
    
    // 创建JSON文档
    DynamicJsonDocument doc(64 * 1024); // 64KB应该足够加载大多数录制数据
    
    // 解析JSON
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    // 检查解析结果
    if (error) {
        Serial.print("错误：解析JSON失败：");
        Serial.println(error.c_str());
        return false;
    }
    
    // 读取帧数
    frameIndex = doc["frameCount"];
    
    // 检查帧数是否超出限制
    if (frameIndex > MAX_RECORD_FRAMES) {
        Serial.print("警告：文件包含的帧数（");
        Serial.print(frameIndex);
        Serial.print("）超出了最大限制（");
        Serial.print(MAX_RECORD_FRAMES);
        Serial.println("），将被截断");
        
        frameIndex = MAX_RECORD_FRAMES;
    }
    
    // 读取所有帧
    JsonArray framesArray = doc["frames"];
    
    uint16_t i = 0;
    for (JsonObject frameObj : framesArray) {
        // 超出最大帧数，退出循环
        if (i >= MAX_RECORD_FRAMES) break;
        
        // 解析帧数据
        frames[i].timestamp = frameObj["timestamp"];
        frames[i].type = (ServoCommandType)((int)frameObj["type"]);
        frames[i].time = frameObj["time"];
        
        // 解析舵机数据
        JsonArray servosArray = frameObj["servos"];
        frames[i].servoCount = min((size_t)servosArray.size(), (size_t)SERVO_COUNT);
        
        uint8_t j = 0;
        for (JsonObject servoObj : servosArray) {
            // 超出最大舵机数，退出循环
            if (j >= SERVO_COUNT) break;
            
            frames[i].servos[j].ID = servoObj["id"];
            frames[i].servos[j].Position = servoObj["position"];
            j++;
        }
        
        i++;
    }
    
    // 更新实际加载的帧数
    frameIndex = i;
    
    Serial.print("成功从文件加载 ");
    Serial.print(frameIndex);
    Serial.println(" 帧数据");
    
    return true;
}

// 开始回放
bool ServoCommandRecorder::startPlayback(const char* fileName) {
    // 检查当前状态
    if (state != STATE_IDLE) {
        Serial.println("错误：录制器不处于空闲状态，无法开始回放");
        return false;
    }
    
    // 检查SPIFFS挂载状态
    if (!SPIFFS.begin(true)) {
        Serial.println("错误：SPIFFS挂载失败，无法开始回放");
        return false;
    }
    
    // 确保文件路径以"/"开头
    String normalizedPath = fileName;
    if (!normalizedPath.startsWith("/")) {
        normalizedPath = "/" + normalizedPath;
        Serial.print("修正文件路径格式：");
        Serial.println(normalizedPath);
    }
    
    // 检查文件是否存在
    if (!SPIFFS.exists(normalizedPath)) {
        Serial.print("错误：回放文件不存在：");
        Serial.println(normalizedPath);
        
        // 列出所有可用的文件作为调试信息
        Serial.println("可用文件列表:");
        File root = SPIFFS.open("/");
        if (root && root.isDirectory()) {
            File file = root.openNextFile();
            while (file) {
                Serial.print("- ");
                Serial.println(file.name());
                file = root.openNextFile();
            }
        }
        return false;
    }
    
    // 检查文件大小
    File checkFile = SPIFFS.open(normalizedPath, FILE_READ);
    if (!checkFile) {
        Serial.print("错误：无法打开文件 ");
        Serial.println(normalizedPath);
        return false;
    }
    
    size_t fileSize = checkFile.size();
    checkFile.close();
    
    if (fileSize == 0) {
        Serial.println("错误：回放文件为空");
        return false;
    }
    
    Serial.print("回放文件大小: ");
    Serial.print(fileSize);
    Serial.println(" 字节");
    
    // 从文件加载录制数据
    if (!loadRecordFromFile(normalizedPath.c_str())) {
        return false;
    }
    
    // 验证帧数据有效性
    if (frameIndex == 0) {
        Serial.println("错误：回放文件中没有有效的帧数据");
        return false;
    }
    
    // 更新状态参数
    recordFileName = normalizedPath.c_str();
    startTime = millis();
    state = STATE_PLAYING;
    
    Serial.print("开始回放舵机动作，文件：");
    Serial.println(recordFileName);
    Serial.print("帧数据验证: 共有 ");
    Serial.print(frameIndex);
    Serial.println(" 帧数据");
    
    // 打印前几帧数据作为验证
    for (int i = 0; i < min((int)5, (int)frameIndex); i++) {
        Serial.print("帧 ");
        Serial.print(i);
        Serial.print(": 时间戳=");
        Serial.print(frames[i].timestamp);
        Serial.print(", 类型=");
        Serial.print((int)frames[i].type);
        Serial.print(", 舵机数=");
        Serial.print(frames[i].servoCount);
        
        if (frames[i].servoCount > 0) {
            Serial.print(", 舵机ID: ");
            for (uint8_t j = 0; j < min((int)frames[i].servoCount, (int)3); j++) {  // 只打印前3个舵机
                Serial.print(frames[i].servos[j].ID);
                Serial.print("=");
                Serial.print(frames[i].servos[j].Position);
                Serial.print(" ");
            }
        }
        Serial.println();
    }
    
    return true;
}

// 停止回放
void ServoCommandRecorder::stopPlayback() {
    // 检查当前状态
    if (state != STATE_PLAYING) {
        return;
    }
    
    // 更新状态
    state = STATE_IDLE;
    
    Serial.println("停止回放");
}

// 回放下一帧
bool ServoCommandRecorder::playNextFrame(ServoCommand_t& command) {
    // 检查当前状态
    if (state != STATE_PLAYING) {
        return false;
    }
    
    // 静态变量用于记录当前回放的帧索引，添加一个标记变量来跟踪是否首次调用
    static uint16_t currentFrame = 0;
    static unsigned long lastPlaybackStartTime = 0;
    
    // 如果回放开始时间改变，说明这是新的回放，重置索引
    if (lastPlaybackStartTime != startTime) {
        currentFrame = 0;
        lastPlaybackStartTime = startTime;
        Serial.println("检测到新的回放，重置帧索引");
    }
    
    // 如果是刚开始回放，重置回放索引
    if (currentFrame >= frameIndex) {
        currentFrame = 0;
        
        // 如果没有帧可回放，停止回放
        if (frameIndex == 0) {
            Serial.println("错误: 没有可回放的帧数据");
            stopPlayback();
            return false;
        }
    }
    
    // 计算当前时间与回放开始的时间差
    unsigned long currentTime = millis() - startTime;
    
    // 检查是否到了播放下一帧的时间
    if (currentTime >= frames[currentFrame].timestamp) {
        // 填充命令结构体
        command.type = frames[currentFrame].type;
        command.servoCount = frames[currentFrame].servoCount;
        command.time = frames[currentFrame].time;
        
        // 复制舵机数据
        for (uint8_t i = 0; i < command.servoCount; i++) {
            command.servos[i] = frames[currentFrame].servos[i];
        }
        
        // 增强调试信息
        Serial.print("回放第 ");
        Serial.print(currentFrame + 1);
        Serial.print("/");
        Serial.print(frameIndex);
        Serial.print(" 帧，时间戳 ");
        Serial.print(frames[currentFrame].timestamp);
        Serial.print(" ms, 命令类型 ");
        Serial.print(command.type);
        Serial.print(", 舵机数量 ");
        Serial.print(command.servoCount);
        
        if (command.servoCount > 0) {
            Serial.print(", 舵机ID: ");
            for (uint8_t i = 0; i < min((int)command.servoCount, (int)3); i++) {  // 只打印前3个舵机
                Serial.print(command.servos[i].ID);
                Serial.print("=");
                Serial.print(command.servos[i].Position);
                Serial.print(" ");
            }
        }
        Serial.println();
        
        // 移动到下一帧
        currentFrame++;
        
        // 检查是否到达最后一帧
        if (currentFrame >= frameIndex) {
            Serial.println("回放完成");
            stopPlayback();
            currentFrame = 0; // 重置以便下次回放
        }
        
        return true;
    }
    
    return false; // 还没到播放时间
}

// 列出所有录制文件
void ServoCommandRecorder::listRecordFiles(JsonArray& filesArray) {
    // 检查SPIFFS挂载状态
    if (!SPIFFS.begin(true)) {
        Serial.println("错误：SPIFFS挂载失败，无法列出录制文件");
        return;
    }
    
    // 遍历SPIFFS上的所有文件
    File root = SPIFFS.open("/");
    if (!root) {
        Serial.println("错误：无法打开SPIFFS根目录");
        return;
    }
    
    if (!root.isDirectory()) {
        Serial.println("错误：根目录不是目录");
        return;
    }
    
    File file = root.openNextFile();
    while (file) {
        String fileName = file.name();
        
        // 只添加.json文件
        if (fileName.endsWith(".json")) {
            // 直接添加文件名
            filesArray.add(fileName);
        }
        
        file = root.openNextFile();
    }
    
    // 输出找到的文件数量
    Serial.print("找到 ");
    Serial.print(filesArray.size());
    Serial.println(" 个录制文件");
}

// 删除录制文件
bool ServoCommandRecorder::deleteRecordFile(const char* fileName) {
    // 检查SPIFFS挂载状态
    if (!SPIFFS.begin(true)) {
        Serial.println("错误：SPIFFS挂载失败，无法删除录制文件");
        return false;
    }
    
    // 检查文件是否存在
    if (!SPIFFS.exists(fileName)) {
        Serial.print("错误：要删除的录制文件不存在：");
        Serial.println(fileName);
        return false;
    }
    
    // 删除文件
    if (!SPIFFS.remove(fileName)) {
        Serial.print("错误：删除文件失败：");
        Serial.println(fileName);
        return false;
    }
    
    Serial.print("成功删除录制文件：");
    Serial.println(fileName);
    return true;
}