/******************************************************
* FileName:      RobotArmWebPage.h
* Date:          2025/05/05
* Description:   机械臂Web控制界面HTML代码
*****************************************************/

#ifndef ROBOT_ARM_WEB_PAGE_H
#define ROBOT_ARM_WEB_PAGE_H

#include <Arduino.h>

// Web页面HTML - 控制界面
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>机械臂控制</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 20px;
      background-color: #f0f0f0;
    }
    .container {
      max-width: 800px;
      margin: 0 auto;
      background-color: white;
      padding: 20px;
      border-radius: 8px;
      box-shadow: 0 4px 8px rgba(0,0,0,0.1);
    }
    h1 {
      color: #333;
      text-align: center;
    }
    .servo-control {
      margin-bottom: 20px;
      padding: 15px;
      background-color: #f9f9f9;
      border-radius: 5px;
    }
    .slider-container {
      display: flex;
      align-items: center;
      margin-bottom: 10px;
    }
    label {
      width: 120px;
      margin-right: 10px;
    }
    input[type="range"] {
      flex-grow: 1;
      margin: 0 10px;
    }
    .value-display {
      width: 40px;
      text-align: center;
    }
    .control-btn {
      width: 40px;
      height: 40px;
      font-size: 18px;
      font-weight: bold;
      padding: 0;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .buttons {
      display: flex;
      justify-content: center;
      margin-top: 20px;
      gap: 15px;
    }
    button {
      background-color: #4CAF50;
      border: none;
      color: white;
      padding: 10px 15px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
      margin: 4px 2px;
      cursor: pointer;
      border-radius: 5px;
    }
    button:hover {
      background-color: #45a049;
    }
    .unload {
      background-color: #f44336;
    }
    .unload:hover {
      background-color: #d32f2f;
    }
    .home {
      background-color: #2196F3;
    }
    .home:hover {
      background-color: #0b7dda;
    }
    .action-buttons {
      display: flex;
      justify-content: space-between;
      margin-top: 20px;
    }
    /* 添加录制和回放按钮的样式 */
    .record {
      background-color: #ff0000;
    }
    .record:hover {
      background-color: #cc0000;
    }
    .stop-record {
      background-color: #ff6600;
    }
    .stop-record:hover {
      background-color: #cc5200;
    }
    .play {
      background-color: #00cc00;
    }
    .play:hover {
      background-color: #009900;
    }
    /* 预设位置相关样式 */
    .preset-section {
      margin-top: 30px;
      padding: 15px;
      background-color: #e9f7ef;
      border-radius: 5px;
      border: 1px solid #ccc;
    }
    /* 添加录制文件管理区域样式 */
    .files-section {
      margin-top: 30px;
      padding: 15px;
      background-color: #e9f2f9;
      border-radius: 5px;
      border: 1px solid #c2d6e4;
    }
    .files-list {
      max-height: 200px;
      overflow-y: auto;
      background-color: #fff;
      border: 1px solid #ddd;
      padding: 10px;
      margin-top: 10px;
      border-radius: 3px;
    }
    .file-item {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 5px 0;
      border-bottom: 1px solid #eee;
    }
    .file-name {
      flex-grow: 1;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
      margin-right: 10px;
    }
    .delete-file {
      background-color: #f44336;
      padding: 5px 10px;
      font-size: 14px;
    }
    .delete-file:hover {
      background-color: #d32f2f;
    }
    .file-buttons {
      margin-top: 10px;
      display: flex;
      justify-content: flex-end;
    }
    .refresh-files {
      background-color: #2196F3;
      margin-right: 10px;
    }
    .refresh-files:hover {
      background-color: #0b7dda;
    }
    /* 添加录制命令显示区样式 */
    .record-commands-section {
      margin-top: 30px;
      padding: 15px;
      background-color: #f9f2f4;
      border-radius: 5px;
      border: 1px solid #e3bfc7;
    }
    .record-commands-area {
      max-height: 200px;
      overflow-y: auto;
      background-color: #fff;
      border: 1px solid #ddd;
      padding: 10px;
      font-family: monospace;
      margin-top: 10px;
      border-radius: 3px;
    }
    .record-status {
      font-weight: bold;
      margin-bottom: 10px;
      color: #333;
    }
    .recording {
      color: #ff0000;
    }
    .playing {
      color: #00cc00;
    }
    .idle {
      color: #666;
    }
    /* 添加运动学模式相关样式 */
    .kinematics-section {
      margin-top: 30px;
      padding: 15px;
      background-color: #f2f8ef;
      border-radius: 5px;
      border: 1px solid #ccc;
      display: none; /* 默认隐藏 */
    }
    .mode-toggle {
      margin-top: 20px;
      text-align: center;
    }
    .mode-btn {
      padding: 10px 20px;
      margin: 0 10px;
      font-weight: bold;
    }
    .active-mode {
      background-color: #28a745;
      color: white;
    }
    .input-group {
      display: flex;
      align-items: center;
      margin-bottom: 10px;
    }
    .input-group label {
      width: 100px;
      margin-right: 10px;
    }
    .input-group input {
      width: 80px;
      text-align: right;
      padding: 5px;
      border: 1px solid #ccc;
      border-radius: 3px;
    }
    .input-group .unit {
      margin-left: 5px;
      color: #666;
      width: 30px;
    }
    .coordinate-controls {
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
    }
    .coordinate-group {
      width: 48%;
      margin-bottom: 15px;
      padding: 10px;
      background-color: #f9f9f9;
      border-radius: 5px;
      border: 1px solid #eee;
    }
    .coordinate-title {
      font-weight: bold;
      margin-bottom: 10px;
      color: #333;
    }
    .move-buttons {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      grid-gap: 5px;
      margin-top: 10px;
    }
    .move-btn {
      padding: 8px 0;
      font-size: 14px;
      text-align: center;
    }
    .move-value {
      grid-column: span 3;
      margin-bottom: 5px;
    }
    .move-value input {
      width: 100%;
      padding: 5px;
      box-sizing: border-box;
      text-align: center;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>机械臂控制面板</h1>
    
    <!-- 添加控制模式切换按钮 -->
    <div class="mode-toggle">
      <button id="jointModeBtn" class="mode-btn active-mode" onclick="switchControlMode(0)">关节角度模式</button>
      <button id="kinematicsModeBtn" class="mode-btn" onclick="switchControlMode(1)">运动学模式</button>
    </div>
    
    <!-- 关节控制区域 -->
    <div id="jointControlArea">
      <div class="servo-control">
        <h2>底座舵机</h2>
        <div class="slider-container">
          <label for="base">旋转控制:</label>
          <button class="control-btn" onclick="decrementServo(1, 'base', 10)">-</button>
          <input type="range" id="base" min="0" max="1000" value="500" class="slider" 
                 oninput="updateSliderValueAndMove(this, 'baseValue', 1)">
          <button class="control-btn" onclick="incrementServo(1, 'base', 10)">+</button>
          <span id="baseValue" class="value-display">500</span>
        </div>
      </div>

      <div class="servo-control">
        <h2>肩部舵机</h2>
        <div class="slider-container">
          <label for="shoulder">俯仰控制:</label>
          <button class="control-btn" onclick="decrementServo(2, 'shoulder', 10)">-</button>
          <input type="range" id="shoulder" min="0" max="1000" value="500" class="slider" 
                 oninput="updateSliderValueAndMove(this, 'shoulderValue', 2)">
          <button class="control-btn" onclick="incrementServo(2, 'shoulder', 10)">+</button>
          <span id="shoulderValue" class="value-display">500</span>
        </div>
        <div class="slider-container">
          <label for="shoulderRoll">滚转控制:</label>
          <button class="control-btn" onclick="decrementServo(3, 'shoulderRoll', 10)">-</button>
          <input type="range" id="shoulderRoll" min="0" max="1000" value="500" class="slider" 
                 oninput="updateSliderValueAndMove(this, 'shoulderRollValue', 3)">
          <button class="control-btn" onclick="incrementServo(3, 'shoulderRoll', 10)">+</button>
          <span id="shoulderRollValue" class="value-display">500</span>
        </div>
      </div>

      <div class="servo-control">
        <h2>肘部舵机</h2>
        <div class="slider-container">
          <label for="elbow">俯仰控制:</label>
          <button class="control-btn" onclick="decrementServo(4, 'elbow', 10)">-</button>
          <input type="range" id="elbow" min="0" max="1000" value="500" class="slider" 
                 oninput="updateSliderValueAndMove(this, 'elbowValue', 4)">
          <button class="control-btn" onclick="incrementServo(4, 'elbow', 10)">+</button>
          <span id="elbowValue" class="value-display">500</span>
        </div>
      </div>

      <div class="servo-control">
        <h2>腕部舵机</h2>
        <div class="slider-container">
          <label for="wrist">俯仰控制:</label>
          <button class="control-btn" onclick="decrementServo(5, 'wrist', 10)">-</button>
          <input type="range" id="wrist" min="0" max="1000" value="500" class="slider" 
                 oninput="updateSliderValueAndMove(this, 'wristValue', 5)">
          <button class="control-btn" onclick="incrementServo(5, 'wrist', 10)">+</button>
          <span id="wristValue" class="value-display">500</span>
        </div>
        <div class="slider-container">
          <label for="wristRoll">滚转控制:</label>
          <button class="control-btn" onclick="decrementServo(6, 'wristRoll', 10)">-</button>
          <input type="range" id="wristRoll" min="0" max="1000" value="500" class="slider" 
                 oninput="updateSliderValueAndMove(this, 'wristRollValue', 6)">
          <button class="control-btn" onclick="incrementServo(6, 'wristRoll', 10)">+</button>
          <span id="wristRollValue" class="value-display">500</span>
        </div>
        <div class="slider-container">
          <label for="wristYaw">偏航控制:</label>
          <button class="control-btn" onclick="decrementServo(7, 'wristYaw', 10)">-</button>
          <input type="range" id="wristYaw" min="0" max="1000" value="500" class="slider" 
                 oninput="updateSliderValueAndMove(this, 'wristYawValue', 7)">
          <button class="control-btn" onclick="incrementServo(7, 'wristYaw', 10)">+</button>
          <span id="wristYawValue" class="value-display">500</span>
        </div>
      </div>

      <div class="servo-control">
        <h2>夹爪舵机</h2>
        <div class="slider-container">
          <label for="gripper">开合控制:</label>
          <button class="control-btn" onclick="decrementServo(8, 'gripper', 10)">-</button>
          <input type="range" id="gripper" min="0" max="1000" value="0" class="slider" 
                 oninput="updateSliderValueAndMove(this, 'gripperValue', 8)">
          <button class="control-btn" onclick="incrementServo(8, 'gripper', 10)">+</button>
          <span id="gripperValue" class="value-display">0</span>
        </div>
      </div>
    </div>

    <!-- 运动学控制区域 -->
    <div id="kinematicsControlArea" class="kinematics-section">
      <h2>末端位置姿态控制</div>
      <div class="coordinate-controls">
        <div class="coordinate-group">
          <div class="coordinate-title">位置控制</div>
          <div class="input-group">
            <label for="posX">X:</label>
            <input type="number" id="posX" value="0">
            <span class="unit">mm</span>
          </div>
          <div class="input-group">
            <label for="posY">Y:</label>
            <input type="number" id="posY" value="0">
            <span class="unit">mm</span>
          </div>
          <div class="input-group">
            <label for="posZ">Z:</label>
            <input type="number" id="posZ" value="0">
            <span class="unit">mm</span>
          </div>
        </div>
        <div class="coordinate-group">
          <div class="coordinate-title">姿态控制</div>
          <div class="input-group">
            <label for="rotRoll">滚转:</label>
            <input type="number" id="rotRoll" value="0">
            <span class="unit">°</span>
          </div>
          <div class="input-group">
            <label for="rotPitch">俯仰:</label>
            <input type="number" id="rotPitch" value="0">
            <span class="unit">°</span>
          </div>
          <div class="input-group">
            <label for="rotYaw">偏航:</label>
            <input type="number" id="rotYaw" value="0">
            <span class="unit">°</span>
          </div>
        </div>
      </div>
      <div class="move-buttons">
        <div class="move-value">
          <input type="number" id="moveStep" value="10" min="1" max="100" step="1">
        </div>
        <button class="move-btn" onclick="moveEndEffector('x', -1)">X-</button>
        <button class="move-btn" onclick="moveEndEffector('x', 1)">X+</button>
        <button class="move-btn" onclick="moveEndEffector('y', -1)">Y-</button>
        <button class="move-btn" onclick="moveEndEffector('y', 1)">Y+</button>
        <button class="move-btn" onclick="moveEndEffector('z', -1)">Z-</button>
        <button class="move-btn" onclick="moveEndEffector('z', 1)">Z+</button>
        <button class="move-btn" onclick="rotateEndEffector('roll', -1)">滚转-</button>
        <button class="move-btn" onclick="rotateEndEffector('roll', 1)">滚转+</button>
        <button class="move-btn" onclick="rotateEndEffector('pitch', -1)">俯仰-</button>
        <button class="move-btn" onclick="rotateEndEffector('pitch', 1)">俯仰+</button>
        <button class="move-btn" onclick="rotateEndEffector('yaw', -1)">偏航-</button>
        <button class="move-btn" onclick="rotateEndEffector('yaw', 1)">偏航+</button>
      </div>
    </div>

    <div class="action-buttons">
      <button class="home" onclick="moveHome()">回到初始位置</button>
      <button class="record" onclick="startRecording()">录制动作组</button>
      <button class="stop-record" onclick="stopRecording()">停止录制</button>
      <button class="play" onclick="playRecording()">播放动作组</button>
      <button class="unload" onclick="unloadServos()">掉电所有舵机</button>
    </div>
    
    <!-- 添加预设位置功能区域 -->
    <div class="preset-section">
      <h2>预设位置</h2>
      <div class="preset-controls">
        <input type="text" id="presetName" placeholder="输入预设名称..." />
        <select id="presetSelect">
          <option value="">-- 选择预设位置 --</option>
        </select>
      </div>
      <div class="preset-buttons">
        <button class="save-preset" onclick="savePreset()">保存当前位置</button>
        <button class="load-preset" onclick="loadPreset()">加载选中预设</button>
        <button class="delete-preset" style="background-color: #f44336;" onclick="deletePreset()">删除预设</button>
      </div>
    </div>
    
    <!-- 添加录制命令显示区域 -->
    <div class="record-commands-section">
      <h2>录制状态</h2>
      <div class="record-status">
        当前状态: <span id="recorderStatus" class="idle">空闲</span>
        <span id="frameCounter"></span>
      </div>
      <div class="record-commands-area" id="recordCommands">
        <!-- 录制的命令将显示在这里 -->
        <div>等待录制命令...</div>
      </div>
    </div>

    <!-- 添加录制文件管理区域 -->
    <div class="files-section">
      <h2>录制文件管理</h2>
      <div class="files-list" id="filesContainer">
        <!-- 录制的文件将显示在这里 -->
        <div>加载录制文件列表中...</div>
      </div>
      <div class="file-buttons">
        <button class="refresh-files" onclick="refreshRecordFiles()">刷新文件列表</button>
        <button class="play" onclick="showPlayDialog()">选择文件播放</button>
      </div>
    </div>
  </div>

  <script>
    // 全局变量，控制轮询间隔
    var pollingInterval = 10; // 毫秒
    var pollingTimer = null;
    var recordStatusTimer = null; // 录制状态轮询计时器
    var lastPositions = [0, 0, 0, 0, 0, 0, 0, 0];
    var sliderDebounceTimers = {}; // 用于滑块的防抖计时器
    var servoMotion = {}; // 用于存储舵机运动状态
    var buttonPressTimer = {}; // 用于按钮长按功能的计时器
    var moveSpeed = 50; // 舵机恒定运动速度(单位:ms),值越小速度越快
    
    // 初始化页面加载时更新当前位置
    window.onload = function() {
      fetchAllServoPositions();
      // 获取当前控制模式
      fetchControlMode();
      // 启动定期轮询
      startPolling();
      // 启动录制状态轮询
      startRecordStatusPolling();
      // 为按钮添加长按事件
      setupButtonPressEvents();
      // 加载预设列表
      loadPresetList();
      // 设置滑条事件
      setupSliderEvents();
      // 获取录制文件列表
      fetchRecordFiles();
    }
    
    // 启动轮询
    function startPolling() {
      if (pollingTimer === null) {
        pollingTimer = setInterval(fetchAllServoPositions, pollingInterval);
        console.log("开始位置数据轮询，间隔：" + pollingInterval + "ms");
      }
    }
    
    // 停止轮询
    function stopPolling() {
      if (pollingTimer !== null) {
        clearInterval(pollingTimer);
        pollingTimer = null;
        console.log("停止位置数据轮询");
      }
    }

    // 启动录制状态轮询
    function startRecordStatusPolling() {
      if (recordStatusTimer === null) {
        recordStatusTimer = setInterval(fetchRecorderStatus, 500); // 每500ms轮询一次
        console.log("开始录制状态轮询，间隔：500ms");
      }
    }
    
    // 停止录制状态轮询
    function stopRecordStatusPolling() {
      if (recordStatusTimer !== null) {
        clearInterval(recordStatusTimer);
        recordStatusTimer = null;
        console.log("停止录制状态轮询");
      }
    }

    // 获取录制状态和命令信息
    function fetchRecorderStatus() {
      fetch('/api/record/status')
      .then(response => response.json())
      .then(data => {
        // 更新状态显示
        const statusElem = document.getElementById('recorderStatus');
        const frameCounterElem = document.getElementById('frameCounter');
        
        // 移除所有类名并添加新的类名
        statusElem.className = '';
        
        if (data.state === 'recording') {
          statusElem.classList.add('recording');
          statusElem.textContent = '录制中';
          frameCounterElem.textContent = ` - 已录制 ${data.frameCount} 帧`;
          
          // 获取命令列表
          fetchRecordedCommands();
        } else if (data.state === 'playing') {
          statusElem.classList.add('playing');
          statusElem.textContent = '播放中';
          frameCounterElem.textContent = '';
          
          // 获取命令列表
          fetchRecordedCommands();
        } else {
          statusElem.classList.add('idle');
          statusElem.textContent = '空闲';
          frameCounterElem.textContent = '';
        }
      })
      .catch(error => {
        console.error("获取录制状态时出错：", error);
      });
    }
    
    // 获取录制的命令信息
    function fetchRecordedCommands() {
      fetch('/api/record/commands')
      .then(response => response.json())
      .then(data => {
        const commandsArea = document.getElementById('recordCommands');
        
        // 如果有命令数据，则更新显示
        if (data.commands && data.commands.length > 0) {
          // 清空现有内容
          commandsArea.innerHTML = '';
          
          // 添加新的命令
          data.commands.forEach(cmd => {
            const cmdElem = document.createElement('div');
            
            // 根据命令类型设置样式
            if (cmd.type === 'info') {
              cmdElem.className = 'info-message';
            } else if (cmd.type === 'error') {
              cmdElem.className = 'error-message';
            } else {
              cmdElem.className = 'command-message';
            }
            
            // 设置内容
            cmdElem.textContent = cmd.text;
            
            // 添加到显示区域
            commandsArea.appendChild(cmdElem);
          });
          
          // 滚动到最底部
          commandsArea.scrollTop = commandsArea.scrollHeight;
        }
      })
      .catch(error => {
        console.error("获取录制命令时出错：", error);
      });
    }

    // 获取所有舵机位置
    function fetchAllServoPositions() {
      fetch('/api/servos')
      .then(response => response.json())
      .then(data => {
        // 检查每个舵机位置是否发生变化
        let positionsChanged = false;
        
        for (let i = 0; i < 8; i++) {
          if (lastPositions[i] !== data.positions[i]) {
            positionsChanged = true;
            lastPositions[i] = data.positions[i];
          }
        }
        
        // 只有当位置发生变化时才更新UI
        if (positionsChanged) {
          updateUIWithPositions(data.positions);
          console.log("舵机位置已更新");
        }
      })
      .catch(error => {
        console.error("获取舵机位置时出错：", error);
      });
    }
    
    // 更新UI显示舵机位置
    function updateUIWithPositions(positions) {
      document.getElementById('base').value = positions[0];
      document.getElementById('baseValue').innerHTML = positions[0];
      
      document.getElementById('shoulder').value = positions[1];
      document.getElementById('shoulderValue').innerHTML = positions[1];
      
      document.getElementById('shoulderRoll').value = positions[2];
      document.getElementById('shoulderRollValue').innerHTML = positions[2];
      
      document.getElementById('elbow').value = positions[3];
      document.getElementById('elbowValue').innerHTML = positions[3];
      
      document.getElementById('wrist').value = positions[4];
      document.getElementById('wristValue').innerHTML = positions[4];
      
      document.getElementById('wristRoll').value = positions[5];
      document.getElementById('wristRollValue').innerHTML = positions[5];
      
      document.getElementById('wristYaw').value = positions[6];
      document.getElementById('wristYawValue').innerHTML = positions[6];
      
      document.getElementById('gripper').value = positions[7];
      document.getElementById('gripperValue').innerHTML = positions[7];
    }

    // 设置按钮长按事件
    function setupButtonPressEvents() {
      const allButtons = document.querySelectorAll('.control-btn');
      allButtons.forEach(button => {
        button.addEventListener('mousedown', startButtonPress);
        button.addEventListener('mouseup', stopButtonPress);
        button.addEventListener('mouseleave', stopButtonPress);
        button.addEventListener('touchstart', function(e) {
          e.preventDefault(); // 阻止默认行为，防止长按触发复制栏
          startButtonPress(e);
        });
        button.addEventListener('touchend', stopButtonPress);
        button.addEventListener('touchcancel', stopButtonPress); // 添加触摸取消事件处理
      });
    }
    
    // 按钮按下开始
    function startButtonPress(event) {
      const button = event.currentTarget;
      const isIncrement = button.textContent === '+';
      const sliderId = isIncrement 
                      ? button.previousElementSibling.id 
                      : button.nextElementSibling.id;
      const servoId = getServoIdFromSliderId(sliderId);
      const step = 5; // 每次移动的步长
      
      // 清除已有的计时器（如果存在）
      if (buttonPressTimer[sliderId]) {
        clearInterval(buttonPressTimer[sliderId]);
        buttonPressTimer[sliderId] = null;
      }
      
      // 立即执行一次
      if (isIncrement) {
        incrementServo(servoId, sliderId, step);
      } else {
        decrementServo(servoId, sliderId, step);
      }
      
      // 设置长按计时器，持续移动
      buttonPressTimer[sliderId] = setInterval(() => {
        if (isIncrement) {
          incrementServo(servoId, sliderId, step);
        } else {
          decrementServo(servoId, sliderId, step);
        }
      }, 100); // 100ms执行一次，实现连续移动效果
    }
    
    // 按钮释放停止
    function stopButtonPress(event) {
      const button = event.currentTarget;
      const isIncrement = button.textContent === '+';
      const sliderId = isIncrement 
                      ? button.previousElementSibling.id 
                      : button.nextElementSibling.id;
      
      // 确保计时器被清除
      if (buttonPressTimer[sliderId]) {
        clearInterval(buttonPressTimer[sliderId]);
        buttonPressTimer[sliderId] = null;
      }
    }

    // 从滑块ID获取舵机ID
    function getServoIdFromSliderId(sliderId) {
      switch(sliderId) {
        case 'base': return 1;
        case 'shoulder': return 2;
        case 'shoulderRoll': return 3;
        case 'elbow': return 4;
        case 'wrist': return 5;
        case 'wristRoll': return 6;
        case 'wristYaw': return 7;
        case 'gripper': return 8;
        default: return 1;
      }
    }

    // 更新滑块值显示
    function updateSliderValue(slider, outputId) {
      document.getElementById(outputId).innerHTML = slider.value;
    }

    // 更新滑块值显示并移动舵机 - 恒定速度版
    function updateSliderValueAndMove(slider, outputId, servoId) {
      const currentValue = parseInt(document.getElementById(outputId).innerHTML);
      const targetValue = parseInt(slider.value);
      document.getElementById(outputId).innerHTML = targetValue;
      
      // 如果值相同，不需要移动
      if(currentValue === targetValue) return;
      
      // 停止之前的运动（如果有）
      if(servoMotion[servoId]) {
        clearInterval(servoMotion[servoId]);
      }
      
      // 计算移动方向和步数
      const direction = targetValue > currentValue ? 1 : -1;
      let currentPos = currentValue;
      
      // 设置恒定速度移动
      servoMotion[servoId] = setInterval(() => {
        // 移动一步
        currentPos += direction * 5; // 每次移动5单位
        
        // 检查是否已达到或超过目标位置
        if((direction > 0 && currentPos >= targetValue) || 
           (direction < 0 && currentPos <= targetValue)) {
          currentPos = targetValue;
          clearInterval(servoMotion[servoId]);
          delete servoMotion[servoId];
        }
        
        // 发送当前位置到舵机
        setServo(servoId, currentPos);
      }, moveSpeed);
    }

    // 设置单个舵机位置 - 简化版，消除防抖，实现实时控制
    function setServo(servoId, position) {
      // 暂时停止轮询，避免用户设置值与轮询获取的值冲突
      stopPolling();
      
      // 创建舵机控制JSON对象
      const servoData = {
        servos: [{
          id: servoId,
          position: parseInt(position)
        }],
        time: 10 // 减少响应时间，提高灵敏度
      };
      
      fetch('/api/servo', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(servoData)
      })
      .then(response => {
        if (response.ok) {
          // 更新本地存储的位置
          lastPositions[servoId-1] = parseInt(position);
        } else {
          console.error('舵机位置设置失败');
        }
        // 设置完成后，延迟重新启动轮询
        setTimeout(startPolling, 100);
      })
      .catch(error => {
        console.error("设置舵机位置时出错：", error);
        setTimeout(startPolling, 100);
      });
    }

    // 增加舵机位置
    function incrementServo(servoId, sliderId, step) {
      var slider = document.getElementById(sliderId);
      var newValue = parseInt(slider.value) + step;
      if (newValue > slider.max) newValue = slider.max;
      slider.value = newValue;
      updateSliderValueAndMove(slider, sliderId + 'Value', servoId);
    }

    // 减少舵机位置
    function decrementServo(servoId, sliderId, step) {
      var slider = document.getElementById(sliderId);
      var newValue = parseInt(slider.value) - step;
      if (newValue < slider.min) newValue = slider.min;
      slider.value = newValue;
      updateSliderValueAndMove(slider, sliderId + 'Value', servoId);
    }

    // 回到初始位置
    function moveHome() {
      // 暂时停止轮询
      stopPolling();
      
      // 添加"回到初始位置中..."提示
      const homeButton = document.querySelector('.home');
      const originalText = homeButton.textContent;
      homeButton.textContent = '位置复位中...';
      homeButton.disabled = true;
      
      // 先立即更新界面上的滑块到初始位置值
      const initialPositions = [500, 500, 500, 500, 500, 500, 500, 0]; // 初始位置值
      updateUIWithPositions(initialPositions);
      
      // 更新本地存储的位置
      for (let i = 0; i < initialPositions.length; i++) {
        lastPositions[i] = initialPositions[i];
      }
      
      fetch('/api/home', {
        method: 'POST'
      })
      .then(response => {
        if (response.ok) {
          console.log('已发送回到初始位置命令');
          
          // 创建动画效果，显示舵机正在移动
          const animationInterval = setInterval(() => {
            homeButton.textContent += '.';
            if (homeButton.textContent.endsWith('位置复位中......')) {
              homeButton.textContent = '位置复位中';
            }
          }, 300);
          
          // 延迟后获取最新位置并重新启动轮询
          setTimeout(() => {
            clearInterval(animationInterval);
            homeButton.textContent = originalText;
            homeButton.disabled = false;
            fetchAllServoPositions();
            startPolling();
          }, 2000); // 增加延迟时间，确保动作完成
        } else {
          homeButton.textContent = originalText;
          homeButton.disabled = false;
          startPolling();
        }
      })
      .catch(error => {
        console.error('回到初始位置出错:', error);
        homeButton.textContent = originalText;
        homeButton.disabled = false;
        startPolling();
      });
    }

    // 掉电所有舵机
    function unloadServos() {
      fetch('/api/unload', {
        method: 'POST'
      })
      .then(response => {
        if (response.ok) {
          console.log('已发送掉电命令');
        }
      });
    }

    // 加载预设位置列表
    function loadPresetList() {
      fetch('/api/presets')
      .then(response => response.json())
      .then(data => {
        const presetSelect = document.getElementById('presetSelect');
        
        // 清除现有选项，保留默认选项
        while (presetSelect.options.length > 1) {
          presetSelect.remove(1);
        }
        
        // 添加从服务器获取的预设
        data.presets.forEach(preset => {
          const option = document.createElement('option');
          option.value = preset.id;
          option.text = preset.name;
          presetSelect.add(option);
        });
      })
      .catch(error => {
        console.error("获取预设列表时出错：", error);
      });
    }
    
    // 保存当前位置为预设
    function savePreset() {
      const presetName = document.getElementById('presetName').value.trim();
      
      if (!presetName) {
        alert('请输入预设位置的名称！');
        return;
      }
      
      // 停止轮询以确保获取最新位置
      stopPolling();
      
      // 获取当前所有舵机位置
      fetch('/api/servos')
      .then(response => response.json())
      .then(data => {
        // 构建预设数据
        const presetData = {
          name: presetName,
          positions: data.positions
        };
        
        // 发送保存请求
        return fetch('/api/preset/save', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(presetData)
        });
      })
      .then(response => {
        if (response.ok) {
          alert('预设位置保存成功！');
          // 清空输入框
          document.getElementById('presetName').value = '';
          // 重新加载预设列表
          loadPresetList();
        } else {
          alert('保存预设位置失败，请重试！');
        }
        // 恢复轮询
        startPolling();
      })
      .catch(error => {
        console.error("保存预设位置时出错：", error);
        alert('保存预设位置时发生错误！');
        startPolling();
      });
    }
    
    // 加载预设位置
    function loadPreset() {
      const presetSelect = document.getElementById('presetSelect');
      const presetId = presetSelect.value;
      
      if (!presetId) {
        alert('请选择要加载的预设位置！');
        return;
      }
      
      // 停止轮询
      stopPolling();
      
      // 发送加载预设请求
      fetch('/api/preset/load', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ name: presetId }) // 使用name参数而非id
      })
      .then(response => {
        if (response.ok) {
          console.log('预设位置加载成功');
          // 延迟后获取最新位置
          setTimeout(() => {
            fetchAllServoPositions();
            startPolling();
          }, 1000);
        } else {
          alert('加载预设位置失败，请重试！');
          startPolling();
        }
      })
      .catch(error => {
        console.error("加载预设位置时出错：", error);
        alert('加载预设位置时发生错误！');
        startPolling();
      });
    }

    // 删除预设位置
    function deletePreset() {
      const presetSelect = document.getElementById('presetSelect');
      const presetId = presetSelect.value;
      
      if (!presetId) {
        alert('请选择要删除的预设位置！');
        return;
      }
      
      if (!confirm(`确定要删除预设"${presetSelect.options[presetSelect.selectedIndex].text}"吗？`)) {
        return; // 用户取消删除
      }
      
      // 发送删除预设请求
      fetch('/api/preset/delete', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ id: presetId })
      })
      .then(response => {
        if (response.ok) {
          alert('预设位置删除成功！');
          // 重新加载预设列表
          loadPresetList();
        } else {
          alert('删除预设位置失败，请重试！');
        }
      })
      .catch(error => {
        console.error("删除预设位置时出错：", error);
        alert('删除预设位置时发生错误！');
      });
    }

    // 设置滑条控件事件
    function setupSliderEvents() {
      const sliders = document.querySelectorAll('input[type="range"]');
      sliders.forEach(slider => {
        // 阻止滑条的默认触摸行为，避免与页面滑动冲突
        slider.addEventListener('touchstart', function(e) {
          e.preventDefault();
        }, { passive: false });
        
        slider.addEventListener('touchmove', function(e) {
          e.preventDefault();
          const touch = e.touches[0];
          const sliderRect = this.getBoundingClientRect();
          const percent = (touch.clientX - sliderRect.left) / sliderRect.width;
          const min = parseInt(this.min);
          const max = parseInt(this.max);
          const value = Math.round(min + percent * (max - min));
          
          // 确保值在有效范围内
          if (value >= min && value <= max) {
            this.value = value;
            // 触发change事件，确保值更新
            const event = new Event('change');
            this.dispatchEvent(event);
          }
        }, { passive: false });
        
        // 当滑条值改变时
        slider.addEventListener('change', function() {
          const servoId = getServoIdFromSliderId(this.id);
          const value = this.value;
          
          // 使用已有的setServo函数发送请求，确保格式一致
          setServo(servoId, value);
        });
      });
    }

    // 禁用整个页面的长按复制行为
    document.addEventListener('contextmenu', function(e) {
      if(e.target.classList.contains('control-btn') || e.target.type === 'range') {
        e.preventDefault();
      }
    });

    // 初始化所有控件
    document.addEventListener('DOMContentLoaded', function() {
      setupButtonPressEvents();
      setupSliderEvents();
      // 禁用页面上控制元素的文本选择
      const controls = document.querySelectorAll('.control-btn, input[type="range"]');
      controls.forEach(control => {
        control.style.webkitUserSelect = 'none';
        control.style.userSelect = 'none';
      });
    });

    // 开始录制动作组
    function startRecording() {
      // 生成唯一的文件名，使用时间戳
      const fileName = `/record_web_${Date.now()}.json`;
      
      fetch('/api/record/start', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ fileName: fileName })
      })
      .then(response => {
        if (response.ok) {
          alert('开始录制动作组！记录文件名：' + fileName);
        } else {
          alert('开始录制失败，请重试！');
        }
      })
      .catch(error => {
        console.error('开始录制出错：', error);
        alert('开始录制时发生错误！');
      });
    }
    
    // 停止录制动作组
    function stopRecording() {
      fetch('/api/record/stop', {
        method: 'POST'
      })
      .then(response => {
        if (response.ok) {
          return response.json();
        } else {
          throw new Error('停止录制失败');
        }
      })
      .then(data => {
        alert(`停止录制成功！共录制了 ${data.frameCount} 帧动作`);
      })
      .catch(error => {
        console.error('停止录制出错：', error);
        alert('停止录制时发生错误！');
      });
    }
    
    // 播放动作组
    function playRecording() {
      fetch('/api/record/files')
      .then(response => {
        if (response.ok) {
          return response.json();
        } else {
          throw new Error('获取录制文件列表失败');
        }
      })
      .then(data => {
        if (data.files && data.files.length > 0) {
          // 获取最新的录制文件
          const latestFile = data.files[data.files.length - 1];
          
          // 播放最新录制的动作组
          return fetch('/api/record/play', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json'
            },
            body: JSON.stringify({ fileName: latestFile })
          });
        } else {
          alert('没有找到录制文件！');
          throw new Error('没有录制文件');
        }
      })
      .then(response => {
        if (response.ok) {
          alert('开始播放动作组');
        } else {
          alert('播放动作组失败，请重试！');
        }
      })
      .catch(error => {
        console.error('播放动作组出错：', error);
      });
    }
    
    // 获取录制文件列表
    function fetchRecordFiles() {
      const container = document.getElementById('filesContainer');
      container.innerHTML = '<div>加载录制文件列表中...</div>';
      
      fetch('/api/record/files')
      .then(response => response.json())
      .then(data => {
        // 清空容器
        container.innerHTML = '';
        
        if (data.files && data.files.length > 0) {
          // 为每个文件创建一个条目
          data.files.forEach(file => {
            // 创建文件项容器
            const fileItem = document.createElement('div');
            fileItem.className = 'file-item';
            
            // 创建文件名显示
            const fileName = document.createElement('div');
            fileName.className = 'file-name';
            // 显示没有路径前缀的文件名
            const displayName = file.replace(/^\//, ''); // 删除开头的斜杠
            fileName.textContent = displayName;
            fileItem.appendChild(fileName);
            
            // 创建操作按钮
            const buttons = document.createElement('div');
            
            // 播放按钮
            const playButton = document.createElement('button');
            playButton.className = 'play';
            playButton.textContent = '播放';
            playButton.onclick = () => playSpecificFile(file);
            buttons.appendChild(playButton);
            
            // 删除按钮
            const deleteButton = document.createElement('button');
            deleteButton.className = 'delete-file';
            deleteButton.textContent = '删除';
            deleteButton.onclick = () => deleteRecordFile(file);
            buttons.appendChild(deleteButton);
            
            fileItem.appendChild(buttons);
            
            // 添加到容器
            container.appendChild(fileItem);
          });
        } else {
          container.innerHTML = '<div>没有找到录制文件</div>';
        }
      })
      .catch(error => {
        console.error("获取录制文件列表时出错：", error);
        container.innerHTML = '<div>获取文件列表失败</div>';
      });
    }
    
    // 刷新文件列表
    function refreshRecordFiles() {
      fetchRecordFiles();
    }
    
    // 删除指定的录制文件
    function deleteRecordFile(fileName) {
      if (!confirm(`确定要删除文件 ${fileName} 吗？`)) {
        return; // 用户取消删除
      }
      
      // 确保文件名以/开头
      const normalizedFileName = fileName.startsWith('/') ? fileName : '/' + fileName;
      
      fetch('/api/record/delete', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ fileName: normalizedFileName })
      })
      .then(response => {
        if (response.ok) {
          alert(`文件 ${fileName} 已成功删除`);
          // 刷新文件列表
          fetchRecordFiles();
        } else {
          alert(`删除文件 ${fileName} 失败，请重试`);
        }
      })
      .catch(error => {
        console.error('删除文件时出错：', error);
        alert('删除文件时发生错误');
      });
    }
    
    // 播放指定的录制文件
    function playSpecificFile(fileName) {
      // 确保文件名以/开头
      const normalizedFileName = fileName.startsWith('/') ? fileName : '/' + fileName;
      
      fetch('/api/record/play', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ fileName: normalizedFileName })
      })
      .then(response => {
        if (response.ok) {
          alert(`开始播放文件 ${fileName}`);
        } else {
          alert(`播放文件 ${fileName} 失败，请重试`);
        }
      })
      .catch(error => {
        console.error('播放文件时出错：', error);
        alert('播放文件时发生错误！');
      });
    }
    
    // 显示播放对话框
    function showPlayDialog() {
      // 获取文件列表
      fetch('/api/record/files')
      .then(response => response.json())
      .then(data => {
        if (data.files && data.files.length > 0) {
          // 创建选择框
          let options = data.files.map(file => 
            `<option value="${file}">${file.replace(/^\//, '')}</option>`
          ).join('');
          
          // 创建简单的对话框
          const fileSelect = prompt(
            "请选择要播放的文件编号:\n\n" + 
            data.files.map((file, index) => 
              `${index + 1}. ${file.replace(/^\//, '')}`
            ).join('\n')
          );
          
          // 检查用户输入
          if (fileSelect !== null) {
            const index = parseInt(fileSelect) - 1;
            if (index >= 0 && index < data.files.length) {
              playSpecificFile(data.files[index]);
            } else {
              alert('请输入有效的文件编号');
            }
          }
        } else {
          alert('没有找到录制文件！');
        }
      })
      .catch(error => {
        console.error('获取文件列表时出错：', error);
        alert('获取文件列表失败');
      });
    }

    // 切换控制模式
    function switchControlMode(mode) {
      const jointControlArea = document.getElementById('jointControlArea');
      const kinematicsControlArea = document.getElementById('kinematicsControlArea');
      const jointModeBtn = document.getElementById('jointModeBtn');
      const kinematicsModeBtn = document.getElementById('kinematicsModeBtn');
      
      // 发送控制模式切换请求
      fetch('/api/control_mode', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ mode: mode })
      })
      .then(response => {
        if (response.ok) {
          if (mode === 0) {
            jointControlArea.style.display = 'block';
            kinematicsControlArea.style.display = 'none';
            jointModeBtn.classList.add('active-mode');
            kinematicsModeBtn.classList.remove('active-mode');
            // 获取最新关节位置
            fetchAllServoPositions();
          } else {
            jointControlArea.style.display = 'none';
            kinematicsControlArea.style.display = 'block';
            jointModeBtn.classList.remove('active-mode');
            kinematicsModeBtn.classList.add('active-mode');
            // 获取最新末端位姿
            fetchEndEffectorPose();
          }
        } else {
          alert('切换控制模式失败，请重试！');
        }
      })
      .catch(error => {
        console.error('切换控制模式时出错：', error);
        alert('切换控制模式时发生错误！');
      });
    }

    // 增量移动末端执行器位置和姿态
    function moveEndEffector(axis, direction) {
      const step = parseFloat(document.getElementById('moveStep').value) || 10;
      let moveData = {
        dx: 0,
        dy: 0,
        dz: 0,
        droll: 0,
        dpitch: 0,
        dyaw: 0
      };
      
      // 设置移动增量
      if (axis === 'x') moveData.dx = direction * step;
      else if (axis === 'y') moveData.dy = direction * step;
      else if (axis === 'z') moveData.dz = direction * step;
      
      // 发送增量移动请求
      fetch('/api/move_end_effector', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(moveData)
      })
      .then(response => {
        if (response.ok) {
          // 移动成功，更新显示的位置
          fetchEndEffectorPose();
        } else {
          return response.json().then(data => {
            alert('移动失败：' + (data.message || '超出工作空间'));
          });
        }
      })
      .catch(error => {
        console.error('增量移动时出错：', error);
      });
    }

    // 增量旋转末端执行器
    function rotateEndEffector(axis, direction) {
      const step = parseFloat(document.getElementById('moveStep').value) || 5;
      let rotateData = {
        dx: 0,
        dy: 0,
        dz: 0,
        droll: 0,
        dpitch: 0,
        dyaw: 0
      };
      
      // 设置旋转增量 (角度值)
      if (axis === 'roll') rotateData.droll = direction * step;
      else if (axis === 'pitch') rotateData.dpitch = direction * step;
      else if (axis === 'yaw') rotateData.dyaw = direction * step;
      
      // 发送增量旋转请求
      fetch('/api/move_end_effector', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(rotateData)
      })
      .then(response => {
        if (response.ok) {
          // 旋转成功，更新显示的姿态
          fetchEndEffectorPose();
        } else {
          return response.json().then(data => {
            alert('旋转失败：' + (data.message || '超出工作空间'));
          });
        }
      })
      .catch(error => {
        console.error('增量旋转时出错：', error);
      });
    }

    // 获取末端执行器位姿
    function fetchEndEffectorPose() {
      fetch('/api/end_effector')
      .then(response => {
        if (response.ok) {
          return response.json();
        } else {
          throw new Error('获取末端位姿失败');
        }
      })
      .then(data => {
        if (data.success) {
          // 更新输入框中的值
          document.getElementById('posX').value = data.position.x.toFixed(1);
          document.getElementById('posY').value = data.position.y.toFixed(1);
          document.getElementById('posZ').value = data.position.z.toFixed(1);
          document.getElementById('rotRoll').value = data.orientation.roll.toFixed(1);
          document.getElementById('rotPitch').value = data.orientation.pitch.toFixed(1);
          document.getElementById('rotYaw').value = data.orientation.yaw.toFixed(1);
        }
      })
      .catch(error => {
        console.error('获取末端位姿时出错：', error);
      });
    }

    // 直接设置末端位姿
    function setEndEffectorPose() {
      // 获取输入框中的值
      const x = parseFloat(document.getElementById('posX').value);
      const y = parseFloat(document.getElementById('posY').value);
      const z = parseFloat(document.getElementById('posZ').value);
      const roll = parseFloat(document.getElementById('rotRoll').value);
      const pitch = parseFloat(document.getElementById('rotPitch').value);
      const yaw = parseFloat(document.getElementById('rotYaw').value);
      
      // 检查输入值是否有效
      if (isNaN(x) || isNaN(y) || isNaN(z) || isNaN(roll) || isNaN(pitch) || isNaN(yaw)) {
        alert('请输入有效的位姿值！');
        return;
      }
      
      const poseData = {
        position: { x, y, z },
        orientation: { roll, pitch, yaw }
      };
      
      // 发送位姿设置请求
      fetch('/api/end_effector', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(poseData)
      })
      .then(response => {
        if (response.ok) {
          return response.json();
        } else {
          return response.json().then(data => {
            throw new Error(data.message || '设置位姿失败');
          });
        }
      })
      .then(data => {
        if (data.success) {
          console.log('末端位姿设置成功');
        } else {
          alert('设置位姿失败：' + (data.message || '可能超出工作空间'));
        }
      })
      .catch(error => {
        console.error('设置末端位姿时出错：', error);
        alert('设置末端位姿失败：' + error.message);
      });
    }

    // 获取当前控制模式
    function fetchControlMode() {
      fetch('/api/control_mode')
      .then(response => {
        if (response.ok) {
          return response.json();
        } else {
          throw new Error('获取控制模式失败');
        }
      })
      .then(data => {
        // 根据当前模式更新界面
        switchControlMode(data.mode);
      })
      .catch(error => {
        console.error('获取控制模式时出错：', error);
      });
    }

    // 为末端位姿输入框添加事件处理
    document.addEventListener('DOMContentLoaded', function() {
      const poseInputs = document.querySelectorAll('#posX, #posY, #posZ, #rotRoll, #rotPitch, #rotYaw');
      poseInputs.forEach(input => {
        input.addEventListener('change', function() {
          setEndEffectorPose();
        });
      });
      
      // 添加末端坐标直接设置按钮
      const coordSection = document.querySelector('.coordinate-controls');
      if (coordSection) {
        const applyBtn = document.createElement('button');
        applyBtn.textContent = '应用位姿';
        applyBtn.className = 'home';
        applyBtn.style.marginTop = '10px';
        applyBtn.style.width = '100%';
        applyBtn.onclick = setEndEffectorPose;
        coordSection.appendChild(applyBtn);
      }
    });
  </script>
</body>
</html>
)rawliteral";

#endif // ROBOT_ARM_WEB_PAGE_H