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
      margin-right: 10px;
    }
    .value-display {
      width: 40px;
      text-align: center;
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
  </style>
</head>
<body>
  <div class="container">
    <h1>机械臂控制面板</h1>
    
    <div class="servo-control">
      <h2>底座舵机</h2>
      <div class="slider-container">
        <label for="base">旋转控制:</label>
        <input type="range" id="base" min="0" max="1000" value="500" class="slider" oninput="updateSliderValue(this, 'baseValue')">
        <span id="baseValue" class="value-display">500</span>
        <button onclick="setServo(1, document.getElementById('base').value)">设置</button>
      </div>
    </div>

    <div class="servo-control">
      <h2>肩部舵机</h2>
      <div class="slider-container">
        <label for="shoulder">俯仰控制:</label>
        <input type="range" id="shoulder" min="0" max="1000" value="500" class="slider" oninput="updateSliderValue(this, 'shoulderValue')">
        <span id="shoulderValue" class="value-display">500</span>
        <button onclick="setServo(2, document.getElementById('shoulder').value)">设置</button>
      </div>
    </div>

    <div class="servo-control">
      <h2>肘部舵机</h2>
      <div class="slider-container">
        <label for="elbow">俯仰控制:</label>
        <input type="range" id="elbow" min="0" max="1000" value="500" class="slider" oninput="updateSliderValue(this, 'elbowValue')">
        <span id="elbowValue" class="value-display">500</span>
        <button onclick="setServo(4, document.getElementById('elbow').value)">设置</button>
      </div>
    </div>

    <div class="servo-control">
      <h2>腕部舵机</h2>
      <div class="slider-container">
        <label for="wrist">俯仰控制:</label>
        <input type="range" id="wrist" min="0" max="1000" value="500" class="slider" oninput="updateSliderValue(this, 'wristValue')">
        <span id="wristValue" class="value-display">500</span>
        <button onclick="setServo(5, document.getElementById('wrist').value)">设置</button>
      </div>
    </div>

    <div class="servo-control">
      <h2>夹爪舵机</h2>
      <div class="slider-container">
        <label for="gripper">开合控制:</label>
        <input type="range" id="gripper" min="0" max="1000" value="0" class="slider" oninput="updateSliderValue(this, 'gripperValue')">
        <span id="gripperValue" class="value-display">0</span>
        <button onclick="setServo(8, document.getElementById('gripper').value)">设置</button>
      </div>
    </div>

    <div class="action-buttons">
      <button class="home" onclick="moveHome()">回到初始位置</button>
      <button class="unload" onclick="unloadServos()">掉电所有舵机</button>
    </div>
  </div>

  <script>
    // 初始化页面加载时更新当前位置
    window.onload = function() {
      fetchAllServoPositions();
    }

    // 获取所有舵机位置
    function fetchAllServoPositions() {
      fetch('/api/servos')
      .then(response => response.json())
      .then(data => {
        document.getElementById('base').value = data.positions[0];
        document.getElementById('baseValue').innerHTML = data.positions[0];
        
        document.getElementById('shoulder').value = data.positions[1];
        document.getElementById('shoulderValue').innerHTML = data.positions[1];
        
        document.getElementById('elbow').value = data.positions[3];
        document.getElementById('elbowValue').innerHTML = data.positions[3];
        
        document.getElementById('wrist').value = data.positions[4];
        document.getElementById('wristValue').innerHTML = data.positions[4];
        
        document.getElementById('gripper').value = data.positions[7];
        document.getElementById('gripperValue').innerHTML = data.positions[7];
      });
    }

    // 更新滑块值显示
    function updateSliderValue(slider, outputId) {
      document.getElementById(outputId).innerHTML = slider.value;
    }

    // 设置单个舵机位置
    function setServo(servoId, position) {
      fetch('/api/servo', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          id: servoId,
          position: parseInt(position),
          time: 1000
        })
      })
      .then(response => {
        if (response.ok) {
          console.log('舵机位置设置成功');
        } else {
          console.error('舵机位置设置失败');
        }
      });
    }

    // 回到初始位置
    function moveHome() {
      fetch('/api/home', {
        method: 'POST'
      })
      .then(response => {
        if (response.ok) {
          console.log('已发送回到初始位置命令');
          setTimeout(fetchAllServoPositions, 1500);
        }
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
  </script>
</body>
</html>
)rawliteral";

#endif // ROBOT_ARM_WEB_PAGE_H