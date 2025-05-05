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
    /* 预设位置相关样式 */
    .preset-section {
      margin-top: 30px;
      padding: 15px;
      background-color: #e9f7ef;
      border-radius: 5px;
      border: 1px solid #ccc;
    }
    .preset-controls {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      align-items: center;
      margin-bottom: 10px;
    }
    .preset-controls input, .preset-controls select {
      padding: 8px;
      border-radius: 4px;
      border: 1px solid #ccc;
    }
    .preset-controls input {
      flex-grow: 1;
      min-width: 150px;
    }
    .preset-buttons {
      display: flex;
      gap: 10px;
      margin-top: 10px;
    }
    .save-preset {
      background-color: #ff9800;
    }
    .save-preset:hover {
      background-color: #e68a00;
    }
    .load-preset {
      background-color: #9c27b0;
    }
    .load-preset:hover {
      background-color: #7b1fa2;
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

    <div class="action-buttons">
      <button class="home" onclick="moveHome()">回到初始位置</button>
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
      </div>
    </div>
  </div>

  <script>
    // 全局变量，控制轮询间隔
    var pollingInterval = 10; // 毫秒
    var pollingTimer = null;
    var lastPositions = [0, 0, 0, 0, 0, 0, 0, 0];
    var sliderDebounceTimers = {}; // 用于滑块的防抖计时器
    var servoMotion = {}; // 用于存储舵机运动状态
    var buttonPressTimer = {}; // 用于按钮长按功能的计时器
    var moveSpeed = 50; // 舵机恒定运动速度(单位:ms),值越小速度越快
    
    // 初始化页面加载时更新当前位置
    window.onload = function() {
      fetchAllServoPositions();
      // 启动定期轮询
      startPolling();
      // 为按钮添加长按事件
      setupButtonPressEvents();
      // 加载预设列表
      loadPresetList();
      // 设置滑条事件
      setupSliderEvents();
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
      
      fetch('/api/home', {
        method: 'POST'
      })
      .then(response => {
        if (response.ok) {
          console.log('已发送回到初始位置命令');
          // 延迟后获取最新位置并重新启动轮询
          setTimeout(() => {
            fetchAllServoPositions();
            startPolling();
          }, 2000);
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
        body: JSON.stringify({ id: presetId })
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
  </script>
</body>
</html>
)rawliteral";

#endif // ROBOT_ARM_WEB_PAGE_H