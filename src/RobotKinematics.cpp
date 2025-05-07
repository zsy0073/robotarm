#include "RobotKinematics.h"
#include <math.h>

RobotKinematics::RobotKinematics() {
    resetToDefaultParams();
}

void RobotKinematics::resetToDefaultParams() {
    // 设置默认MDH参数
    _alpha[0] = 0.0f;
    _alpha[1] = -PI/2;
    _alpha[2] = PI/2;
    _alpha[3] = -PI/2;
    _alpha[4] = PI/2;
    _alpha[5] = -PI/2;
    _alpha[6] = PI/2;
    
    // 默认连杆长度
    for(int i = 0; i < ARM_DOF; i++) {
        _a[i] = 0.0f;
    }
    
    // 默认连杆偏距
    _d[0] = 0.1875f;
    _d[1] = 0.0f;
    _d[2] = 0.162f;
    _d[3] = 0.0f;
    _d[4] = 0.162f;
    _d[5] = 0.0f;
    _d[6] = 0.138f;
    
    // 设置默认关节限位 (±120°)
    for(int i = 0; i < ARM_DOF; i++) {
        _joint_lower[i] = -120.0f * PI / 180.0f;
        _joint_upper[i] = 120.0f * PI / 180.0f;
    }
    
    // 默认使用关节限位
    _use_joint_limits = true;
    
    // 设置默认求解参数
    _max_iter = DEFAULT_MAX_ITER;
    _tol = DEFAULT_TOL;
    _damping = DEFAULT_DAMPING;
    _damping_factor = DEFAULT_DAMPING_FACTOR;
    
    // 初始化状态变量
    _last_iter_count = 0;
    _last_error = 0.0f;
}

// 设置MDH参数
void RobotKinematics::setMDHParams(float* alpha_arr, float* a_arr, float* d_arr) {
    for(int i = 0; i < ARM_DOF; i++) {
        _alpha[i] = alpha_arr[i];
        _a[i] = a_arr[i];
        _d[i] = d_arr[i];
    }
}

// 获取MDH参数
void RobotKinematics::getMDHParams(float* alpha_out, float* a_out, float* d_out) const {
    for(int i = 0; i < ARM_DOF; i++) {
        if(alpha_out) alpha_out[i] = _alpha[i];
        if(a_out) a_out[i] = _a[i];
        if(d_out) d_out[i] = _d[i];
    }
}

// 设置关节限位
void RobotKinematics::setJointLimits(float* lower_limits, float* upper_limits) {
    for(int i = 0; i < ARM_DOF; i++) {
        _joint_lower[i] = lower_limits[i];
        _joint_upper[i] = upper_limits[i];
    }
}

// 启用或禁用关节限位
void RobotKinematics::enableJointLimits(bool enable) {
    _use_joint_limits = enable;
}

// 设置求解参数
void RobotKinematics::setSolverParams(int max_iter, float tolerance, float damping) {
    _max_iter = max_iter;
    _tol = tolerance;
    _damping = damping;
}

// 正运动学 - 计算末端执行器位姿
bool RobotKinematics::forwardKinematics(const float* q, float T[4][4]) {
    // 初始化为单位矩阵
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            T[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 对每个关节计算变换矩阵并串联
    float A[4][4];
    float temp[4][4];
    
    for(int i = 0; i < ARM_DOF; i++) {
        float ca = cos(_alpha[i]);
        float sa = sin(_alpha[i]);
        float ct = cos(q[i]);
        float st = sin(q[i]);
        
        // MDH变换矩阵
        A[0][0] = ct;    A[0][1] = -st;   A[0][2] = 0.0f;  A[0][3] = _a[i];
        A[1][0] = st*ca; A[1][1] = ct*ca; A[1][2] = -sa;   A[1][3] = -sa*_d[i];
        A[2][0] = st*sa; A[2][1] = ct*sa; A[2][2] = ca;    A[2][3] = ca*_d[i];
        A[3][0] = 0.0f;  A[3][1] = 0.0f;  A[3][2] = 0.0f;  A[3][3] = 1.0f;
        
        // 复制当前T到临时变量
        for(int r = 0; r < 4; r++) {
            for(int c = 0; c < 4; c++) {
                temp[r][c] = T[r][c];
            }
        }
        
        // T = T * A
        matrixMultiply4x4(temp, A, T);
    }
    
    return true;
}

// 逆运动学 - 计算关节角度
bool RobotKinematics::inverseKinematics(const float T_des[4][4], const float* q0, float* q_result) {
    // 初始化
    float q[ARM_DOF];
    float best_q[ARM_DOF];
    float min_error = 1e6f; // 足够大的初始值
    int stalled_count = 0;
    float last_error = INFINITY;
    
    // 复制初始关节角度
    for(int i = 0; i < ARM_DOF; i++) {
        q[i] = q0[i];
        best_q[i] = q0[i];
    }
    
    // 迭代求解
    bool success = false;
    for(int iter = 0; iter < _max_iter; iter++) {
        // 计算当前位姿
        float T_current[4][4];
        if(!forwardKinematics(q, T_current)) {
            _last_iter_count = iter;
            _last_error = min_error;
            return false;
        }
        
        // 计算误差
        float error[6]; // [位置误差(3); 姿态误差(3)]
        computeFullError(T_current, T_des, error);
        float error_norm = vectorNorm(error, 6);
        
        // 记录最佳解
        if(error_norm < min_error) {
            min_error = error_norm;
            for(int i = 0; i < ARM_DOF; i++) {
                best_q[i] = q[i];
            }
            stalled_count = 0;  // 重置停滞计数
            
            // 如果达到精度要求，提前结束
            if(error_norm < _tol) {
                for(int i = 0; i < ARM_DOF; i++) {
                    q_result[i] = q[i];
                }
                success = true;
                _last_iter_count = iter + 1;
                _last_error = error_norm;
                break;
            }
        } else {
            stalled_count++;  // 误差没有改善
        }
        
        // 早期终止条件: 误差已经足够小且迭代次数足够多
        if(error_norm < 0.01f && iter > _max_iter/4) {
            for(int i = 0; i < ARM_DOF; i++) {
                q_result[i] = best_q[i];
            }
            success = true;
            _last_iter_count = iter + 1;
            _last_error = min_error;
            break;
        }
        
        // 随机扰动策略
        if(stalled_count > 10) {
            // 创建随机扰动
            for(int i = 0; i < ARM_DOF; i++) {
                float perturb = (random(2000) / 2000.0f - 0.5f) * 0.2f;
                q[i] += perturb;
            }
            
            // 应用关节限位
            if(_use_joint_limits) {
                applyJointLimits(q);
            }
            
            stalled_count = 0;
            continue;
        }
        
        // 计算雅可比矩阵
        float J[6][ARM_DOF];
        getJacobian(q, J);
        
        // 自适应阻尼策略
        float damping = _damping;
        float cond_J = approximateConditionNumber(J);
        if(cond_J > 1e4) {  // 条件数过大，增大阻尼
            damping = _damping * 10.0f;
        } else {
            // 在误差大时使用大阻尼，误差小时减小阻尼
            damping = _damping * (1.0f + _damping_factor * min(1.0f, error_norm));
        }
        
        // 计算阻尼伪逆
        float J_pinv[ARM_DOF][6];
        pseudoInverse(J, J_pinv);
        
        // 计算关节角度增量
        float dq[ARM_DOF] = {0};
        for(int i = 0; i < ARM_DOF; i++) {
            dq[i] = 0.0f;
            for(int j = 0; j < 6; j++) {
                dq[i] += J_pinv[i][j] * error[j];
            }
        }
        
        // 自适应步长控制
        float alpha;
        if(error_norm > 1.0f) {
            alpha = 0.1f;  // 误差大时，使用小步长
        } else if(error_norm > 0.5f) {
            alpha = 0.2f;
        } else if(error_norm > 0.2f) {
            alpha = 0.5f;
        } else {
            alpha = 1.0f;  // 误差小时，可以使用全步长
        }
        
        // 线搜索 - 最多尝试4次
        bool step_accepted = false;
        float current_error = error_norm;
        
        for(int ls = 0; ls < 4 && !step_accepted; ls++) {
            // 尝试新的关节角
            float q_new[ARM_DOF];
            for(int i = 0; i < ARM_DOF; i++) {
                q_new[i] = q[i] + alpha * dq[i];
            }
            
            // 应用关节限位
            if(_use_joint_limits) {
                for(int i = 0; i < ARM_DOF; i++) {
                    if(q_new[i] < _joint_lower[i]) q_new[i] = _joint_lower[i];
                    else if(q_new[i] > _joint_upper[i]) q_new[i] = _joint_upper[i];
                }
            }
            
            // 计算新位姿和误差
            float T_new[4][4];
            forwardKinematics(q_new, T_new);
            
            float error_new[6];
            computeFullError(T_new, T_des, error_new);
            float new_error = vectorNorm(error_new, 6);
            
            // 接受误差减小的步长
            if(new_error < current_error) {
                for(int i = 0; i < ARM_DOF; i++) {
                    q[i] = q_new[i];
                }
                step_accepted = true;
                break;
            }
            
            // 减小步长继续尝试
            alpha *= 0.5f;
        }
        
        // 如果线搜索失败，使用更小的步长
        if(!step_accepted) {
            float fixed_alpha = 0.005f;
            for(int i = 0; i < ARM_DOF; i++) {
                q[i] += fixed_alpha * dq[i];
            }
            
            // 应用关节限位
            if(_use_joint_limits) {
                applyJointLimits(q);
            }
        }
        
        // 检测是否停滞
        if(iter > 1 && iter % 20 == 0) {
            if(abs(last_error - error_norm) < 0.001f) {
                // 误差几乎不变，尝试随机重启
                if(min_error > 0.1f && iter < _max_iter * 0.8f) {
                    // 如果当前最佳解不够好，尝试重新开始
                    for(int i = 0; i < ARM_DOF; i++) {
                        q[i] = q0[i] + (random(2000) / 2000.0f - 0.5f) * 0.5f;
                    }
                    if(_use_joint_limits) {
                        applyJointLimits(q);
                    }
                    last_error = INFINITY;
                    continue;
                } else {
                    // 误差停滞，提前结束
                    break;
                }
            }
        }
        last_error = error_norm;
    }
    
    // 所有迭代结束，返回最佳解
    if(!success) {
        for(int i = 0; i < ARM_DOF; i++) {
            q_result[i] = best_q[i];
        }
        success = (min_error < 0.2f);  // 放宽成功标准
        _last_error = min_error;
    }
    
    return success;
}

// 计算雅可比矩阵 (使用数值微分方法)
void RobotKinematics::getJacobian(const float* q, float J[6][ARM_DOF]) {
    // 扰动幅度
    const float h = 1e-6f;
    
    // 计算当前位姿
    float T0[4][4];
    forwardKinematics(q, T0);
    
    // 对每个关节进行扰动
    for(int i = 0; i < ARM_DOF; i++) {
        // 创建扰动关节角
        float q_perturb[ARM_DOF];
        for(int j = 0; j < ARM_DOF; j++) {
            q_perturb[j] = q[j];
        }
        q_perturb[i] += h;
        
        // 计算扰动后的位姿
        float T_perturb[4][4];
        forwardKinematics(q_perturb, T_perturb);
        
        // 提取位置雅可比
        // 位置导数 = (扰动后位置 - 当前位置) / 扰动量
        J[0][i] = (T_perturb[0][3] - T0[0][3]) / h;
        J[1][i] = (T_perturb[1][3] - T0[1][3]) / h;
        J[2][i] = (T_perturb[2][3] - T0[2][3]) / h;
        
        // 计算姿态雅可比 - 使用旋转矩阵差异转换为角速度
        float R0[3][3], R_perturb[3][3], R_diff[3][3];
        float R_diffT[3][3], R_temp[3][3];
        
        // 提取旋转矩阵
        for(int r = 0; r < 3; r++) {
            for(int c = 0; c < 3; c++) {
                R0[r][c] = T0[r][c];
                R_perturb[r][c] = T_perturb[r][c];
            }
        }
        
        // 计算R_diff = R_perturb * R0^T
        matrixTranspose3x3(R0, R_diffT);
        
        // R_diff = R_perturb * R0^T
        for(int r = 0; r < 3; r++) {
            for(int c = 0; c < 3; c++) {
                R_temp[r][c] = 0;
                for(int k = 0; k < 3; k++) {
                    R_temp[r][c] += R_perturb[r][k] * R_diffT[k][c];
                }
            }
        }
        
        // 提取角速度 (skew-symmetric部分)
        float theta = acos((R_temp[0][0] + R_temp[1][1] + R_temp[2][2] - 1) / 2);
        
        if(theta < 1e-10f) {
            // 几乎没有旋转变化
            J[3][i] = 0.0f;
            J[4][i] = 0.0f;
            J[5][i] = 0.0f;
        } else if(abs(theta - PI) < 1e-6f) {
            // 处理180度旋转的特殊情况
            // 使用旋转矩阵对角线元素识别轴
            if(R_temp[0][0] > 0.9f) {
                J[3][i] = PI / h;
                J[4][i] = 0.0f;
                J[5][i] = 0.0f;
            } else if(R_temp[1][1] > 0.9f) {
                J[3][i] = 0.0f;
                J[4][i] = PI / h;
                J[5][i] = 0.0f;
            } else {
                J[3][i] = 0.0f;
                J[4][i] = 0.0f;
                J[5][i] = PI / h;
            }
        } else {
            // 正常情况 - 使用skew-symmetric部分提取轴
            float axis[3];
            axis[0] = (R_temp[2][1] - R_temp[1][2]) / (2 * sin(theta));
            axis[1] = (R_temp[0][2] - R_temp[2][0]) / (2 * sin(theta));
            axis[2] = (R_temp[1][0] - R_temp[0][1]) / (2 * sin(theta));
            
            // 归一化轴
            float norm = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
            if(norm > 1e-10f) {
                axis[0] /= norm;
                axis[1] /= norm;
                axis[2] /= norm;
            }
            
            // 雅可比值 = 角速度 = 轴*角度/扰动量
            J[3][i] = axis[0] * theta / h;
            J[4][i] = axis[1] * theta / h;
            J[5][i] = axis[2] * theta / h;
        }
    }
}

// 计算阻尼伪逆
void RobotKinematics::pseudoInverse(const float J[6][ARM_DOF], float J_pinv[ARM_DOF][6]) {
    // 使用阻尼最小二乘法计算伪逆
    // J_pinv = J^T * (J * J^T + lambda^2 * I)^(-1)
    
    float lambda = _damping;
    float lambda_sq = lambda * lambda;
    
    // 计算J * J^T (6x6)
    float JJT[6][6];
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            JJT[i][j] = 0.0f;
            for(int k = 0; k < ARM_DOF; k++) {
                JJT[i][j] += J[i][k] * J[j][k];
            }
            
            // 增加阻尼正则项
            if(i == j) {
                JJT[i][j] += lambda_sq;
            }
        }
    }
    
    // 计算(J * J^T)^(-1) 使用高斯-约旦消元法
    float JJT_inv[6][6];
    float aug[6][12]; // 增广矩阵 [JJT|I]
    
    // 创建增广矩阵
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            aug[i][j] = JJT[i][j];
            aug[i][j+6] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 高斯-约旦消元
    for(int i = 0; i < 6; i++) {
        // 找到主元
        float max_val = fabs(aug[i][i]);
        int max_row = i;
        
        for(int j = i+1; j < 6; j++) {
            if(fabs(aug[j][i]) > max_val) {
                max_val = fabs(aug[j][i]);
                max_row = j;
            }
        }
        
        // 交换行
        if(max_row != i) {
            for(int j = 0; j < 12; j++) {
                float temp = aug[i][j];
                aug[i][j] = aug[max_row][j];
                aug[max_row][j] = temp;
            }
        }
        
        // 缩放主元行使主元为1
        float pivot = aug[i][i];
        if(fabs(pivot) < 1e-10f) {
            // 矩阵接近奇异，使用正则化
            pivot = (pivot >= 0) ? 1e-10f : -1e-10f;
        }
        
        for(int j = 0; j < 12; j++) {
            aug[i][j] /= pivot;
        }
        
        // 消元，使其他行在主元列都为0
        for(int j = 0; j < 6; j++) {
            if(j != i) {
                float factor = aug[j][i];
                for(int k = 0; k < 12; k++) {
                    aug[j][k] -= factor * aug[i][k];
                }
            }
        }
    }
    
    // 提取逆矩阵
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            JJT_inv[i][j] = aug[i][j+6];
        }
    }
    
    // 计算J_pinv = J^T * JJT_inv
    for(int i = 0; i < ARM_DOF; i++) {
        for(int j = 0; j < 6; j++) {
            J_pinv[i][j] = 0.0f;
            for(int k = 0; k < 6; k++) {
                J_pinv[i][j] += J[k][i] * JJT_inv[k][j];
            }
        }
    }
}

// 计算位姿误差
void RobotKinematics::computeFullError(const float T_current[4][4], const float T_des[4][4], float* error) {
    // 位置误差 (从目标位置减去当前位置)
    error[0] = T_des[0][3] - T_current[0][3];
    error[1] = T_des[1][3] - T_current[1][3];
    error[2] = T_des[2][3] - T_current[2][3];
    
    // 姿态误差 (使用对数映射)
    // 计算旋转误差矩阵 R_error = R_des * R_current^T
    float R_current[3][3], R_des[3][3], R_currentT[3][3], R_error[3][3];
    
    // 提取旋转矩阵
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R_current[i][j] = T_current[i][j];
            R_des[i][j] = T_des[i][j];
        }
    }
    
    // 计算R_current的转置
    matrixTranspose3x3(R_current, R_currentT);
    
    // 计算R_error = R_des * R_current^T
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R_error[i][j] = 0.0f;
            for(int k = 0; k < 3; k++) {
                R_error[i][j] += R_des[i][k] * R_currentT[k][j];
            }
        }
    }
    
    // 提取轴角表示
    float cos_theta = (R_error[0][0] + R_error[1][1] + R_error[2][2] - 1.0f) / 2.0f;
    // 确保在[-1,1]范围内，避免数值问题
    cos_theta = constrain(cos_theta, -1.0f, 1.0f);
    
    float theta = acos(cos_theta);
    
    if(theta < 1e-10f) {
        // 如果角度几乎为零，方向轴任意
        error[3] = error[4] = error[5] = 0.0f;
    } else if(abs(theta - PI) < 1e-6f) {
        // 处理180度旋转的特殊情况
        // 尝试从对角线元素提取轴
        if(R_error[0][0] > -0.99f) {
            error[3] = PI * sqrt((R_error[0][0] + 1.0f) / 2.0f);
            error[4] = error[5] = 0.0f;
        } else if(R_error[1][1] > -0.99f) {
            error[4] = PI * sqrt((R_error[1][1] + 1.0f) / 2.0f);
            error[3] = error[5] = 0.0f;
        } else {
            error[5] = PI * sqrt((R_error[2][2] + 1.0f) / 2.0f);
            error[3] = error[4] = 0.0f;
        }
    } else {
        // 正常情况
        float axis[3];
        axis[0] = (R_error[2][1] - R_error[1][2]) / (2.0f * sin(theta));
        axis[1] = (R_error[0][2] - R_error[2][0]) / (2.0f * sin(theta));
        axis[2] = (R_error[1][0] - R_error[0][1]) / (2.0f * sin(theta));
        
        // 归一化
        float norm = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
        if(norm > 1e-10f) {
            axis[0] /= norm;
            axis[1] /= norm;
            axis[2] /= norm;
        }
        
        // 误差向量 = 轴 * 角度
        error[3] = axis[0] * theta;
        error[4] = axis[1] * theta;
        error[5] = axis[2] * theta;
    }
}

// 应用关节限位
void RobotKinematics::applyJointLimits(float* q) {
    for(int i = 0; i < ARM_DOF; i++) {
        if(q[i] < _joint_lower[i]) {
            q[i] = _joint_lower[i];
        }
        else if(q[i] > _joint_upper[i]) {
            q[i] = _joint_upper[i];
        }
    }
}

// 矩阵乘法 4x4
void RobotKinematics::matrixMultiply4x4(const float A[4][4], const float B[4][4], float C[4][4]) {
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            C[i][j] = 0.0f;
            for(int k = 0; k < 4; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 矩阵转置 3x3
void RobotKinematics::matrixTranspose3x3(const float in[3][3], float out[3][3]) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            out[j][i] = in[i][j];
        }
    }
}

// 计算向量的范数
float RobotKinematics::vectorNorm(const float* v, int size) const {
    float sum = 0.0f;
    for(int i = 0; i < size; i++) {
        sum += v[i] * v[i];
    }
    return sqrt(sum);
}

// 计算条件数的近似值
float RobotKinematics::approximateConditionNumber(const float J[6][ARM_DOF]) {
    // 使用简单的方法近似计算条件数
    // 找到J的列向量的最大和最小范数
    float max_norm = 0.0f;
    float min_norm = 1e10f;
    
    for(int i = 0; i < ARM_DOF; i++) {
        float col_norm = 0.0f;
        for(int j = 0; j < 6; j++) {
            col_norm += J[j][i] * J[j][i];
        }
        col_norm = sqrt(col_norm);
        
        if(col_norm > max_norm) max_norm = col_norm;
        if(col_norm < min_norm) min_norm = col_norm;
    }
    
    // 防止除以零
    if(min_norm < 1e-10f) min_norm = 1e-10f;
    
    return max_norm / min_norm;
}

// 欧拉角转换到位姿矩阵
void RobotKinematics::eulerToMatrix(float x, float y, float z, float roll, float pitch, float yaw, float T[4][4]) {
    float cr = cos(roll);
    float sr = sin(roll);
    float cp = cos(pitch);
    float sp = sin(pitch);
    float cy = cos(yaw);
    float sy = sin(yaw);
    
    // 使用Z-Y-X旋转顺序的旋转矩阵
    T[0][0] = cy * cp;
    T[0][1] = cy * sp * sr - sy * cr;
    T[0][2] = cy * sp * cr + sy * sr;
    T[0][3] = x;
    
    T[1][0] = sy * cp;
    T[1][1] = sy * sp * sr + cy * cr;
    T[1][2] = sy * sp * cr - cy * sr;
    T[1][3] = y;
    
    T[2][0] = -sp;
    T[2][1] = cp * sr;
    T[2][2] = cp * cr;
    T[2][3] = z;
    
    T[3][0] = 0.0f;
    T[3][1] = 0.0f;
    T[3][2] = 0.0f;
    T[3][3] = 1.0f;
}

// 位姿矩阵提取位置和欧拉角
void RobotKinematics::matrixToEuler(const float T[4][4], float* position, float* euler) {
    // 提取位置
    if(position) {
        position[0] = T[0][3];
        position[1] = T[1][3];
        position[2] = T[2][3];
    }
    
    // 提取欧拉角 (Z-Y-X顺序，即RPY)
    if(euler) {
        float r11 = T[0][0];
        float r12 = T[0][1];
        float r21 = T[1][0];
        float r22 = T[1][1];
        float r31 = T[2][0];
        float r32 = T[2][1];
        float r33 = T[2][2];
        
        // 处理万向锁问题 (pitch接近±90度)
        if(abs(r31) > 0.99999f) {
            // 万向锁情况 - roll和yaw有无数解，选择yaw=0
            euler[0] = 0.0f; // yaw
            euler[1] = -asin(r31); // pitch
            euler[2] = atan2(-r12, r22); // roll
        } else {
            euler[0] = atan2(r21, r11); // yaw
            euler[1] = -asin(r31); // pitch
            euler[2] = atan2(r32, r33); // roll
        }
    }
}

// 调试输出
void RobotKinematics::printMatrix(const float mat[4][4]) const {
    for(int i = 0; i < 4; i++) {
        Serial.print("[ ");
        for(int j = 0; j < 4; j++) {
            Serial.print(mat[i][j], 6);
            if(j < 3) Serial.print(", ");
        }
        Serial.println(" ]");
    }
}

void RobotKinematics::printVector(const float* vec, int size) const {
    Serial.print("[ ");
    for(int i = 0; i < size; i++) {
        Serial.print(vec[i], 6);
        if(i < size-1) Serial.print(", ");
    }
    Serial.println(" ]");
}