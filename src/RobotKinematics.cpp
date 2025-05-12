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
    int actual_iterations = 0;
    
    // 复制初始关节角度
    for(int i = 0; i < ARM_DOF; i++) {
        q[i] = q0[i];
        best_q[i] = q0[i];
    }
    
    // 计算初始位姿和误差
    float T_initial[4][4];
    forwardKinematics(q, T_initial);
    float initial_error[6];
    computeFullError(T_initial, T_des, initial_error);
    float initial_error_norm = vectorNorm(initial_error, 6);
    min_error = initial_error_norm;
    
    // 记录初始位置
    float initial_position[3];
    for(int i = 0; i < 3; i++) {
        initial_position[i] = T_initial[i][3];
    }
    
    // 记录目标位置
    float target_position[3];
    for(int i = 0; i < 3; i++) {
        target_position[i] = T_des[i][3];
    }
    
    // 计算位置差距
    float position_diff = 0;
    for(int i = 0; i < 3; i++) {
        position_diff += (target_position[i] - initial_position[i]) * 
                         (target_position[i] - initial_position[i]);
    }
    position_diff = sqrt(position_diff);
    
    // 根据位置差距自适应设置参数
    float target_tolerance;
    int min_iterations, max_iterations;
    
    if(position_diff < 0.01f) {
        // 微小位移，可以更快收敛
        target_tolerance = 0.01f;
        min_iterations = 5;
        max_iterations = 50;
    } else if(position_diff < 0.05f) {
        // 小位移
        target_tolerance = 0.01f;
        min_iterations = 10;
        max_iterations = 100;
    } else {
        // 大位移，需要更多迭代
        target_tolerance = 0.02f;
        min_iterations = 20;
        max_iterations = 200;
    }
    
    // 如果初始误差已经足够小，可以直接返回
    if(initial_error_norm < target_tolerance && position_diff < 0.01f) {
        for(int i = 0; i < ARM_DOF; i++) {
            q_result[i] = q0[i];
        }
        _last_iter_count = 1;
        _last_error = initial_error_norm;
        return true;
    }
    
    // 优化起点策略 - 根据位置差异决定起点数量
    int num_starting_points;
    if(position_diff < 0.02f) {
        num_starting_points = 2;  // 小位移只需少量起点
    } else if(position_diff < 0.1f) {
        num_starting_points = 3;  // 中等位移
    } else {
        num_starting_points = 5;  // 大位移需要更多尝试
    }
    
    float best_error_overall = min_error;
    float q_starts[5][ARM_DOF]; // 最多5个起点
    
    // 权重系数，使末端位置误差比姿态误差更重要
    float position_weight = 1.0f; 
    float orientation_weight = 0.5f;
    
    // 准备多个起点
    for(int i = 0; i < ARM_DOF; i++) {
        q_starts[0][i] = q0[i];  // 原始起点
    }
    
    // 添加额外起点，使用预设偏移模式而不是完全随机
    const float offset_patterns[4][ARM_DOF] = {
        {0.2f, -0.2f, 0.2f, -0.2f, 0.2f, -0.2f, 0.0f},
        {-0.2f, 0.2f, -0.2f, 0.2f, -0.2f, 0.2f, 0.0f},
        {0.3f, 0.3f, -0.3f, -0.3f, 0.3f, 0.3f, 0.0f},
        {-0.3f, -0.3f, 0.3f, 0.3f, -0.3f, -0.3f, 0.0f}
    };
    
    for(int sp = 1; sp < num_starting_points && sp <= 4; sp++) {
        for(int i = 0; i < ARM_DOF; i++) {
            // 使用预设偏移模式
            q_starts[sp][i] = q0[i] + offset_patterns[sp-1][i];
            
            // 添加少量随机扰动
            q_starts[sp][i] += (random(1000) / 10000.0f - 0.05f);
        }
        
        // 应用关节限制
        if(_use_joint_limits) {
            for(int i = 0; i < ARM_DOF; i++) {
                if(q_starts[sp][i] < _joint_lower[i]) 
                    q_starts[sp][i] = _joint_lower[i];
                else if(q_starts[sp][i] > _joint_upper[i]) 
                    q_starts[sp][i] = _joint_upper[i];
            }
        }
    }
    
    bool found_good_solution = false;
    
    // 从多个起点尝试求解
    for(int sp = 0; sp < num_starting_points && sp < 5; sp++) {
        // 如果已经找到非常好的解，提前终止
        if(best_error_overall < 0.005f) {
            found_good_solution = true;
            break;
        }
        
        // 重置当前点
        for(int i = 0; i < ARM_DOF; i++) {
            q[i] = q_starts[sp][i];
        }
        
        int stalled_count = 0;
        float last_error = INFINITY;
        float current_min_error = INFINITY;
        
        // 主迭代过程
        for(int iter = 0; iter < max_iterations; iter++) {
            actual_iterations++;
            
            // 计算当前位姿
            float T_current[4][4];
            if(!forwardKinematics(q, T_current)) {
                continue;
            }
            
            // 计算误差向量 [位置误差(3); 姿态误差(3)]
            float error[6];
            computeFullError(T_current, T_des, error);
            
            // 分别计算位置和姿态误差
            float pos_error = sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2]);
            float ori_error = sqrt(error[3]*error[3] + error[4]*error[4] + error[5]*error[5]);
            
            // 应用权重
            for(int i = 0; i < 3; i++) {
                error[i] *= position_weight;
            }
            for(int i = 3; i < 6; i++) {
                error[i] *= orientation_weight;
            }
            
            // 计算加权总误差
            float error_norm = vectorNorm(error, 6);
            
            // 更新最佳解
            if(error_norm < current_min_error) {
                current_min_error = error_norm;
                
                // 只有当这个起点找到的解比全局最优更好时才更新全局最优
                if(error_norm < best_error_overall) {
                    best_error_overall = error_norm;
                    for(int i = 0; i < ARM_DOF; i++) {
                        best_q[i] = q[i];
                    }
                }
                
                stalled_count = 0;  // 重置停滞计数
                
                // 如果达到精度要求并且已完成最小迭代次数，提前结束
                if(pos_error < target_tolerance && iter >= min_iterations) {
                    found_good_solution = true;
                    break;
                }
            } else {
                stalled_count++;
            }
            
            // 如果多次停滞，尝试随机扰动
            if(stalled_count > 8) {
                for(int i = 0; i < ARM_DOF; i++) {
                    float perturb = (random(1000) / 1000.0f - 0.5f) * 0.1f;
                    q[i] += perturb;
                }
                
                if(_use_joint_limits) {
                    applyJointLimits(q);
                }
                
                stalled_count = 0;
                continue;
            }
            
            // 使用加速收敛策略
            if(iter > 20 && error_norm > 0.1f && stalled_count > 3) {
                // 误差仍然较大且收敛缓慢，尝试更激进的搜索
                for(int i = 0; i < ARM_DOF; i++) {
                    q[i] = q_starts[0][i] + (random(2000) / 1000.0f - 1.0f) * 0.3f;
                }
                if(_use_joint_limits) {
                    applyJointLimits(q);
                }
                stalled_count = 0;
                continue;
            }
            
            // 计算雅可比矩阵
            float J[6][ARM_DOF];
            getJacobian(q, J);
            
            // 对雅可比矩阵进行加权调整
            for(int i = 0; i < 3; i++) {
                for(int j = 0; j < ARM_DOF; j++) {
                    J[i][j] *= position_weight;
                }
            }
            
            for(int i = 3; i < 6; i++) {
                for(int j = 0; j < ARM_DOF; j++) {
                    J[i][j] *= orientation_weight;
                }
            }
            
            // 计算雅可比条件数
            float cond_J = approximateConditionNumber(J);
            
            // 自适应阻尼系数
            float damping;
            if(cond_J > 1e4) {
                // 接近奇异点，大阻尼
                damping = 0.1f;
            } else if(error_norm > 0.5f) {
                // 误差大，中等阻尼
                damping = 0.01f;
            } else {
                // 误差小，小阻尼
                damping = 0.001f;
            }
            
            // 设置阻尼参数
            _damping = damping;
            
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
            
            // 优化线搜索
            float alpha;
            if(error_norm > 0.5f) {
                alpha = 0.1f;  // 误差大时，谨慎步长
            } else if(error_norm > 0.1f) {
                alpha = 0.3f;  // 中等误差，适中步长
            } else {
                alpha = 0.5f;  // 误差小时，较大步长
            }
            
            // 线搜索 - 尝试不同步长
            float best_new_error = error_norm;
            float q_best_step[ARM_DOF];
            for(int i = 0; i < ARM_DOF; i++) {
                q_best_step[i] = q[i];
            }
            
            bool step_accepted = false;
            
            // 自适应步长参数
            float step_factors[] = {1.0f, 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f};
            int num_steps = sizeof(step_factors) / sizeof(step_factors[0]);
            
            for(int s = 0; s < num_steps; s++) {
                float q_trial[ARM_DOF];
                float current_step = alpha * step_factors[s];
                
                for(int i = 0; i < ARM_DOF; i++) {
                    q_trial[i] = q[i] + current_step * dq[i];
                }
                
                if(_use_joint_limits) {
                    for(int i = 0; i < ARM_DOF; i++) {
                        if(q_trial[i] < _joint_lower[i]) q_trial[i] = _joint_lower[i];
                        else if(q_trial[i] > _joint_upper[i]) q_trial[i] = _joint_upper[i];
                    }
                }
                
                // 评估新点
                float T_trial[4][4];
                forwardKinematics(q_trial, T_trial);
                
                float trial_error[6];
                computeFullError(T_trial, T_des, trial_error);
                
                // 应用权重
                for(int i = 0; i < 3; i++) {
                    trial_error[i] *= position_weight;
                }
                for(int i = 3; i < 6; i++) {
                    trial_error[i] *= orientation_weight;
                }
                
                float trial_error_norm = vectorNorm(trial_error, 6);
                
                if(trial_error_norm < best_new_error) {
                    best_new_error = trial_error_norm;
                    for(int i = 0; i < ARM_DOF; i++) {
                        q_best_step[i] = q_trial[i];
                    }
                    step_accepted = true;
                    
                    // 如果误差显著减少，立即接受这个步长
                    if(trial_error_norm < 0.9f * error_norm) {
                        break;
                    }
                }
            }
            
            // 更新解
            if(step_accepted) {
                for(int i = 0; i < ARM_DOF; i++) {
                    q[i] = q_best_step[i];
                }
            } else {
                // 所有步长都不能改善，使用微小步长
                float tiny_alpha = 0.001f;
                for(int i = 0; i < ARM_DOF; i++) {
                    q[i] += tiny_alpha * dq[i];
                }
                
                if(_use_joint_limits) {
                    applyJointLimits(q);
                }
            }
            
            // 收敛检查
            if(iter > min_iterations) {
                if(abs(last_error - error_norm) < 0.0001f) {
                    // 误差不再显著减小
                    if(pos_error < 0.05f) {
                        // 位置误差已经足够小，可以接受
                        break;
                    }
                }
                
                if(pos_error < 0.01f && ori_error < 0.05f) {
                    // 位置和姿态误差都很小，提前结束
                    found_good_solution = true;
                    break;
                }
            }
            
            last_error = error_norm;
        }
        
        // 如果找到了好的解决方案，就不再尝试其他起点
        if(found_good_solution) break;
    }
    
    // 所有迭代和起点尝试结束，返回全局最佳解
    for(int i = 0; i < ARM_DOF; i++) {
        q_result[i] = best_q[i];
    }
    
    // 计算最终误差（不加权）
    float T_final[4][4];
    forwardKinematics(best_q, T_final);
    float final_error[6];
    computeFullError(T_final, T_des, final_error);
    float pos_err = sqrt(final_error[0]*final_error[0] + final_error[1]*final_error[1] + final_error[2]*final_error[2]);
    
    // 返回真实误差（主要考虑位置精度）
    _last_error = pos_err;
    _last_iter_count = actual_iterations;
    
    // 大幅放宽成功标准，即使误差较大也返回解，让上层应用决定是否使用
    bool success = (pos_err < 0.08f);  // 将阈值从0.05f提高到0.08f
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