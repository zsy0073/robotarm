#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <Arduino.h>

// 机械臂参数常量
#define ARM_DOF 7              // 7自由度
#define DEFAULT_MAX_ITER 50    // 逆解最大迭代次数
#define DEFAULT_TOL 1e-5f      // 收敛阈值
#define DEFAULT_DAMPING 0.05f  // 基础阻尼因子
#define DEFAULT_DAMPING_FACTOR 5.0f // 阻尼调整系数

class RobotKinematics {
public:
    // 构造函数
    RobotKinematics();
    
    // 默认参数设置
    void resetToDefaultParams();
    
    // 设置MDH参数
    void setMDHParams(float* alpha_arr, float* a_arr, float* d_arr);
    
    // 设置关节限位
    void setJointLimits(float* lower_limits, float* upper_limits);
    void enableJointLimits(bool enable);
    
    // 设置求解参数
    void setSolverParams(int max_iter, float tolerance, float damping);
    
    // 正运动学 - 计算末端执行器位姿
    // 输入: q - 关节角度数组(弧度)
    // 输出: T - 4x4齐次变换矩阵
    bool forwardKinematics(const float* q, float T[4][4]);
    
    // 逆运动学 - 计算关节角度
    // 输入: T_des - 目标位姿矩阵, q0 - 初始关节角度
    // 输出: q_result - 计算得到的关节角度
    bool inverseKinematics(const float T_des[4][4], const float* q0, float* q_result);
    
    // 转换欧拉角到位姿矩阵
    void eulerToMatrix(float x, float y, float z, float roll, float pitch, float yaw, float T[4][4]);
    
    // 位姿矩阵提取位置和欧拉角
    void matrixToEuler(const float T[4][4], float* position, float* euler);
    
    // 获取上次逆运动学计算的迭代次数
    int getLastIterCount() const { return _last_iter_count; }
    
    // 获取上次逆运动学计算的误差
    float getLastError() const { return _last_error; }
    
    // 获取当前MDH参数
    void getMDHParams(float* alpha_out, float* a_out, float* d_out) const;
    
    // 调试输出
    void printMatrix(const float mat[4][4]) const;
    void printVector(const float* vec, int size) const;
    
private:
    // MDH参数
    float _alpha[ARM_DOF]; // 连杆扭角
    float _a[ARM_DOF];     // 连杆长度
    float _d[ARM_DOF];     // 连杆偏距
    
    // 关节限位
    float _joint_lower[ARM_DOF]; // 关节下限
    float _joint_upper[ARM_DOF]; // 关节上限
    bool _use_joint_limits;      // 是否使用关节限位
    
    // 求解参数
    int _max_iter;           // 最大迭代次数
    float _tol;              // 收敛阈值
    float _damping;          // 基础阻尼因子
    float _damping_factor;   // 阻尼调整系数
    
    // 状态变量
    int _last_iter_count;    // 上次迭代次数
    float _last_error;       // 上次误差
    
    // 计算雅可比矩阵 (使用数值微分方法)
    void getJacobian(const float* q, float J[6][ARM_DOF]);
    
    // 计算伪逆
    void pseudoInverse(const float J[6][ARM_DOF], float J_pinv[ARM_DOF][6]);
    
    // 计算位姿误差 (位置+姿态)
    void computeFullError(const float T_current[4][4], const float T_des[4][4], float* error);
    
    // 应用关节限位
    void applyJointLimits(float* q);
    
    // 矩阵乘法 4x4
    void matrixMultiply4x4(const float A[4][4], const float B[4][4], float C[4][4]);
    
    // 矩阵转置 3x3
    void matrixTranspose3x3(const float in[3][3], float out[3][3]);
    
    // 计算向量的范数
    float vectorNorm(const float* v, int size) const;
    
    // 计算条件数的近似值
    float approximateConditionNumber(const float J[6][ARM_DOF]);
};

#endif // ROBOT_KINEMATICS_H