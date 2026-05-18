#include <stdio.h>
 
// 定义增量型PID控制器结构体
typedef struct {
    double Kp;    // 比例增益
    double Ki;    // 积分增益
    double Kd;    // 微分增益
    double prev_error; // 上一次误差
    double prev_prev_error; // 前两次误差
    double integral;  // 积分值
    double output;    // 控制输出
} IncrementalPID;
 
// 初始化增量型PID控制器
void init_pid(IncrementalPID *pid, double Kp, double Ki, double Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0;
    pid->prev_prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
}
 
// 计算增量型PID控制器输出
double compute_pid(IncrementalPID *pid, double setpoint, double actual_value) {
    // 计算当前误差
    double error = setpoint - actual_value;
 
    // 计算PID增量
    double delta_output = pid->Kp * (error - pid->prev_error) 
                          + pid->Ki * error 
                          + pid->Kd * (error - 2 * pid->prev_error + pid->prev_prev_error);
 
    // 更新输出
    pid->output += delta_output;
 
    // 更新历史误差
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
 
    return pid->output;
}
