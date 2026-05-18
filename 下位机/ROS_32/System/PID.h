#ifndef __PID_H
#define __PID_H

typedef struct {
    double Kp;    // 比例增益
    double Ki;    // 积分增益
    double Kd;    // 微分增益
    double prev_error; // 上一次误差
    double prev_prev_error; // 前两次误差
    double integral;  // 积分值
    double output;    // 控制输出
} IncrementalPID;


void init_pid(IncrementalPID *pid, double Kp, double Ki, double Kd);
double compute_pid(IncrementalPID *pid, double setpoint, double actual_value);




#endif
