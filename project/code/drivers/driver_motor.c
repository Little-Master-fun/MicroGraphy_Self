/*********************************************************************************************************************
* 文件名称          driver_motor.c
* 功能说明          双电机驱动程序实现文件（双PWM控制）
* 作者              LittleMaster
* 版本信息          v3.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v 
* 2025-09-17        LittleMaster       2.0v              创建基础功能
* 2025-09-23        LittleMaster       3.0v              改用双PWM控制，移除方向GPIO
* 
* 文件作用说明：
* 本文件是双电机驱动系统的实现文件，采用双PWM控制方式
* 控制逻辑：正转使用PWM1，反转使用PWM2，两个PWM通道互斥工作
* 
********************************************************************************************************************/

#include "driver_motor.h"
#include "zf_common_headfile.h"



//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机系统初始化
// 参数说明     void
// 返回参数     motor_status_enum   初始化状态
// 使用示例     motor_init();
// 备注信息     初始化双PWM通道，无需GPIO方向控制
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_init(void)
{
    // 初始化左电机双PWM通道
    pwm_init(MOTOR_LEFT_PWM1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);   // 左电机正转通道
    pwm_init(MOTOR_LEFT_PWM2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);   // 左电机反转通道
    
    // 初始化右电机双PWM通道
    pwm_init(MOTOR_RIGHT_PWM1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);  // 右电机正转通道
    pwm_init(MOTOR_RIGHT_PWM2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);  // 右电机反转通道
    
    // 确保电机初始状态为停止
    pwm_set_duty(MOTOR_LEFT_PWM1, 0);
    pwm_set_duty(MOTOR_LEFT_PWM2, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
    
    return MOTOR_STATUS_OK;
}




//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM值保护函数
// 参数说明     pwm_input           输入PWM值
// 返回参数     int16               保护后的PWM值
// 使用示例     protected_pwm = motor_pwm_protect(input_pwm);
// 备注信息     限制PWM值在有效范围内，防止电机损坏
//-------------------------------------------------------------------------------------------------------------------
int16 motor_pwm_protect(int16 pwm_input)
{
    int16 pwm_output = 0;
    
    // 上限保护
    if (pwm_input > MOTOR_PWM_MAX)
    {
        pwm_output = MOTOR_PWM_MAX;
    }
    // 下限保护
    else if (pwm_input < -MOTOR_PWM_MAX)
    {
        pwm_output = -MOTOR_PWM_MAX;
    }
    // 死区保护（防止电机抖动）
    else if ((pwm_input > -MOTOR_PWM_MIN) && (pwm_input < MOTOR_PWM_MIN))
      {
          pwm_output = 0;
      }
      else
      {
          pwm_output = pwm_input;
      } 
      
      return pwm_output;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机PWM（双PWM控制方式）
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 参数说明     pwm_value           电机PWM值 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_pwm(MOTOR_LEFT, 5000);
// 备注信息     正值正转使用PWM1，负值反转使用PWM2，两PWM通道互斥
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_pwm(motor_id_enum motor_id, int16 pwm_value)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    // PWM值保护
    int16 protected_pwm = motor_pwm_protect(pwm_value);
    
    // 处理MOTOR_BOTH情况，并行调用设置左右电机
    if (motor_id == MOTOR_BOTH)
    {
        status |= motor_set_pwm(MOTOR_LEFT, protected_pwm);
        status |= motor_set_pwm(MOTOR_RIGHT, protected_pwm);
        return status;
    }
    
    // 双PWM控制逻辑：计算速度和方向
    int16 abs_pwm = (protected_pwm >= 0) ? protected_pwm : -protected_pwm;
    uint8 is_forward = (protected_pwm >= 0) ? 0 : 1;
    
    // 设置左电机
    if (motor_id == MOTOR_LEFT)
    {
        if (abs_pwm == 0)
        {
            // 停止模式：PWM1=0, PWM2=0
            pwm_set_duty(MOTOR_LEFT_PWM1, 0);
            pwm_set_duty(MOTOR_LEFT_PWM2, 0);
        }
        else if (is_forward)
        {
            // 正转：PWM1=速度, PWM2=0
            pwm_set_duty(MOTOR_LEFT_PWM1, abs_pwm);
            pwm_set_duty(MOTOR_LEFT_PWM2, 0);
        }
        else
        {
            // 反转：PWM1=0, PWM2=速度
            pwm_set_duty(MOTOR_LEFT_PWM1, 0);
            pwm_set_duty(MOTOR_LEFT_PWM2, abs_pwm);
        }
    }
    // 设置右电机  
    else if (motor_id == MOTOR_RIGHT)
    {
        if (abs_pwm == 0)
        {
            // 停止模式：PWM1=0, PWM2=0
            pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
            pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
        }
        else if (is_forward)
        {
            // 正转：PWM1=速度, PWM2=0
            pwm_set_duty(MOTOR_RIGHT_PWM1, abs_pwm);
            pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
        }
        else
        {
            // 反转：PWM1=0, PWM2=速度
            pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
            pwm_set_duty(MOTOR_RIGHT_PWM2, abs_pwm);
        }
    }
    else
    {
        return MOTOR_STATUS_ERROR;
    }
    
    return MOTOR_STATUS_OK;
}
