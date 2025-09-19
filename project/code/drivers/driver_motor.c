/*********************************************************************************************************************
* 文件名称          driver_motor.c
* 功能说明          双电机驱动程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件是双电机驱动系统的实现文件，提供完整的电机控制功能
* 
********************************************************************************************************************/

#include "driver_motor.h"
#include "zf_common_headfile.h"

//=================================================内部函数声明================================================
static motor_status_enum motor_set_pwm_single(motor_id_enum motor_id, int16 pwm_value);
static motor_status_enum motor_set_direction_single(motor_id_enum motor_id, motor_direction_enum direction);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机系统初始化
// 参数说明     void
// 返回参数     motor_status_enum   初始化状态
// 使用示例     motor_init();
// 备注信息     初始化PWM通道和GPIO引脚
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_init(void)
{
    // 初始化左电机PWM通道
    pwm_init(MOTOR_LEFT_PWM1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    pwm_init(MOTOR_LEFT_PWM2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    
    // 初始化右电机PWM通道
    pwm_init(MOTOR_RIGHT_PWM1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    pwm_init(MOTOR_RIGHT_PWM2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    
    // 初始化左电机方向控制GPIO
    gpio_init(MOTOR_LEFT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_LEFT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // 初始化右电机方向控制GPIO
    gpio_init(MOTOR_RIGHT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // 确保电机初始状态为停止
    motor_stop(MOTOR_BOTH);
    
    return MOTOR_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机方向
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 参数说明     direction           电机方向 (MOTOR_STOP/MOTOR_FORWARD/MOTOR_BACKWARD)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
// 备注信息     设置电机转动方向
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_direction(motor_id_enum motor_id, motor_direction_enum direction)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    if (motor_id == MOTOR_LEFT || motor_id == MOTOR_BOTH)
    {
        status |= motor_set_direction_single(MOTOR_LEFT, direction);
    }
    
    if (motor_id == MOTOR_RIGHT || motor_id == MOTOR_BOTH)
    {
        status |= motor_set_direction_single(MOTOR_RIGHT, direction);
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机速度
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 参数说明     speed               电机速度 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_speed(MOTOR_LEFT, 5000);
// 备注信息     正值正转，负值反转，自动处理方向控制
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_speed(motor_id_enum motor_id, int16 speed)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    // PWM值保护
    int16 protected_speed = motor_pwm_protect(speed);
    
    // 根据速度正负确定方向
    motor_direction_enum direction;
    int16 abs_speed;
    
    if (protected_speed > 0)
    {
        direction = MOTOR_FORWARD;
        abs_speed = protected_speed;
    }
    else if (protected_speed < 0)
    {
        direction = MOTOR_BACKWARD;
        abs_speed = -protected_speed;
    }
    else
    {
        direction = MOTOR_STOP;
        abs_speed = 0;
    }
    
    // 设置方向
    status |= motor_set_direction(motor_id, direction);
    
    // 设置PWM
    if (direction != MOTOR_STOP)
    {
        if (motor_id == MOTOR_LEFT || motor_id == MOTOR_BOTH)
        {
            status |= motor_set_pwm_single(MOTOR_LEFT, abs_speed);
        }
        
        if (motor_id == MOTOR_RIGHT || motor_id == MOTOR_BOTH)
        {
            status |= motor_set_pwm_single(MOTOR_RIGHT, abs_speed);
        }
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止电机
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_stop(MOTOR_BOTH);
// 备注信息     停止指定电机
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_stop(motor_id_enum motor_id)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    if (motor_id == MOTOR_LEFT || motor_id == MOTOR_BOTH)
    {
        // 停止左电机
        gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
        gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
        pwm_set_duty(MOTOR_LEFT_PWM1, 0);
        pwm_set_duty(MOTOR_LEFT_PWM2, 0);
    }
    
    if (motor_id == MOTOR_RIGHT || motor_id == MOTOR_BOTH)
    {
        // 停止右电机
        gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
        gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
        pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
        pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
    }
    
    return status;
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
// 函数简介     电机测试函数
// 参数说明     void
// 返回参数     void
// 使用示例     motor_test();
// 备注信息     用于测试电机基本功能
//-------------------------------------------------------------------------------------------------------------------
void motor_test(void)
{
    // 测试左电机正转
    motor_set_speed(MOTOR_LEFT, 3000);
    system_delay_ms(2000);
    
    // 测试左电机反转
    motor_set_speed(MOTOR_LEFT, -3000);
    system_delay_ms(2000);
    
    // 停止左电机
    motor_stop(MOTOR_LEFT);
    system_delay_ms(1000);
    
    // 测试右电机正转
    motor_set_speed(MOTOR_RIGHT, 3000);
    system_delay_ms(2000);
    
    // 测试右电机反转
    motor_set_speed(MOTOR_RIGHT, -3000);
    system_delay_ms(2000);
    
    // 停止右电机
    motor_stop(MOTOR_RIGHT);
    system_delay_ms(1000);
    
    // 测试双电机同步
    motor_set_speed(MOTOR_BOTH, 2000);
    system_delay_ms(2000);
    
    // 停止所有电机
    motor_stop(MOTOR_BOTH);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差速运动控制
// 参数说明     linear_speed        线速度 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 参数说明     angular_speed       角速度 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_differential_drive(3000, 1000);
// 备注信息     基于线速度和角速度控制差速运动
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_differential_drive(int16 linear_speed, int16 angular_speed)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    // 差速运动学解算
    int16 left_speed = linear_speed - angular_speed;
    int16 right_speed = linear_speed + angular_speed;
    
    // 设置左右电机速度
    status |= motor_set_speed(MOTOR_LEFT, left_speed);
    status |= motor_set_speed(MOTOR_RIGHT, right_speed);
    
    return status;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单个电机PWM设置（内部函数）
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT)
// 参数说明     pwm_value           PWM值 (0 ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 备注信息     内部函数，用于设置单个电机的PWM输出
//-------------------------------------------------------------------------------------------------------------------
static motor_status_enum motor_set_pwm_single(motor_id_enum motor_id, int16 pwm_value)
{
    if (motor_id == MOTOR_LEFT)
    {
        pwm_set_duty(MOTOR_LEFT_PWM1, pwm_value);
        pwm_set_duty(MOTOR_LEFT_PWM2, 0);
    }
    else if (motor_id == MOTOR_RIGHT)
    {
        pwm_set_duty(MOTOR_RIGHT_PWM1, pwm_value);
        pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
    }
    else
    {
        return MOTOR_STATUS_ERROR;
    }
    
    return MOTOR_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单个电机方向设置（内部函数）
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT)
// 参数说明     direction           电机方向
// 返回参数     motor_status_enum   执行状态
// 备注信息     内部函数，用于设置单个电机的转动方向
//-------------------------------------------------------------------------------------------------------------------
static motor_status_enum motor_set_direction_single(motor_id_enum motor_id, motor_direction_enum direction)
{
    if (motor_id == MOTOR_LEFT)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:
                gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
                gpio_set_level(MOTOR_LEFT_IN2, GPIO_HIGH);
                break;
            case MOTOR_BACKWARD:
                gpio_set_level(MOTOR_LEFT_IN1, GPIO_HIGH);
                gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
                break;
            case MOTOR_STOP:
            default:
                gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
                gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
                break;
        }
    }
    else if (motor_id == MOTOR_RIGHT)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:
                gpio_set_level(MOTOR_RIGHT_IN1, GPIO_HIGH);
                gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
                break;
            case MOTOR_BACKWARD:
                gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
                gpio_set_level(MOTOR_RIGHT_IN2, GPIO_HIGH);
                break;
            case MOTOR_STOP:
            default:
                gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
                gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
                break;
        }
    }
    else
    {
        return MOTOR_STATUS_ERROR;
    }
    
    return MOTOR_STATUS_OK;
}