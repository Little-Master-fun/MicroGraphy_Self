/*********************************************************************************************************************
* 文件名称          driver_motor.c
* 功能说明          双电机驱动程序实现文件（TB67H420FTG驱动芯片）
* 作者              LittleMaster
* 版本信息          v4.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v              创建基础功能
* 2025-09-17        LittleMaster       2.0v              修复控制逻辑
* 2025-09-23        LittleMaster       3.0v              改用双PWM控制
* 2025-11-05        AI Assistant       4.0v              修改为TB67H420FTG标准控制方式
* 
* 文件作用说明：
* 本文件是双电机驱动系统的实现文件，使用TB67H420FTG双H桥驱动芯片
* 控制逻辑：通过IN1/IN2 GPIO控制方向，PWM控制速度
* 
********************************************************************************************************************/

#include "driver_motor.h"
#include "zf_common_headfile.h"

//=================================================内部函数声明================================================
static void motor_set_direction_pins(motor_id_enum motor_id, uint8 direction);



//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机系统初始化
// 参数说明     void
// 返回参数     motor_status_enum   初始化状态
// 使用示例     motor_init();
// 备注信息     初始化PWM通道和GPIO方向控制引脚
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_init(void)
{
    // 初始化左电机PWM通道
    pwm_init(MOTOR_LEFT_PWMA, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    
    // 初始化右电机PWM通道
    pwm_init(MOTOR_RIGHT_PWMA, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    
    // 初始化左电机方向控制GPIO
    gpio_init(MOTOR_LEFT_INA1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_LEFT_INA2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // 初始化右电机方向控制GPIO
    gpio_init(MOTOR_RIGHT_INA1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_INA2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // 确保电机初始状态为停止（刹车模式：IN1=0, IN2=0）
    gpio_set_level(MOTOR_LEFT_INA1, 0);
    gpio_set_level(MOTOR_LEFT_INA2, 0);
    gpio_set_level(MOTOR_RIGHT_INA1, 0);
    gpio_set_level(MOTOR_RIGHT_INA2, 0);
    pwm_set_duty(MOTOR_LEFT_PWMA, 0);
    pwm_set_duty(MOTOR_RIGHT_PWMA, 0);
    
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
// 函数简介     设置电机速度（有符号PWM值）
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 参数说明     pwm_value           电机PWM值 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_pwm(MOTOR_LEFT, 5000);
// 备注信息     正值正转，负值反转，自动控制方向引脚和PWM速度
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_pwm(motor_id_enum motor_id, int16 pwm_value)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    // PWM值保护
    int16 protected_pwm = motor_pwm_protect(pwm_value);
    
    // 处理MOTOR_BOTH情况，递归调用设置左右电机
    if (motor_id == MOTOR_BOTH)
    {
        status |= motor_set_pwm(MOTOR_LEFT, protected_pwm);
        status |= motor_set_pwm(MOTOR_RIGHT, protected_pwm);
        return status;
    }
    
    // 判断方向和速度
    uint8 direction;
    uint16 speed;
    
    if (protected_pwm > 0)
    {
        direction = MOTOR_DIR_CW;       // 正转
        speed = (uint16)protected_pwm;
    }
    else if (protected_pwm < 0)
    {
        direction = MOTOR_DIR_CCW;      // 反转
        speed = (uint16)(-protected_pwm);
    }
    else
    {
        direction = MOTOR_DIR_BRAKE;    // 刹车
        speed = 0;
    }
    
    // 调用底层函数
    return motor_set_direction_speed(motor_id, direction, speed);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接设置电机方向和速度
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT)
// 参数说明     direction           方向 (MOTOR_DIR_CW/CCW/BRAKE/STOP)
// 参数说明     speed               速度 (0 ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_direction_speed(MOTOR_LEFT, MOTOR_DIR_CW, 5000);
// 备注信息     直接控制方向和速度，用于高级应用
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_direction_speed(motor_id_enum motor_id, uint8 direction, uint16 speed)
{
    if (motor_id == MOTOR_BOTH)
    {
        return MOTOR_STATUS_ERROR;  // 此函数不支持MOTOR_BOTH
    }
    
    // 速度限幅
    if (speed > MOTOR_PWM_MAX)
    {
        speed = MOTOR_PWM_MAX;
    }
    
    // 设置方向
    motor_set_direction_pins(motor_id, direction);
    
    // 设置速度（PWM占空比）
    if (motor_id == MOTOR_LEFT)
    {
        pwm_set_duty(MOTOR_LEFT_PWMA, speed);
    }
    else if (motor_id == MOTOR_RIGHT)
    {
        pwm_set_duty(MOTOR_RIGHT_PWMA, speed);
    }
    else
    {
        return MOTOR_STATUS_ERROR;
    }
    
    return MOTOR_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机紧急停止（刹车模式）
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_brake(MOTOR_BOTH);
// 备注信息     使用短路刹车模式快速停止电机
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_brake(motor_id_enum motor_id)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    if (motor_id == MOTOR_BOTH || motor_id == MOTOR_LEFT)
    {
        motor_set_direction_pins(MOTOR_LEFT, MOTOR_DIR_BRAKE);
        pwm_set_duty(MOTOR_LEFT_PWMA, 0);
    }
    
    if (motor_id == MOTOR_BOTH || motor_id == MOTOR_RIGHT)
    {
        motor_set_direction_pins(MOTOR_RIGHT, MOTOR_DIR_BRAKE);
        pwm_set_duty(MOTOR_RIGHT_PWMA, 0);
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置双电机速度（兼容接口）
// 参数说明     left_speed          左电机速度 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 参数说明     right_speed         右电机速度 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_speed(1000, 1000);
// 备注信息     用于差速控制，正值正转，负值反转
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_speed(int16 left_speed, int16 right_speed)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    status |= motor_set_pwm(MOTOR_LEFT, left_speed);
    status |= motor_set_pwm(MOTOR_RIGHT, right_speed);
    
    return status;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机方向引脚
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT)
// 参数说明     direction           方向 (MOTOR_DIR_CW/CCW/BRAKE/STOP)
// 返回参数     void
// 备注信息     内部函数，根据TB67H420FTG真值表设置IN1/IN2引脚
//-------------------------------------------------------------------------------------------------------------------
static void motor_set_direction_pins(motor_id_enum motor_id, uint8 direction)
{
    uint8 in1_level, in2_level;
    
    // 根据方向确定IN1/IN2电平（已反转方向以匹配编码器）
    switch (direction)
    {
        case MOTOR_DIR_CW:      // 正转（已反转）: IN1=0, IN2=1
            in1_level = 0;
            in2_level = 1;
            break;
            
        case MOTOR_DIR_CCW:     // 反转（已反转）: IN1=1, IN2=0
            in1_level = 1;
            in2_level = 0;
            break;
            
        case MOTOR_DIR_BRAKE:   // 刹车: IN1=0, IN2=0
            in1_level = 0;
            in2_level = 0;
            break;
            
        case MOTOR_DIR_STOP:    // 停止（高阻）: IN1=1, IN2=1
            in1_level = 1;
            in2_level = 1;
            break;
            
        default:
            in1_level = 0;
            in2_level = 0;
            break;
    }
    
    // 设置对应电机的IN引脚
    if (motor_id == MOTOR_LEFT)
    {
        gpio_set_level(MOTOR_LEFT_INA1, in1_level);
        gpio_set_level(MOTOR_LEFT_INA2, in2_level);
    }
    else if (motor_id == MOTOR_RIGHT)
    {
        gpio_set_level(MOTOR_RIGHT_INA1, in1_level);
        gpio_set_level(MOTOR_RIGHT_INA2, in2_level);
    }
}
