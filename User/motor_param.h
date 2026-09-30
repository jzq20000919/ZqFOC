/**
 * @file    motor_param.h
 * @brief   电机、采样及控制参数
 */
#ifndef MOTOR_PARAM_H
#define MOTOR_PARAM_H

// 电机参数
#define MOTOR_POLE_PAIRS            7          // 极对数
#define MOTOR_RS                    2.55f      // 相电阻，Ω
#define MOTOR_LD                    0.00086f   // d轴电感，H
#define MOTOR_LQ                    0.00086f   // q轴电感，H
#define MOTOR_FLUX                  0.0035f    // 磁链，Wb
#define MOTOR_RATED_CURRENT         0.5f       // 额定电流，A
#define MOTOR_MAX_CURRENT           2.0f       // 最大电流，A
#define MOTOR_MAX_SPEED_RPM         2600.0f    // 最大机械转速，rpm
#define MOTOR_START_SPEED_RPM       100.0f     // 正常模式按键启动后的目标转速，rpm
#define MOTOR_SPEED_BUTTON_STEP_RPM 100.0f     // SW2/SW3每次按下调节的转速，rpm
#define MOTOR_INERTIA               3.7e-6f    // 转动惯量，kg·m²
#define MOTOR_TORQUE_CONSTANT       0.0434f    // 转矩常数，N·m/A

// 编码器参数
#define ENCODER_CPR                 4096U      // 每机械圈计数
#define ENCODER_ELECTRICAL_OFFSET   0.0f       // 初始电角度偏置，rad

// 控制频率与周期
#define PWM_FREQUENCY_HZ            16000.0f   // PWM频率，Hz
#define CURRENT_LOOP_FREQUENCY      PWM_FREQUENCY_HZ // 电流环频率，Hz
#define CURRENT_LOOP_TS             (1.0f / CURRENT_LOOP_FREQUENCY) // 电流环周期，s
#define SPEED_LOOP_FREQUENCY        1000.0f    // 速度环频率，Hz
#define SPEED_LOOP_TS               (1.0f / SPEED_LOOP_FREQUENCY) // 速度环周期，s
#define POSITION_LOOP_FREQUENCY     200.0f     // 位置环频率，Hz
#define POSITION_LOOP_TS            (1.0f / POSITION_LOOP_FREQUENCY) // 位置环周期，s

// 电流环PI参数
#define CURRENT_KP_D                2.70f      // d轴比例增益
#define CURRENT_KI_D                8000.0f    // d轴积分增益
#define CURRENT_KP_Q                2.70f      // q轴比例增益
#define CURRENT_KI_Q                8000.0f    // q轴积分增益
#define CURRENT_PI_VOLTAGE_LIMIT    24.0f      // PI输出电压限幅，V

// 速度环PI参数
#define SPEED_KP                    0.0008f    // 比例增益
#define SPEED_KI                    0.035f     // 积分增益

// 位置环PI参数
#define POSITION_KP                 120.0f     // 比例增益
#define POSITION_KI                 0.0f       // 积分增益

// 电流采样参数
#define CURRENT_SHUNT_RESISTANCE    0.003f     // 分流电阻，Ω
#define CURRENT_AMP_GAIN            10.0f      // 电流采样放大倍数
#define ADC_VREF                    3.3f       // ADC参考电压，V
#define ADC_FULL_SCALE              4095.0f    // 12位ADC最大计数
#define ADC_CURRENT_OFFSET_A        2030.5f    // A相零电流ADC计数
#define ADC_CURRENT_OFFSET_B        2019.0f    // B相零电流ADC计数
#define ADC_CURRENT_OFFSET_C        2031.0f    // C相零电流ADC计数
#define CURRENT_OFFSET_SAMPLE_COUNT 24U        // 上电零电流采样次数
#define CURRENT_OFFSET_SAMPLE_INTERVAL_MS 2U   // 相邻零点采样间隔，ms
#define CURRENT_OFFSET_SAMPLE_TIMEOUT_MS 5U    // 单次等待ADC注入转换超时，ms
#define CURRENT_SCALE \
    (ADC_VREF / (ADC_FULL_SCALE * CURRENT_SHUNT_RESISTANCE * CURRENT_AMP_GAIN)) // 电流换算系数，A/计数

// 母线电压采样参数
#define BUS_VOLTAGE_DIVIDER_RATIO   26.0f      // 电阻分压倍率

// SVPWM参数
#define SVPWM_DUTY_MIN              0.20f      // 最小占空比，0～1
#define SVPWM_DUTY_MAX              0.80f      // 最大占空比，0～1
#define VOLTAGE_UTILIZATION         0.90f      // 母线电压利用系数

// 数学常量
#define FOC_PI                      3.14159265358979323846f // π
#define FOC_TWO_PI                  6.28318530717958647692f // 2π
#define FOC_INV_SQRT3               0.5773502691896258f    // 1/√3
#define FOC_SQRT3                   1.7320508075688772f    // √3

// 编码器电角度对齐参数
#define ENCODER_ALIGN_VOLTAGE       1.0f       // 对齐电压，V
#define ENCODER_ALIGN_TIME_MS       1000U      // 对齐保持时间，ms
#define ENCODER_ALIGN_ANGLE         0.0f       // 对齐电角度，rad
#define ENCODER_ALIGN_SAMPLE_COUNT  10U        // 对齐后编码器采样次数
#define ENCODER_ALIGN_SAMPLE_INTERVAL_MS 2U    // 相邻编码器采样间隔，ms
#define ENCODER_ALIGN_SAMPLE_TIMEOUT_MS 100U   // 对齐后的采样总超时，ms
#define ENCODER_ALIGN_RELEASE_TIME_MS 800U     // 撤掉对齐电压后的释放等待，ms
#endif // MOTOR_PARAM_H
