#ifndef MOTOR_PARAM_H
#define MOTOR_PARAM_H

/* Motor: Rs [Ohm], Ld/Lq [H], flux [Wb], current [A], speed [rpm],
 * inertia [kg*m^2], torque constant [Nm/A]. */
#define MOTOR_POLE_PAIRS            7
#define MOTOR_RS                    2.55f
#define MOTOR_LD                    0.00086f
#define MOTOR_LQ                    0.00086f
#define MOTOR_FLUX                  0.0035f
#define MOTOR_RATED_CURRENT         0.5f
#define MOTOR_MAX_CURRENT           2.0f
#define MOTOR_MAX_SPEED_RPM         2600.0f
#define MOTOR_INERTIA               3.7e-6f
#define MOTOR_TORQUE_CONSTANT       0.0434f

/* Encoder */
#define ENCODER_CPR                 4096U
#define ENCODER_ELECTRICAL_OFFSET   0.0f

/* Control frequencies and sampling periods */
#define PWM_FREQUENCY_HZ            16000.0f
#define CURRENT_LOOP_FREQUENCY      PWM_FREQUENCY_HZ
#define CURRENT_LOOP_TS             (1.0f / CURRENT_LOOP_FREQUENCY)
#define SPEED_LOOP_FREQUENCY        1000.0f
#define SPEED_LOOP_TS               (1.0f / SPEED_LOOP_FREQUENCY)
#define POSITION_LOOP_FREQUENCY     200.0f
#define POSITION_LOOP_TS            (1.0f / POSITION_LOOP_FREQUENCY)

/* Current-loop PI */
#define CURRENT_KP_D                2.70f
#define CURRENT_KI_D                8000.0f
#define CURRENT_KP_Q                2.70f
#define CURRENT_KI_Q                8000.0f
#define CURRENT_PI_VOLTAGE_LIMIT    24.0f

/* Speed-loop PI */
#define SPEED_KP                    0.0008f
#define SPEED_KI                    0.035f

/* Position-loop PI */
#define POSITION_KP                 120.0f
#define POSITION_KI                 0.0f

/* Current sampling */
#define CURRENT_SHUNT_RESISTANCE    0.003f
#define CURRENT_AMP_GAIN            10.0f
#define ADC_VREF                    3.3f
#define ADC_FULL_SCALE              4095.0f
#define ADC_CURRENT_OFFSET          2048.0f
#define CURRENT_SCALE \
    (ADC_VREF / (ADC_FULL_SCALE * CURRENT_SHUNT_RESISTANCE * CURRENT_AMP_GAIN))

/* Bus-voltage sampling */
#define BUS_VOLTAGE_DIVIDER_RATIO   26.0f

/* SVPWM */
#define SVPWM_DUTY_MIN              0.05f
#define SVPWM_DUTY_MAX              0.95f
#define VOLTAGE_UTILIZATION         0.90f

/* Math constants */
#define FOC_PI                      3.14159265358979323846f
#define FOC_TWO_PI                  6.28318530717958647692f
#define FOC_INV_SQRT3               0.5773502691896258f
#define FOC_SQRT3                   1.7320508075688772f

/* Encoder electrical alignment */
#define ENCODER_ALIGN_VOLTAGE       1.0f
#define ENCODER_ALIGN_TIME_MS       1000U
#define ENCODER_ALIGN_ANGLE         0.0f
#endif // MOTOR_PARAM_H

