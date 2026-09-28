#ifndef MOTOR_PARAM_H
#define MOTOR_PARAM_H
/* ================= Motor ================= */
#define MOTOR_POLE_PAIRS        7
#define MOTOR_RS                2.55f
#define MOTOR_LD                0.00086f
#define MOTOR_LQ                0.00086f
#define MOTOR_FLUX              0.0035f

#define MOTOR_RATED_CURRENT     0.5f
#define MOTOR_MAX_CURRENT       2.0f
#define MOTOR_MAX_SPEED_RPM     2600.0f
/* ================= Control ================= */
#define PWM_FREQUENCY_HZ        16000.0f
#define CURRENT_LOOP_TS         (1.0f / PWM_FREQUENCY_HZ)

/* ================= Math ================= */

#define FOC_PI                  3.14159265358979323846f
#define FOC_TWO_PI              6.28318530717958647692f
#define FOC_INV_SQRT3           0.5773502691896258f
#define FOC_SQRT3               1.7320508075688772f
#endif // MOTOR_PARAM_H

/* ================= Encoder ================= */
#define ENCODER_CPR    4096U