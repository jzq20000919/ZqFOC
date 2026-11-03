/**
 * @file    foc_encoder.c
 * @brief   编码器角度与转速实现
 */
#include "foc_encoder.h"
#include "motor_param.h"
#include "foc_math.h"
#include <math.h>
#include <string.h>

// G431RB最后2KB只保存零点，链接脚本已排除这一区域。
// 首次只写一个64位字，不在线擦页。重标定时用烧录工具擦除此页后复位。
#define ENCODER_ZERO_ADDRESS 0x0801F800U
#define ENCODER_ZERO_MAGIC   (0x5A000000U | (MOTOR_POLE_PAIRS << 16) | ENCODER_CPR)
static FOC_ENCODER_HandleTypeDef *saving_encoder;
static uint8_t alignment_started;
static uint8_t alignment_done;
static uint8_t sample_count;
static uint32_t alignment_tick;
static uint32_t sample_tick;
static uint32_t release_tick;
static float alignment_samples[ENCODER_ALIGN_SAMPLE_COUNT];

void FOC_ENCODER_Init(FOC_ENCODER_HandleTypeDef *encoder, TIM_HandleTypeDef *htim)
{
    encoder->htim = htim;
    encoder->spd_last_count = 0;
    encoder->count = 0;
    encoder->angle_m = 0.0f;
    encoder->angle_e = 0.0f;
    encoder->angle_e_offset = ENCODER_ELECTRICAL_OFFSET;
    encoder->speed = 0.0f;
    encoder->speed_raw = 0.0f;
    encoder->aligned_raw_angle = 0.0f;
    encoder->home_from_z = 0.0f;
    encoder->zero_raw_angle = 0.0f;
    encoder->zero_saved = 0U;
    encoder->zero_known = 0U;
    encoder->index_armed = 0U;
    encoder->index_found = 0U;
    encoder->index_count = 0U;
    saving_encoder = NULL;
    alignment_started = 0U;
    alignment_done = 0U;
    sample_count = 0U;
    HAL_NVIC_SetPriority(FLASH_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FLASH_IRQn);

    uint32_t saved[2];
    memcpy(saved, (const void *)ENCODER_ZERO_ADDRESS, sizeof(saved));
    if (saved[1] == ENCODER_ZERO_MAGIC)
    {
        memcpy(&encoder->home_from_z, &saved[0], sizeof(float));
        encoder->zero_saved = 1U;
    }
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL); 
}
void FOC_ENCODER_UpdateAngle(FOC_ENCODER_HandleTypeDef *encoder)
{
    // 1. 用定时器计数换算单圈机械角。
    encoder->count = __HAL_TIM_GET_COUNTER(encoder->htim);

    float raw_angle = (float)encoder->count * FOC_TWO_PI / (float)ENCODER_CPR;
    encoder->angle_e = FOC_MATH_Normalize(raw_angle * MOTOR_POLE_PAIRS + encoder->angle_e_offset);
    encoder->angle_m = FOC_MATH_Normalize(raw_angle - encoder->zero_raw_angle);
}
void FOC_ENCODER_UpdateSpeed(FOC_ENCODER_HandleTypeDef *encoder, float dt)
{
    int32_t count_now;
    int32_t d_count;
    const int32_t counts_per_rev = (int32_t)ENCODER_CPR;
    count_now = __HAL_TIM_GET_COUNTER(encoder->htim);
    d_count = count_now - encoder->spd_last_count;
    // 用有符号计数比较，避免无符号转换把微小计数差误判为整圈回绕。
    if (d_count > (counts_per_rev / 2))
    {
        d_count -= counts_per_rev;
    }
    else if (d_count < -(counts_per_rev / 2))
    {
        d_count += counts_per_rev;
    }
    if(dt >0.0f)
    {
        encoder->speed_raw = (float)d_count * 60.0f / (float)ENCODER_CPR / dt;
        float alpha = dt / (ENCODER_SPEED_FILTER_TS + dt); // 时间常数越大越平滑；5ms、1ms采样时，新测速值权重为1/6
        encoder->speed += alpha * (encoder->speed_raw - encoder->speed); // 控制环与VOFA使用同一滤波反馈，不能只美化显示
    }
    else
    {
        encoder->speed = 0.0f;
        encoder->speed_raw = 0.0f;
    }
    encoder->spd_last_count = count_now;
}
uint8_t FOC_ENCODER_Align(FOC_ENCODER_HandleTypeDef *encoder,
                         FOC_SVPWM_HandleTypeDef *svpwm, float v_bus)
{
    if (alignment_done) return 1U;
    uint32_t now = HAL_GetTick();
    if (alignment_started == 0U)
    {
        FOC_SVPWM_Update(svpwm, 0.0f, 0.0f, v_bus);
        FOC_SVPWM_Start(svpwm);
        FOC_SVPWM_Update(svpwm, ENCODER_ALIGN_VOLTAGE * cosf(ENCODER_ALIGN_ANGLE),
                         ENCODER_ALIGN_VOLTAGE * sinf(ENCODER_ALIGN_ANGLE), v_bus);
        alignment_started = 1U;
        alignment_tick = now;
        sample_tick = now;
    }
    if (sample_count == ENCODER_ALIGN_SAMPLE_COUNT)
    {
        if ((uint32_t)(now - release_tick) < ENCODER_ALIGN_RELEASE_TIME_MS) return 0U;
        FOC_SVPWM_Stop(svpwm); // 对齐结束先关闭通道，后续闭环才能重新启动PWM。
        alignment_done = 1U;
        return 1U;
    }
    if ((uint32_t)(now - alignment_tick) < ENCODER_ALIGN_TIME_MS) return 0U;
    if ((uint32_t)(now - sample_tick) < ENCODER_ALIGN_SAMPLE_INTERVAL_MS) return 0U;

    sample_tick = now;
    alignment_samples[sample_count++] = (float)__HAL_TIM_GET_COUNTER(encoder->htim)
                                      * FOC_TWO_PI / (float)ENCODER_CPR;
    if (sample_count == ENCODER_ALIGN_SAMPLE_COUNT)
    {
        FOC_ENCODER_CalibrateElectricalOffsetSamples(
            encoder, ENCODER_ALIGN_ANGLE, alignment_samples, sample_count);
        FOC_SVPWM_Update(svpwm, 0.0f, 0.0f, v_bus);
        release_tick = now;
    }
    return 0U;
}

void FOC_ENCODER_SetElectricalOffset(FOC_ENCODER_HandleTypeDef *encoder, float offset)
{
    encoder->angle_e_offset = FOC_MATH_Normalize(offset);
}

void FOC_ENCODER_CalibrateElectricalOffset(FOC_ENCODER_HandleTypeDef *encoder,float align_angle)
{
    float offset;
    // 以固定磁场对齐后的转子位置计算电角度零点。
    encoder->count = __HAL_TIM_GET_COUNTER(encoder->htim);
    encoder->aligned_raw_angle = (float)encoder->count * FOC_TWO_PI / (float)ENCODER_CPR;
    offset = align_angle - encoder->aligned_raw_angle * MOTOR_POLE_PAIRS;
    FOC_ENCODER_SetElectricalOffset(encoder, offset);
    // 同步角度与测速历史，避免首次速度更新出现计数跳变。
    FOC_ENCODER_UpdateAngle(encoder);
    encoder->spd_last_count = encoder->count;
    encoder->speed = 0.0f;
    encoder->speed_raw = 0.0f;
}

void FOC_ENCODER_CalibrateElectricalOffsetSamples(
    FOC_ENCODER_HandleTypeDef *encoder,
    float align_angle,
    const float *angle_samples,
    uint8_t sample_count)
{
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;

    // 圆周平均可避免机械角在0/2π处跨界时得到错误的平均值。
    for (uint8_t i = 0U; i < sample_count; ++i)
    {
        sum_sin += sinf(angle_samples[i]);
        sum_cos += cosf(angle_samples[i]);
    }
    float angle_m_avg = FOC_MATH_Normalize(atan2f(sum_sin, sum_cos));
    encoder->aligned_raw_angle = angle_m_avg; // 输入样本使用原始计数角度。
    float offset = align_angle - angle_m_avg * MOTOR_POLE_PAIRS;
    FOC_ENCODER_SetElectricalOffset(encoder, offset);
    FOC_ENCODER_UpdateAngle(encoder);
}

void FOC_ENCODER_CalibrateZero(FOC_ENCODER_HandleTypeDef *encoder)
{
    float z_angle = (float)encoder->index_count * FOC_TWO_PI / (float)ENCODER_CPR;
    encoder->home_from_z = FOC_MATH_Normalize(encoder->aligned_raw_angle - z_angle);
    FOC_ENCODER_RestoreZero(encoder);

    uint32_t words[2];
    uint64_t data;
    memcpy(&words[0], &encoder->home_from_z, sizeof(float));
    words[1] = ENCODER_ZERO_MAGIC;
    memcpy(&data, words, sizeof(data));
    saving_encoder = encoder;
    HAL_FLASH_Unlock();
    HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_DOUBLEWORD, ENCODER_ZERO_ADDRESS, data);
}

void FOC_ENCODER_RestoreZero(FOC_ENCODER_HandleTypeDef *encoder)
{
    float z_angle = (float)encoder->index_count * FOC_TWO_PI / (float)ENCODER_CPR;
    encoder->zero_raw_angle = FOC_MATH_Normalize(z_angle + encoder->home_from_z);
    encoder->angle_e_offset = FOC_MATH_Normalize(
        ENCODER_ALIGN_ANGLE - encoder->zero_raw_angle * MOTOR_POLE_PAIRS);
    encoder->zero_known = 1U;
    FOC_ENCODER_UpdateAngle(encoder);
}

uint8_t FOC_ENCODER_AtZero(FOC_ENCODER_HandleTypeDef *encoder)
{
    float error = encoder->angle_m;
    if (error > FOC_PI) error -= FOC_TWO_PI;
    return fabsf(error) <= ENCODER_HOME_ERROR_RAD && fabsf(encoder->speed) <= ENCODER_HOME_SPEED_ERROR;
}

void HAL_FLASH_EndOfOperationCallback(uint32_t address)
{
    (void)address;
    HAL_FLASH_Lock();
    saving_encoder->zero_saved = 1U;
}
