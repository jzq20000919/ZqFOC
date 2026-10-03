/**
 * @file    foc_key.c
 * @brief   FOC启停与调速按键初始化、消抖及轮询处理
 */
#include "foc_key.h"
#include "foc_manager.h"
#include "foc_vofa.h"
#include "motor_param.h"
#include "stm32g4xx_hal.h"

typedef struct
{
  GPIO_PinState last_level;
  GPIO_PinState stable_level;
  uint32_t last_change_tick;
} FOC_SPEED_BUTTON_State;

#define FOC_BUTTON_PORT        GPIOD
#define FOC_BUTTON_PIN         GPIO_PIN_2
#define FOC_BUTTON_DEBOUNCE_MS 30U
#define FOC_SPEED_UP_PORT      GPIOB
#define FOC_SPEED_UP_PIN       GPIO_PIN_6
#define FOC_SPEED_DOWN_PORT    GPIOC
#define FOC_SPEED_DOWN_PIN     GPIO_PIN_9

static GPIO_PinState button_last_level;
static GPIO_PinState button_stable_level;
static uint32_t button_last_change_tick;
static FOC_SPEED_BUTTON_State speed_up_button;
static FOC_SPEED_BUTTON_State speed_down_button;

/**
 * @brief  轮询SW1并在稳定按下沿切换启停
 */
static void FOC_KEY_ProcessButton(void)
{
  GPIO_PinState level = HAL_GPIO_ReadPin(FOC_BUTTON_PORT, FOC_BUTTON_PIN);
  uint32_t now = HAL_GetTick();

  if (level != button_last_level)
  {
    button_last_level = level;
    button_last_change_tick = now;
  }
  if (level != button_stable_level &&
      (uint32_t)(now - button_last_change_tick) >= FOC_BUTTON_DEBOUNCE_MS)
  {
    button_stable_level = level;
    if (level == GPIO_PIN_SET)
    {
      if (foc_motor.motor_state == FOC_STATE_STOPPED)
      {
        FOC_Motor_Start();
      }
      else
      {
        FOC_Motor_Stop();
      }
    }
  }
}
/**
 * @brief  检测调速按键经过消抖后的按下沿
 */
static uint8_t FOC_KEY_SpeedButtonPressed(GPIO_TypeDef *port, uint16_t pin,
                                         FOC_SPEED_BUTTON_State *button,
                                         uint32_t now)
{
  GPIO_PinState level = HAL_GPIO_ReadPin(port, pin);

  if (level != button->last_level)
  {
    button->last_level = level;
    button->last_change_tick = now;
  }
  if (level != button->stable_level &&
      (uint32_t)(now - button->last_change_tick) >= FOC_BUTTON_DEBOUNCE_MS)
  {
    button->stable_level = level;
    return (level == GPIO_PIN_SET) ? 1U : 0U;
  }
  return 0U;
}

/**
 * @brief  SW2/SW3按当前方向增减目标转速，每次按下只调整一次
 */
static void FOC_KEY_ProcessSpeedButtons(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t speed_up = FOC_KEY_SpeedButtonPressed(
      FOC_SPEED_UP_PORT, FOC_SPEED_UP_PIN, &speed_up_button, now);
  uint8_t speed_down = FOC_KEY_SpeedButtonPressed(
      FOC_SPEED_DOWN_PORT, FOC_SPEED_DOWN_PIN, &speed_down_button, now);
  if (foc_motor.motor_state != FOC_STATE_RUNNING ||
      FOC_VOFA_GetControlMode() != FOC_CONTROL_SPEED || speed_up == speed_down)
  {
    return;
  }

  float speed_ref = foc_motor.loop_spd.speed_ref;
  float speed_magnitude = (speed_ref < 0.0f) ? -speed_ref : speed_ref;
  if (speed_up != 0U)
  {
    speed_magnitude += MOTOR_SPEED_BUTTON_STEP_RPM;
  }
  else
  {
    // 减速最低到零，避免一次按压跨过零速而突然反转。
    speed_magnitude = (speed_magnitude > MOTOR_SPEED_BUTTON_STEP_RPM)
                          ? speed_magnitude - MOTOR_SPEED_BUTTON_STEP_RPM
                          : 0.0f;
  }
  FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd,
                           (speed_ref < 0.0f) ? -speed_magnitude : speed_magnitude);
}

/**
 * @brief  记录初始电平，避免上电时按住按键就触发动作
 */
void FOC_KEY_Init(void)
{
  // 记录上电时的按键电平；若一直按住，必须先松开再按才启动。
  button_last_level = HAL_GPIO_ReadPin(FOC_BUTTON_PORT, FOC_BUTTON_PIN);
  button_stable_level = button_last_level;
  button_last_change_tick = HAL_GetTick();
  speed_up_button.last_level = HAL_GPIO_ReadPin(FOC_SPEED_UP_PORT, FOC_SPEED_UP_PIN);
  speed_up_button.stable_level = speed_up_button.last_level;
  speed_up_button.last_change_tick = button_last_change_tick;
  speed_down_button.last_level = HAL_GPIO_ReadPin(FOC_SPEED_DOWN_PORT, FOC_SPEED_DOWN_PIN);
  speed_down_button.stable_level = speed_down_button.last_level;
  speed_down_button.last_change_tick = button_last_change_tick;
}

/**
 * @brief  先处理启停按键，再处理加减速按键
 */
void FOC_KEY_Task(void)
{
  FOC_KEY_ProcessButton();
  FOC_KEY_ProcessSpeedButtons();
}
