#include "motor_ui.h"
#include "comm_can.h"   // CAN通信接口
#include "esp_log.h"    // ESP-IDF日志输出
#include <stdio.h>   // 提供snprintf字符串格式化函数
/* ==================== 界面颜色 ==================== */
#define UI_COLOR_BG      0x07121F   // 页面背景色
#define UI_COLOR_CARD    0x0D2235   // 卡片背景色
#define UI_COLOR_BLUE    0x2196F3   // 速度主题色
#define UI_COLOR_TEXT    0xEAF4FF   // 主要文字颜色
#define UI_COLOR_DIM     0x8FA9BD   // 次要文字颜色
#define UI_COLOR_GREEN   0x32E875   // 在线状态颜色
#define UI_COLOR_RED  0xF04452   // STOP按钮颜色
/* ==================== 页面对象 ==================== */
static lv_obj_t *speed_screen = NULL;   // 创建一个页面对象
/* ==================== 动态UI对象 ==================== */
static lv_obj_t *can_status_label = NULL;      // CAN状态
static lv_obj_t *speed_slider = NULL;          // 目标速度滑块
static lv_obj_t *target_value_label = NULL;        // 顶部目标速度数值
static lv_obj_t *target_speed_label = NULL;        // 信息区目标速度
static lv_obj_t *actual_speed_label = NULL;        // 实际速度
static lv_obj_t *iq_info_label = NULL;             // Iq信息
static lv_obj_t *id_info_label = NULL;             // Id信息
static lv_obj_t *motor_state_label = NULL;   // 电机运行状态显示
/* ==================== UI状态 ==================== */
static int32_t target_speed_rpm = 500;         // 当前目标速度

/* ==================== 速度滑块事件 ==================== */
static void UI_SpeedSliderEvent(lv_event_t *event)
{
    lv_obj_t *slider = lv_event_get_target(event);   // 获取触发事件的滑块对象
    target_speed_rpm = lv_slider_get_value(slider);   // 读取当前滑块值
    lv_label_set_text_fmt(target_value_label, "%ld rpm", (long)target_speed_rpm);   // 更新顶部目标速度
    lv_label_set_text_fmt(target_speed_label, "Target: %ld rpm", (long)target_speed_rpm);   // 更新信息区目标速度
}
/* ==================== 速度滑块CAN发送事件 ==================== */
static void UI_SpeedSliderSendEvent(lv_event_t *event)
{
    (void)event;   // 不使用事件参数

    esp_err_t ret = comm_can_send_command(CAN_CMD_SET_SPEED, (uint16_t)(int16_t)target_speed_rpm);   // 发送目标速度

    if (ret != ESP_OK)
    {
        ESP_LOGE("MOTOR_UI", "Set speed failed: %s", esp_err_to_name(ret));   // 输出发送错误
    }
}
/* ==================== START按钮事件 ==================== */
static void UI_StartButtonEvent(lv_event_t *event)
{
    (void)event;   // 不使用事件参数

    esp_err_t ret = comm_can_send_command(CAN_CMD_SET_MODE, CAN_MODE_SPEED);   // 1.设置速度模式
    if (ret != ESP_OK)
    {
        ESP_LOGE("MOTOR_UI", "Set mode failed: %s", esp_err_to_name(ret));
        return;   // 发送失败则不继续启动
    }
    ret = comm_can_send_command(CAN_CMD_SET_SPEED, (uint16_t)(int16_t)target_speed_rpm);   //2.设置目标速度
    if (ret != ESP_OK)
    { 
        ESP_LOGE("MOTOR_UI", "Set speed failed: %s", esp_err_to_name(ret));
        return;   // 发送失败则不继续启动
    }
    ret = comm_can_send_command(CAN_CMD_START, 0);   // 3.发送启动命令
    if (ret != ESP_OK)
    {
        ESP_LOGE("MOTOR_UI", "START failed: %s", esp_err_to_name(ret));
    }
}

/* ==================== STOP按钮事件 ==================== */
static void UI_StopButtonEvent(lv_event_t *event)
{
    (void)event;   // 暂时不使用事件参数
    esp_err_t ret = comm_can_send_command(CAN_CMD_STOP, 0);   // 发送停止命令
    if (ret != ESP_OK)
    {
        ESP_LOGE("MOTOR_UI", "STOP command failed: %s", esp_err_to_name(ret));   // 记录发送错误
    }
}


/* ==================== 速度控制页面函数 ==================== */
static void UI_Create_SpeedScreen(void)//配置页面对象属性
{
    speed_screen = lv_obj_create(NULL);   // 创建一个新的Screen页面
    lv_obj_set_style_bg_color(speed_screen, lv_color_hex(UI_COLOR_BG), 0);   // 设置页面背景色
    lv_obj_set_style_bg_opa(speed_screen, LV_OPA_COVER, 0);   // 设置页面背景透明度, 完全不透明
    lv_obj_clear_flag(speed_screen, LV_OBJ_FLAG_SCROLLABLE);   // 禁止页面滚动
//创建标题
    lv_obj_t *title = lv_label_create(speed_screen);   // 创建标题并放入速度控制页面
    lv_label_set_text(title,"SPEED CONTROL");   // 设置标题文字属性
    lv_obj_set_pos(title, 8, 6);   // 设置标题在页面中的位属性
    lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);   // 设置标题文字颜色属性
//创建CAN状态指示灯
    can_status_label = lv_label_create(speed_screen);   // 创建CAN状态文字
    lv_label_set_text(can_status_label,"CAN: OFFLINE");   // 当前暂时显示离线
    lv_obj_set_pos(can_status_label,224,6);   // 设置CAN状态指示灯在页面中的位置
    lv_obj_set_style_text_color(can_status_label, lv_color_hex(UI_COLOR_DIM), 0);   // 设置CAN状态指示灯文字颜色属性
//创建目标速度卡片
    lv_obj_t *target_speed_card = lv_obj_create(speed_screen);   // 创建目标速度卡片
    lv_obj_set_style_bg_color(target_speed_card, lv_color_hex(UI_COLOR_CARD), 0);   // 设置卡片背景色
    lv_obj_set_style_bg_opa(target_speed_card, LV_OPA_COVER, 0);   // 设置卡片背景透明度
    lv_obj_set_style_radius(target_speed_card, 8, 0);   // 设置卡片圆角半径
    lv_obj_set_style_border_width(target_speed_card, 1, 0);   // 设置卡片边框宽度
    lv_obj_set_size(target_speed_card, 308, 62);   // 设置卡片大小
    lv_obj_set_pos(target_speed_card, 6, 28);   // 设置卡片在页面中的位置
    lv_obj_set_style_pad_all(target_speed_card, 6, 0);   // 设置卡片内边距
    lv_obj_clear_flag(target_speed_card, LV_OBJ_FLAG_SCROLLABLE);   // 禁止卡片滚动
//创建目标速度卡片的文字
    lv_obj_t *target_title = lv_label_create(target_speed_card);   // 创建目标速度卡片的标题文字
    lv_label_set_text(target_title,"TARGET SPEED");   // 设置标题文字属性
    lv_obj_set_pos(target_title, 4, 1);   // 设置标题在卡片中的位置
    lv_obj_set_style_text_color(target_title, lv_color_hex(UI_COLOR_DIM), 0);   // 设置标题文字颜色属性
//创建目标速度数值
    target_value_label = lv_label_create(target_speed_card);   // 创建目标速度数值文字
    lv_label_set_text_fmt(target_value_label, "%ld rpm", (long)target_speed_rpm);   // 设置初始数值
    lv_obj_set_pos(target_value_label, 220, 1);   // 设置数值在卡片中的位置
    lv_obj_set_style_text_color(target_value_label, lv_color_hex(0x2196F3), 0);   // 设置数值文字颜色属性

/* ==================== 速度滑块 ==================== */
speed_slider = lv_slider_create(target_speed_card);   // 创建速度滑块
lv_obj_set_size(speed_slider, 286, 12);                   // 设置滑块大小
lv_obj_set_pos(speed_slider, 4, 31);                      // 放在卡片下方
lv_slider_set_range(speed_slider, -2600, 2600);           // 设置速度范围：-2600~2600 rpm
lv_slider_set_value(speed_slider, target_speed_rpm, LV_ANIM_OFF);      // 初始目标速度：500 rpm
lv_obj_set_style_bg_color(speed_slider, lv_color_hex(0x24384A), LV_PART_MAIN);          // 未选中的滑轨颜色
lv_obj_set_style_bg_color(speed_slider, lv_color_hex(0x2196F3), LV_PART_INDICATOR);     // 已选中的滑轨颜色
lv_obj_set_style_bg_color(speed_slider, lv_color_hex(0x2196F3), LV_PART_KNOB);          // 滑块圆点颜色
lv_obj_add_event_cb(speed_slider, UI_SpeedSliderEvent, LV_EVENT_VALUE_CHANGED, NULL);   // 绑定滑块数值变化事件
lv_obj_add_event_cb(speed_slider, UI_SpeedSliderSendEvent, LV_EVENT_RELEASED, NULL);   // 松开滑块时发送CAN命令

/* ==================== START按钮 ==================== */
lv_obj_t *start_button = lv_button_create(speed_screen);   // 创建START按钮
lv_obj_set_size(start_button, 150, 34);                    // 设置按钮大小
lv_obj_set_pos(start_button, 6, 96);                       // 设置按钮位置
lv_obj_set_style_bg_color(start_button, lv_color_hex(UI_COLOR_GREEN), 0);   // 设置绿色背景
lv_obj_set_style_border_width(start_button, 0, 0);                          // 不显示边框
lv_obj_set_style_radius(start_button, 7, 0);                                // 设置圆角
lv_obj_t *start_label = lv_label_create(start_button);     // 在按钮中创建文字
lv_label_set_text(start_label, "START");                   // 设置按钮文字
lv_obj_set_style_text_color(start_label, lv_color_hex(0x06140B), 0);        // 设置文字颜色
lv_obj_center(start_label);                                // 文字居中
lv_obj_add_event_cb(start_button, UI_StartButtonEvent, LV_EVENT_CLICKED, NULL);   // 添加START按钮事件回调
/* ==================== STOP按钮 ==================== */

lv_obj_t *stop_button = lv_button_create(speed_screen);    // 创建STOP按钮
lv_obj_set_size(stop_button, 152, 34);                     // 设置按钮大小
lv_obj_set_pos(stop_button, 162, 96);                      // 放在START右侧
lv_obj_set_style_bg_color(stop_button, lv_color_hex(UI_COLOR_RED), 0);      // 设置红色背景
lv_obj_set_style_border_width(stop_button, 0, 0);                           // 不显示边框
lv_obj_set_style_radius(stop_button, 7, 0);                                 // 设置圆角
lv_obj_t *stop_label = lv_label_create(stop_button);       // 在按钮中创建文字
lv_label_set_text(stop_label, "STOP");                     // 设置按钮文字
lv_obj_set_style_text_color(stop_label, lv_color_hex(UI_COLOR_TEXT), 0);    // 设置白色文字
lv_obj_center(stop_label);                                 // 文字居中
lv_obj_add_event_cb(stop_button, UI_StopButtonEvent, LV_EVENT_CLICKED, NULL);       // 添加STOP按钮事件回调
/* ==================== Speed Information 卡片 ==================== */

lv_obj_t *speed_info_card = lv_obj_create(speed_screen);   // 创建速度信息卡片
lv_obj_set_size(speed_info_card, 308, 70);                 // 设置卡片大小
lv_obj_set_pos(speed_info_card, 6, 136);                   // 放在按钮下方
lv_obj_set_style_bg_color(speed_info_card, lv_color_hex(UI_COLOR_CARD), 0);   // 卡片背景色
lv_obj_set_style_bg_opa(speed_info_card, LV_OPA_COVER, 0);               // 背景完全不透明
lv_obj_set_style_border_width(speed_info_card, 0, 0);                    // 不显示边框
lv_obj_set_style_radius(speed_info_card, 8, 0);                          // 设置圆角
lv_obj_set_style_pad_all(speed_info_card, 6, 0);                         // 设置内部留白
lv_obj_clear_flag(speed_info_card, LV_OBJ_FLAG_SCROLLABLE);              // 禁止卡片滚动
//创建Speed标题
lv_obj_t *speed_info_title = lv_label_create(speed_info_card);   // 创建Speed标题
lv_label_set_text(speed_info_title, "Speed Information");        // 设置标题文字
lv_obj_set_pos(speed_info_title, 4, 0);                          // 设置标题位置
lv_obj_set_style_text_color(speed_info_title, lv_color_hex(UI_COLOR_BLUE), 0);   // 蓝色标题

//创建目标速度信息
target_speed_label = lv_label_create(speed_info_card);   // 创建目标速度信息Label
lv_label_set_text_fmt(target_speed_label, "Target: %ld rpm", (long)target_speed_rpm);
lv_obj_set_pos(target_speed_label, 4, 24);
lv_obj_set_style_text_color(target_speed_label, lv_color_hex(UI_COLOR_TEXT), 0);

//创建实际速度信息
actual_speed_label = lv_label_create(speed_info_card);   // 创建实际速度信息
lv_label_set_text(actual_speed_label, "Actual: 0 rpm");            // 当前实际速度
lv_obj_set_pos(actual_speed_label, 4, 45);                         // 设置位置
lv_obj_set_style_text_color(actual_speed_label, lv_color_hex(UI_COLOR_TEXT), 0);   // 白色文字

//创建电流信息
lv_obj_t *current_info_title = lv_label_create(speed_info_card);   // 创建电流信息标题
lv_label_set_text(current_info_title, "Current Information");      // 设置标题文字
lv_obj_set_pos(current_info_title, 158, 0);                        // 放在卡片右半边
lv_obj_set_style_text_color(current_info_title, lv_color_hex(UI_COLOR_BLUE), 0);   // 蓝色标题
iq_info_label = lv_label_create(speed_info_card);              // 创建Iq信息
lv_label_set_text(iq_info_label, "Iq Ref: 0.00   Iq: 0.00");             // 初始值均为0
lv_obj_set_pos(iq_info_label, 158, 24);                                  // 设置位置
lv_obj_set_style_text_color(iq_info_label, lv_color_hex(UI_COLOR_TEXT), 0);   // 白色文字
id_info_label = lv_label_create(speed_info_card);              // 创建Id信息
lv_label_set_text(id_info_label, "Id Ref: 0.00   Id: 0.00");             // 初始值均为0
lv_obj_set_pos(id_info_label, 158, 45);                                  // 设置位置
lv_obj_set_style_text_color(id_info_label, lv_color_hex(UI_COLOR_TEXT), 0);   // 白色文字

motor_state_label = lv_label_create(speed_screen);   // 创建运行状态标签
lv_label_set_text(motor_state_label, "Motor: UNKNOWN");   // 初始状态未知
lv_obj_set_pos(motor_state_label, 8, 215);   // 放在信息卡片下方
lv_obj_set_style_text_color(motor_state_label, lv_color_hex(UI_COLOR_DIM), 0);   // 设置文字颜色

}

/* ==================== CAN数据接收与UI更新 ==================== */
static void UI_CanReceiveTimer(lv_timer_t *timer)
{
    (void)timer;   // 不使用定时器参数

    CAN_RxMessage_t message;   // 原始CAN报文
    CAN_Status_t status;       // 解析后的电机状态
    CAN_Current_t current;   // 保存解析后的电流数据
    CAN_Speed_t speed;   // 解析后的速度数据
    while (comm_can_receive(&message))//获取原始CAN报文
    {
        if (comm_can_parse_status(&message, &status))
        {
            lv_label_set_text(can_status_label, "CAN: RX OK");   // 表示已收到有效状态报文
            switch (status.state)
            {
                case CAN_STATE_STOPPED:
                    lv_label_set_text(motor_state_label, "Motor: STOPPED");
                    break;

                case CAN_STATE_ALIGNING:
                    lv_label_set_text(motor_state_label, "Motor: ALIGNING");
                    break;

                case CAN_STATE_ALIGN_RELEASE:
                    lv_label_set_text(motor_state_label, "Motor: ALIGN RELEASE");
                    break;

                case CAN_STATE_RUNNING:
                    lv_label_set_text(motor_state_label, "Motor: RUNNING");
                    break;

                default:
                    break;
            }
        }
        else if (comm_can_parse_speed(&message, &speed))
        {
            lv_label_set_text_fmt(actual_speed_label, "Actual: %d rpm", (int)speed.actual_speed_rpm);   // 更新实际速度
        }
        else if (comm_can_parse_current(&message, &current))
        {
            char iq_text[64];   // 保存Iq显示字符串
            char id_text[64];   // 保存Id显示字符串
            snprintf(iq_text, sizeof(iq_text), "Iq Ref: %.2f  Iq: %.2f", (double)current.iq_ref, (double)current.iq);   // 格式化Iq数据
            snprintf(id_text, sizeof(id_text), "Id Ref: %.2f  Id: %.2f", (double)current.id_ref, (double)current.id);   // 格式化Id数据
            lv_label_set_text(iq_info_label, iq_text);   // 更新Iq标签
            lv_label_set_text(id_info_label, id_text);   // 更新Id标签  
        }
    }
}



void motor_ui_create(lv_display_t *display)
{
    if (display == NULL) return;   // 防止传入无效显示器
    lv_display_set_default(display);   // 设置当前默认显示器
    UI_Create_SpeedScreen();   // 创建速度控制页面
    lv_screen_load(speed_screen);   // 显示速度控制页面
    lv_timer_create(UI_CanReceiveTimer, 50, NULL);   // 每50ms调用一次UI_CanReceiveTimer
}