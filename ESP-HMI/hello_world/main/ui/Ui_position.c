#include "Ui_position.h"
#include "Ui_speed.h"   // 返回速度页面
#include "comm_can.h"   // CAN通信接口
#include "esp_log.h"    // ESP-IDF日志输出
#include <stdio.h>   // 提供snprintf字符串格式化函数
/* ==================== 界面颜色 ==================== */
#define UI_COLOR_BG      0x07121F   // 页面背景色
#define UI_COLOR_CARD    0x0D2235   // 卡片背景色
#define UI_COLOR_BLUE    0x2196F3   // 位置主题色
#define UI_COLOR_TEXT    0xEAF4FF   // 主要文字颜色
#define UI_COLOR_DIM     0x8FA9BD   // 次要文字颜色
#define UI_COLOR_GREEN   0x32E875   // 在线状态颜色
#define UI_COLOR_RED  0xF04452   // STOP按钮颜色
/* ==================== 页面对象 ==================== */
static lv_obj_t *position_screen = NULL;   // 创建一个页面对象
/* ==================== 动态UI对象 ==================== */
static lv_obj_t *can_status_label = NULL;      // CAN状态
static lv_obj_t *position_slider = NULL;          // 目标位置滑块
static lv_obj_t *target_value_label = NULL;        // 顶部目标位置数值
static lv_obj_t *target_position_label = NULL;        // 信息区目标位置
static lv_obj_t *actual_position_label = NULL;        // 实际位置
static lv_obj_t *iq_info_label = NULL;             // Iq信息
static lv_obj_t *id_info_label = NULL;             // Id信息
static lv_obj_t *motor_state_label = NULL;   // 电机运行状态显示
/* ==================== UI状态 ==================== */
static int32_t target_position_cdeg = 0;   // 上电默认0度；滑块以0.01度为一个计数

/* ==================== 位置滑块事件 ==================== */
static void UI_PositionSliderEvent(lv_event_t *event)
{
    lv_obj_t *slider = lv_event_get_target(event);   // 获取触发事件的滑块对象
    target_position_cdeg = lv_slider_get_value(slider);   // 读取当前滑块值
    lv_label_set_text_fmt(target_value_label, "%.2f deg", (double)target_position_cdeg / CAN_POSITION_SCALE);   // 更新顶部目标位置
}
/* ==================== 位置滑块CAN发送事件 ==================== */
static void UI_PositionSliderSendEvent(lv_event_t *event)
{
    (void)event;   // 不使用事件参数

    esp_err_t ret = comm_can_send_command(CAN_CMD_SET_MODE, CAN_MODE_POSITION);   // 先选择位置模式，避免位置命令被速度模式忽略
    if (ret != ESP_OK)
    {
        ESP_LOGE("POSITION_UI", "Set mode request failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = comm_can_send_command(CAN_CMD_SET_POSITION, (uint16_t)target_position_cdeg);   // Byte1/2发送无符号角度×100

    if (ret != ESP_OK)
    {
        ESP_LOGE("POSITION_UI", "Set position request failed: %s", esp_err_to_name(ret));   // 仅表示发送请求未提交成功
    }
}
/* ==================== START按钮事件 ==================== */
static void UI_StartButtonEvent(lv_event_t *event)
{
    (void)event;   // 不使用事件参数

    esp_err_t ret = comm_can_send_command(CAN_CMD_SET_MODE, CAN_MODE_POSITION);   // 1.设置位置模式
    if (ret != ESP_OK)
    {
        ESP_LOGE("POSITION_UI", "Set mode request failed: %s", esp_err_to_name(ret));
        return;   // 发送失败则不继续启动
    }
    ret = comm_can_send_command(CAN_CMD_SET_POSITION, (uint16_t)target_position_cdeg);   //2.设置目标位置
    if (ret != ESP_OK)
    { 
        ESP_LOGE("POSITION_UI", "Set position request failed: %s", esp_err_to_name(ret));
        return;   // 发送失败则不继续启动
    }
    ret = comm_can_send_command(CAN_CMD_START, 0);   // 3.发送启动命令
    if (ret != ESP_OK)
    {
        ESP_LOGE("POSITION_UI", "START request failed: %s", esp_err_to_name(ret));
    }
}

/* ==================== STOP按钮事件 ==================== */
static void UI_StopButtonEvent(lv_event_t *event)
{
    (void)event;   // 暂时不使用事件参数
    esp_err_t ret = comm_can_send_command(CAN_CMD_STOP, 0);   // 发送停止命令
    if (ret != ESP_OK)
    {
        ESP_LOGE("POSITION_UI", "STOP request failed: %s", esp_err_to_name(ret));   // 记录请求提交错误，不混同总线发送结果
    }
}


/* ==================== 返回速度页面 ==================== */
static void UI_SpeedButtonEvent(lv_event_t *event)
{
    (void)event;
    UI_Speed_Show();   // 页面切换只改变显示，不立即切换电机控制模式
}

/* ==================== 位置控制页面函数 ==================== */
static void UI_Create_PositionScreen(void)//配置页面对象属性
{
    position_screen = lv_obj_create(NULL);   // 创建一个新的Screen页面
    lv_obj_set_style_bg_color(position_screen, lv_color_hex(UI_COLOR_BG), 0);   // 设置页面背景色
    lv_obj_set_style_bg_opa(position_screen, LV_OPA_COVER, 0);   // 设置页面背景透明度, 完全不透明
    lv_obj_clear_flag(position_screen, LV_OBJ_FLAG_SCROLLABLE);   // 禁止页面滚动
//创建标题
    lv_obj_t *title = lv_label_create(position_screen);   // 创建标题并放入位置控制页面
    lv_label_set_text(title,"POSITION CONTROL");   // 设置标题文字属性
    lv_obj_set_pos(title, 8, 6);   // 设置标题在页面中的位属性
    lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_TEXT), 0);   // 设置标题文字颜色属性
//创建CAN状态指示灯
    can_status_label = lv_label_create(position_screen);   // 创建CAN状态文字
    lv_label_set_text(can_status_label,"CAN: OFFLINE");   // 当前暂时显示离线
    lv_obj_set_pos(can_status_label,224,6);   // 设置CAN状态指示灯在页面中的位置
    lv_obj_set_style_text_color(can_status_label, lv_color_hex(UI_COLOR_DIM), 0);   // 设置CAN状态指示灯文字颜色属性
//创建目标位置卡片
    lv_obj_t *target_position_card = lv_obj_create(position_screen);   // 创建目标位置卡片
    lv_obj_set_style_bg_color(target_position_card, lv_color_hex(UI_COLOR_CARD), 0);   // 设置卡片背景色
    lv_obj_set_style_bg_opa(target_position_card, LV_OPA_COVER, 0);   // 设置卡片背景透明度
    lv_obj_set_style_radius(target_position_card, 8, 0);   // 设置卡片圆角半径
    lv_obj_set_style_border_width(target_position_card, 1, 0);   // 设置卡片边框宽度
    lv_obj_set_size(target_position_card, 308, 62);   // 设置卡片大小
    lv_obj_set_pos(target_position_card, 6, 28);   // 设置卡片在页面中的位置
    lv_obj_set_style_pad_all(target_position_card, 6, 0);   // 设置卡片内边距
    lv_obj_clear_flag(target_position_card, LV_OBJ_FLAG_SCROLLABLE);   // 禁止卡片滚动
//创建目标位置卡片的文字
    lv_obj_t *target_title = lv_label_create(target_position_card);   // 创建目标位置卡片的标题文字
    lv_label_set_text(target_title,"TARGET POSITION");   // 设置标题文字属性
    lv_obj_set_pos(target_title, 4, 1);   // 设置标题在卡片中的位置
    lv_obj_set_style_text_color(target_title, lv_color_hex(UI_COLOR_DIM), 0);   // 设置标题文字颜色属性
//创建目标位置数值
    target_value_label = lv_label_create(target_position_card);   // 创建目标位置数值文字
    lv_label_set_text_fmt(target_value_label, "%.2f deg", (double)target_position_cdeg / CAN_POSITION_SCALE);   // 设置初始数值
    lv_obj_set_pos(target_value_label, 194, 1);   // 设置数值在卡片中的位置
    lv_obj_set_style_text_color(target_value_label, lv_color_hex(0x2196F3), 0);   // 设置数值文字颜色属性

/* ==================== 位置滑块 ==================== */
position_slider = lv_slider_create(target_position_card);   // 创建位置滑块
lv_obj_set_size(position_slider, 286, 12);                   // 设置滑块大小
lv_obj_set_pos(position_slider, 4, 31);                      // 放在卡片下方
lv_slider_set_range(position_slider, 0, CAN_POSITION_FULL_CIRCLE - 1);   // 单圈范围0～359.99度，以0.01度计数
lv_slider_set_value(position_slider, target_position_cdeg, LV_ANIM_OFF);      // 初始目标位置：0度
lv_obj_set_style_bg_color(position_slider, lv_color_hex(0x24384A), LV_PART_MAIN);          // 未选中的滑轨颜色
lv_obj_set_style_bg_color(position_slider, lv_color_hex(0x2196F3), LV_PART_INDICATOR);     // 已选中的滑轨颜色
lv_obj_set_style_bg_color(position_slider, lv_color_hex(0x2196F3), LV_PART_KNOB);          // 滑块圆点颜色
lv_obj_add_event_cb(position_slider, UI_PositionSliderEvent, LV_EVENT_VALUE_CHANGED, NULL);   // 绑定滑块数值变化事件
lv_obj_add_event_cb(position_slider, UI_PositionSliderSendEvent, LV_EVENT_RELEASED, NULL);   // 松开滑块时发送CAN命令

/* ==================== START按钮 ==================== */
lv_obj_t *start_button = lv_button_create(position_screen);   // 创建START按钮
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

lv_obj_t *stop_button = lv_button_create(position_screen);    // 创建STOP按钮
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
/* ==================== Position Information 卡片 ==================== */

lv_obj_t *position_info_card = lv_obj_create(position_screen);   // 创建位置信息卡片
lv_obj_set_size(position_info_card, 308, 70);                 // 设置卡片大小
lv_obj_set_pos(position_info_card, 6, 136);                   // 放在按钮下方
lv_obj_set_style_bg_color(position_info_card, lv_color_hex(UI_COLOR_CARD), 0);   // 卡片背景色
lv_obj_set_style_bg_opa(position_info_card, LV_OPA_COVER, 0);               // 背景完全不透明
lv_obj_set_style_border_width(position_info_card, 0, 0);                    // 不显示边框
lv_obj_set_style_radius(position_info_card, 8, 0);                          // 设置圆角
lv_obj_set_style_pad_all(position_info_card, 6, 0);                         // 设置内部留白
lv_obj_clear_flag(position_info_card, LV_OBJ_FLAG_SCROLLABLE);              // 禁止卡片滚动
//创建Speed标题
lv_obj_t *position_info_title = lv_label_create(position_info_card);   // 创建Speed标题
lv_label_set_text(position_info_title, "Position Information");        // 设置标题文字
lv_obj_set_pos(position_info_title, 4, 0);                          // 设置标题位置
lv_obj_set_style_text_color(position_info_title, lv_color_hex(UI_COLOR_BLUE), 0);   // 蓝色标题

//创建目标位置信息
target_position_label = lv_label_create(position_info_card);   // 创建目标位置信息Label
lv_label_set_text(target_position_label, "Ref: -- deg");   // 收到反馈后显示STM32实际使用的目标
lv_obj_set_pos(target_position_label, 4, 24);
lv_obj_set_style_text_color(target_position_label, lv_color_hex(UI_COLOR_TEXT), 0);

//创建实际位置信息
actual_position_label = lv_label_create(position_info_card);   // 创建实际位置信息
lv_label_set_text(actual_position_label, "Actual: -- deg");            // 当前实际位置
lv_obj_set_pos(actual_position_label, 4, 42);                         // 留出默认字体16像素行高，避免卡片底部裁切
lv_obj_set_style_text_color(actual_position_label, lv_color_hex(UI_COLOR_TEXT), 0);   // 白色文字

//创建电流信息
lv_obj_t *current_info_title = lv_label_create(position_info_card);   // 创建电流信息标题
lv_label_set_text(current_info_title, "Ref/Act (A)");      // 两个电流数值依次为目标/实际，单位A
lv_obj_set_pos(current_info_title, 158, 0);                        // 放在卡片右半边
lv_obj_set_style_text_color(current_info_title, lv_color_hex(UI_COLOR_BLUE), 0);   // 蓝色标题
iq_info_label = lv_label_create(position_info_card);              // 创建Iq信息
lv_label_set_text(iq_info_label, "Iq:0.00/0.00");             // 初始值均为0，缩短文字以容纳负电流
lv_obj_set_pos(iq_info_label, 158, 24);                                  // 设置位置
lv_obj_set_width(iq_info_label, 138);   // 卡片右侧可用宽度：308-12-158
lv_label_set_long_mode(iq_info_label, LV_LABEL_LONG_CLIP);   // 限制在右侧区域，避免跨越卡片边界
lv_obj_set_style_text_color(iq_info_label, lv_color_hex(UI_COLOR_TEXT), 0);   // 白色文字
id_info_label = lv_label_create(position_info_card);              // 创建Id信息
lv_label_set_text(id_info_label, "Id:0.00/0.00");             // 初始值均为0，格式与Iq一致
lv_obj_set_pos(id_info_label, 158, 42);                                  // 与实际位置同一行，16像素行高完整放入卡片
lv_obj_set_width(id_info_label, 138);   // 与Iq使用相同宽度
lv_label_set_long_mode(id_info_label, LV_LABEL_LONG_CLIP);   // 限制在右侧区域
lv_obj_set_style_text_color(id_info_label, lv_color_hex(UI_COLOR_TEXT), 0);   // 白色文字

motor_state_label = lv_label_create(position_screen);   // 创建运行状态标签
lv_label_set_text(motor_state_label, "Motor: UNKNOWN");   // 初始状态未知
lv_obj_set_pos(motor_state_label, 8, 215);   // 放在信息卡片下方
lv_obj_set_style_text_color(motor_state_label, lv_color_hex(UI_COLOR_DIM), 0);   // 设置文字颜色

lv_obj_set_width(motor_state_label, 200);   // 页脚右侧放置页面切换按钮
lv_label_set_long_mode(motor_state_label, LV_LABEL_LONG_CLIP);
lv_obj_t *speed_button = lv_button_create(position_screen);
lv_obj_set_size(speed_button, 98, 24);
lv_obj_set_style_pad_all(speed_button, 0, 0);   // 与速度页入口保持一致，保证24像素高度内显示文字
lv_obj_set_pos(speed_button, 216, 211);
lv_obj_set_style_bg_color(speed_button, lv_color_hex(UI_COLOR_BLUE), 0);
lv_obj_set_style_border_width(speed_button, 0, 0);
lv_obj_set_style_radius(speed_button, 5, 0);
lv_obj_t *speed_label = lv_label_create(speed_button);
lv_label_set_text(speed_label, "SPEED");
lv_obj_center(speed_label);
lv_obj_add_event_cb(speed_button, UI_SpeedButtonEvent, LV_EVENT_CLICKED, NULL);
}

/* ==================== 共用接收定时器调用的显示更新 ==================== */
void UI_Position_SetCanOnline(bool online)
{
    if (can_status_label == NULL) return;
    lv_label_set_text(can_status_label, online ? "CAN: RX OK" : "CAN: OFFLINE");
    lv_obj_set_style_text_color(can_status_label, lv_color_hex(online ? UI_COLOR_GREEN : UI_COLOR_DIM), 0);
}

void UI_Position_UpdateStatus(const CAN_Status_t *status)
{
    if (status == NULL || motor_state_label == NULL) return;
    switch (status->state)   // 两页显示同一个电机的真实状态
    {
        case CAN_STATE_STOPPED: lv_label_set_text(motor_state_label, "Motor: STOPPED"); break;
        case CAN_STATE_ALIGNING: lv_label_set_text(motor_state_label, "Motor: ALIGNING"); break;
        case CAN_STATE_ALIGN_RELEASE: lv_label_set_text(motor_state_label, "Motor: ALIGN RELEASE"); break;
        case CAN_STATE_RUNNING: lv_label_set_text(motor_state_label, "Motor: RUNNING"); break;
        default: break;
    }
}

void UI_Position_UpdateCurrent(const CAN_Current_t *current)
{
    if (current == NULL || iq_info_label == NULL || id_info_label == NULL) return;
    char iq_text[64];
    char id_text[64];
    snprintf(iq_text, sizeof(iq_text), "Iq:%.2f/%.2f", (double)current->iq_ref, (double)current->iq);   // 目标/实际电流，单位A
    snprintf(id_text, sizeof(id_text), "Id:%.2f/%.2f", (double)current->id_ref, (double)current->id);
    lv_label_set_text(iq_info_label, iq_text);
    lv_label_set_text(id_info_label, id_text);
}

void UI_Position_UpdatePosition(const CAN_Position_t *position)
{
    if (position == NULL || actual_position_label == NULL || target_position_label == NULL) return;
    lv_label_set_text_fmt(actual_position_label, "Actual: %.2f deg", (double)position->actual_position_cdeg / CAN_POSITION_SCALE);
    lv_label_set_text_fmt(target_position_label, "Ref: %.2f deg", (double)position->target_position_cdeg / CAN_POSITION_SCALE);   // 下方Ref来自STM32反馈，上方滑块目标由用户设置
}

/* ==================== 位置页面公开接口 ==================== */
void UI_Position_Show(void)
{
    if (position_screen != NULL) lv_screen_load(position_screen);   // 保留目标滑块值，不重复创建页面
}

void UI_Position_Create(lv_display_t *display)
{
    if (display == NULL || position_screen != NULL) return;
    lv_display_set_default(display);
    UI_Create_PositionScreen();   // 不创建接收定时器，两页共用Ui_speed.c中的50ms定时器
}

