//#include "ui.h"
//#include "ui_priority.h"
//#include "cap.h"
//#include "communicate.h"
//#include "judge_protocol.h"

//void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle);
//void Get_Hp(float *current_hp,float *max_hp,robot_type_e robot_type);

//my_ui_config_t my_ui_config = {
//  .armor_radius = 100,
//};

//ui_info_t dynamic_ui_info [DYNAMIC_UI_NUM] = 
//{
//  [D_CAP_VOLTAGE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = ARC, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 20, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X , // 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y , //  坐标
//  .ui_config.end_x = 360, // x 半轴长度
//  .ui_config.end_y = 360, // y 半轴长度
//  .ui_config.start_angel = 240, // 起始角度
//  .ui_config.end_angel = 240, // 终止角度
//  },
//  [D_FRIC_B_SPEED] = {
//    /*不变配置*/
//    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//    .ui_config.ui_type = FLOAT, // UI内容类型
//    /*可变配置*/
//    .ui_config.operate_type = MODIFY, // 操作类型
//    .ui_config.layer = 0, // 图层数，0~9
//    .ui_config.color = YELLOW, // 颜色
//    .ui_config.size = 20, // 字体大小
//    .ui_config.width = 3, // 线条宽度
//    .ui_config.start_x = CLIENT_MID_POSITION_X + 470, // 起点 x 坐标
//    .ui_config.start_y = CLIENT_MID_POSITION_Y + 320, // 起点 y 坐标
//    .ui_config.float_num = 66.66, // 显示的数字
//    .ui_config.decimal = 0, // 小数位有效个数
//  },
//  [D_FRIC_F_SPEED] = {
//    /*不变配置*/
//    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//    .ui_config.ui_type = FLOAT, // UI内容类型
//    /*可变配置*/
//    .ui_config.operate_type = MODIFY, // 操作类型
//    .ui_config.layer = 0, // 图层数，0~9
//    .ui_config.color = PINK, // 颜色
//    .ui_config.size = 20, // 字体大小 
//    .ui_config.width = 3, // 线条宽度
//    .ui_config.start_x = CLIENT_MID_POSITION_X +600, // 起点 x 坐标
//    .ui_config.start_y = CLIENT_MID_POSITION_Y +320, // 起点 y 坐标
//    .ui_config.float_num = 66.66, // 显示的数字
//    .ui_config.decimal = 0, // 小数位有效个数
//  },
//  [D_UWB_YAW] = {
//    /*不变配置*/
//    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
//    .ui_config.ui_type = FLOAT, // UI内容类型
//    /*可变配置*/
//    .ui_config.operate_type = MODIFY, // 操作类型
//    .ui_config.layer = 0, // 图层数，0~9
//    .ui_config.color = GREEN, // 颜色
//    .ui_config.size = 20, // 字体大小 
//    .ui_config.width = 2, // 线条宽度
//    .ui_config.start_x = CLIENT_MID_POSITION_X + 730, // 起点 x 坐标
//    .ui_config.start_y = CLIENT_MID_POSITION_Y + 320, // 起点 y 坐标
//    .ui_config.float_num = 66.66, // 显示的数字
//    .ui_config.decimal = 2, // 小数位有效个数
//  },
//  [D_PTICH_IMU_ANGLE] = {
//  /*不变配置*/
//  .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = FLOAT, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = GREEN, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + 80,  // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 350, // 起点 y 坐标
//  .ui_config.float_num = 66.66, // 显示的数字
//  .ui_config.decimal = 2, // 小数位有效个数
//  },

//  [D_VISION_CIRCLE] = {
//  /*不变配置*/
//  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 60, // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  
//  [D_SPEED_ADAPT_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 172, // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  [D_CAP_ON_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 116, // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  [D_CAR_MODE] = {
//  /*不变配置*/
//  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = GREEN, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 17, // 起点 y 坐标
//  .ui_config.text = "DUNE", // 显示的文字
//  }, 
//  [D_FRIC_STATE_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + 650, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - 20, // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  
//  [D_HIT_TARGET_DISTANCE] = {
//  /*不变配置*/
//  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = FLOAT, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + -50, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 350, // 起点 y 坐标
//  .ui_config.decimal = 2,
//  .ui_config.float_num = 0, // 显示的数字
//  },

//  [D_RFID_CYCLE] = {
// /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 238 , // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  [D_MID_RECTANGEL] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = RECTANGEL, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = BLACK, // 颜色
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - _VISION__RECTANGEL_X_WIDTH, // 左上角 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + _VISION__RECTANGEL_Y_OFFSET, // 左上角 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + _VISION__RECTANGEL_X_WIDTH, // 右下角 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y - _VISION__RECTANGEL_Y_OFFSET, // 右下角 y 坐标
//  },
//  [D_HIT_TARGET_TRIANGLE_1] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 10, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 120, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X , // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y + 137, // 终点 y 坐标
//  },
//  [D_HIT_TARGET_TRIANGLE_2] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X , // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 137, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + 10, // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y + 120, // 终点 y 坐标
//  }, 
//  [D_HIT_HIGHLIGHT_LINE_1] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 3, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_HIT_HIGHLIGHT_LINE_2] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 3, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_HEAD_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X , // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y +130, // 圆心 y 坐标
//  .ui_config.radius = 20, // 半径
//  },
//  #ifndef UI_SIMPLIFY
//  [D_VISION_ARMOR_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = BLACK, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = 0 , // 圆心 x 坐标
//  .ui_config.start_y = 0 , // 圆心 y 坐标
//  .ui_config.radius = 11, // 半径
//  },
//  [D_VISION_HP_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = ARC, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = GREEN, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = 0, // 圆心 x 坐标
//  .ui_config.start_y = 0, // 圆心 y 坐标
//  .ui_config.end_x = 11, // x 半轴长度
//  .ui_config.end_y = 11, // y 半轴长度
//  .ui_config.start_angel = 0, // 起始角度
//  .ui_config.end_angel = 0, // 终止角度
//  },
//  [D_VISION_WHITE_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = ARC, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = 0, // 圆心 x 坐标
//  .ui_config.start_y = 0, // 圆心 y 坐标
//  .ui_config.end_x = 11, // x 半轴长度
//  .ui_config.end_y = 11, // y 半轴长度
//  .ui_config.start_angel = 0, // 起始角度
//  .ui_config.end_angel = 0, // 终止角度
//  },
//  [D_ROI_LEFT] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_ROI_UP] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_ROI_RIGHT] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_ROI_DOWN] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_ROI_MID] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = FUCHSIA, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  #endif
//  [D_PITCH_POINTER] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 5, // 线条宽度
//  .ui_config.start_x = 1260, // 起点 x 坐标
//  .ui_config.start_y = 540, // 起点 y 坐标
//  .ui_config.end_x = 1340, // 终点 x 坐标
//  .ui_config.end_y = 540, // 终点 y 坐标
//  },
// 
//};

//ui_info_t const_ui_info [CONST_UI_NUM] = 
//{
//  [C_VISION_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 72, // 起点 y 坐标
//  .ui_config.text = "VISION", // 显示的文字
//  },
//  [C_FRIC_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + 530, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y , // 起点 y 坐标
//  .ui_config.text = "FRIC", // 显示的文字
//  },
//  [C_FRIC_ADAPT_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 190, // 起点 y 坐标
//  .ui_config.text = "FRIC_ADAPT", // 显示的文字
//  },
//  [C_RFID_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 255, // 起点 y 坐标
//  .ui_config.text = "RFID", // 显示的文字
//  },
//  [C_CAP_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 127, // 起点 y 坐标
//  .ui_config.text = "CAP", // 显示的文字
//  },
//  [C_CAR_MODE_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 17, // 起点 y 坐标
//  .ui_config.text = "MODE", // 显示的文字
//  },
//  [C_MID_LINE] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = ORANGE, // 颜色
//  .ui_config.width = 2, // 线条宽度
//	  #if HERO_TYPE!=2
//	   .ui_config.start_x = CLIENT_MID_POSITION_X+0, // 起点 x 坐标
//	  #else
//	   .ui_config.start_x = CLIENT_MID_POSITION_X-15, // 起点 x 坐标
//	  #endif
//		.ui_config.start_y = CLIENT_MID_POSITION_Y, // 起点 y 坐标
//	  
//	  #if HERO_TYPE!=2
//	   .ui_config.end_x = CLIENT_MID_POSITION_X+0, // 起点 x 坐标
//	  #else
//	   .ui_config.end_x = CLIENT_MID_POSITION_X-15, // 起点 x 坐标
//	  #endif
//	  
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [C_TOILET_DOWN_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = FUCHSIA, // 颜色
//  .ui_config.size = 10, // 字体大小
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + _TOILET_DOWN_WIDTH + 20, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _TOILET_DOWN_OFFSET + 5, // 起点 y 坐标
//  .ui_config.text = "TOILET_DOWN", // 显示的文字
//  },
//  [C_OUTPOST_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = FUCHSIA, // 颜色
//  .ui_config.size = 10, // 字体大小
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + _OUTPOST_WIDTH + 20, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _OUTPOST_OFFSET + 5, // 起点 y 坐标
//  .ui_config.text = "OUTPOST", // 显示的文字
//  },
//  [C_HEIGHT_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 10, // 字体大小
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + _HEIGHT_WIDTH + 20, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _HEIGHT_OFFSET + 5, // 起点 y 坐标
//  .ui_config.text = "HEIGHT", // 显示的文字
//  },
//  [C_HEIGHT_LINE] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - _HEIGHT_WIDTH, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _HEIGHT_OFFSET, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + _HEIGHT_WIDTH, // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y - _HEIGHT_OFFSET, // 终点 y 坐标
//  },
//  [C_OUTPOST_LINE] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - _OUTPOST_WIDTH, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _OUTPOST_OFFSET, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + _OUTPOST_WIDTH, // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y - _OUTPOST_OFFSET, // 终点 y 坐标
//  },
//  [C_TOILET_DOWN_LINE] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - _TOILET_DOWN_WIDTH, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _TOILET_DOWN_OFFSET, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + _TOILET_DOWN_WIDTH, // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y - _TOILET_DOWN_OFFSET, // 终点 y 坐标
//  },
//  [C_PASS_LINE_LEFT] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 155, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - 60, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X - 420, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [C_PASS_LINE_RIGHT] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色·
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + 155, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - 60, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + 420, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
////  #ifndef UI_SIMPLIFY
//  [C_PITCH_LINE_0] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1300, // 起点 x 坐标
//  .ui_config.start_y = 540, // 起点 y 坐标
//  .ui_config.end_x = 1330, // 终点 x 坐标
//  .ui_config.end_y = 540, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_10] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1295, // 起点 x 坐标
//  .ui_config.start_y = 599, // 起点 y 坐标
//  .ui_config.end_x = 1324, // 终点 x 坐标
//  .ui_config.end_y = 604, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_20] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1279, // 起点 x 坐标
//  .ui_config.start_y = 656, // 起点 y 坐标
//  .ui_config.end_x = 1307, // 终点 x 坐标
//  .ui_config.end_y = 666, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_30] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1254, // 起点 x 坐标
//  .ui_config.start_y = 710, // 起点 y 坐标
//  .ui_config.end_x = 1280, // 终点 x 坐标
//  .ui_config.end_y = 725, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_40] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1220, // 起点 x 坐标
//  .ui_config.start_y = 758, // 起点 y 坐标
//  .ui_config.end_x = 1243, // 终点 x 坐标
//  .ui_config.end_y = 778, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_50] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1178, // 起点 x 坐标
//  .ui_config.start_y = 800, // 起点 y 坐标
//  .ui_config.end_x = 1198, // 终点 x 坐标
//  .ui_config.end_y = 823, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_N_10] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1294, // 起点 x 坐标
//  .ui_config.start_y = 480, // 起点 y 坐标
//  .ui_config.end_x = 1324, // 终点 x 坐标
//  .ui_config.end_y = 476, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_N_20] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1279, // 起点 x 坐标
//  .ui_config.start_y = 424, // 起点 y 坐标
//  .ui_config.end_x = 1308, // 终点 x 坐标
//  .ui_config.end_y = 413, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_N_30] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1254, // 起点 x 坐标
//  .ui_config.start_y = 370, // 起点 y 坐标
//  .ui_config.end_x = 1280, // 终点 x 坐标
//  .ui_config.end_y = 355, // 终点 y 坐标
//  }, 
//  [C_PITCH_50_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 1200, // 起点 x 坐标
//  .ui_config.start_y = 865 , // 起点 y 坐标
//  .ui_config.text = "50", // 显示的文字
//  },
//  [C_PITCH_0_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 1340, // 起点 x 坐标
//  .ui_config.start_y = 550 , // 起点 y 坐标
//  .ui_config.text = "0", // 显示的文字
//  },
//  [C_PITCH_N_30_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 1290, // 起点 x 坐标
//  .ui_config.start_y = 346 , // 起点 y 坐标
//  .ui_config.text = "-30", // 显示的文字
//  },
//  [C_PITCH_30_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 1300, // 起点 x 坐标
//  .ui_config.start_y = 760 , // 起点 y 坐标
//  .ui_config.text = "30", // 显示的文字
//  },
////  #endif
//};


///**
// * @brief 初始化UI 在main.c中调用
// * 
// */
//void My_Ui_Init(void)
//{
//  Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
//}

//// float max_hp,current_hp;

///**
// * @brief UI信息更新 任务中调用
// * 
// */

//void Ui_Info_Update(void)
//{
//	client_info_update();
//  /*超电UI更新****************************************************/
//   static float last_cap_voltage = 0;
//	//#define BUFFER
//	#ifdef BUFFER
//	float cap_voltage = (float)judge.power_heat_data.chassis_power_buffer;
//	dynamic_ui_info[D_CAP_VOLTAGE].ui_config.end_angel = 240.f + 75.f*(cap_voltage/60.f);
//	#else
//	 
//	float cap_voltage = My_Cap.cap_v;
//	 dynamic_ui_info[D_CAP_VOLTAGE].ui_config.end_angel = 240.f + 75.f*(cap_voltage/24.f)*(cap_voltage/24.f);
//	#endif
//  
//  if (last_cap_voltage != cap_voltage)
//  {
//    
//	 #ifdef BUFFER
//	if (cap_voltage < 25)//电压低于15V变红
//	#else
//	if (cap_voltage < 15)//电压低于15V变红
//	#endif 
//    {
//      dynamic_ui_info[D_CAP_VOLTAGE].ui_config.color = PINK;
//    }
//	else if(cap_voltage>23.4f)
//	{
//      dynamic_ui_info[D_CAP_VOLTAGE].ui_config.color = GREEN;
//    }
//    else //电压高于15V变白+
//    {
//      dynamic_ui_info[D_CAP_VOLTAGE].ui_config.color = WHITE;
//    }
////	if (cap.info.wireless_rx_data.is_charging==1)
////	{
////		dynamic_ui_info[D_CAP_VOLTAGE].ui_config.color=YELLOW;
////	}
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_CAP_VOLTAGE]);
//  }
//  last_cap_voltage = cap_voltage;
//  /*UWB XY更新*****************************************/
////  static float last_fric_b_speed = 0, last_fric_f_speed = 0, last_uwb_yaw = 0;
////  float fric_f_speed =  communicate.car_data1_rx_info->fric_f_speed;
////  float fric_b_speed =  communicate.car_data1_rx_info->fric_b_speed;
////  float uwb_yaw = judge.game_robot_pos.angle;
////  if (last_fric_f_speed != fric_f_speed)
////  {
////    dynamic_ui_info[D_FRIC_F_SPEED].ui_config.float_num = fric_f_speed;
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_FRIC_F_SPEED]);
////  }
////  if (last_fric_b_speed != fric_b_speed)
////  {
////    dynamic_ui_info[D_FRIC_B_SPEED].ui_config.float_num = fric_b_speed;
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_FRIC_B_SPEED]);
////  }
////  if (last_uwb_yaw != uwb_yaw)
////  {
////    dynamic_ui_info[D_UWB_YAW].ui_config.float_num = uwb_yaw;
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_UWB_YAW]);
////  }
////  last_uwb_yaw = uwb_yaw;
////  last_fric_f_speed = fric_f_speed;
////  last_fric_b_speed = fric_b_speed;
//  /*云台机械角度更新***************************************************/
//  /*云台角度UI更新****************************************************/
//  static float last_pitch_imu_angle = 0;
//  float pitch_imu_angle = (float)Board_Rx_Info.pitch_imu / 100.f; // 云台角度转浮点数
//  if (last_pitch_imu_angle != pitch_imu_angle)
//  {
//    dynamic_ui_info[D_PTICH_IMU_ANGLE].ui_config.float_num = pitch_imu_angle;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_PTICH_IMU_ANGLE]);
//  }
//  last_pitch_imu_angle = pitch_imu_angle;
// /*视觉允许打弹状态UI更新****************************************************/
//  static uint8_t last_is_vision_online = 0;
//  uint8_t is_vision_online = Board_Rx_Info.vision_state;
//  if (last_is_vision_online != is_vision_online)
//  {
//    if (is_vision_online) // 视觉允许打弹
//    {
//      dynamic_ui_info[D_VISION_CIRCLE].ui_config.color = GREEN;
//    }
//    else
//    {
//      dynamic_ui_info[D_VISION_CIRCLE].ui_config.color = WHITE;
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_CIRCLE]);
//  }
//  last_is_vision_online = is_vision_online;
//  /*无自适应UI更新****************************************************/
//  static uint8_t last_is_open_adapt_flag = 0;
////  uint8_t is_open_adapt_flag = communicate.car_data0_rx_info->car_state.bit.is_open_adapt ;
////  if (is_open_adapt_flag != last_is_open_adapt_flag)
////  {
////    if (is_open_adapt_flag==1) 
////    {
////      dynamic_ui_info[D_SPEED_ADAPT_CYCLE].ui_config.color = GREEN;
////    }
////    else
////    {
////      dynamic_ui_info[D_SPEED_ADAPT_CYCLE].ui_config.color = WHITE;
////    }
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_SPEED_ADAPT_CYCLE]);
////  }
////  last_is_open_adapt_flag = is_open_adapt_flag;
//  /*超电UI更新****************************************************/
//  static uint8_t last_is_on_cap = 0;
////  uint8_t is_on_cap = communicate.car_data0_rx_info->car_state.bit.is_on_cap;
////  if (last_is_on_cap != is_on_cap)
////  {
////    if (is_on_cap) 
////    {
////      dynamic_ui_info[D_CAP_ON_CYCLE].ui_config.color = GREEN;
////    }
////    else
////    {
////      dynamic_ui_info[D_CAP_ON_CYCLE].ui_config.color = WHITE;
////    }
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_CAP_ON_CYCLE]);
////  }
////  last_is_on_cap = is_on_cap;
//  /*车辆模式UI更新****************************************************/
//  static uint8_t last_car_mode = 0;
////  uint8_t car_mode = communicate.car_data0_rx_info->car_move_mode;
////  if (car_mode == vision_cycle_CAR) car_mode = cycle_CAR;
////  if (car_mode == vision_gyro_CAR) car_mode = gyro_CAR;

////  if (last_car_mode != car_mode)
////  {
////    memset(dynamic_ui_info[D_CAR_MODE].ui_config.text,0,sizeof(dynamic_ui_info[D_CAR_MODE].ui_config.text));
////    switch (car_mode)
////    {
////    case offline_CAR:
////      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "OFFLINE");
////      break;
////    case init_CAR:
////      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "INIT");
////      break;
////    case cycle_CAR:
////      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "CYLE");
////      break;
////    case gyro_CAR:
////      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "GYRO");
////      break;
////    case mec_CAR:
////      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "MEC");
////      break;
////    case lob_CAR:
////      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "LOB");
////      break;
////    default:
////      break;
////    }
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_CAR_MODE]);
////  }
////  last_car_mode = car_mode;
//  /*摩擦轮速度UI更新****************************************************/
//  /*中间框颜色****************************************************/
//  static uint8_t last_vision_mode_state = 0;
//  uint8_t vision_mode_state = Board_Rx_Info.vision_state;
//  static uint8_t last_is_find_target = 0;
//  uint8_t is_find_target = Board_Rx_Info.is_find_Target;

//  if (last_vision_mode_state != vision_mode_state || last_is_find_target != is_find_target)
//  {
//    if (vision_mode_state && is_find_target) // 视觉模式且找到目标
//    {
//      dynamic_ui_info[D_MID_RECTANGEL].ui_config.color = GREEN;
//    }
//    else if (vision_mode_state && !is_find_target) // 视觉模式但未找到目标
//    {
//      dynamic_ui_info[D_MID_RECTANGEL].ui_config.color = WHITE;
//    }
//    else if (!vision_mode_state && is_find_target) // 非视觉模式但找到目标
//    {
//      dynamic_ui_info[D_MID_RECTANGEL].ui_config.color = FUCHSIA;
//    }
//    else // 非视觉模式且未找到目标
//    {
//      dynamic_ui_info[D_MID_RECTANGEL].ui_config.color = BLACK;
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_MID_RECTANGEL]);
//  }
//  last_vision_mode_state = vision_mode_state;
//  last_is_find_target = is_find_target;
//  /*头朝向指示****************************************************/
//  static float last_chassis_angle = 0;
////  float chassis_angle = (float)(communicate.car_data1_rx_info->chassis_angel) * 2.f * 3.14 / 180.f ; // 底盘角度解算
////  if(last_chassis_angle!=chassis_angle)
////  {
////	rotate_point(&dynamic_ui_info[D_HEAD_CYCLE].ui_config.start_x,
////                    &dynamic_ui_info[D_HEAD_CYCLE].ui_config.start_y,
////                    CLIENT_MID_POSITION_X ,
////                    CLIENT_MID_POSITION_Y +142,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y  ,
////                    chassis_angle);
////  }
////  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HEAD_CYCLE]);
////  last_chassis_angle =chassis_angle;
//  /*击打目标指示****************************************************/
//  static float last_target_angle_err = 0;

////  float target_angle_err = (float)(communicate.game_robot_pos_tx_info->angle_err) * 3.14 / 180.f ; // 底盘角度转浮点数
////  if (last_target_angle_err != target_angle_err)
////  {
////   // 旋转头部三角形
////    rotate_point(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1].ui_config.start_x,
////                    &dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1].ui_config.start_y,
////                    CLIENT_MID_POSITION_X - 10,
////                    CLIENT_MID_POSITION_Y + 125,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y,
////                    target_angle_err);
////    rotate_point(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1].ui_config.end_x,
////                    &dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1].ui_config.end_y,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y + 142,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y,
////                    target_angle_err);
////    rotate_point(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2].ui_config.start_x,
////                    &dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2].ui_config.start_y,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y + 142,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y,
////                    target_angle_err);
////    rotate_point(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2].ui_config.end_x,  
////                    &dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2].ui_config.end_y,
////                    CLIENT_MID_POSITION_X + 10,
////                    CLIENT_MID_POSITION_Y + 125,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y,
////                    target_angle_err);

////                    
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1]);
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2]);
////  }
////  last_target_angle_err = target_angle_err;
//  /*击打命中提示叉叉************************************************/
////  uint16_t enemy_outpost_HP;static uint16_t last_enemy_outpost_HP;
////  uint16_t enemy_base_HP;static uint16_t last_enemy_base_HP;
////  static uint8_t hit_highlight_flag;//显示图标为0，不显示时为0
////  static uint16_t hit_tick;//击打到时刻开始计时
////  static uint16_t last_hit_tick;//用来检测跳变
////  //更新血量
////  if(communicate.game_robot_status_tx_info->game_process.bit.car_color==0)//红方
////  {
////	
////	  enemy_outpost_HP=judge.game_robot_HP.blue_outpost_HP;
////	  enemy_base_HP=judge.game_robot_HP.blue_base_HP;
////  }
////  else
////  {
////	  enemy_outpost_HP=judge.game_robot_HP.red_outpost_HP;
////	  enemy_base_HP=judge.game_robot_HP.red_base_HP;
////  }
//  //检测建筑物掉血符合范围
////  if((last_enemy_outpost_HP-enemy_outpost_HP>=100&&last_enemy_outpost_HP-enemy_outpost_HP<=400)
////	  ||(last_enemy_base_HP-enemy_base_HP>=100&&last_enemy_base_HP-enemy_base_HP<=400))
////  {
////	  hit_highlight_flag=1;
////	  hit_tick=0;//复位时间
////	  //画叉叉
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.start_x=CLIENT_MID_POSITION_X-50;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.start_y=CLIENT_MID_POSITION_Y+50;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.end_x=CLIENT_MID_POSITION_X+50;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.end_y=CLIENT_MID_POSITION_Y-50;
////	  
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.start_x=CLIENT_MID_POSITION_X+50;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.start_y=CLIENT_MID_POSITION_Y+50;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.end_x=CLIENT_MID_POSITION_X-50;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.end_y=CLIENT_MID_POSITION_Y-50;
////	  //发送
////	  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1]);
////	  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2]);
////  }
////  if(hit_highlight_flag==1)
////  {
////	hit_tick++;
////  }
////  if(hit_tick>=600)//显示时间结束，标志位和时间清零
////  {
////	 //清除
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.start_x=0;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.start_y=0;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.end_x=0;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.end_y=0;
////																
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.start_x=0;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.start_y=0;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.end_x=0;
////	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.end_y=0;
////	  //发送
////	  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1]);
////	  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2]);
////	hit_tick=0;
////	hit_highlight_flag=0;//不再计时，实现跳变效果
////  }
////  
////  last_enemy_outpost_HP=enemy_outpost_HP;
////  last_enemy_base_HP=enemy_base_HP;
////  
//  /*弹速显示**************************************************/
//  /*摩擦轮转速差显示*******************************************/
//  /*摩擦轮状态显示********************************************/
////  static uint8_t last_fric_state = 0;
////   static uint8_t last_fric_online = 0;
////  uint8_t fric_state = communicate.car_data0_rx_info->car_state.bit.fri_speed_state;
////  uint8_t fric_online = judge.game_robot_status.power_management_shooter_output;//读裁判系统
////  
////  if (fric_state != last_fric_state||last_fric_online!=fric_online)//如果摩擦轮转速变或者在线状态变
////  {
////    if ( fric_online== 0)//摩擦轮掉线，黑
////    {
////      dynamic_ui_info[D_FRIC_STATE_CYCLE].ui_config.color = BLACK;
////    }
////    else if(fric_online== 1&&fric_state==0)//摩擦轮在线但是没转，青色
////    {
////      dynamic_ui_info[D_FRIC_STATE_CYCLE].ui_config.color = CYAN_BLUE;      
////    }
////	else if(fric_online== 1&&fric_state==1)//摩擦轮电机在线并且在转，绿色
////	{
////	  dynamic_ui_info[D_FRIC_STATE_CYCLE].ui_config.color = GREEN;   
////	}
////	
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_FRIC_STATE_CYCLE]);
////  }
////  last_fric_state = fric_state;
////	last_fric_online=fric_online;
//  /*RFID显示********************************************/
////  static uint8_t last_rfid_state = 0;
////  uint8_t rfid_state = (judge.rfid_status.rfid_status != 0);
////  if (last_rfid_state != rfid_state)
////  {
////    if (rfid_state == 1)
////    {
////      dynamic_ui_info[D_RFID_CYCLE].ui_config.color = GREEN;
////    }
////    else
////    {
////      dynamic_ui_info[D_RFID_CYCLE].ui_config.color = WHITE;
////    }
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_RFID_CYCLE]);
////  }
////  last_rfid_state = rfid_state;

//  /*视觉装甲板显示********************************************/
//  #ifndef UI_SIMPLIFY
////  static uint16_t last_vision_armor_x,last_vision_armor_y;
////  uint16_t vision_armor_x = communicate.car_data2_rx_info->ui_x;
////  uint16_t vision_armor_y = 1080 - communicate.car_data2_rx_info->ui_y;
////  if (communicate.car_data0_rx_info->car_state.bit.is_find_target == 1)
////  {
////    if (last_vision_armor_x != vision_armor_x || last_vision_armor_y != vision_armor_y)
////    {
////      //位置更新
////      dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_x = vision_armor_x;
////      dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_y = vision_armor_y;
////      dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.start_x = vision_armor_x;
////      dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.start_y = vision_armor_y;
////      dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_x = vision_armor_x;
////      dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_y = vision_armor_y;
////      //角度更新
////      float max_hp,current_hp;
////      Get_Hp(&current_hp,&max_hp,communicate.car_data2_rx_info->detect_num);
////      dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.end_angel = current_hp/max_hp*360.f;
////      if (current_hp == 0)
////      {
////        dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.end_angel = 1;
////      }
////      //白色过渡动画
////      dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_angel = dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.end_angel;
////      float target_white_end_angel = dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_angel + 1;
////      if (dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.end_angel > target_white_end_angel)//会逐渐靠近目标角度（也就是血条结束角度，也是自身的开始角度），但是永远不会到达，因为每次都是差值的20%
////      {
////        dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.end_angel -= 0.2*(dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.end_angel - target_white_end_angel);
////      }
////      else//初始化过渡动画结束角度
////      {
////        dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.end_angel = target_white_end_angel;
////      }
////      
////      
////      //颜色更新
////      if (current_hp > (0.5*max_hp))
////      {
////        dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.color = GREEN;
////      }
////      else if (current_hp >= 200)
////      {
////        dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.color = YELLOW;
////      }
////      else
////      {
////        dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.color = FUCHSIA;
////      }
////        
////      Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_HP_CYCLE]);
////      Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_ARMOR_CYCLE]);
////      Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_WHITE_CYCLE]);
////    }
////  }
////  else if (dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_x != 0)
////  {
////    dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_x = 0;
////    dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_y = 0;
////    dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.start_x = 0;
////    dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.start_y = 0;
////    dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_x = 0;
////    dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_y = 0;
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_HP_CYCLE]);
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_ARMOR_CYCLE]);
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_WHITE_CYCLE]);
////  }
////  last_vision_armor_x = vision_armor_x;
////  last_vision_armor_y = vision_armor_y;
//  #endif
//  /*基地距离********************************************/
////  static float last_base_distance;
////  float target_distance = communicate.game_robot_pos_tx_info->target_distance / 100.F;
////  if (last_base_distance != target_distance)
////  {
////    dynamic_ui_info[D_HIT_TARGET_DISTANCE].ui_config.float_num = target_distance;
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_TARGET_DISTANCE]);
////  }
////  last_base_distance = target_distance;
//  /*PITCH指针****************************************************/
////	#ifndef UI_SIMPLIFY
////  float pitch_angel=(communicate.car_data1_rx_info->pitch_motor_angle)/180.f*3.14;
////  static float  last_pitch_angel;
////  // 旋转线条
////  if(last_pitch_angel!=pitch_angel)
////  {
////	rotate_point(&dynamic_ui_info[D_PITCH_POINTER].ui_config.start_x,
////                    &dynamic_ui_info[D_PITCH_POINTER].ui_config.start_y,
////                    1260,
////                    540,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y,
////                    pitch_angel);
////    rotate_point(&dynamic_ui_info[D_PITCH_POINTER].ui_config.end_x,
////                    &dynamic_ui_info[D_PITCH_POINTER].ui_config.end_y,
////                    1340,
////                    540,
////                    CLIENT_MID_POSITION_X,
////                    CLIENT_MID_POSITION_Y,
////                    pitch_angel);
////  }
////    last_pitch_angel=pitch_angel;
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_PITCH_POINTER]);
//////  #endif
////  #ifndef UI_SIMPLIFY
////  /*ROI*/
////  static uint8_t last_roi;
////  uint8_t current_roi = communicate.car_data3_rx_info->uix_lb;
////  if (last_roi != current_roi)
////  {
////    dynamic_ui_info[D_ROI_UP].ui_config.start_x = communicate.car_data3_rx_info->uix_lt * 10;
////    dynamic_ui_info[D_ROI_UP].ui_config.start_y = 1080 - communicate.car_data3_rx_info->uiy_lt * 5;
////    dynamic_ui_info[D_ROI_UP].ui_config.end_x = communicate.car_data3_rx_info->uix_rt * 10;
////    dynamic_ui_info[D_ROI_UP].ui_config.end_y = 1080 - communicate.car_data4_rx_info->uiy_rt * 5;
////    dynamic_ui_info[D_ROI_DOWN].ui_config.start_x = communicate.car_data3_rx_info->uix_lb * 10;
////    dynamic_ui_info[D_ROI_DOWN].ui_config.start_y = 1080 - communicate.car_data3_rx_info->uiy_lb * 5;
////    dynamic_ui_info[D_ROI_DOWN].ui_config.end_x = communicate.car_data3_rx_info->uix_rb * 10;
////    dynamic_ui_info[D_ROI_DOWN].ui_config.end_y = 1080 - communicate.car_data3_rx_info->uiy_rb * 5;
////    dynamic_ui_info[D_ROI_LEFT].ui_config.start_x = communicate.car_data3_rx_info->uix_lt * 10;
////    dynamic_ui_info[D_ROI_LEFT].ui_config.start_y = 1080 - communicate.car_data3_rx_info->uiy_lt * 5;
////    dynamic_ui_info[D_ROI_LEFT].ui_config.end_x = communicate.car_data3_rx_info->uix_lb * 10;
////    dynamic_ui_info[D_ROI_LEFT].ui_config.end_y = 1080 - communicate.car_data3_rx_info->uiy_lb * 5;
////    dynamic_ui_info[D_ROI_RIGHT].ui_config.start_x = communicate.car_data3_rx_info->uix_rt * 10;
////    dynamic_ui_info[D_ROI_RIGHT].ui_config.start_y = 1080 - communicate.car_data4_rx_info->uiy_rt * 5;
////    dynamic_ui_info[D_ROI_RIGHT].ui_config.end_x = communicate.car_data3_rx_info->uix_rb * 10;
////    dynamic_ui_info[D_ROI_RIGHT].ui_config.end_y = 1080 - communicate.car_data3_rx_info->uiy_rb * 5;
////    dynamic_ui_info[D_ROI_MID].ui_config.start_x = communicate.car_data4_rx_info->uix_left * 10;
////    dynamic_ui_info[D_ROI_MID].ui_config.start_y = 1080 - communicate.car_data4_rx_info->uiy_left * 5;
////    dynamic_ui_info[D_ROI_MID].ui_config.end_x = communicate.car_data2_rx_info->uix_right * 10;
////    dynamic_ui_info[D_ROI_MID].ui_config.end_y = 1080 - communicate.car_data2_rx_info->uiy_right * 5;
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_UP]);
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_DOWN]);
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_LEFT]);
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_RIGHT]);
////    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_MID]);
////  }
////  last_roi = current_roi;
////  #endif
// 
//}

















///*图形处理**********************************************************************************/

//#include <rp_math.h>

///**
// * @brief 把某点绕某点旋转一定角度
// * 
// * @param x 存储旋转后x的地址
// * @param y 存储旋转后y的地址
// * @param raw_x 旋转前x的值
// * @param raw_y 旋转后y的值
// * @param mid_x 旋转原点x
// * @param mid_y 旋转原点y
// * @param angle 旋转的角度rad
// */
//void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle) 
//{
//  float s = sin(angle);
//  float c = cos(angle);
//  // 平移到原点
//  float origin_x = raw_x - mid_x;
//  float origin_y = raw_y - mid_y;
//  // 旋转
//  float new_x = origin_x * c - origin_y * s;
//  float new_y = origin_x * s + origin_y * c;
//  // 平移回去并更新原始坐标
//  *x = new_x + mid_x;
//  *y = new_y + mid_y;
//}

///**
// * @brief 按比例放大某点
// * 
// * @param x 存储放大后x的地址
// * @param y 存储放大后y的地址
// * @param raw_x 原始x的值
// * @param raw_y 原始y的值
// * @param mid_x 放大中心点x
// * @param mid_y 放大中心点y
// * @param scale 放大的比例
// */
//void scale_point(uint16_t *x, uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float scale) 
//{
//  // 平移到原点
//  float origin_x = raw_x - mid_x;
//  float origin_y = raw_y - mid_y;
//  // 按比例放大
//  float new_x = origin_x * scale;
//  float new_y = origin_y * scale;
//  // 平移回去并更新原始坐标
//  *x = new_x + mid_x;
//  *y = new_y + mid_y;
//}

///**
// * @brief 旋转圆弧
// * 
// * @param start_angle 起始角度 
// * @param end_angel 终止角度
// * @param rotation_angle 
// */
//void rotate_arc(uint16_t *start_angle, uint16_t *end_angel, float rotation_angle) 
//{
//  *start_angle = fmod(*start_angle + rotation_angle, 360);
//  *end_angel = fmod(*end_angel + rotation_angle, 360);
//}

///*其他******************************************/
//void Get_Hp(float *current_hp,float *max_hp,robot_type_e robot_type)
//{
//  //最大血量
//  switch (robot_type)
//  {
//  case HERO:
//    *max_hp = 500;
//    break;
//  case INFANRTY_3:
//  case INFANRTY_4:
//  case INFANRTY_5:
//    *max_hp = 400;
//    break;
//  case OUTPOST:
//    *max_hp = 1500;
//    break;
//  case SENTRY:
//    *max_hp = 400;
//    break;
//  case BASE:
//    *max_hp = 5000;
//    break;
//  default:
//    break;
//  }
//  //判断红蓝
//  if(My_Judge.org_info->game_robot_status.robot_id <= 10)//红方
//  {
//    switch (robot_type)
//    {
////    case HERO:
////      *current_hp = judge.game_robot_HP.blue_1_robot_HP;
////      break;
////    case ENGINEER:
////      *current_hp = judge.game_robot_HP.blue_2_robot_HP;
////      break;
////    case INFANRTY_3:
////      *current_hp = judge.game_robot_HP.blue_3_robot_HP;
////      break;
////    case INFANRTY_4:
////      *current_hp = judge.game_robot_HP.blue_4_robot_HP;
////      break;
////    case INFANRTY_5:
////      *current_hp = judge.game_robot_HP.blue_5_robot_HP;
////      break;
////    case OUTPOST:
////      *current_hp = judge.game_robot_HP.blue_outpost_HP;
////      break;
////    case SENTRY:
////      *current_hp = judge.game_robot_HP.blue_7_robot_HP;
////      break;
////    case BASE:
////      *current_hp = judge.game_robot_HP.blue_base_HP;
//      break;
//    default:
//      break;
//    }
//  }
//  else//蓝方
//  {
//    switch (robot_type)
//    {
////    case HERO:
////      *current_hp = judge.game_robot_HP.red_1_robot_HP;
////      break;
////    case ENGINEER:
////      *current_hp = judge.game_robot_HP.red_2_robot_HP;
////      break;
////    case INFANRTY_3:
////      *current_hp = judge.game_robot_HP.red_3_robot_HP;
////      break;
////    case INFANRTY_4:
////      *current_hp = judge.game_robot_HP.red_4_robot_HP;
////      break;
////    case INFANRTY_5:
////      *current_hp = judge.game_robot_HP.red_5_robot_HP;
////      break;
////    case OUTPOST:
////      *current_hp = judge.game_robot_HP.red_outpost_HP;
////      break;
////    case SENTRY:
////      *current_hp = judge.game_robot_HP.red_7_robot_HP;
////      break;
////    case BASE:
////      *current_hp = judge.game_robot_HP.red_base_HP;
//      break;
//    default:
//      break;
//    }
//    
//  }
//}

#include "ui.h"
#include "ui_priority.h"
#include "arm_math.h"
#include "ui_protocol.h"
#include "Balance.h"
#include "Chassis.h"
#include "judge.h"
//#include "math_support.h"
#include "car_info.h"
#include "gimbal.h"
#include "communicate.h"
#include "cap.h"
#include "gimbal_motor.h"
#include "cap.h"
//#include "speed_estimate.h"


//UI_Dynamic_Info_t My_UI_Dynamic_Info;
#define LEFT_UP_X (Client_mid_position_x - 820)
#define LEFT_UP_Y (Client_mid_position_y + 320)
#define RIGHT_UP_X (Client_mid_position_x + 820)
#define RIGHT_UP_Y (Client_mid_position_y + 320)

#define CHAS_CIRCLE_X     (Client_mid_position_x)
#define CHAS_CIRCLE_Y     (Client_mid_position_y + 250 - 5)
#define CHAS_CIRCLE_R     (65)//65

#define PITCH_CENTER_X    260
#define PITCH_CENTER_Y    690

#define LEG_LENGTH_X      (Client_mid_position_x + 220)   //腿长线起始，以最下面的线为准,高度120，0.24:120
#define LEG_LENGTH_Y      (Client_mid_position_y - 475)
// 右大腿
#define R_THIGH_START_X   (Client_mid_position_x +730)
#define R_THIGH_START_Y   (Client_mid_position_y - 105)
#define R_THIGH_END_X0    (Client_mid_position_x + 620)
#define R_THIGH_END_Y0    (Client_mid_position_y - 105)

// 右小腿（初始状态时，小腿沿大腿方向延伸，起点即为大腿终点）
#define R_SHANK_START_X0  R_THIGH_END_X0
#define R_SHANK_START_Y0  R_THIGH_END_Y0
#define R_SHANK_END_X0    (R_SHANK_START_X0 + 125)   // 长度125，水平方向
#define R_SHANK_END_Y0    R_SHANK_START_Y0


// 左大腿
#define L_THIGH_START_X   (Client_mid_position_x +630)
#define L_THIGH_START_Y   (Client_mid_position_y - 105)
#define L_THIGH_END_X0    (Client_mid_position_x + 520)
#define L_THIGH_END_Y0    (Client_mid_position_y - 105)

// 左小腿（初始状态时，小腿沿大腿方向延伸，起点即为大腿终点）
#define L_SHANK_START_X0  L_THIGH_END_X0
#define L_SHANK_START_Y0  L_THIGH_END_Y0
#define L_SHANK_END_X0    (L_SHANK_START_X0 + 125)   // 长度110，水平方向
#define L_SHANK_END_Y0    L_SHANK_START_Y0

UI_Dynamic_Info_t My_UI_Dynamic_Info;

void UI_Info_Update_Leg_length(void);
void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle);
void My_Chas_Pitch_Update(float angle);
void My_Chas_Theta_Update(float angle);
void My_Chas_Circle_Update(float angle);
void My_Front_Leg_Update(float angle1,float angle2);
void My_Back_Leg_Update(float angle1,float angle2);
ui_info_t dynamic_ui_info [DYNAMIC_NUM] = 
{
	
	[TOP_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d1",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标,110
    .ui_config.start_y = Client_mid_position_y + 130 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 740 ,
		.ui_config.end_y = Client_mid_position_y + 70 ,
                  
	},
	
	[FLY_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d2",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 220 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 740 ,
		.ui_config.end_y = Client_mid_position_y + 160 ,
	},
	
	[UPSTEP_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d3",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
     .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 310 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 870 ,
		.ui_config.end_y = Client_mid_position_y + 250 ,
		
	},
	
//	[UPSTEP_NUM] = {
//		/*******不变配置*********/
//		.ui_config.priority = HIGH_PRIORITY,
//    .ui_config.ui_type = INT,           // UI内容类型
//    .ui_config.name = "d4",              // 图形名称
//    /*******可变配置*********/
//		.ui_config.operate_type = MODIFY,    // 操作类型
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = CYAN_BLUE,            // 颜色
//    .ui_config.size = 30,                // 字体大小
//    .ui_config.width = 2,                // 线条宽度
//    .ui_config.start_x = Client_mid_position_x + 830,              // 起点 x 坐标
//    .ui_config.start_y = Client_mid_position_y + 297,              // 起点 y 坐标
//    .ui_config.int_num = 0,
//	},
//	[POWER] = {
//		/*******不变配置*********/
//		.ui_config.priority = MID_PRIORITY,
//    .ui_config.ui_type = CHAR,           // UI内容类型
//    .ui_config.name = "d3",              // 图形名称
//    /*******可变配置*********/
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = WHITE,            // 颜色
//    .ui_config.size = 30,                // 字体大小
//    .ui_config.width = 2,                // 线条宽度
//    .ui_config.start_x = RIGHT_UP_X - 165,              // 起点 x 坐标
//    .ui_config.start_y = RIGHT_UP_Y - 30,              // 起点 y 坐标
//    .ui_config.text = "POWER",            // 显示的文字
//		
//	},
//	
//	[POWER_FRAME] = {
//		/*******不变配置*********/
//    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//    .ui_config.ui_type = RECTANGEL,         // UI内容类型
//    .ui_config.name = "d4",              // 图形名称
//    /*******可变配置*********/
//    .ui_config.operate_type = MODIFY,    // 操作类型
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = WHITE,            // 颜色
//    .ui_config.width = 5,                // 线条宽度
//    .ui_config.start_x = RIGHT_UP_X - 190,              // 起点 x 坐标
//    .ui_config.start_y = RIGHT_UP_Y - 15 ,              // 起点 y 坐标
//		.ui_config.end_x = RIGHT_UP_X ,
//		.ui_config.end_y = RIGHT_UP_Y - 90 ,
//		
//	},
	
	[BUFF_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d5",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
     .ui_config.start_x = 170 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 310 ,              // 起点 y 坐标
		.ui_config.end_x = 350 ,
		.ui_config.end_y = Client_mid_position_y + 250 ,
		
	},
	
	[BUFF_NUM] = {
		/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = INT,           // UI内容类型
    .ui_config.name = "d6",              // 图形名称
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = 310,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 297,              // 起点 y 坐标
    .ui_config.int_num = 0,
		
	},
	[FRIC_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d23",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 40 ,              // 起点 y 坐标
	  .ui_config.end_x = Client_mid_position_x + 740 ,
	  .ui_config.end_y = Client_mid_position_y - 20 ,
	},
	[JUMP_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d24",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 750 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y +130 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 860 ,
		.ui_config.end_y = Client_mid_position_y +70 ,
		
	},
	[VISION_TAR] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d30",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 750 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y +40 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 860 ,
		.ui_config.end_y = Client_mid_position_y - 20 ,
		
	},
	[SAVE_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d25",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = 170 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 230 ,              // 起点 y 坐标
		.ui_config.end_x = 300 ,
		.ui_config.end_y = Client_mid_position_y +180 ,
		
	},
	
	[BULLET_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = INT,            // UI内容类型
    .ui_config.name = "d7",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.size = 60,                // 字体大小
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 310,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 90,              // 起点 y 坐标
    .ui_config.int_num = 0,              // 显示的数字
	},
	
	[CHAS_HEAD_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d8",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = CHAS_CIRCLE_X,              // 起点 x 坐标
    .ui_config.start_y = CHAS_CIRCLE_Y,              // 起点 y 坐标
    .ui_config.end_x = CHAS_CIRCLE_X,                // 终点 x 坐标
    .ui_config.end_y = CHAS_CIRCLE_Y + CHAS_CIRCLE_R,                // 终点 y 坐标
  },
	
	[CHAS_SIDE_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d9",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R,              // 起点 x 坐标
    .ui_config.start_y = CHAS_CIRCLE_Y ,              // 起点 y 坐标
    .ui_config.end_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R,                // 终点 x 坐标
    .ui_config.end_y = CHAS_CIRCLE_Y,                // 终点 y 坐标		
	},
		
	[PITCH_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d10",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 7,                // 线条宽度
    .ui_config.start_x = PITCH_CENTER_X - 55,              // 起点 x 坐标
    .ui_config.start_y = PITCH_CENTER_Y,              // 起点 y 坐标
    .ui_config.end_x = PITCH_CENTER_X + 55,                // 终点 x 坐标
    .ui_config.end_y = PITCH_CENTER_Y,                // 终点 y 坐标	
	},
	
	[THETA_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d11",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = PINK,            // 颜色
    .ui_config.width = 7,                // 线条宽度
    .ui_config.start_x = PITCH_CENTER_X,              // 起点 x 坐标
    .ui_config.start_y = PITCH_CENTER_Y,              // 起点 y 坐标
    .ui_config.end_x = PITCH_CENTER_X,                // 终点 x 坐标
    .ui_config.end_y = PITCH_CENTER_Y - 80,  
	},
	
	[R_LEG_LENGTH] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d12",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 30,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X + 60,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 60,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y + 120,                // 终点 y 坐标
	},
	
	[L_LEG_LENGTH] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d13",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = PINK,            // 颜色
    .ui_config.width = 30,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X + 20,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 20,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y + 120,                // 终点 y 坐标
	},
	
	[CAP_LINE] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d14",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = GREEN,            // 颜色
    .ui_config.width = 25,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 250,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 332,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x + 250,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y + 332,                // 终点 y 坐标
	},
	
	[VISION_AIM] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CIRCLE,         // UI内容类型
    .ui_config.name = "d15",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 1,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y ,              // 起点 y 坐标
		.ui_config.radius = 3,
	},
	
	[AUTO_CATCH_FRAME] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d16",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
     .ui_config.start_x = Client_mid_position_x - 280,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 170 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 280 ,
		.ui_config.end_y = Client_mid_position_y - 180 ,
	},
	
	[CAR_SPEED] = {
		/*不变配置*/
  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = FLOAT, // UI内容类型
	.ui_config.name = "d17",
  /*可变配置*/
  .ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 1, // 图层数，0~9
  .ui_config.color = CYAN_BLUE, // 颜色
  .ui_config.size = 30, // 字体大小
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = Client_mid_position_x - 60, // 起点 x 坐标
  .ui_config.start_y = Client_mid_position_y - 400, // 起点 y 坐标
  .ui_config.float_num = 0, // 显示的数字
  .ui_config.decimal = 2, // 小数位有效个数
	},
	
	[LENGTH_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d18",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = YELLOW,            // 颜色
    .ui_config.width = 3,                // 线条宽度
     .ui_config.start_x = Client_mid_position_x - 350,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 125 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x - 310 ,
		.ui_config.end_y = Client_mid_position_y - 165 ,
	},
	[FRONT_LEG_R] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d19",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 5,                // 线条宽度
     .ui_config.start_x =R_THIGH_START_X,              // 起点 x 坐标
    .ui_config.start_y = R_THIGH_START_Y ,              // 起点 y 坐标
		.ui_config.end_x = R_THIGH_END_X0 ,
		.ui_config.end_y = R_THIGH_END_Y0 ,
	},
	
	[BACK_LEG_R] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d20",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 5,                // 线条宽度
     .ui_config.start_x = 0,              // 起点 x 坐标
    .ui_config.start_y = 0,              // 起点 y 坐标
		.ui_config.end_x = 0 ,
		.ui_config.end_y = 0,
	},
	[FRONT_LEG_L] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d21",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 5,                // 线条宽度
     .ui_config.start_x = L_THIGH_START_X,              // 起点 x 坐标
    .ui_config.start_y = L_THIGH_START_Y ,              // 起点 y 坐标
		.ui_config.end_x = L_THIGH_END_X0 ,
		.ui_config.end_y = L_THIGH_END_Y0 ,
	},
	
	[BACK_LEG_L] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d22",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 5,                // 线条宽度
     .ui_config.start_x = 0,              // 起点 x 坐标
    .ui_config.start_y = 0 ,              // 起点 y 坐标
		.ui_config.end_x = 0 ,
		.ui_config.end_y = 0 ,
	},
	[PITCH_OFFSET_NUM] = {
		/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = FLOAT,           // UI内容类型
    .ui_config.name = "d26",              // 图形名称
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.size = 15,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 450,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 80,              // 起点 y 坐标
    .ui_config.int_num = 0,
		
	},
};

ui_info_t const_ui_info [CONST_NUM] = 
{
	[TOP_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g1",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度,没用
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 117,              // 起点 y 坐标
    .ui_config.text = "TOP",            // 显示的文字
	},
	
	[UPSTEP_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g2",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 297,              // 起点 y 坐标
    .ui_config.text = "UPSTEP",            // 显示的文字
		
	},
	
	[FLY_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g3",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 207,              // 起点 y 坐标
    .ui_config.text = "FLY",            // 显示的文字
	},
	
//	[AUTO_CATCH_FRAME] = {
//		/*******不变配置*********/
//    .ui_config.ui_type = RECTANGEL,         // UI内容类型
//    .ui_config.name = "g4",              // 图形名称
//    /*******可变配置*********/
// 
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = WHITE,            // 颜色
//    .ui_config.width = 1,                // 线条宽度
//    .ui_config.start_x = Client_mid_position_x - 260,              // 起点 x 坐标
//    .ui_config.start_y = Client_mid_position_y + 180 ,              // 起点 y 坐标
//		.ui_config.end_x = Client_mid_position_x + 260 ,
//		.ui_config.end_y = Client_mid_position_y - 180 ,
//	},
	
//	[VISION_CHAR] = {
//		/*******不变配置*********/
//    .ui_config.ui_type = CHAR,           // UI内容类型
//    .ui_config.name = "g4",              // 图形名称
//    /*******可变配置*********/
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = WHITE,            // 颜色
//    .ui_config.size = 20,                // 字体大小
//    .ui_config.width = 2,                // 线条宽度
//    .ui_config.start_x = RIGHT_UP_X - 40,              // 起点 x 坐标
//    .ui_config.start_y = RIGHT_UP_Y - 60,              // 起点 y 坐标
//    .ui_config.text = "BUFF",            // 显示的文字
//	},
	 [BUFF_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g5",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = 180,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 297,              // 起点 y 坐标
    .ui_config.text = "BUFF",            // 显示的文字
	 },
	 
	 [CHAS_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CIRCLE,         // UI内容类型
    .ui_config.name = "g6",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = GREEN,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = CHAS_CIRCLE_X ,              // 圆心 x 坐标
    .ui_config.start_y = CHAS_CIRCLE_Y,              // 圆心 y 坐标
    .ui_config.radius = 65, 
	 },
	 
	 
	 [STANDARD_LINE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g8",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y + 40,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 100,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y + 40,                // 终点 y 坐标
	 },
	 
	 [HIGH_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g9",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = YELLOW,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y + 120,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 100,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y + 120,                // 终点 y 坐标
	 },
	 
	 [MINIMUM_LINE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g10",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 100,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y ,                // 终点 y 坐标
	 },
	 
	 [CAP_FRAME] = {
		 /*******不变配置*********/
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "g11",              // 图形名称
    /*******可变配置*********/
 
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 253,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 345 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 253 ,
		.ui_config.end_y = Client_mid_position_y + 317 ,
	 },
	 
	 [MOVE_L_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g12",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 1,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 600,//334,              // 起点 x 坐标
    .ui_config.start_y = 0,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x - 160,//82,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y - 150 ,                // 终点 y 坐标
		 
	 },

	 [MOVE_R_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g13",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 1,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 600,//334,              // 起点 x 坐标
    .ui_config.start_y = 0,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x + 160,//82,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y - 150 ,                // 终点 y 坐标
		 
	 },
	 
	 [LOW_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g14",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 340,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 130,              // 起点 y 坐标
    .ui_config.text = "L",            // 显示的文字
	 },
	 
	 [MID_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g15",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size =  30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 340,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 70,              // 起点 y 坐标
    .ui_config.text = "M",            // 显示的文字
	 },
	 
	 [HIGH_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g16",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 340,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 10,              // 起点 y 坐标
    .ui_config.text = "H",            // 显示的文字
	 },
	 
	 [CAP_DIVISION_1] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g17",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 250 + 70,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 355,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x - 250 + 70,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y + 305 ,                // 终点 y 坐标
	 },
	 
	 [CAP_DIVISION_2] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g18",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 250 + 195,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 355,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x - 250 + 195,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y + 305 ,                // 终点 y 坐标
	 },
	[PITCH_OFFSET] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g19",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 15,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 50,              // 起点 y 坐标
    .ui_config.text = "PIT_OFFSET:",            // 显示的文字
	},
	[FRIC_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g20",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 27,              // 起点 y 坐标
    .ui_config.text = "FRC",            // 显示的文字
	},
	[PITCH_OFFSET] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g21",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 15,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 300,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 80,              // 起点 y 坐标
    .ui_config.text = "PIT_OFFSET",            // 显示的文字
	},
	
	[JUMP_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g22",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度,没用
    .ui_config.start_x = Client_mid_position_x + 760,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 117,              // 起点 y 坐标
    .ui_config.text = "JMP",            // 显示的文字
	},
	[VISION_MODE] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g23",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 760,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 27,              // 起点 y 坐标
    .ui_config.text = "OTP",            // 显示的文字
	},

};

void My_Ui_Init(void)
{
  Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
}

	float thigh_angle_l = 0.f;
	float shank_angle_l = 0.f;
	float thigh_angle_r = 0.f;
	float shank_angle_r = 0.f;

void Ui_Info_Update(void)
{
	client_info_update();
	
	//陀螺框更新
	static uint8_t top_last_mode = false;
	
	if(top_last_mode != Balance.Flag->Cycle_Flag)
	{
	  if(Balance.Flag->Cycle_Flag == true)
	  {
		  dynamic_ui_info[TOP_FRAME].ui_config.color = GREEN;
  	}
	  else
	  {
		  dynamic_ui_info[TOP_FRAME].ui_config.color = WHITE;
	  }
	  Enqueue_Ui_For_Sending(&dynamic_ui_info[TOP_FRAME]);
  }
	
	top_last_mode = Balance.Flag->Cycle_Flag;
	
	//上台阶框更新
	static uint8_t upstep_last_mode = false;
	
	if(upstep_last_mode != (Balance.Flag->Knee_Strike_1_Flag || Balance.Flag->Knee_Strike_2_Flag))
	{
		if(Balance.Flag->Knee_Strike_1_Flag == true || Balance.Flag->Knee_Strike_2_Flag == true)
		{
			dynamic_ui_info[UPSTEP_FRAME].ui_config.color = GREEN;
		}
		else
		{
			dynamic_ui_info[UPSTEP_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[UPSTEP_FRAME]);
	}
	
	upstep_last_mode = (Balance.Flag->Knee_Strike_1_Flag || Balance.Flag->Knee_Strike_2_Flag);

	//腿角更新
	float front_angle_last_l = 0.f;
	float back_angle_last_l = 0.f;
	float front_angle_last_r = 0.f;
	float back_angle_last_r = 0.f;

	
	//余弦定理，还是太复杂了，后续看看能不能解算好phi4和phi3 2026.3.24
	thigh_angle_r =PI- Chassis.Leg_Unit[R_Leg]->Straight->info->thetal_err  -  acos((l4*l4 + (Chassis.Leg_Unit[R_Leg]->Link->info->length->l0) * (Chassis.Leg_Unit[R_Leg]->Link->info->length->l0) - l3*l3) / 2 * l4 *(Chassis.Leg_Unit[R_Leg]->Link->info->length->l0))- PI/4;
	thigh_angle_l = PI - Chassis.Leg_Unit[L_Leg]->Straight->info->thetal_err -  acos((l4*l4 + (Chassis.Leg_Unit[L_Leg]->Link->info->length->l0) * (Chassis.Leg_Unit[L_Leg]->Link->info->length->l0) - l3*l3) / 2 * l4 *(Chassis.Leg_Unit[L_Leg]->Link->info->length->l0))- PI/4;
	shank_angle_r = 2*PI-acos((l4*l4 + l3*l3 - (Chassis.Leg_Unit[R_Leg]->Link->info->length->l0) * (Chassis.Leg_Unit[R_Leg]->Link->info->length->l0)) / (2 * l4 * l3));
	shank_angle_l = 2*PI-acos((l4*l4 + l3*l3 - (Chassis.Leg_Unit[L_Leg]->Link->info->length->l0) * (Chassis.Leg_Unit[L_Leg]->Link->info->length->l0)) / (2 * l4 * l3))-PI/8;
	if(thigh_angle_r < -2*PI)
	{
		thigh_angle_r += 2*PI;
	}
	if(thigh_angle_l < -2*PI)
	{
		thigh_angle_l += 2*PI;
	}
	//thigh_angle_r = My_State_Var.theta_r;
	
//	front_angle_now_r = lowpass(front_angle_last_r,front_angle_now_r,0.5);
//	front_angle_now_l = lowpass(front_angle_last_l,front_angle_now_l,0.5);
//	back_angle_now_r = lowpass(back_angle_last_r,back_angle_now_r,0.5);
//	back_angle_now_l = lowpass(back_angle_last_l,back_angle_now_l,0.5);
	
	    My_Front_Leg_Update(thigh_angle_r, thigh_angle_l);
        My_Back_Leg_Update(shank_angle_r, shank_angle_l);

    // 发送更新后的 UI 数据
    Enqueue_Ui_For_Sending(&dynamic_ui_info[FRONT_LEG_R]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[FRONT_LEG_L]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[BACK_LEG_R]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[BACK_LEG_L]);
	
	//	
//	if(My_Judge.org_info->power_heat_data.chassis_power_buffer <= 25.f)
//	{
//		dynamic_ui_info[POWER_FRAME].ui_config.color = ORANGE;
//		dynamic_ui_info[POWER].ui_config.color = WHITE;
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[POWER_FRAME]);
//	}
//	else if(My_Judge.org_info->power_heat_data.chassis_power_buffer <= 8.f)
//	{
//		dynamic_ui_info[POWER_FRAME].ui_config.color = FUCHSIA;
//		dynamic_ui_info[POWER].ui_config.color = FUCHSIA;
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[POWER_FRAME]);
//	}
//	else
//	{
//		dynamic_ui_info[POWER_FRAME].ui_config.color = WHITE;
//		dynamic_ui_info[POWER].ui_config.color = WHITE;
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[POWER_FRAME]);
//	}

  //上台阶数字更新，2025.7.29不更新了
//	static uint8_t upstep_num = 0,upstep_last_num = 0;
//	if(My_Chassis.Knee_Strike_Flag_1 == true)
//	{
//		upstep_num = 1;
//	}
//	else if(My_Chassis.Knee_Strike_Flag_2 == true)
//	{
//		upstep_num = 2;
//	}
//	else 
//	{
//		upstep_num = 0;
//	}
//	
//	if(upstep_last_num != upstep_num)
//	{
//		if(upstep_num == 1)
//		{
//			dynamic_ui_info[UPSTEP_NUM].ui_config.int_num = 1;
//		}
//		else if(upstep_num == 2)
//		{
//			dynamic_ui_info[UPSTEP_NUM].ui_config.int_num = 2;
//		}
//		else
//		{
//			dynamic_ui_info[UPSTEP_NUM].ui_config.int_num = 0;
//		}
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[UPSTEP_NUM]);
//	}
//	
//	upstep_last_num = upstep_num;
	
	//剩余发弹量更新
	static int16_t bullet_remain = 0;
	
//	if(bullet_remain != My_Judge.org_info->ext_bullet_remaining.bullet_remaining_num_17mm)
//	{
//		dynamic_ui_info[BULLET_NUM].ui_config.int_num = (int16_t)My_Judge.org_info->ext_bullet_remaining.bullet_remaining_num_17mm;
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[BULLET_NUM]);
//	}
//	
//	bullet_remain =  My_Judge.org_info->ext_bullet_remaining.bullet_remaining_num_17mm;
//	
	//飞坡框更新
	
	static uint8_t fly_last_mode = false;
	
	if(fly_last_mode != Balance.Flag->Middle_Flag)
	{
		if(Balance.Flag->Middle_Flag == true)
		{
			dynamic_ui_info[FLY_FRAME].ui_config.color = GREEN;
		}
		else if(Balance.Flag->Middle_Flag == false)
		{
			dynamic_ui_info[FLY_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[FLY_FRAME]);
	}
	fly_last_mode = Balance.Flag->Middle_Flag;
	
	//跳跃框更新
	static uint8_t jump_last_mode = false;
	
	if(jump_last_mode != Balance.Flag->Jumping_Flag)
	{
		if(Balance.Flag->Jumping_Flag == true)
		{
			dynamic_ui_info[JUMP_FRAME].ui_config.color = GREEN;
		}
		else if(Balance.Flag->Jumping_Flag == false)
		{
			dynamic_ui_info[JUMP_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[JUMP_FRAME]);
	}
	jump_last_mode = Balance.Flag->Jumping_Flag;
	//摩擦轮更新
	static uint8_t fric_last_flag = false;
	static uint8_t fric_is_online = 1;
	if(fric_last_flag != Board_Tx_Info.is_fric_on)
	{
			if(Board_Tx_Info.is_fric_on == 1)
			{
				dynamic_ui_info[FRIC_FRAME].ui_config.color = GREEN;
			}
			else if (Board_Tx_Info.is_fric_on == 0)
			{
				dynamic_ui_info[FRIC_FRAME].ui_config.color = WHITE;
			}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[FRIC_FRAME]);
	}
	fric_last_flag = Board_Tx_Info.is_fric_on;
	//视觉数字更新
	static uint8_t vision_last_mode = 0;
	
	if(vision_last_mode != Board_Tx_Info.vision_mode)
	{
		if(Board_Tx_Info.vision_mode == 1)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 1;
		}
		else if(Board_Tx_Info.vision_mode == 2)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 2;
		}
		else if(Board_Tx_Info.vision_mode == 3)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 3;
		}
		else if(Board_Tx_Info.vision_mode == 5)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 5;
		}
		else
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 0;
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[BUFF_NUM]);
	}
	vision_last_mode = Board_Tx_Info.vision_mode;
	
	//视觉框更新
	static uint8_t vision_last_flag = 0;
	
	if(vision_last_flag != (Balance.Vision.Auto_Catch_Flag || Balance.Vision.Auto_Base_Flag))
	{
		if((Balance.Vision.Auto_Catch_Flag || Balance.Vision.Auto_Base_Flag) == 1)
		{
			dynamic_ui_info[BUFF_FRAME].ui_config.color = GREEN;
		}
		else
		{
			dynamic_ui_info[BUFF_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[BUFF_FRAME]);
	}
	vision_last_flag = (Balance.Vision.Auto_Catch_Flag || Balance.Vision.Auto_Base_Flag);
	
//	//腿长长度更新
	UI_Info_Update_Leg_length();
	
	static uint32_t l_last = 0, r_last = 0;
	
	if(l_last != My_UI_Dynamic_Info.l_leg_length)
	{
		dynamic_ui_info[L_LEG_LENGTH].ui_config.end_y = dynamic_ui_info[L_LEG_LENGTH].ui_config.start_y + My_UI_Dynamic_Info.l_leg_length;
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[L_LEG_LENGTH]);
	}
	
	l_last = My_UI_Dynamic_Info.l_leg_length;
	
	if(r_last != My_UI_Dynamic_Info.r_leg_length)
	{
		dynamic_ui_info[R_LEG_LENGTH].ui_config.end_y = dynamic_ui_info[R_LEG_LENGTH].ui_config.start_y + My_UI_Dynamic_Info.r_leg_length;
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[R_LEG_LENGTH]);
	}
	
	r_last = My_UI_Dynamic_Info.r_leg_length;
	
	//超电条更新,24.f:500
	float cap_V = 0;
	static uint16_t cap_line = 0,cap_line_last = 0;
	
	cap_line = (uint16_t)(((My_Cap.cap_v * My_Cap.cap_v) / (24.f * 24.f)) * 500);
	
	if(cap_line_last != cap_line)
	{
		dynamic_ui_info[CAP_LINE].ui_config.end_x = (Client_mid_position_x -250) + cap_line;
//		dynamic_ui_info[CAP_LINE].ui_config.end_y = Client_mid_position_y + 320;
		if(((My_Cap.cap_v * My_Cap.cap_v) / (24.f * 24.f)) <= 0.14)
		{
			dynamic_ui_info[CAP_LINE].ui_config.color = FUCHSIA;
		}			
		else if(((My_Cap.cap_v * My_Cap.cap_v) / (24.f * 24.f)) <= 0.39)
		{
			dynamic_ui_info[CAP_LINE].ui_config.color = ORANGE;
		}
		else
		{
			dynamic_ui_info[CAP_LINE].ui_config.color = GREEN;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[CAP_LINE]);
	}
	cap_line_last = cap_line;
	
	//机体线更新
	static float pitch_err_last = 0.f,pitch_err_now;
	
	pitch_err_now = Chassis.Leg_Unit[R_Leg]->Straight->info->thetab_err;
	pitch_err_now = Lowpass(pitch_err_last,pitch_err_now,0.5f);
	if(pitch_err_last != pitch_err_now)
	{
		My_Chas_Pitch_Update(pitch_err_now);
		Enqueue_Ui_For_Sending(&dynamic_ui_info[PITCH_LINE]);
	}
	pitch_err_last = pitch_err_now;
	
	//摆角线更新
	static float theta_err_last = 0.f,theta_err_now;
	
	theta_err_now = Chassis.Leg_Unit[R_Leg]->Straight->info->thetal_err;//右视图
	theta_err_now = Lowpass(theta_err_last,theta_err_now,0.5f);
	if(theta_err_last != theta_err_now)
	{
		My_Chas_Theta_Update(theta_err_now);
		Enqueue_Ui_For_Sending(&dynamic_ui_info[THETA_LINE]);
	}
	theta_err_last = theta_err_now;
	
	//底盘方位角更新
	static float chas_angle_err_last = 0.f,test_chas_angle = 0.f;
	
	test_chas_angle = (gimbal.base_info.yaw_motor_angle);
	if(my_abs(test_chas_angle) > PI)
	{
		test_chas_angle -= sgn(test_chas_angle) * 2 * PI;
	}
	
//	test_chas_angle = (test_chas_angle / 4096.f) * PI;
	if(chas_angle_err_last != test_chas_angle)
	{
		My_Chas_Circle_Update(test_chas_angle);
		
	}
	
	//自瞄框更新
	static uint8_t vision_last_state = 0,vision_now_state = 0;
	static uint8_t vision_lost = 0;
	
	if((Board_Tx_Info.vision_mode == 1 || Board_Tx_Info.vision_mode == 5) && Board_Rx_Info.is_find_Target == 1)
	{
		vision_now_state = 1;
	}
	else if((Board_Tx_Info.vision_mode == 2 || Board_Tx_Info.vision_mode == 3) && Board_Rx_Info.is_find_base == 1)
	{
		vision_now_state = 2;
	}
	else
	{
		vision_now_state = 0;
	}
	
	if(Board_Rx_Info.hit_enable == 1 && Board_Rx_Info.is_find_Target == 1 && Board_Rx_Info.vision_state == 1)
	{
		vision_lost = 0;
	}
	
	if(vision_last_state != vision_now_state)
	{
		if(vision_now_state == 1)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = CYAN_BLUE;
		}
		else if(vision_now_state == 2)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = GREEN;
		}
		else if(vision_now_state == 0)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[AUTO_CATCH_FRAME]);
	}
	
	vision_last_state = vision_now_state;
	
	//视觉丢失
	if(Board_Rx_Info.vision_state == 0 && vision_lost == 0)//只更新一次
	{
		dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = BLACK;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[AUTO_CATCH_FRAME]);
		vision_lost = 1;
	}
	
	//车体速度更新
	static float speed_last = 0.f,speed_now = 0.f;
	
	speed_now = XEstimateKF.FilteredValue[1];
	if(speed_last != speed_now)
	{
		dynamic_ui_info[CAR_SPEED].ui_config.float_num = speed_now;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[CAR_SPEED]);
	}
	speed_last = speed_now;
	
	//腿长模式更新
	static uint8_t length_mode = 1,length_mode_last = 1;
	if(Balance.Flag->Knee_Strike_1_Flag == true)
	{
		length_mode = 3;
	}
	else if(Balance.Flag->Middle_Flag == true)
	{
		length_mode = 2;
	}
	else
	{
		length_mode = 1;
	}
	if(length_mode != length_mode_last)
	{
		if(length_mode == 1)
		{
			dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 125;
			dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 165;
		}
		else if(length_mode == 2)
		{
			dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 65;
			dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 105;
		}
		else if(length_mode == 3)
		{
			dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 5;
			dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 45;
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[LENGTH_FRAME]);
	}
	length_mode_last = length_mode;
	
	//视觉pitch偏置
  dynamic_ui_info[PITCH_OFFSET_NUM].ui_config.float_num = gimbal.offset_info->lob_pitch_gyro_offset;
	Enqueue_Ui_For_Sending(&dynamic_ui_info[PITCH_OFFSET_NUM]);

	if(Balance.Chassis_Com->COMMON_OUTPOST_SHOOT == true)
	{
		dynamic_ui_info[VISION_TAR].ui_config.color = GREEN;
	}
	else
	{
		dynamic_ui_info[VISION_TAR].ui_config.color = WHITE;
	}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[VISION_TAR]);
	
}
//2201-2431

float test_l = 0.f;
float test_r = 0.f;
uint8_t face = 0;//0前1后
float testt = 0.f;

void UI_Info_Update_Leg_length(void)//0.24:120
{	
	if(fabsf(Chassis.Posture->info->yaw) <= 0.7f)
	{
	  test_l = constrain(Chassis.Leg_Unit[L_Leg]->Link->info->length->l0, MIN_LEG_LENGTH, MAX_LEG_LENGTH) - MIN_LEG_LENGTH;
	  test_r = constrain(Chassis.Leg_Unit[R_Leg]->Link->info->length->l0, MIN_LEG_LENGTH, MAX_LEG_LENGTH) - MIN_LEG_LENGTH;
	}
	else if(fabsf(Chassis.Posture->info->yaw) >= 0.7f)
	{
		test_r = constrain(Chassis.Leg_Unit[L_Leg]->Link->info->length->l0, MIN_LEG_LENGTH, MAX_LEG_LENGTH) - MIN_LEG_LENGTH;
	  test_l = constrain(Chassis.Leg_Unit[R_Leg]->Link->info->length->l0, MIN_LEG_LENGTH, MAX_LEG_LENGTH) - MIN_LEG_LENGTH;
	}
	float real_range = (MAX_LEG_LENGTH - MIN_LEG_LENGTH);
	
	float gragh_range = my_abs((float)const_ui_info[HIGH_LINE].ui_config.start_y - (float)const_ui_info[MINIMUM_LINE].ui_config.start_y);
	
	My_UI_Dynamic_Info.l_leg_length = (uint32_t)((test_l / real_range) * gragh_range);
	
	My_UI_Dynamic_Info.r_leg_length = (uint32_t)((test_r / real_range) * gragh_range);	
	
}
//2174  2460
/**
 * @brief 把某点绕某点旋转一定角度
 * 
 * @param x 存储旋转后x的地址
 * @param y 存储旋转后y的地址
 * @param raw_x 旋转前x的值
 * @param raw_y 旋转后y的值
 * @param mid_x 旋转原点x
 * @param mid_y 旋转原点y
 * @param angle 旋转的角度rad
 */
void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle) 
{
  float s = sin(angle);
  float c = cos(angle);
  // 平移到原点
  float origin_x = raw_x - mid_x;
  float origin_y = raw_y - mid_y;
  // 旋转
  float new_x = origin_x * c - origin_y * s;
  float new_y = origin_x * s + origin_y * c;
  // 平移回去并更新原始坐标
  *x = new_x + mid_x;
  *y = new_y + mid_y;
}

/*底盘pitch倾角更新*/
void My_Chas_Pitch_Update(float angle)
{
	rotate_point(&dynamic_ui_info[PITCH_LINE].ui_config.start_x,&dynamic_ui_info[PITCH_LINE].ui_config.start_y,
	              PITCH_CENTER_X - 55,PITCH_CENTER_Y,
	              PITCH_CENTER_X,PITCH_CENTER_Y,
	              angle);
	
	rotate_point(&dynamic_ui_info[PITCH_LINE].ui_config.end_x,&dynamic_ui_info[PITCH_LINE].ui_config.end_y,
	              PITCH_CENTER_X + 55,PITCH_CENTER_Y,
	              PITCH_CENTER_X,PITCH_CENTER_Y,
	              angle);
}
/*底盘摆角更新*/
void My_Chas_Theta_Update(float angle)
{
	rotate_point(&dynamic_ui_info[THETA_LINE].ui_config.end_x,&dynamic_ui_info[THETA_LINE].ui_config.end_y,
	              PITCH_CENTER_X,PITCH_CENTER_Y - 80,
	              PITCH_CENTER_X,PITCH_CENTER_Y,
	              angle);
}

/*底盘方位更新*/
void My_Chas_Circle_Update(float angle)
{
	rotate_point(&dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_x,&dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y + CHAS_CIRCLE_R,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	rotate_point(&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_x,&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_y,
	              CHAS_CIRCLE_X - CHAS_CIRCLE_R,CHAS_CIRCLE_Y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	rotate_point(&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_x,&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_y,
	              CHAS_CIRCLE_X + CHAS_CIRCLE_R,CHAS_CIRCLE_Y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	Enqueue_Ui_For_Sending(&dynamic_ui_info[CHAS_HEAD_LINE]);
	Enqueue_Ui_For_Sending(&dynamic_ui_info[CHAS_SIDE_LINE]);
}
/*大腿摆角更新*/
void My_Front_Leg_Update(float angle_r, float angle_l)
{
    // 更新右大腿
    rotate_point(&dynamic_ui_info[FRONT_LEG_R].ui_config.end_x,
                 &dynamic_ui_info[FRONT_LEG_R].ui_config.end_y,
                 R_THIGH_END_X0, R_THIGH_END_Y0,
                 R_THIGH_START_X, R_THIGH_START_Y,
                 angle_r);
	
	rotate_point(&dynamic_ui_info[FRONT_LEG_L].ui_config.end_x,
                 &dynamic_ui_info[FRONT_LEG_L].ui_config.end_y,
                 L_THIGH_END_X0, L_THIGH_END_Y0,
                 L_THIGH_START_X, L_THIGH_START_Y,
                 angle_l);
    // 更新左大腿（类似，定义左腿初始坐标）

    // 将小腿起点更新为大腿终点
    dynamic_ui_info[BACK_LEG_R].ui_config.start_x = 
        dynamic_ui_info[FRONT_LEG_R].ui_config.end_x;
    dynamic_ui_info[BACK_LEG_R].ui_config.start_y = 
        dynamic_ui_info[FRONT_LEG_R].ui_config.end_y;
		
	 dynamic_ui_info[BACK_LEG_L].ui_config.start_x = 
        dynamic_ui_info[FRONT_LEG_L].ui_config.end_x;
    dynamic_ui_info[BACK_LEG_L].ui_config.start_y = 
        dynamic_ui_info[FRONT_LEG_L].ui_config.end_y;

	rotate_point(&dynamic_ui_info[BACK_LEG_R].ui_config.end_x,
                 &dynamic_ui_info[BACK_LEG_R].ui_config.end_y,
                 R_SHANK_END_X0, R_SHANK_END_Y0,
                 R_SHANK_START_X0, R_SHANK_START_Y0,
                 angle_r);
				 
	rotate_point(&dynamic_ui_info[BACK_LEG_L].ui_config.end_x,
                 &dynamic_ui_info[BACK_LEG_L].ui_config.end_y,
                 L_SHANK_END_X0, L_SHANK_END_Y0,
                 L_SHANK_START_X0, L_SHANK_START_Y0,
                 angle_r);
    // 左腿同理
}

void My_Back_Leg_Update(float angle_r, float angle_l)
{
    // 右小腿：基于小腿起点旋转固定偏移量
    rotate_point(&dynamic_ui_info[BACK_LEG_R].ui_config.end_x,
                 &dynamic_ui_info[BACK_LEG_R].ui_config.end_y,
                 R_SHANK_END_X0, R_SHANK_END_Y0,
                 dynamic_ui_info[BACK_LEG_R].ui_config.start_x,
                 dynamic_ui_info[BACK_LEG_R].ui_config.start_y,
                 angle_r);
				 
	rotate_point(&dynamic_ui_info[BACK_LEG_L].ui_config.end_x,
                 &dynamic_ui_info[BACK_LEG_L].ui_config.end_y,
                 L_SHANK_END_X0, L_SHANK_END_Y0,
                 dynamic_ui_info[BACK_LEG_L].ui_config.start_x,
                 dynamic_ui_info[BACK_LEG_L].ui_config.start_y,
                 angle_r);
    // 左小腿同理
}
