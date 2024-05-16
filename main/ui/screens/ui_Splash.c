// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: Smart_Gadget

#include "../ui.h"

void ui_Splash_screen_init(void)
{
    ui_Splash = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Splash, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Splash, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Splash, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Demo = ui_Small_Label_create(ui_Splash);
    lv_obj_set_x(ui_Demo, 0);
    lv_obj_set_y(ui_Demo, 75);
    lv_obj_set_align(ui_Demo, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Demo, "Demo");
    lv_obj_set_style_text_color(ui_Demo, lv_color_hex(0x9C9CD9), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Demo, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Smart_Gadget = ui_Small_Label_create(ui_Splash);
    lv_obj_set_x(ui_Smart_Gadget, 0);
    lv_obj_set_y(ui_Smart_Gadget, 50);
    lv_obj_set_align(ui_Smart_Gadget, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Smart_Gadget, "Smart Gadget");
    lv_obj_set_style_text_color(ui_Smart_Gadget, lv_color_hex(0x000746), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Smart_Gadget, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SLS_Logo = lv_img_create(ui_Splash);
    lv_img_set_src(ui_SLS_Logo, &ui_img_sls_logo_png);
    lv_obj_set_width(ui_SLS_Logo, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_SLS_Logo, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SLS_Logo, 0);
    lv_obj_set_y(ui_SLS_Logo, -50);
    lv_obj_set_align(ui_SLS_Logo, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_SLS_Logo, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_SLS_Logo, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    lv_obj_add_event_cb(ui_Splash, ui_event_Splash, LV_EVENT_ALL, NULL);

}
