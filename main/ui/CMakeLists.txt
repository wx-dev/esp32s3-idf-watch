SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_compass_png.c
    images/ui_img_overlay_png.c
    images/ui_img_indicator_1_png.c)

add_library(ui ${SOURCES})
