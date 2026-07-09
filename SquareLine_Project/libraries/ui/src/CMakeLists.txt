SET(SOURCES ui_MainScreen.c
    ui_Settings.c
    ui_LogScreen.c
    ui_theme_manager.c
    ui_themes.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_1791433664.c
    ui_img_387995924.c
    fonts/ui_font_bat.c
    fonts/ui_font_big.c
    fonts/ui_font_mid.c
    fonts/ui_font_qq99.c
    fonts/ui_font_set.c
    fonts/ui_font_title.c)

add_library(ui ${SOURCES})
