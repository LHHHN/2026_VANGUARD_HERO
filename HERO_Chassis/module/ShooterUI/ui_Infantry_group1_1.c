//
// Created by RM UI Designer
//

#include "ui_Infantry_group1_1.h"
#include "string.h"

#if USE_RAW == 1

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 1

ui_string_frame_t ui_Infantry_group1_1;

ui_interface_string_t* ui_Infantry_group1_fric = &ui_Infantry_group1_1.option;

void _ui_init_Infantry_group1_1() {
    ui_Infantry_group1_1.option.figure_name[0] = FRAME_ID;
    ui_Infantry_group1_1.option.figure_name[1] = GROUP_ID;
    ui_Infantry_group1_1.option.figure_name[2] = START_ID;
    ui_Infantry_group1_1.option.operate_tpyel = 1;
    ui_Infantry_group1_1.option.figure_tpye = 7;
    ui_Infantry_group1_1.option.layer = 0;
    ui_Infantry_group1_1.option.font_size = 20;
    ui_Infantry_group1_1.option.start_x = 229;
    ui_Infantry_group1_1.option.start_y = 747;
    ui_Infantry_group1_1.option.color = 8;
    ui_Infantry_group1_1.option.str_length = 4;
    ui_Infantry_group1_1.option.width = 2;
    strcpy(ui_Infantry_group1_fric->string, "FRIC");

    ui_proc_string_frame(&ui_Infantry_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_1, sizeof(ui_Infantry_group1_1));
}

void _ui_update_Infantry_group1_1() {
    ui_Infantry_group1_1.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_Infantry_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_1, sizeof(ui_Infantry_group1_1));
}

void _ui_remove_Infantry_group1_1() {
    ui_Infantry_group1_1.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_Infantry_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_1, sizeof(ui_Infantry_group1_1));
}

#endif