//
// Created by RM UI Designer
//

#include "ui_Infantry_group1_4.h"

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 10
#define OBJ_NUM 3
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_Infantry_group1_4;
ui_interface_line_t *ui_Infantry_group1_horizen2 = (ui_interface_line_t *)&(ui_Infantry_group1_4.data[0]);
ui_interface_line_t *ui_Infantry_group1_horizen3 = (ui_interface_line_t *)&(ui_Infantry_group1_4.data[1]);
ui_interface_arc_t *ui_Infantry_group1_auto_range = (ui_interface_arc_t *)&(ui_Infantry_group1_4.data[2]);

void _ui_init_Infantry_group1_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group1_4.data[i].figure_name[0] = FRAME_ID;
        ui_Infantry_group1_4.data[i].figure_name[1] = GROUP_ID;
        ui_Infantry_group1_4.data[i].figure_name[2] = i + START_ID;
        ui_Infantry_group1_4.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_Infantry_group1_4.data[i].operate_tpyel = 0;
    }

    ui_Infantry_group1_horizen2->figure_tpye = 0;
    ui_Infantry_group1_horizen2->layer = 0;
    ui_Infantry_group1_horizen2->start_x = 862;
    ui_Infantry_group1_horizen2->start_y = 480;
    ui_Infantry_group1_horizen2->end_x = 1030;
    ui_Infantry_group1_horizen2->end_y = 480;
    ui_Infantry_group1_horizen2->color = 2;
    ui_Infantry_group1_horizen2->width = 2;

    ui_Infantry_group1_horizen3->figure_tpye = 0;
    ui_Infantry_group1_horizen3->layer = 0;
    ui_Infantry_group1_horizen3->start_x = 889;
    ui_Infantry_group1_horizen3->start_y = 496;
    ui_Infantry_group1_horizen3->end_x = 1005;
    ui_Infantry_group1_horizen3->end_y = 496;
    ui_Infantry_group1_horizen3->color = 6;
    ui_Infantry_group1_horizen3->width = 2;

    ui_Infantry_group1_auto_range->figure_tpye = 4;
    ui_Infantry_group1_auto_range->layer = 0;
    ui_Infantry_group1_auto_range->rx = 392;
    ui_Infantry_group1_auto_range->ry = 306;
    ui_Infantry_group1_auto_range->start_x = 958;
    ui_Infantry_group1_auto_range->start_y = 535;
    ui_Infantry_group1_auto_range->color = 6;
    ui_Infantry_group1_auto_range->width = 3;
    ui_Infantry_group1_auto_range->start_angle = 0;
    ui_Infantry_group1_auto_range->end_angle = 360;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group1_4);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_4, sizeof(ui_Infantry_group1_4));
}

void _ui_update_Infantry_group1_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group1_4.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group1_4);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_4, sizeof(ui_Infantry_group1_4));
}

void _ui_remove_Infantry_group1_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group1_4.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group1_4);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_4, sizeof(ui_Infantry_group1_4));
}
