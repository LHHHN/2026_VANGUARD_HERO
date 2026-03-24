//
// Created by RM UI Designer
//

#include "ui_Infantry_group1_3.h"

#if USE_RAW == 1

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 3
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_Infantry_group1_3;
ui_interface_line_t *ui_Infantry_group1_midline = (ui_interface_line_t *)&(ui_Infantry_group1_3.data[0]);
ui_interface_line_t *ui_Infantry_group1_left = (ui_interface_line_t *)&(ui_Infantry_group1_3.data[1]);
ui_interface_line_t *ui_Infantry_group1_right = (ui_interface_line_t *)&(ui_Infantry_group1_3.data[2]);
ui_interface_line_t *ui_Infantry_group1_horizen1 = (ui_interface_line_t *)&(ui_Infantry_group1_3.data[3]);
ui_interface_rect_t *ui_Infantry_group1_body_rect = (ui_interface_rect_t *)&(ui_Infantry_group1_3.data[4]);
ui_interface_rect_t *ui_Infantry_group1_capasitorRect = (ui_interface_rect_t *)&(ui_Infantry_group1_3.data[5]);
ui_interface_line_t *ui_Infantry_group1_frontLine = (ui_interface_line_t *)&(ui_Infantry_group1_3.data[6]);

void _ui_init_Infantry_group1_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group1_3.data[i].figure_name[0] = FRAME_ID;
        ui_Infantry_group1_3.data[i].figure_name[1] = GROUP_ID;
        ui_Infantry_group1_3.data[i].figure_name[2] = i + START_ID;
        ui_Infantry_group1_3.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_Infantry_group1_3.data[i].operate_tpyel = 0;
    }

    ui_Infantry_group1_midline->figure_tpye = 0;
    ui_Infantry_group1_midline->layer = 0;
    ui_Infantry_group1_midline->start_x = 945;
    ui_Infantry_group1_midline->start_y = 387;
    ui_Infantry_group1_midline->end_x = 945;
    ui_Infantry_group1_midline->end_y = 542;
    ui_Infantry_group1_midline->color = 5;
    ui_Infantry_group1_midline->width = 3;

    ui_Infantry_group1_left->figure_tpye = 0;
    ui_Infantry_group1_left->layer = 0;
    ui_Infantry_group1_left->start_x = 398;
    ui_Infantry_group1_left->start_y = 53;
    ui_Infantry_group1_left->end_x = 613;
    ui_Infantry_group1_left->end_y = 387;
    ui_Infantry_group1_left->color = 1;
    ui_Infantry_group1_left->width = 3;

    ui_Infantry_group1_right->figure_tpye = 0;
    ui_Infantry_group1_right->layer = 0;
    ui_Infantry_group1_right->start_x = 1518;
    ui_Infantry_group1_right->start_y = 51;
    ui_Infantry_group1_right->end_x = 1303;
    ui_Infantry_group1_right->end_y = 385;
    ui_Infantry_group1_right->color = 1;
    ui_Infantry_group1_right->width = 3;

    ui_Infantry_group1_horizen1->figure_tpye = 0;
    ui_Infantry_group1_horizen1->layer = 0;
    ui_Infantry_group1_horizen1->start_x = 789;
    ui_Infantry_group1_horizen1->start_y = 441;
    ui_Infantry_group1_horizen1->end_x = 1105;
    ui_Infantry_group1_horizen1->end_y = 441;
    ui_Infantry_group1_horizen1->color = 5;
    ui_Infantry_group1_horizen1->width = 3;

    ui_Infantry_group1_body_rect->figure_tpye = 1;
    ui_Infantry_group1_body_rect->layer = 0;
    ui_Infantry_group1_body_rect->start_x = 1636;
    ui_Infantry_group1_body_rect->start_y = 553;
    ui_Infantry_group1_body_rect->color = 0;
    ui_Infantry_group1_body_rect->width = 3;
    ui_Infantry_group1_body_rect->end_x = 1737;
    ui_Infantry_group1_body_rect->end_y = 683;

    ui_Infantry_group1_capasitorRect->figure_tpye = 1;
    ui_Infantry_group1_capasitorRect->layer = 0;
    ui_Infantry_group1_capasitorRect->start_x = 1496;
    ui_Infantry_group1_capasitorRect->start_y = 397;
    ui_Infantry_group1_capasitorRect->color = 8;
    ui_Infantry_group1_capasitorRect->width = 2;
    ui_Infantry_group1_capasitorRect->end_x = 1509;
    ui_Infantry_group1_capasitorRect->end_y = 797;

    ui_Infantry_group1_frontLine->figure_tpye = 0;
    ui_Infantry_group1_frontLine->layer = 0;
    ui_Infantry_group1_frontLine->start_x = 1657;
    ui_Infantry_group1_frontLine->start_y = 693;
    ui_Infantry_group1_frontLine->end_x = 1721;
    ui_Infantry_group1_frontLine->end_y = 693;
    ui_Infantry_group1_frontLine->color = 0;
    ui_Infantry_group1_frontLine->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group1_3);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_3, sizeof(ui_Infantry_group1_3));
}

void _ui_update_Infantry_group1_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group1_3.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group1_3);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_3, sizeof(ui_Infantry_group1_3));
}

void _ui_remove_Infantry_group1_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group1_3.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group1_3);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group1_3, sizeof(ui_Infantry_group1_3));
}

#endif