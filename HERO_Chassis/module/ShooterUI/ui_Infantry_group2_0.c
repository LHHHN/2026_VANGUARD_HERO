//
// Created by RM UI Designer
//

#include "ui_Infantry_group2_0.h"

#if USE_RAW == 1

#define FRAME_ID 1
#define GROUP_ID 1
#define START_ID 0
#define OBJ_NUM 5
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_Infantry_group2_0;
ui_interface_line_t *ui_Infantry_group2_Front = (ui_interface_line_t *)&(ui_Infantry_group2_0.data[0]);
ui_interface_line_t *ui_Infantry_group2_electricity = (ui_interface_line_t *)&(ui_Infantry_group2_0.data[1]);
ui_interface_rect_t *ui_Infantry_group2_fric_point = (ui_interface_rect_t *)&(ui_Infantry_group2_0.data[2]);
ui_interface_rect_t *ui_Infantry_group2_auto_point = (ui_interface_rect_t *)&(ui_Infantry_group2_0.data[3]);
ui_interface_number_t *ui_Infantry_group2_error_num = (ui_interface_number_t *)&(ui_Infantry_group2_0.data[4]);

void _ui_init_Infantry_group2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group2_0.data[i].figure_name[0] = FRAME_ID;
        ui_Infantry_group2_0.data[i].figure_name[1] = GROUP_ID;
        ui_Infantry_group2_0.data[i].figure_name[2] = i + START_ID;
        ui_Infantry_group2_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_Infantry_group2_0.data[i].operate_tpyel = 0;
    }

    ui_Infantry_group2_Front->figure_tpye = 0;
    ui_Infantry_group2_Front->layer = 0;
    ui_Infantry_group2_Front->start_x = 1687;
    ui_Infantry_group2_Front->start_y = 625;
    ui_Infantry_group2_Front->end_x = 1687;
    ui_Infantry_group2_Front->end_y = 709;
    ui_Infantry_group2_Front->color = 1;
    ui_Infantry_group2_Front->width = 3;

    ui_Infantry_group2_electricity->figure_tpye = 0;
    ui_Infantry_group2_electricity->layer = 0;
    ui_Infantry_group2_electricity->start_x = 1503;
    ui_Infantry_group2_electricity->start_y = 397;
    ui_Infantry_group2_electricity->end_x = 1503;
    ui_Infantry_group2_electricity->end_y = 799;
    ui_Infantry_group2_electricity->color = 2;
    ui_Infantry_group2_electricity->width = 7;

    ui_Infantry_group2_fric_point->figure_tpye = 1;
    ui_Infantry_group2_fric_point->layer = 0;
    ui_Infantry_group2_fric_point->start_x = 328;
    ui_Infantry_group2_fric_point->start_y = 730;
    ui_Infantry_group2_fric_point->color = 8;
    ui_Infantry_group2_fric_point->width = 12;
    ui_Infantry_group2_fric_point->end_x = 339;
    ui_Infantry_group2_fric_point->end_y = 741;

    ui_Infantry_group2_auto_point->figure_tpye = 1;
    ui_Infantry_group2_auto_point->layer = 0;
    ui_Infantry_group2_auto_point->start_x = 328;
    ui_Infantry_group2_auto_point->start_y = 669;
    ui_Infantry_group2_auto_point->color = 8;
    ui_Infantry_group2_auto_point->width = 12;
    ui_Infantry_group2_auto_point->end_x = 339;
    ui_Infantry_group2_auto_point->end_y = 680;

    ui_Infantry_group2_error_num->figure_tpye = 6;
    ui_Infantry_group2_error_num->layer = 0;
    ui_Infantry_group2_error_num->font_size = 30;
    ui_Infantry_group2_error_num->start_x = 326;
    ui_Infantry_group2_error_num->start_y = 824;
    ui_Infantry_group2_error_num->color = 3;
    ui_Infantry_group2_error_num->number = 0;
    ui_Infantry_group2_error_num->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group2_0, sizeof(ui_Infantry_group2_0));
}

void _ui_update_Infantry_group2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group2_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group2_0, sizeof(ui_Infantry_group2_0));
}

void _ui_remove_Infantry_group2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_Infantry_group2_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_Infantry_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_Infantry_group2_0, sizeof(ui_Infantry_group2_0));
}

#endif