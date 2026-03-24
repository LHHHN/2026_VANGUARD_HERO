//
// Created by RM UI Designer
//

#ifndef UI_Infantry_group1_3_H
#define UI_Infantry_group1_3_H

#include "ui_interface.h"

#if USE_RAW == 1

extern ui_interface_line_t *ui_Infantry_group1_midline;
extern ui_interface_line_t *ui_Infantry_group1_left;
extern ui_interface_line_t *ui_Infantry_group1_right;
extern ui_interface_line_t *ui_Infantry_group1_horizen1;
extern ui_interface_rect_t *ui_Infantry_group1_body_rect;
extern ui_interface_rect_t *ui_Infantry_group1_capasitorRect;
extern ui_interface_line_t *ui_Infantry_group1_frontLine;

void _ui_init_Infantry_group1_3();
void _ui_update_Infantry_group1_3();
void _ui_remove_Infantry_group1_3();

#endif

#endif //UI_Infantry_group1_3_H
