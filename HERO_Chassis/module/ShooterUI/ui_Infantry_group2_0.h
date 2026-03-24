//
// Created by RM UI Designer
//

#ifndef UI_Infantry_group2_0_H
#define UI_Infantry_group2_0_H

#include "ui_interface.h"

#if USE_RAW == 1

extern ui_interface_line_t *ui_Infantry_group2_Front;
extern ui_interface_line_t *ui_Infantry_group2_electricity;
extern ui_interface_rect_t *ui_Infantry_group2_fric_point;
extern ui_interface_rect_t *ui_Infantry_group2_auto_point;
extern ui_interface_number_t *ui_Infantry_group2_error_num;

void _ui_init_Infantry_group2_0();
void _ui_update_Infantry_group2_0();
void _ui_remove_Infantry_group2_0();

#endif

#endif //UI_Infantry_group2_0_H
