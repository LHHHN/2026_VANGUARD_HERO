#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_Infantry_group1_0.h"
#include "ui_Infantry_group1_1.h"
#include "ui_Infantry_group1_2.h"
#include "ui_Infantry_group1_3.h"
#include "ui_Infantry_group1_4.h"
#include "ui_Infantry_group2_0.h"


#define ui_init_Infantry_group() \
_ui_init_Infantry_group1_0(); \
_ui_init_Infantry_group1_1(); \
_ui_init_Infantry_group1_2(); \
_ui_init_Infantry_group1_3(); \
_ui_init_Infantry_group1_4()

#define ui_update_Infantry_group1() \
_ui_update_Infantry_group1_0(); \
_ui_update_Infantry_group1_1(); \
_ui_update_Infantry_group1_2(); \
_ui_update_Infantry_group1_3(); \
_ui_update_Infantry_group1_4()

#define ui_remove_Infantry_group1() \
_ui_remove_Infantry_group1_0(); \
_ui_remove_Infantry_group1_1(); \
_ui_remove_Infantry_group1_2(); \
_ui_remove_Infantry_group1_3(); \
_ui_remove_Infantry_group1_4()


#define ui_init_Infantry_group2() \
_ui_init_Infantry_group2_0()

#define ui_update_Infantry_group2() \
_ui_update_Infantry_group2_0()

#define ui_remove_Infantry_group2() \
_ui_remove_Infantry_group2_0()

typedef struct
{
	/* data */
}__attribute__((packed)) ui_behaviour_t;

typedef struct
{
	/* data */
	
}__attribute__((packed)) ui_cmd_t;

void UI_Task_Init(void);

void ui_init();
void ui_reinit();
void update_shooter_ui(float gimbal_delta, uint8_t cap_level, uint8_t auto_status, uint8_t fric_status, uint8_t error_num);


#ifdef __cplusplus
}
#endif

#endif //UI_H
