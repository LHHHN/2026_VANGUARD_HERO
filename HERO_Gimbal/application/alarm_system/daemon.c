/**
******************************************************************************
 * @file    daemon.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "daemon.h"
#include "defense_center.h"

void Daemon_Init(void)
{
	// supervisor 实例在各模块调用 Supervisor_Register() 时已统一登记，这里无需额外收集。
}

void Daemon_Proc(void)
{
	Supervisor_Task();
}