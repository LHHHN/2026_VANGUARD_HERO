#ifndef __VPC_H__
#define __VPC_H__

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"

#include "INS.h"


extern void VPC_Init(void);
extern void VPC_UpdatePackets(void);
extern void Choose_VPC_Type(void);
//extern void VPC_SendTarget(void);
#endif