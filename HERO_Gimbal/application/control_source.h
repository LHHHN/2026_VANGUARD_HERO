/**
 * @file control_source.h
 * @brief Unified control-source arbitration.
 */

#ifndef __CONTROL_SOURCE_H__
#define __CONTROL_SOURCE_H__

#include <stddef.h>
#include <stdint.h>

#include "remote_control.h"
#include "remote_vt03.h"

typedef enum
{
    CONTROL_SOURCE_NONE = 0,
    CONTROL_SOURCE_RC,
    CONTROL_SOURCE_VT03,
} control_source_e;

static inline control_source_e Control_Get_Source(const RC_ctrl_t *rc, const VT03_ctrl_t *vt03)
{
    if (rc != NULL && rc->online == 1U)
    {
        return CONTROL_SOURCE_RC;
    }

    if (vt03 != NULL && vt03->online == 1U)
    {
        return CONTROL_SOURCE_VT03;
    }

    return CONTROL_SOURCE_NONE;
}

static inline uint8_t Control_Is_VT03(control_source_e source)
{
    return (uint8_t)(source == CONTROL_SOURCE_VT03);
}

#endif /* __CONTROL_SOURCE_H__ */
