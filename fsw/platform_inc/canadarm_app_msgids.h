/************************************************************************
**
**
** File: canadarm_app_msgids.h
**
** Purpose:
**  Define Canadarm App Message IDs
**
** Notes:
**
**
*************************************************************************/
#ifndef _canadarm_app_msgids_h_
#define _canadarm_app_msgids_h_

#include "cfe_msgids.h"

#define CANADARM_APP_CMD_MID     (CFE_PLATFORM_CMD_MID_BASE + 0x37)
#define CANADARM_APP_SEND_HK_MID (CFE_PLATFORM_CMD_MID_BASE + 0x38)
#define CANADARM_APP_ROBOT_STATE_MID (CFE_PLATFORM_CMD_MID_BASE + 0x39)

#define CANADARM_APP_HK_TLM_MID      (CFE_PLATFORM_TLM_MID_BASE + 0x36)
#define CANADARM_APP_ROBOT_CONTROL_MID   (CFE_PLATFORM_TLM_MID_BASE + 0x37)
#define CANADARM_APP_HR_CONTROL_MID  (CFE_PLATFORM_TLM_MID_BASE + 0x38)
#endif /* _canadarm_app_msgids_h_ */

/*********************************/
/* End of File Comment           */
/*********************************/
