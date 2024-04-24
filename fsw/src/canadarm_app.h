/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: canadarm_app.h
**
** Purpose:
**   This file is main hdr file for the ros application.
**
**
*******************************************************************************/

#ifndef _canadarm_app_h_
#define _canadarm_app_h_

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include "canadarm_app_perfids.h"
#include "canadarm_app_msgids.h"
#include "canadarm_app_msg.h"

// #include "canadarm_app_msgids.h"

/***********************************************************************/
#define CANADARM_APP_PIPE_DEPTH 32 /* Depth of the Command Pipe for Application */
/************************************************************************
** Type Definitions
*************************************************************************/

/*
** Global Data
*/

typedef struct
{
    /*
    ** Command interface counters...
    */
    uint8 CmdCounter;
    uint8 ErrCounter;

    uint32 square_counter;
    uint32 hk_counter;

    //Housekeeping telemetry packet...
    CanadarmAppHkTlm_t HkTlm;
    // Goal joint state sent to robot on flight side
    CanadarmAppRobotCommand_t FlightGoal;
    
    // Run Status variable used in the main processing loop
    uint32 RunStatus;

    /*
    ** Operational data (not reported in housekeeping)...
    */
    CFE_SB_PipeId_t CommandPipe;

    /*
    ** Initialization data (not reported in housekeeping)...
    */
    char   PipeName[CFE_MISSION_MAX_API_LEN];
    uint16 PipeDepth;

    CFE_EVS_BinFilter_t EventFilters[CANADARM_APP_EVENT_COUNTS];

} CanadarmAppData_t;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (CanadarmAppMain), these
**       functions are not called from any other source module.
*/
void  CanadarmAppMain(void);

int32 CanadarmAppInit(void);

void  CanadarmAppProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr);
void  CanadarmAppProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr);

int32 CanadarmAppReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg);
void CanadarmAppProcessRobotState(CFE_SB_Buffer_t *SBBufPtr);

int32 CanadarmAppNoop(const CanadarmAppNoopCmd_t *Msg);
int32 updateRobotCommand(const CanadarmAppCmd_t *Msg);

bool CanadarmAppVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);


#endif /* _CANADARM_APP_h_ */
