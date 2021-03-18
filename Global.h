/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2016  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.34 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The  software has  been licensed  to Silicon Labs Norway, a subsidiary
of Silicon Labs Inc. whose registered office is 400 West Cesar Chavez,
Austin,  TX 78701, USA solely for  the purposes of creating  libraries 
for its  ARM Cortex-M3, M4F  processor-based devices,  sublicensed and 
distributed  under the  terms and conditions  of the  End User License  
Agreement supplied by Silicon Labs.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information

Licensor:                 SEGGER Software GmbH
Licensed to:              Silicon Laboratories Norway
Licensed SEGGER software: emWin
License number:           GUI-00140
License model:            See Agreement, dated 20th April 2012
Licensed product:         - 
Licensed platform:        Cortex M3, Cortex M4F
Licensed number of seats: -
----------------------------------------------------------------------
File    : GLOBAL.h
Purpose : Global types etc.
---------------------------END-OF-HEADER------------------------------
*/

#ifndef GLOBAL_H            // Guard against multiple inclusion
#define GLOBAL_H

#include "os.h"
/*********************************************************************
*
*       Macros
*
**********************************************************************
*/
#ifndef   U8
  #define U8  unsigned char
#endif
#ifndef   U16
  #define U16 unsigned short
#endif
#ifndef   U32
  #define U32 unsigned long
#endif
#ifndef   I8
  #define I8  signed char
#endif
#ifndef   I16
  #define I16 signed short
#endif
#ifndef   I32
  #define I32 signed long
#endif

#define MUTEX_ADDx
typedef enum {
  PROP_STATUS_SEND                  = 0x00,
  PROP_TIMER_EXPIRED                = 0x01,
  PROP_TOGGLE_MODE                  = 0x02,
  PROP_TOGGLE_RXD                   = 0x03,
  V3_EVT_INDICATION                 = 0x04
} V3_Msg;

typedef enum {
  DEMO_EVT_NONE                     = 0x00,
  DEMO_EVT_BOOTED                   = 0x01,
  DEMO_EVT_BLUETOOTH_CONNECTED      = 0x02,
  DEMO_EVT_BLUETOOTH_DISCONNECTED   = 0x03,
  DEMO_EVT_RAIL_READY               = 0x04,
  DEMO_EVT_RAIL_ADVERTISE           = 0x05,
  DEMO_EVT_LIGHT_CHANGED_BLUETOOTH  = 0x06,
  DEMO_EVT_LIGHT_CHANGED_RAIL       = 0x07,
  DEMO_EVT_INDICATION               = 0x08,
  DEMO_EVT_INDICATION_SUCCESSFUL    = 0x09,
  DEMO_EVT_INDICATION_FAILED        = 0x0A,
  DEMO_EVT_BUTTON0_PRESSED          = 0x0B,
  DEMO_EVT_BUTTON1_PRESSED          = 0x0C,
  DEMO_EVT_CLEAR_DIRECTION          = 0x0D
} I2CMsg;

extern OS_MUTEX I2CMutex;
extern OS_Q     I2C_Queue;
extern OS_Q     V3_Queue;

static inline void I2CPend(RTOS_ERR* err)
{
  OSMutexPend((OS_MUTEX *)&I2CMutex,
              (OS_TICK   ) 0,
              (OS_OPT    ) OS_OPT_PEND_BLOCKING,
              (CPU_TS   *) DEF_NULL,
              (RTOS_ERR *) err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(*err) == RTOS_ERR_NONE), 1);
}

static inline void I2CPost(RTOS_ERR* err)
{
  OSMutexPost((OS_MUTEX *)&I2CMutex,
              (OS_OPT    ) OS_OPT_POST_NONE,
              (RTOS_ERR *) err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(*err) == RTOS_ERR_NONE), 1);
}

static inline I2CMsg I2CQueuePend(RTOS_ERR* err)
{
  I2CMsg i2cMsg;
  OS_MSG_SIZE demoMsgSize;
  i2cMsg = (I2CMsg)OSQPend((OS_Q*       )&I2C_Queue,
                             (OS_TICK     ) 0,
                             (OS_OPT      ) OS_OPT_PEND_NON_BLOCKING,                             //OS_OPT_PEND_BLOCKING
                             (OS_MSG_SIZE*)&demoMsgSize,
                             (CPU_TS*     ) DEF_NULL,
                             (RTOS_ERR*   ) err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(*err) == RTOS_ERR_NONE), 1);
  return i2cMsg;
}

static inline void I2CQueuePost(I2CMsg msg, RTOS_ERR* err)
{
  OSQPost((OS_Q*      )&I2C_Queue,
          (void*      ) msg,
          (OS_MSG_SIZE) sizeof(void*),
          (OS_OPT     ) OS_OPT_POST_FIFO + OS_OPT_POST_ALL,
          (RTOS_ERR*  ) err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(*err) == RTOS_ERR_NONE), 1);
}

static inline V3_Msg V3_QueuePend(RTOS_ERR* err)
{
  V3_Msg v3Msg;
  OS_MSG_SIZE propMsgSize;
  v3Msg = (V3_Msg)OSQPend((OS_Q*       )&V3_Queue,
                             (OS_TICK     ) 0,
                             (OS_OPT      ) OS_OPT_PEND_BLOCKING,
                             (OS_MSG_SIZE*)&propMsgSize,
                             (CPU_TS     *) DEF_NULL,
                             (RTOS_ERR   *) err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(*err) == RTOS_ERR_NONE), 1);
  return v3Msg;
}

static inline void V3_QueuePost(V3_Msg msg, RTOS_ERR* err)
{
  OSQPost((OS_Q*      )&V3_Queue,
          (void*      ) msg,
          (OS_MSG_SIZE) sizeof(void*),
          (OS_OPT     ) OS_OPT_POST_FIFO + OS_OPT_POST_ALL,
          (RTOS_ERR*  ) err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(*err) == RTOS_ERR_NONE), 1);
}

#endif                      // Avoid multiple inclusion

/*************************** End of file ****************************/
