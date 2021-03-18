/***************************************************************************//**
 * @file
 * @brief Implements the Thermometer (GATT Server) Role of the Health
 * Thermometer Profile, which enables a Collector device to connect and
 * interact with a Thermometer.  The device acts as a connection
 * Peripheral. The sample application is based on Micrium OS RTOS.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                            INCLUDE FILES
 *********************************************************************************************************
 *********************************************************************************************************
 */

#include "bsp/siliconlabs/generic/include/bsp_os.h"

#include <cpu/include/cpu.h>
#include <common/include/common.h>
#include <kernel/include/os.h>

#include <common/include/lib_def.h>
#include <common/include/rtos_utils.h>
#include <common/include/toolchains.h>
#include <common/include/rtos_prio.h>
#include  <common/include/platform_mgr.h>

#include "sleep.h"
#include <stdio.h>

#include "rtos_bluetooth.h"
//Bluetooth API definition
#include "rtos_gecko.h"
//GATT DB
#include "gatt_db.h"

/* Board Headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "hal-config-app-common.h"

#include "em_core.h"
#include "em_cmu.h"

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                             LOCAL DEFINES
 *********************************************************************************************************
 *********************************************************************************************************
 */


#define OTA

#define  EX_MAIN_START_TASK_PRIO            21u
#define  EX_MAIN_START_TASK_STK_SIZE        512u

#define  APP_CFG_TASK_START_PRIO            2u
#define  APP_CFG_TASK_BLUETOOTH_LL_PRIO     3u
#define  APP_CFG_TASK_BLUETOOTH_STACK_PRIO  4u
#define  APP_CFG_TASK_APPLICATION_PRIO      5u
#define  APP_CFG_TASK_THERMOMETER_PRIO      6u

#define  APP_CFG_TASK_THERMOMETER_STK_SIZE  (1200 / sizeof(CPU_STK))
#define  APPLICATION_STACK_SIZE             (1024 / sizeof(CPU_STK))

// MTM: Added configs for all Micrium OS tasks
// Timer Task Configuration
#if (OS_CFG_TMR_EN == DEF_ENABLED)
#define  TIMER_TASK_PRIO            4u
#define  TIMER_TASK_STK_SIZE        256u
#define  TIMER_TASK_CFG             .TmrTaskCfg = \
{                                                 \
    .StkBasePtr = &TimerTaskStk[0],               \
    .StkSize    = TIMER_TASK_STK_SIZE,            \
    .Prio       = TIMER_TASK_PRIO,                \
    .RateHz     = 10u                             \
},
#else
#define  TIMER_TASK_CFG
#endif

// ISR Configuration
#define  ISR_STK_SIZE               256u
#define  ISR_CFG                        .ISR = \
{                                              \
    .StkBasePtr  = (CPU_STK*) &ISRStk[0],      \
    .StkSize     = (ISR_STK_SIZE)              \
},

/* Define RTOS_DEBUG_MODE=DEF_ENABLED at the project level,
 * for enabling debug information for Micrium Probe.*/
#if (RTOS_DEBUG_MODE == DEF_ENABLED)
#define STAT_TASK_CFG          .StatTaskCfg = \
{                                             \
    .StkBasePtr = DEF_NULL,                   \
    .StkSize    = 256u,                       \
    .Prio       = KERNEL_STAT_TASK_PRIO_DFLT, \
    .RateHz     = 10u                         \
},

#define  OS_INIT_CFG_APP            { \
    ISR_CFG                           \
    TIMER_TASK_CFG                    \
    STAT_TASK_CFG                     \
    .MsgPoolSize     = 0u,            \
    .TaskStkLimit    = 0u,            \
    .MemSeg          = DEF_NULL,      \
    .TickRate        = 1000u          \
}
#elif 0
#define  OS_INIT_CFG_APP            { \
    ISR_CFG                           \
    TIMER_TASK_CFG                    \
    .MsgPoolSize     = 0u,            \
    .TaskStkLimit    = 0u,            \
    .MemSeg          = DEF_NULL,      \
    .TickRate        = 1000u          \
}
#else
// Added by Jason Chen, 2021.03.15
#define  OS_INIT_CFG_APP                    { \
	ISR_CFG                                   \
    TIMER_TASK_CFG                            \
    .MsgPoolSize = 100u,                      \
    .TaskStkLimit = 10u,                      \
    .StatTaskCfg =                            \
    {                                         \
      .StkBasePtr = DEF_NULL,                 \
      .StkSize = 256u,                        \
      .Prio = KERNEL_STAT_TASK_PRIO_DFLT,     \
      .RateHz = 10u                           \
    },                                        \
    .MemSeg = DEF_NULL,                       \
    .TickRate = 1000u                         \
}
#endif

#define  COMMON_INIT_CFG_APP        { \
    .CommonMemSegPtr = DEF_NULL       \
}

#define  PLATFORM_MGR_INIT_CFG_APP  { \
    .PoolBlkQtyInit = 0u,             \
    .PoolBlkQtyMax  = 0u              \
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                        LOCAL GLOBAL VARIABLES
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 * Bluetooth stack configuration
 */

#define MAX_CONNECTIONS 4
#define MAX_ADVERTISERS 2
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];
#if 0
/* Gecko configuration parameters (see gecko_configuration.h) */
static const gecko_configuration_t bluetooth_config =
{
  .config_flags = GECKO_CONFIG_FLAG_RTOS,
#if defined(FEATURE_LFXO) || defined(PLFRCO_PRESENT)
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
#else
  .sleep.flags = 0,
#endif
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .gattdb = &bg_gattdb_data,
  .scheduler_callback = BluetoothLLCallback,
  .stack_schedule_callback = BluetoothUpdate,
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
  .rf.flags = APP_RF_CONFIG_ANTENNA,                 /* Enable antenna configuration. */
  .rf.antenna = APP_RF_ANTENNA,                      /* Select antenna path! */
#ifdef OTA
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
  .ota.antenna_defined = APP_RF_CONFIG_ANTENNA,
  .ota.antenna = APP_RF_ANTENNA
#endif
};
#else
/* Bluetooth stack configuration parameters (see "UG136: Silicon Labs Bluetooth C Application Developer's Guide" for details on each parameter) */
static gecko_configuration_t bluetooth_config = {
  .config_flags = GECKO_CONFIG_FLAG_RTOS,                         /* Check flag options from UG136 */
#if defined(FEATURE_LFXO) || defined(PLFRCO_PRESENT) || defined(LFRCO_PRECISION_MODE)
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,                   /* Sleep is enabled */
#else
  .sleep.flags = 0,
#endif
  .bluetooth.max_connections = MAX_CONNECTIONS,                   /* Maximum number of simultaneous connections */
  .bluetooth.max_advertisers = MAX_ADVERTISERS,                   /* Maximum number of advertisement sets */
  .bluetooth.heap            = bluetooth_stack_heap,              /* Bluetooth stack memory for connection management */
  .bluetooth.heap_size       = sizeof(bluetooth_stack_heap),      /* Bluetooth stack memory for connection management */
  .scheduler_callback        = BluetoothLLCallback,
  .stack_schedule_callback   = BluetoothUpdate,
#if defined(FEATURE_LFXO)
  .bluetooth.sleep_clock_accuracy = 100,               /* Accuracy of the Low Frequency Crystal Oscillator in ppm. *
                                                       * Do not modify if you are using a module                  */
#elif defined(PLFRCO_PRESENT) || defined(LFRCO_PRECISION_MODE)
  .bluetooth.sleep_clock_accuracy = 500,               /* In case of internal RCO the sleep clock accuracy is 500 ppm */
#endif
  .gattdb = &bg_gattdb_data,                           /* Pointer to GATT database */
  .ota.flags = 0,                                      /* Check flag options from UG136 */
  .ota.device_name_len = 3,                            /* Length of the device name in OTA DFU mode */
  .ota.device_name_ptr = "OTA",                        /* Device name in OTA DFU mode */
  .pa.config_enable = 1,                               /* Set this to be a valid PA config */
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT,               /* Configure PA input to VBAT */
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,               /* Configure PA input to DCDC */
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
  .rf.flags   = GECKO_RF_CONFIG_ANTENNA,               /* Enable antenna configuration. */
  .rf.antenna = GECKO_RF_ANTENNA,                      /* Select antenna path! */
};

#endif
// Thermometer Task.
static  OS_TCB   App_TaskThermometerTCB;
static  CPU_STK  App_TaskThermometerStk[APP_CFG_TASK_THERMOMETER_STK_SIZE];

// Event Handler Task
static  OS_TCB   ApplicationTaskTCB;
static CPU_STK ApplicationTaskStk[APPLICATION_STACK_SIZE];

// Timer Task
#if (OS_CFG_TMR_EN == DEF_ENABLED)
static  CPU_STK  TimerTaskStk[TIMER_TASK_STK_SIZE];
#endif

static OS_TMR bpt_stateTimer;
static OS_TMR bpt_stateTimer_oneShot;
extern OS_MUTEX I2CMutex;
extern OS_Q    I2C_Queue;
extern OS_Q    V3_Queue;

// ISR Stack
static  CPU_STK  ISRStk[ISR_STK_SIZE];

const   OS_INIT_CFG             OS_InitCfg          = OS_INIT_CFG_APP;
const   COMMON_INIT_CFG         Common_InitCfg      = COMMON_INIT_CFG_APP;
const   PLATFORM_MGR_INIT_CFG   PlatformMgr_InitCfg = PLATFORM_MGR_INIT_CFG_APP;

enum bg_thermometer_temperature_measurement_flag{
  bg_thermometer_temperature_measurement_flag_units    =0x1,
  bg_thermometer_temperature_measurement_flag_timestamp=0x2,
  bg_thermometer_temperature_measurement_flag_type     =0x4,
};

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                       LOCAL FUNCTION PROTOTYPES
 *********************************************************************************************************
 *********************************************************************************************************
 */

static  void     App_TaskThermometer      (void *p_arg);
extern  void     BluetoothApplicationTask (void *p_arg);

//#ifdef OTA
//static uint8_t boot_to_dfu = 0;
//#endif

static inline uint32_t bg_uint32_to_float(uint32_t mantissa, int32_t exponent);
static inline void bg_thermometer_create_measurement(uint8_t* buffer, uint32_t measurement, int fahrenheit);
/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                          GLOBAL FUNCTIONS
 *********************************************************************************************************
 *********************************************************************************************************
 */
extern void bsp_my_init(void);
/*
 *********************************************************************************************************
 *                                                main()
 *
 * Description : This is the standard entry point for C applications. It is assumed that your code will
 *               call main() once you have performed all necessary initialization.
 *
 * Argument(s) : None.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */
int main(void)
{
  RTOS_ERR  err;

  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();
  initVcomEnable();
  CMU_ClockEnable(cmuClock_PRS, true);

  //system already initialized by enter_DefaultMode_from_RESET
  //BSP_SystemInit();                                           /* Initialize System.                                   */

  // MTM: Not needed anymore
  //OS_ConfigureTickTask(&tickTaskCfg);

  OSInit(&err);                                                 /* Initialize the Kernel.                               */
                                                                /*   Check error code.                                  */
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  // MTM: Rename start task to Thermometer task
  OSTaskCreate(&App_TaskThermometerTCB,                         /* Create the Start Task.                               */
               "Thermometer Task",
               App_TaskThermometer,
               0,
               APP_CFG_TASK_THERMOMETER_PRIO,
               &App_TaskThermometerStk[0],
               (APP_CFG_TASK_THERMOMETER_STK_SIZE / 10u),
               APP_CFG_TASK_THERMOMETER_STK_SIZE,
               0,
               0,
               0,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &err);
  /*   Check error code.                                  */
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  OSStart(&err);                                                /* Start the kernel.                                    */
                                                                /*   Check error code.                                  */
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  return (1);
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                           LOCAL FUNCTIONS
 *********************************************************************************************************
 *********************************************************************************************************
 */
/**
 * Convert mantissa & exponent values to IEEE float type
 */
static inline uint32_t bg_uint32_to_float(uint32_t mantissa, int32_t exponent)
{
  return (mantissa & 0xffffff) | (uint32_t)(exponent << 24);
}

/**
 * Create temperature measurement value from IEEE float and temperature type flag
 */
static inline void bg_thermometer_create_measurement(uint8_t* buffer, uint32_t measurement, int fahrenheit)
{
#if 0
  buffer[0] = 'C';//fahrenheit ? bg_thermometer_temperature_measurement_flag_units : 0;
  buffer[1] = measurement & 0xff;

  buffer[2] = (measurement >> 8) & 0xff;
  buffer[3] = (measurement >> 16) & 0xff;
  buffer[4] = (measurement >> 24) & 0xff;
#else
  int ii;
  buffer[0]++;
  if(buffer[0] > 126)
	  buffer[0] = 32;

  for(ii = 0; ii < 15; ii++)
  {
	  buffer[ii + 1] = buffer[ii] + 1;
	  if(buffer[ii + 1] > 126)
		  buffer[ii + 1] = 32;
  }
#endif
}

/***************************************************************************//**
 * Setup the bluetooth init function.
 *
 * @return error code for the gecko_init function
 *
 * All bluetooth specific initialization code should be here like gecko_init(),
 * gecko_init_whitelisting(), gecko_init_multiprotocol() and so on.
 ******************************************************************************/
static errorcode_t initialize_bluetooth()
{
  errorcode_t err = gecko_init(&bluetooth_config);
  APP_RTOS_ASSERT_DBG((err == bg_err_success), 1);
  return err;
}

/*
 *********************************************************************************************************
 *                                          Ex_MainStartTask()
 *
 * Description : This is the task that will be called by the Startup when all services are initializes
 *               successfully.
 *
 * Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
 *
 * Return(s)   : None.
 *
 * Notes       : None.
 *********************************************************************************************************
 */
void bpt_stateTimer_Start(void)
{
	RTOS_ERR  err;
	OSTmrStart(&bpt_stateTimer, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

void bpt_stateTimer_OneShortStart(void)
{
	RTOS_ERR  err;
	OSTmrStart(&bpt_stateTimer_oneShot, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

void bpt_stateTimer_Stop(void)
{
	RTOS_ERR  err;
	OSTmrStop(&bpt_stateTimer, OS_OPT_TMR_NONE, DEF_NULL, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

//static inline void I2CPost_One_time(RTOS_ERR* err)
//{
//  OSMutexPost((OS_MUTEX *)&I2CMutex,
//              (OS_OPT    ) OS_OPT_POST_NONE,
//              (RTOS_ERR *) err);
//  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(*err) == RTOS_ERR_NONE), 1);
//}

extern void bpt_state_runCallback(void *p_tmr, void *p_arg);
extern void bpt_state_runCB(void *p_tmr, void *p_arg);
//extern uint16_t calibrationTimer_read(void);
uint8_t temp_buffer[16];
static  void  App_TaskThermometer(void *p_arg)
{
  RTOS_ERR  err;
  //int temperature_counter = 10;                                 /* A temperature value counting up */
  CORE_DECLARE_IRQ_STATE;

  PP_UNUSED_PARAM(p_arg);                                       /* Prevent compiler warning.                            */

  CORE_ENTER_ATOMIC();
  /* Don't allow EM3, since we use LF clocks. */
  SLEEP_SleepBlockBegin(sleepEM3);
  CORE_EXIT_ATOMIC();

  bluetooth_start(APP_CFG_TASK_BLUETOOTH_LL_PRIO,
                  APP_CFG_TASK_BLUETOOTH_STACK_PRIO,
                  initialize_bluetooth);

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();                                  /* Initialize interrupts disabled measurement.          */
#endif

  BSP_OS_Init();                                                /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
  OSMutexCreate(&I2CMutex,
                "Light Mutex",
                &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  bsp_my_init();

#if 0
  // create one-shot timer for direction array
  OSTmrCreate(&bpt_stateTimer_oneShot,  /*   Pointer to user-allocated timer.   */
              "Demo Timer Direction",   /*   Name used for debugging.           */
              5,                        /*   20 Timer Ticks timeout.            */
              0,                        /*   Unused                             */
              OS_OPT_TMR_ONE_SHOT,      /*   Timer is one-shot.                 */
			  &bpt_state_runCallback,   /*   Called when timer expires.         */
              DEF_NULL,                 /*   No arguments to callback.          */
              &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  OSQCreate((OS_Q     *)&V3_Queue,
		    (CPU_CHAR *)"V3 Queue",
            (OS_MSG_QTY) 32,
            (RTOS_ERR *)&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  OSQCreate((OS_Q     *)&I2C_Queue,
            (CPU_CHAR *)"Proprietary Queue",
            (OS_MSG_QTY) 32,
            (RTOS_ERR *)&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  OSTmrCreate(&bpt_stateTimer,           /* Pointer to user-allocated timer. */
              "bpt state machine",       /* Name used for debugging.         */
              0,                         /* 0 initial delay.                 */
              10,                        /* 100 Timer Ticks period.          */
              OS_OPT_TMR_PERIODIC,       /* Timer is periodic.               */
              &bpt_state_runCallback,    /* Called when timer expires.       */
              DEF_NULL,                  /* No arguments to callback.        */
              &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
#endif
  // Create task for Event handler
  OSTaskCreate((OS_TCB     *)&ApplicationTaskTCB,
               (CPU_CHAR   *)"Bluetooth Application Task",
               (OS_TASK_PTR ) BluetoothApplicationTask,
               (void       *) 0u,
               (OS_PRIO     ) APP_CFG_TASK_APPLICATION_PRIO,
               (CPU_STK    *)&ApplicationTaskStk[0u],
               (CPU_STK_SIZE)(APPLICATION_STACK_SIZE / 10u),
               (CPU_STK_SIZE) APPLICATION_STACK_SIZE,
               (OS_MSG_QTY  ) 0u,
               (OS_TICK     ) 0u,
               (void       *) 0u,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (RTOS_ERR   *)&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

#if 1
  while (DEF_TRUE)
  {
#if 1
	int32_t curren_tick1 = sl_sleeptimer_get_tick_count();
	bpt_state_runCallback(NULL, NULL);
	int32_t diff = sl_sleeptimer_get_tick_count() - curren_tick1;
	diff = sl_sleeptimer_tick_to_ms(diff);

    //V3_state_run();
    //uint8_t temp_buffer[16];
    //bg_thermometer_create_measurement(temp_buffer, bg_uint32_to_float(temperature_counter, 0), 0);

    //temperature_counter++;
    //temp_buffer[8] = diff;
    //if (temperature_counter > 10) {
    //  temperature_counter = 0;
    //  temp_buffer[0] = calibrationTimer_read();
    //  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_gatt_spp_data, 9, temp_buffer);
    //}

    diff = 100 - diff;
    if(diff > 0)
    {
    	OSTimeDlyHMSM(0, 0, 0, diff,
    			OS_OPT_TIME_DLY | OS_OPT_TIME_HMSM_NON_STRICT, &err);
    }
#else
    bpt_state_runCB(NULL, NULL);
#endif
    //gecko_cmd_gatt_server_send_characteristic_notification(0xff, gattdb_temperature_measurement, 5, temp_buffer);
  }
#else
  /* Done starting everyone else so let's exit */
  // MTM: Remove Delete
  OSTaskDel((OS_TCB *)0, &err);
#endif
}

/***************************************************************************//**
 * @brief
 *   This is the idle hook.
 *
 * @detail
 *   This will be called by the Micrium OS idle task when there is no other
 *   task ready to run. We just enter the lowest possible energy mode.
 ******************************************************************************/
void SleepAndSyncProtimer();
void OSIdleContextHook(void)
{
  while (1) {
    /* Put MCU in the lowest sleep mode available, usually EM2 */
    SleepAndSyncProtimer();
  }
}



