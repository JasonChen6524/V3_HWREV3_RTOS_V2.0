/***********************************************************************************************//**
 * \file   spp_server_main.c
 * \brief  SPP server example
 *
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "rtos_gecko.h"
#include "gatt_db.h"


#include <stdio.h>
#include "retargetswo.h"
#include "sleep.h"
#include "spp_utils.h"
#include "em_usart.h"
#include "sl_sleeptimer.h"
#include "infrastructure.h"

/* Vecttor3 */
#include "global.h"
#include "v3.h"
#include "i2c.h"
#include "rtos_err.h"
#include <os.h>
#include <rtos_bluetooth.h>

/***************************************************************************************************
  Local Macros and Definitions
 **************************************************************************************************/
// Moved to V3.h
//#define STATE_ADVERTISING 1
//#define STATE_CONNECTED   2
//#define STATE_SPP_MODE    3
#define OTA
/* Maximum number of iterations when polling UART RX data before sending data over BLE connection
 * set value to 0 to disable optimization -> minimum latency but may decrease throughput */
#define UART_POLL_TIMEOUT  5000

/* soft timer handles */
#define TIC_TIMER_HANDLE 1
#define OS_TIMER_HANDLE 2
#define OS_BIOSENSOR_HANDLE 3                                                            // Added by Jason chen, 2021.03.08

#define TIC_TIMER_CONST 32768
#define TIC_TIMER_PERSEC 10

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

typedef enum {
  PROP_STATUS_SEND                  = 0x00,
  PROP_TIMER_EXPIRED                = 0x01,
  PROP_TOGGLE_MODE                  = 0x02,
  PROP_TOGGLE_RXD                   = 0x03,
  V3_EVT_INDICATION                 = 0x04
} V3_Msg;

enum
{
  //HANDLE_V3, HANDLE_EDDYSTONE, HANDLE_IBEACON  //include this code for Eddystone
  HANDLE_V3, HANDLE_IBEACON
};

//ibeacon setup call
void bcnSetupAdvBeaconing(void);

extern void bpt_main(void); // Temporarily put here, belongs elsewhere
extern void bpt_main_reset(void); // Added by Jason, 2021.01.07

#if 0 //include this code for Eddystone
// Eddystone Advertising data
#define EDDYSTONE_DATA_LEN (23)

static const uint8_t eddystone_data[EDDYSTONE_DATA_LEN] = {
  0x03,          // Length of service list
  0x03,          // Service list
  0xAA, 0xFE,    // Eddystone ID
  0x12,          // Length of service data
  0x16,          // Service data
  0xAA,  0xFE,   // Eddystone ID
  0x10,          // Frame type Eddystone-URL
  0x00,          // Tx power
  0x00,          // 0x00=http://www., 0x01=https://www.
  'a','r','t','a','f','l','e','x','.','c','o','m'
};
#endif

/***************************************************************************************************
 Local Variables
 **************************************************************************************************/
static uint8 _conn_handle = 0xFF;
static int _main_state;

tsCounters _sCounters;

static uint8 _max_packet_size = 20; // Maximum bytes per one packet
static uint8 _min_packet_size = 20; // Target minimum bytes for one packet
static uint8 boot_to_dfu = 0;

OS_MUTEX I2CMutex;
static inline void I2CPend(RTOS_ERR* err);
static inline void I2CPost(RTOS_ERR* err);

OS_Q    I2C_Queue;
static inline I2CMsg I2CQueuePend(RTOS_ERR* err);
static inline void I2CQueuePost(I2CMsg msg, RTOS_ERR* err);

OS_Q    V3_Queue;
static inline V3_Msg V3_QueuePend(RTOS_ERR* err);
static inline void V3_QueuePost(V3_Msg msg, RTOS_ERR* err);

static void reset_variables()
{
	_conn_handle = 0xFF;
	_main_state = STATE_ADVERTISING;
   v3status.spp = STATE_ADVERTISING;
    //gecko_cmd_hardware_set_soft_timer(0, TIC_TIMER_HANDLE, 0);
    
	_max_packet_size = 20;

	memset(&_sCounters, 0, sizeof(_sCounters));
   
   v3CommBuf.rxhead = 0;
   v3CommBuf.rxtail = 0;
   v3CommBuf.txhead = 0;
   v3CommBuf.txtail = 0;
}

static void send_spp_msg()
{
	uint8 len = 0;
	uint8 data[256];
	uint16 result;

	//int c;
	//int timeout = 0;

	if (v3CommBuf.txhead==v3CommBuf.txtail) return;  // no data to send;
   
   // Read up to _max_packet_size characters from local buffer
	while (len < _max_packet_size) 
   {
      data[len++] = v3CommBuf.tx[v3CommBuf.txtail++];
      //v3CommBuf.txtail &= V3BUFMASK;  // not needed for 256 size buffer with U8 index
      if (v3CommBuf.txhead==v3CommBuf.txtail) break;
	}

	if (len > 0) {
		// Stack may return "out-of-memory" error if the local buffer is full -> in that case, just keep trying until the command succeeds
		do {
			result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_gatt_spp_data, len, data)->result;
			_sCounters.num_writes++;
		} while(result == bg_err_out_of_memory);

		if (result != 0) {
			printLog("Unexpected error: %x\r\n", result);
		} else {
			_sCounters.num_pack_sent++;
			_sCounters.num_bytes_sent += len;
		}
	}
}

static uint16_t bpt_second_count = 0;
void bptTimer_callback(void)
{
	bpt_second_count++;
}

uint16_t bptTimer_read(void)
{
	return bpt_second_count;
}

void bptTimer_reset(void)
{
	bpt_second_count = 0;
}

void bptTimer_start(void)
{
	gecko_cmd_hardware_set_soft_timer(TIC_TIMER_CONST, TIC_TIMER_HANDLE, false);                      //1000ms timer
}

void bptTimer_stop(void)
{
	gecko_cmd_hardware_set_soft_timer(0, TIC_TIMER_HANDLE, false);                                    //1000ms timer
}

void bpt_state_runCallback(void *p_tmr, void *p_arg)
{
  RTOS_ERR  err;
  /* Called when timer expires:                            */
  /*   'p_tmr' is pointer to the user-allocated timer.     */
  /*   'p_arg' is argument passed when creating the timer. */
  PP_UNUSED_PARAM(p_tmr);
  PP_UNUSED_PARAM(p_arg);

  //OSTimeDlyHMSM(0, 0, 0, 10,
  //		  OS_OPT_TIME_DLY | OS_OPT_TIME_HMSM_NON_STRICT,
  //		  &err);

  //V3_Msg v3_Msg;

  //ledseq();                          // step the LED player
  //fbseq();                           // Step the feedback player (Haptic and buzzer)

  //v3_Msg = V3_QueuePend(&err);
  //if(v3_Msg == V3_EVT_INDICATION)
  //I2CPend(&err);
  //if((v3status.spp == STATE_CONNECTED)||(v3status.spp == STATE_SPP_MODE))
  {
	  bpt_main();
  }
  //I2CPost(&err);
}

extern void bpt_stateTimer_Start(void);
extern void bpt_stateTimer_OneShortStart(void);
void V3_state_run(void)
{
	static U8 sec_tic = TIC_TIMER_PERSEC;
    // Keep v3_state call before LED and feedback calls to make the response more immediate
    sec_tic++;

    if (sec_tic >= TIC_TIMER_PERSEC)
    {
       v3_state();                     // sequence main V3 state machine
       sec_tic = 0;
    }

    ledseq();                          // step the LED player
    fbseq();                           // Step the feedback player (Haptic and buzzer)

    //Jason // For Bio-Sensor estimation -
    //if((v3status.spp == STATE_CONNECTED)||(v3status.spp == STATE_SPP_MODE))  bpt_main();
    //else  bpt_main_reset();

    if (v3sleep.sleepsec)
    {
    	v3_state();                                                                                     // sequence main V3 state machine
    	gecko_cmd_hardware_set_soft_timer(0, OS_TIMER_HANDLE, false);                                   // turn off timer
    	gecko_cmd_hardware_set_soft_timer((v3sleep.sleepsec*TIC_TIMER_CONST), OS_TIMER_HANDLE, false);  // set new sleep timer
    	v3sleep.sleepsec = 0;                                                                           // clear flag
    	sec_tic = TIC_TIMER_PERSEC;                                                                     // Force state machine to be called next entry
    }

    //SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping
}

static void BluetoothEventHandler(struct gecko_cmd_packet* evt)
{
	//static U8 sectic = TIC_TIMER_PERSEC;
	int i;
	switch (BGLIB_MSG_ID(evt->header))
	{
		case gecko_evt_system_boot_id:
		{
			reset_variables();
			gecko_cmd_gatt_set_max_mtu(247);
			gecko_cmd_le_gap_start_advertising(HANDLE_V3, le_gap_general_discoverable, le_gap_undirected_connectable);

			//iBeacon support
			bcnSetupAdvBeaconing();
		}
		break;

		/* Connection opened event */
		case gecko_evt_le_connection_opened_id:
		{
			_conn_handle = evt->data.evt_le_connection_opened.connection;
			printLog("Connected\r\n");
			_main_state = STATE_CONNECTED;
			v3status.spp = STATE_CONNECTED;

			/* Request connection parameter update.
			 * conn.interval min 20ms, max 40ms, slave latency 4 intervals,
			 * supervision timeout 2 seconds
			 * (These should be compliant with Apple Bluetooth Accessory Design Guidelines, both R7 and R8) */
			gecko_cmd_le_connection_set_timing_parameters(_conn_handle, 24, 40, 0, 200, 0, 0xFFFF);
		}
		break;

		case gecko_evt_le_connection_parameters_id:
			printLog("Conn.parameters: interval %u units, txsize %u\r\n", evt->data.evt_le_connection_parameters.interval, evt->data.evt_le_connection_parameters.txsize);
		break;

		case gecko_evt_gatt_mtu_exchanged_id:
			/* Calculate maximum data per one notification / write-without-response, this depends on the MTU.
			 * up to ATT_MTU-3 bytes can be sent at once  */
			_max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;
			_min_packet_size = _max_packet_size; /* Try to send maximum length packets whenever possible */
			printLog("MTU exchanged: %d\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
		break;

		case gecko_evt_le_connection_closed_id:
			printLog("DISCONNECTED!\r\n");

			/* Show statistics (RX/TX counters) after disconnect: */
			printStats(&_sCounters);

			reset_variables();

		   /* Check if need to boot to OTA DFU mode */
		   if (boot_to_dfu) {
			 /* Enter to OTA DFU mode */
			 gecko_cmd_system_reset(2);
		   } else {
			 /* Restart advertising after client has disconnected */
			 gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
			 SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping
		   }

			/* Restart advertising */
			//gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_undirected_connectable);
		break;

		  /* Check if the user-type OTA Control Characteristic was written.
		   * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
		case gecko_evt_gatt_server_user_write_request_id:

			if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
			  /* Set flag to enter to OTA mode */
			  boot_to_dfu = 1;
			  /* Send response to Write Request */
			  gecko_cmd_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection, gattdb_ota_control, bg_err_success);

			  /* Close connection to enter to DFU OTA mode */
			  gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
			}
			break;

		case gecko_evt_gatt_server_characteristic_status_id:
		{
		   struct gecko_msg_gatt_server_characteristic_status_evt_t *pStatus;
		   pStatus = &(evt->data.evt_gatt_server_characteristic_status);

		   if (pStatus->characteristic == gattdb_gatt_spp_data) {
			  if (pStatus->status_flags == gatt_server_client_config) {
				 // Characteristic client configuration (CCC) for spp_data has been changed
				 if (pStatus->client_config_flags == gatt_notification) {
					_main_state = STATE_SPP_MODE;
					v3status.spp = STATE_SPP_MODE;
					//gecko_cmd_hardware_set_soft_timer(32768, TIC_TIMER_HANDLE, 0);
					SLEEP_SleepBlockBegin(sleepEM2); // Disable sleeping
					//initBoard();
					printLog("SPP Mode ON\r\n");
				 } else {
					printLog("SPP Mode OFF\r\n");
					_main_state = STATE_CONNECTED;
					v3status.spp = STATE_CONNECTED;
					SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping
				 }
			  }
		   }
		}
		break;

		case gecko_evt_gatt_server_attribute_value_id:
		{
			for(i=0;i<evt->data.evt_gatt_server_attribute_value.value.len;i++) {
				//USART_Tx(RETARGET_UART, evt->data.evt_gatt_server_attribute_value.value.data[i]);
				v3CommBuf.rx[v3CommBuf.rxhead++]  = evt->data.evt_gatt_server_attribute_value.value.data[i];
				if(v3CommBuf.rxhead == v3CommBuf.rxtail)
				{
					// make error condition for receive buffer overflow
					break;
				}
			}
			_sCounters.num_pack_received++;
			_sCounters.num_bytes_received += evt->data.evt_gatt_server_attribute_value.value.len;
		}
		break;

		case gecko_evt_hardware_soft_timer_id:

			switch (evt->data.evt_hardware_soft_timer.handle)
			{
		         case OS_TIMER_HANDLE:
		         {
		        	 RTOS_ERR err;
		        	 I2CPend(&err);
		        	 V3_state_run();
		        	 I2CPost(&err);
		        	 //V3_QueuePost(V3_EVT_INDICATION, &err);
		        	 //bpt_stateTimer_OneShortStart();
		         }
		         break;

		         case OS_BIOSENSOR_HANDLE:
		         {
		        	 bpt_state_runCallback(NULL, NULL);
		         }
		         break;

		         case TIC_TIMER_HANDLE:
		         {
		        	 bptTimer_callback();
		         }
		         break;

		         default:
		         break;
			}
		   break;

		default:
		break;

	}
}

/*********************************************************************************************************
 *                                             BluetoothApplicationTask()
 *
 * Description : Bluetooth Application task.
 *
 * Argument(s) : p_arg       the argument passed by 'OSTaskCreate()'.
 *
 * Return(s)   : none.
 *
 * Caller(s)   : This is a task.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */
void  BluetoothApplicationTask(void *p_arg)
{
  RTOS_ERR      os_err;
  (void)p_arg;

  //U8 sectic = TIC_TIMER_PERSEC;

     SLEEP_SleepBlockBegin(sleepEM2); // Disable sleeping

     // Create the soft timer to process biosensor
     //gecko_cmd_hardware_set_soft_timer(TIC_TIMER_CONST/10, OS_BIOSENSOR_HANDLE, false);              //100ms timer
     //bpt_stateTimer_Start();

    //RBG OTA tesing
    // Create soft timer to Handle init I/O and runtime I/O
    gecko_cmd_hardware_set_soft_timer(TIC_TIMER_CONST/TIC_TIMER_PERSEC, OS_TIMER_HANDLE, false);

#if 0
  while (DEF_TRUE) {
    OSFlagPend(&bluetooth_event_flags, (OS_FLAGS)BLUETOOTH_EVENT_FLAG_EVT_WAITING,
               0,
               OS_OPT_PEND_BLOCKING + OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
			   //OS_OPT_PEND_NON_BLOCKING + OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
               NULL,
               &os_err);

    BluetoothEventHandler((struct gecko_cmd_packet*)bluetooth_evt);

    OSFlagPost(&bluetooth_event_flags, (OS_FLAGS)BLUETOOTH_EVENT_FLAG_EVT_HANDLED, OS_OPT_POST_FLAG_SET, &os_err);
  }
#else
  while (DEF_TRUE)
  {
	  OS_FLAGS os_flags_ret;

	  if(_main_state == STATE_SPP_MODE) {

		  /* If SPP data mode is active, use non-blocking gecko_peek_event() */
		  os_flags_ret = OSFlagPend(&bluetooth_event_flags, (OS_FLAGS)BLUETOOTH_EVENT_FLAG_EVT_WAITING,
	               0,
	               OS_OPT_PEND_NON_BLOCKING + OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
	               NULL,
	               &os_err);

		  if(os_flags_ret == 0) {
			  /* No stack events to be handled -> send data from local TX buffer */
			  send_spp_msg(); // send V3 Message, if any from circular buffer
			  recv_spp_msg(); // receive V3 Message, if any from circular buffer
	         continue;  		// Jump directly to next iteration i.e. call gecko_peek_event() again
		  }
	  }
	  else {
		  //      /* if there are no events pending then the next call to gecko_wait_event() may cause
		  //      * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
		  //      if (!gecko_event_pending()){flushLog();}

		  /* SPP data mode not active -> check for stack events using the blocking API */
		  OSFlagPend(&bluetooth_event_flags, (OS_FLAGS)BLUETOOTH_EVENT_FLAG_EVT_WAITING,
	               0,
	               OS_OPT_PEND_BLOCKING + OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
				   //OS_OPT_PEND_NON_BLOCKING + OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
	               NULL,
	               &os_err);
	  }

	  BluetoothEventHandler((struct gecko_cmd_packet*)bluetooth_evt);

	  OSFlagPost(&bluetooth_event_flags, (OS_FLAGS)BLUETOOTH_EVENT_FLAG_EVT_HANDLED, OS_OPT_POST_FLAG_SET, &os_err);
  }
#endif
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

void bcnSetupAdvBeaconing(void)
{

  /* This function sets up a custom advertisement package according to iBeacon specifications.
   * The advertisement package is 30 bytes long. See the iBeacon specification for further details.
   */

  static struct
  {
    uint8_t flagsLen; /* Length of the Flags field. */
    uint8_t flagsType; /* Type of the Flags field. */
    uint8_t flags; /* Flags field. */
    uint8_t mandataLen; /* Length of the Manufacturer Data field. */
    uint8_t mandataType; /* Type of the Manufacturer Data field. */
    uint8_t compId[2]; /* Company ID field. */
    uint8_t beacType[2]; /* Beacon Type field. */
    uint8_t uuid[16]; /* 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon*/
    uint8_t majNum[2]; /* Beacon major number. Used to group related beacons. */
    uint8_t minNum[2]; /* Beacon minor number. Used to specify individual beacons within a group.*/
    uint8_t txPower; /* The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines. */
  } bcnBeaconAdvData = {
  /* Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags. */
  2, /* length  */
  0x01, /* type */
  0x04 | 0x02, /* Flags: LE General Discoverable Mode, BR/EDR is disabled. */

  /* Manufacturer specific data */
  26, /* length of field*/
  0xFF, /* type of field */

  /* The first two data octets shall contain a company identifier code from
   * the Assigned Numbers - Company Identifiers document */
  /* 0x004C = Apple */
  {UINT16_TO_BYTES(0x004C)},

  /* Beacon type */
  /* 0x0215 is iBeacon */
  {UINT16_TO_BYTE1(0x0215), UINT16_TO_BYTE0(0x0215)},

  /* 128 bit / 16 byte UUID */
  {0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0},

  /* Beacon major number */
  /* Set to 34987 and converted to correct format */
  {UINT16_TO_BYTE1(34987), UINT16_TO_BYTE0(34987)},

  /* Beacon minor number */
  /* Set as 1025 and converted to correct format */
  {UINT16_TO_BYTE1(1025), UINT16_TO_BYTE0(1025)},

  /* The Beacon's measured RSSI at 1 meter distance in dBm */
  /* 0xC3 is -61dBm */
  0xC3};

  //
  uint8_t len = sizeof(bcnBeaconAdvData);
  uint8_t *pData = (uint8_t*)(&bcnBeaconAdvData);

  /* Set custom advertising data */

  gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_IBEACON, 0, len, pData);

  /* Set 8 dBm Transmit Power */
  gecko_cmd_le_gap_set_advertise_tx_power(HANDLE_IBEACON, 75);

  /* Set advertising parameters. 200ms (320/1.6) advertisement interval. All channels used.
   * The first two parameters are minimum and maximum advertising interval, both in
   * units of (milliseconds * 1.6).  */
  gecko_cmd_le_gap_set_advertise_timing(HANDLE_IBEACON, 1600, 1600, 0, 0);

  /* Start advertising in user mode */
  gecko_cmd_le_gap_start_advertising(HANDLE_IBEACON, le_gap_user_data, le_gap_non_connectable);

}

