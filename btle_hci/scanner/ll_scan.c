/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ll_scan.h"

#include "btle.h"
#include "nrf_report_disp.h"
#include "radio.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_timer.h"

/*****************************************************************************
* Local definitions
*****************************************************************************/

/**@brief Defining GPIO pins used for looking at timing.
 */

#define DBG_RADIO_END                        0
#define DBG_RADIO_READY                      1
#define DBG_RADIO_TIMER                      2

/**@brief Advertisement report indexes
 */
#define REPORT_BUF_SIZE 2
#define REPORT_ADV_INDEX 0
#define REPORT_RSP_INDEX 1

/**@brief Packet buffer size
 */
//3 6 31
#define RX_BUF_SIZE 40

/**@brief Possible scanner states
 */
typedef enum
{
  SCANNER_STATE_NOT_INITIALIZED = 0,
  SCANNER_STATE_INITIALIZED,
  SCANNER_STATE_IDLE,
  SCANNER_STATE_RECEIVE_ADV,
  SCANNER_STATE_SEND_REQ,
  SCANNER_STATE_RECEIVE_SCAN_RSP
} m_state_t;

	
/**@brief Possible packet types
 */
typedef enum
{
  PACKET_TYPE_ADV_IND = 0x00,
  PACKET_TYPE_ADV_DIRECT_IND = 0x01,
  PACKET_TYPE_ADV_NONCONN_IND = 0x02,
  PACKET_TYPE_CONNECT_REQ = 0x03,
  PACKET_TYPE_SCAN_RSP = 0x04,
  PACKET_TYPE_SCAN_REQ = 0x05,
  PACKET_TYPE_ADV_SCAN_IND = 0x06
} m_packet_type_t;

/**@brief Scanner parameters
 */
typedef struct
{
  btle_scan_types_t scan_type;
  btle_address_type_t own_address_type;
  btle_scan_filter_policy_t scanning_filter_policy;
} scanner_params_t;

/**@brief Scanner variables
 */
static struct
{
  scanner_params_t params;
  m_state_t state;
} m_scanner;
  

/*****************************************************************************
* Static Globals
*****************************************************************************/


static uint32_t m_packets_invalid;
static uint32_t m_packets_valid;

static uint8_t m_rssi;

static uint8_t m_rx_buf[RX_BUF_SIZE];
static uint8_t m_tx_buf[] =
{
  0xC3,                               // BLE Header (PDU_TYPE: SCAN_REQ, TXadd: 1 (random address), RXadd: 1 (random address)
  0x0C,                               // Length of payload: 12
  0x00,                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
  0x20, 0x20, 0x20, 0x20, 0x20, 0x00, // InitAddr LSByte first #local addr
  0x30, 0x30, 0x00, 0x00, 0x00, 0x00, // AdvAddr LSByte first
};

#define ALL_SENSOR_COUNT 1

static uint8_t channel = 37;

// protocol data
static uint8_t SENSOR_MAX=32;

static uint32_t sensor_adv_map=0;
static uint8_t sensor_rsp_count=0;
static uint8_t MY_ADV_PACKET_POS0_POS4[]={0x46,0x1E,0x00,0x30,0x30};
static uint8_t MY_RSP_PACKET_POS0_POS4[]={0x44,0x25,0x00,0x30,0x30};
static uint8_t link_rssi[31];
static uint8_t sensor_adv_count=0;
static uint8_t sync_flag=0; // to be modified!!



/*****************************************************************************
* Static Function prototypes
*****************************************************************************/

static void m_adv_report_generate (uint8_t * const pkt);

static void m_state_init_entry (void);
static void m_state_init_exit (void);
static void m_state_idle_entry (void);
static void m_state_idle_exit (void);
static void m_state_receive_adv_entry (void);
static void m_state_receive_adv_exit (void);
static void m_state_send_scan_req_entry (void);
static void m_state_send_scan_req_exit (void);
static void m_state_receive_scan_rsp_entry (void);
static void m_state_receive_scan_rsp_exit (void);

/*****************************************************************************
* Static Function definitions
*****************************************************************************/


app_timer_id_t my_timer;
uint32_t app_timer_counter=0;

static void my_timer_handler(void * p_context)
{
	app_timer_counter+=1;
}

void get_app_timer_time()
{
	
}


static void m_adv_report_generate (uint8_t * const pkt)
{
  bool has_data = false;
  nrf_report_t report;
  btle_ev_param_le_advertising_report_t *adv_report = &report.event.params.le_advertising_report_event;
  
  /* Validate the RSSI value. It is 7 bits, so a value above 0x7F is invalid */
  if (m_rssi > 0x7F)
  {
    return;
  }

  report.event.event_code = BTLE_EVENT_LE_ADVERTISING_REPORT;
  report.event.opcode = BTLE_CMD_NONE;
  
  switch (pkt[0] & 0x0F)
  {
    case PACKET_TYPE_ADV_IND:
      has_data = true;
      adv_report->event_type = BTLE_REPORT_TYPE_ADV_IND; //0x0
      break;

		// this is needed
    case PACKET_TYPE_ADV_SCAN_IND:
      has_data = true;
      adv_report->event_type = BTLE_REPORT_TYPE_ADV_SCAN_IND; //0x2
      break;

    case PACKET_TYPE_ADV_DIRECT_IND: 
      has_data = false;
      adv_report->event_type = BTLE_REPORT_TYPE_ADV_DIRECT_IND; //0x1
      break;

    case PACKET_TYPE_ADV_NONCONN_IND:
      has_data = true;
      adv_report->event_type = BTLE_REPORT_TYPE_ADV_NONCONN_IND; //0x3
      break;
    
    case PACKET_TYPE_SCAN_RSP:
      has_data = true;
      adv_report->event_type = BTLE_REPORT_TYPE_SCAN_RSP;
      break;
    
    default:
      return;
  }
  
  report.valid_packets = m_packets_valid;
  report.invalid_packets = m_packets_invalid;
  memcpy (adv_report->address, &pkt[3], BTLE_DEVICE_ADDRESS__SIZE);
  


  #define BIT_6                               0x40 /**< The value of bit 6 */
  #define UL_PDU_DD_HEADER_OFFSET             0
  #define UL_PDU_DD_SENDER_PADD_OFFSET        UL_PDU_DD_HEADER_OFFSET   /* Called TxAdd in the spec */
  #define UL_PDU_DD_SENDER_PADD_MASK          BIT_6
  #define UL_PDU_DD_SENDER_PADD_SHIFT         6
  
  adv_report->address_type = (pkt[UL_PDU_DD_SENDER_PADD_OFFSET] & UL_PDU_DD_SENDER_PADD_MASK) >> UL_PDU_DD_SENDER_PADD_SHIFT;
  adv_report->rssi = m_rssi;
  
	// 3 6 31 or 3 6 24
	if(has_data)
	{
		adv_report->length_data = (pkt[1]) - BTLE_DEVICE_ADDRESS__SIZE;
		if (adv_report->length_data > 0x1F)
		{
			return;
		}
		memcpy(adv_report->report_data, &pkt[9], BTLE_ADVERTISING_DATA__SIZE);
	}
	else
		adv_report->length_data=0;
	
  adv_report->num_reports = 1;
  nrf_report_disp_dispatch (&report);
}

void data_report_generate(uint8_t flag,char *const pkt,uint8_t pkt_size)
{
	nrf_report_t report;

  btle_ev_param_le_advertising_report_t *data_report = &report.event.params.le_advertising_report_event;

  report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_SCAN_REQ_REPORT;
  report.event.opcode = BTLE_CMD_NONE;
  
  report.valid_packets = m_packets_valid;
  report.invalid_packets = m_packets_invalid;
  
  data_report->num_reports = 1;
	
	// to be tested
  /*
	memcpy(data_report->report_data,link_rssi,31);
	for(uint8_t i=0;i<31;i++)
		link_rssi[i]=0;
	*/

	data_report->event_type=flag;
  memcpy(data_report->report_data,pkt,pkt_size);
	
	
	nrf_report_disp_dispatch(&report);
}

static void m_state_init_entry (void)
{
  m_scanner.state = SCANNER_STATE_INITIALIZED;
}

static void m_state_init_exit (void)
{
  /* Nothing to do */
}

static void m_state_idle_entry (void)
{
  m_scanner.state = SCANNER_STATE_IDLE;
}

static void m_state_idle_exit (void)
{
  /* Nothing to do */
}

static void m_state_receive_adv_entry (void)
{
  memset ((void *) m_rx_buf, '\0', RX_BUF_SIZE);
  radio_buffer_configure (&m_rx_buf[0]);
  // immediate
  radio_rx_prepare (true);
  radio_rssi_enable ();
 
  /* Only go directly to TX if we're doing active scanning */
  /*
  if (m_scanner.params.scan_type == BTLE_SCAN_TYPE_ACTIVE)
  {
    radio_tx_mode_on_receipt ();
  }
  */
  

  uint8_t change_flag=1;
	if (m_scanner.state!=SCANNER_STATE_RECEIVE_ADV)
		change_flag=0;
  m_scanner.state = SCANNER_STATE_RECEIVE_ADV;
	if(change_flag)
		return;
	if(sync_flag)
		data_report_generate(m_scanner.state,"enter_rsp_scan",sizeof("enter_rsp_scan"));
	else
		data_report_generate(m_scanner.state,"enter_adv_scan",sizeof("enter_adv_scan"));
		
}

static void m_state_receive_adv_exit (void)
{
  m_rssi = radio_rssi_get ();
}

static void m_state_send_scan_req_entry (void)
{
	
  //memcpy(&m_tx_buf[9], &m_rx_buf[3], 6);
	//data_report_generate(m_tx_buf[11],"req_val",sizeof("req_val"));
	
  radio_buffer_configure (&m_tx_buf[0]);
  // 149 us
  radio_tx_prepare ();
  
  m_scanner.state = SCANNER_STATE_SEND_REQ;
	char* LOG_DATA="enter_send_req";
  data_report_generate(m_scanner.state,LOG_DATA,sizeof("enter_send_req"));
}

static void m_state_send_scan_req_exit (void)
{
  /* Nothing to do */
  data_report_generate(m_scanner.state,"exit_send_req",sizeof("exit_send_req"));
}

static void m_state_receive_scan_rsp_entry (void)
{
  memset ((void *) m_rx_buf, '\0', RX_BUF_SIZE);
  radio_buffer_configure (&m_rx_buf[0]);
  // here wait for 149 us (micro second)
  //radio_rx_prepare (false);
  radio_rx_prepare(true);
  radio_rssi_enable ();
  //radio_rx_timeout_enable (); add below to be tested

  /* Only go directly to TX if we're doing active scanning */
  /*
  if (m_scanner.params.scan_type == BTLE_SCAN_TYPE_ACTIVE)
  {
    radio_tx_mode_on_receipt ();
  }
  */
  
  m_scanner.state = SCANNER_STATE_RECEIVE_SCAN_RSP;
  data_report_generate(m_scanner.state,"enter_receive_rsp",sizeof("enter_receive_rsp"));
}

static void m_state_receive_scan_rsp_exit (void)
{
  m_rssi = radio_rssi_get ();
   data_report_generate(m_scanner.state,"exit_receive_rsp",sizeof("exit_receive_rsp"));
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

int8_t get_packet_index(uint8_t * const pkt)
{
	int8_t index=-1;
	uint8_t flag=0;
	for(uint8_t i=5;i<9;i++)
	{
		if (pkt[i]==0)
			index+=8;
		else
		{
			uint8_t value=pkt[i];
			for(uint8_t j=0;j<8;j++)
			{
				if((value&0x1)==1)
				{
					if (flag!=0)
						return -1;
					else
					{
						flag=1;
						index+=j;
						index+=1;
					}
				}
				value=value>>1;
			}
			break;
		}
	}
	return index;
	
}


void deal_sensor_adv(uint8_t index)
{
	m_adv_report_generate (m_rx_buf);
	uint32_t flag=1<<index;
	if ((sensor_adv_map&flag)==0)
	{
		data_report_generate(index,"---a_new_sensor_enters---",sizeof("---a_new_sensor_enters---"));
		sensor_adv_count++;
		sensor_adv_map=sensor_adv_map|flag;
	}
}

bool all_sensor_started()
{
	if (sensor_adv_count==ALL_SENSOR_COUNT)
	{
    // sync_flag=1;
		return true;
	}
  else
	  return false;
}

void ll_scan_rx_cb (bool crc_valid)
{
  /* Received invalid packet */
  if (!crc_valid)
  {
    switch(m_scanner.state)
    {
      case SCANNER_STATE_RECEIVE_ADV:
				m_packets_invalid++;
				m_state_receive_adv_exit ();
				radio_disable ();
				m_state_receive_adv_entry ();
				
        break;

      case SCANNER_STATE_RECEIVE_SCAN_RSP:
        m_packets_invalid++;
      
        m_state_receive_scan_rsp_exit ();
        radio_disable ();
        // modified state machine
        //m_state_receive_adv_entry ();
        m_state_receive_scan_rsp_entry();
        break;

      default:
        break;
    }
  }
	int8_t index=-1;
  
  switch (m_scanner.state)
  {  
    /* Packet received */
    case SCANNER_STATE_RECEIVE_ADV:
      m_packets_valid++;

      if(sync_flag==1)
      {
        m_state_receive_adv_exit();
			  if (memcmp((void*)m_rx_buf,(void*)MY_RSP_PACKET_POS0_POS4,5)==0)
				  m_adv_report_generate (m_rx_buf);
        m_state_receive_adv_entry ();
      }
      else
      switch (m_rx_buf[0] & 0x0F)
      {
        /* If active scanning is enabled, these packets should be reponded to with
         * a SCAN_REQ, and we should wait for a SCAN_RSP.
         */
        case PACKET_TYPE_ADV_IND:
					
          m_state_receive_adv_exit ();
          //m_adv_report_generate (m_rx_buf);
					radio_disable();

          /* If we're doing active scanning, prepare to send SCAN REQ, otherwise
           * loop back around to receive a new advertisement.
           */
          /*
          if (m_scanner.params.scan_type == BTLE_SCAN_TYPE_ACTIVE)
          {
            m_state_send_scan_req_entry ();
          }
          else
          {
            */
            m_state_receive_adv_entry ();
         //}
          break;
				
					// this is the type for sensor adv
        case PACKET_TYPE_ADV_SCAN_IND:
          m_state_receive_adv_exit ();
          if (memcmp((void*)MY_ADV_PACKET_POS0_POS4,(void*)m_rx_buf,5)==0)
          {
            index=get_packet_index(m_rx_buf);
            if (index!=-1&&index<SENSOR_MAX)
            {
              deal_sensor_adv(index);
            }
          }
					m_state_receive_adv_entry ();
          break;

        /* These packets do not require response.
         */
        case PACKET_TYPE_ADV_DIRECT_IND:
          m_state_receive_adv_exit ();
          radio_disable();
          //m_adv_report_generate (m_rx_buf);
          m_state_receive_adv_entry ();
          break;
        
        case PACKET_TYPE_ADV_NONCONN_IND:
          m_state_receive_adv_exit ();
          radio_disable();
          //m_adv_report_generate (m_rx_buf);
          m_state_receive_adv_entry ();
          break;

        /* This should not have happened */
        default:
          m_state_receive_adv_exit ();
          radio_disable();
          m_state_receive_adv_entry();
      }
      break;

    case SCANNER_STATE_RECEIVE_SCAN_RSP:
      m_packets_valid++;

      m_state_receive_scan_rsp_exit ();
			if (memcmp((void*)m_rx_buf,(void*)MY_RSP_PACKET_POS0_POS4,5)==0)
				m_adv_report_generate (m_rx_buf);
      //m_state_receive_adv_entry ();
      m_state_receive_scan_rsp_entry();
      break;

    default:
      break;
  }
}

void ll_scan_tx_cb (void)
{
	//data_report_generate(m_scanner.state,"sync_flag=1_just_send_req_into_scan",sizeof("sync_flag=1_just_send_req_into_scan"));
  if(sync_flag==1)
  {
    // just send the req
    data_report_generate(m_scanner.state,"sync_flag=1_just_send_req_into_scan",sizeof("sync_flag=1_just_send_req_into_scan"));
    m_state_send_scan_req_exit ();
    m_state_receive_adv_entry();
  }
  else
    switch (m_scanner.state)
  {
    /* SCAN_REQ has been transmitted, and we must configure the radio to
     * listen for the incoming SCAN_RSP.
     */
    case SCANNER_STATE_SEND_REQ:
      // sync here!! to be modified
      m_state_send_scan_req_exit ();
      //m_state_receive_scan_rsp_entry ();
      m_state_receive_adv_exit();
      break;

    default:
      break;
  }
}

void ll_scan_timeout_cb (void)
{
  if(sync_flag==1)
  {
    m_state_receive_adv_exit ();
      //radio_disable ();
      m_state_receive_adv_entry ();
  }
  else
    switch (m_scanner.state)
    {
      case SCANNER_STATE_RECEIVE_SCAN_RSP:
        m_state_receive_scan_rsp_exit ();
        //radio_disable ();
        m_state_receive_adv_entry ();
        // might be scan req
        //m_state_receive_scan_rsp_entry();

        break;

      default:
        break;
    }
}

void send_req_for_sync(void)
{
  //data_report_generate(m_scanner.state,"in_sync_func:state",sizeof("in_sync_func:state"));
  if(sync_flag==1)
  {
    m_state_receive_adv_exit();
  }
  else
    switch (m_scanner.state)
    {
    case SCANNER_STATE_RECEIVE_ADV:
      m_state_receive_adv_exit();
      break;

    case SCANNER_STATE_RECEIVE_SCAN_RSP:
      m_state_receive_scan_rsp_exit();
      break;

    default:
      break;
    }
  sync_flag=1;
  m_state_send_scan_req_entry();
}

void app_error_handler(uint32_t error_code,uint32_t line_num,const uint8_t * p_file_name)
{
	
}
btle_status_codes_t ll_scan_init (void)
{

  //APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
	APP_TIMER_INIT(0,2,2,NULL);
  uint8_t err_code=app_timer_create(&my_timer, APP_TIMER_MODE_REPEATED, my_timer_handler);
	APP_ERROR_CHECK(err_code); 
  err_code=app_timer_start(my_timer,APP_TIMER_TICKS(1000,0),NULL);
	APP_ERROR_CHECK(err_code);
	
  m_scanner.state = SCANNER_STATE_NOT_INITIALIZED;
  
  m_state_init_entry ();
  ll_scan_reset ();
  
  return BTLE_STATUS_CODE_SUCCESS;
}

btle_status_codes_t ll_scan_reset (void)
{
  btle_status_codes_t status;
  
  if (m_scanner.state == SCANNER_STATE_IDLE || m_scanner.state == SCANNER_STATE_INITIALIZED)
  {
    status = BTLE_STATUS_CODE_SUCCESS;
  }
  else
  {
    status = BTLE_STATUS_CODE_COMMAND_DISALLOWED;
  }
  
  return status;
}

btle_status_codes_t ll_scan_config (btle_scan_types_t scan_type, btle_address_type_t address_type, btle_scan_filter_policy_t filter_policy)
{
  /* Scanner can only be configured when not running */
  if ((m_scanner.state != SCANNER_STATE_INITIALIZED) && (m_scanner.state != SCANNER_STATE_IDLE))
  {
    return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
  }
  
  m_scanner.params.scan_type = scan_type;
  m_scanner.params.own_address_type = address_type;
  m_scanner.params.scanning_filter_policy = filter_policy;
  
  if (m_scanner.state == SCANNER_STATE_INITIALIZED)
  {
    m_state_init_exit ();
    m_state_idle_entry ();
  }
  
  return BTLE_STATUS_CODE_SUCCESS;
}

btle_status_codes_t ll_scan_start (void)
{ 
  /* Toggle pin when radio reaches END (RX or TX) */
  NRF_GPIOTE->CONFIG[DBG_RADIO_END] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                          GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                          DBG_RADIO_END << GPIOTE_CONFIG_PSEL_Pos |
                          GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;

  NRF_PPI->CH[DBG_RADIO_END].EEP = (uint32_t) (&NRF_RADIO->EVENTS_END);
  NRF_PPI->CH[DBG_RADIO_END].TEP = (uint32_t) (&NRF_GPIOTE->TASKS_OUT[DBG_RADIO_END]);
  NRF_PPI->CHENSET = (1 << DBG_RADIO_END);

  /* Toggle pin when radio reaches READY (RX or TX) */
  NRF_GPIOTE->CONFIG[DBG_RADIO_READY] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                          GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                          DBG_RADIO_READY << GPIOTE_CONFIG_PSEL_Pos |
                          GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;

  NRF_PPI->CH[DBG_RADIO_READY].EEP = (uint32_t) (&NRF_RADIO->EVENTS_READY);
  NRF_PPI->CH[DBG_RADIO_READY].TEP = (uint32_t) (&NRF_GPIOTE->TASKS_OUT[DBG_RADIO_READY]);
  NRF_PPI->CHENSET = (1 << DBG_RADIO_READY);

  /* Toggle pin when timer triggers radio START (TX) */
  NRF_GPIOTE->CONFIG[DBG_RADIO_TIMER] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                          GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                          DBG_RADIO_TIMER << GPIOTE_CONFIG_PSEL_Pos |
                          GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;

  NRF_PPI->CH[DBG_RADIO_TIMER].EEP = (uint32_t) (&(NRF_TIMER0->EVENTS_COMPARE[1]));
  NRF_PPI->CH[DBG_RADIO_TIMER].TEP = (uint32_t) (&NRF_GPIOTE->TASKS_OUT[DBG_RADIO_TIMER]);
  NRF_PPI->CHENSET = (1 << DBG_RADIO_TIMER);

  NVIC_EnableIRQ(TIMER0_IRQn);

  m_state_idle_exit ();
  
  if(channel == 40)
    channel = 37;
  
  radio_init (channel++);
  //radio_rx_timeout_init ();
  
  m_state_receive_adv_entry ();

  return BTLE_STATUS_CODE_SUCCESS;
}

btle_status_codes_t ll_scan_stop (void)
{
  return BTLE_STATUS_CODE_SUCCESS;
}
