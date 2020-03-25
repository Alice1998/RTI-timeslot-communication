/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice, this
	list of conditions and the following disclaimer in the documentation and/or
	other materials provided with the distribution.

	3. Neither the name of Nordic Semiconductor ASA nor the names of other
	contributors to this software may be used to endorse or promote products
	derived from this software without specific prior written permission.

	4. This software must only be used in a processor manufactured by Nordic
	Semiconductor ASA, or in a processor manufactured by a third party that
	is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#include "ts_controller.h"

#include <string.h>
#include <stdio.h>
#include "ts_rng.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_assert.h"
#include "app_error.h"	

#include "btle.h"
#include "ts_peripheral.h"
#include "nrf_advertiser.h"
#include "nrf_report_disp.h"
#include "ts_whitelist.h"

/*****************************************************************************
* Local Definitions
*****************************************************************************/


/* Disable this flag to disable listening for scan requests, and jump straight to 
* next advertisement 
*/
#define TS_SEND_SCAN_RSP (1)

/* Quick macro to scale the interval to the timeslot scale */
#define ADV_INTERVAL_TRANSLATE(interval) (625 * (interval))

/* short macro to check whether the given event is triggered */
#define RADIO_EVENT(x) (NRF_RADIO->x != 0)

/*****************************************************************************
* Static Globals
*****************************************************************************/


/* Buffer for advertisement data */
static uint8_t adv_data_local[BLE_ADDR_OFFSET + BLE_ADDR_LEN + BLE_PAYLOAD_MAXLEN];

#if TS_SEND_SCAN_RSP
/* Buffer for scan response data, only available in scan request/response mode,
* see define at top */
static uint8_t ble_scan_rsp_data[BLE_ADDR_LEN + BLE_PAYLOAD_MAXLEN];

/* Buffer for any message receiving, only available in scan request/response mode,
* see define at top */
static uint8_t ble_rx_buf[BLE_ADDR_LEN + BLE_PAYLOAD_MAXLEN];

/* Packet counter */
static uint16_t packet_count_valid;

/* Faulty packets counter */
static uint16_t packet_count_invalid;
#endif

/* Store the radio channel */
static uint8_t channel;

/* running flag for advertiser. Decision point: end of timeslot (adv event) */
static bool sm_adv_run;

/* Channel map for advertisement */
static btle_dd_channel_map_t channel_map;
	
/* min advertisement interval */
static btle_adv_interval_t adv_int_min;

/* max advertisement interval */
static btle_adv_interval_t adv_int_max;

/* statemachine state */
static ts_state_t sm_state;
static uint8_t START_FLAG=0;

/* advertisement param->type to spec type map */
static const uint8_t ble_adv_type_raw[] = {0, 1, 6, 2};

/* Pool of 255 (the range of the RNG-peripheral) psedorandomly generated values */
static uint8_t rng_pool[255];

/* A pointer into our pool. Will wrap around upon overflow */
static uint8_t pool_index = 0;

static uint8_t GENERAL_SENSOR_ADDR[]={0x30,0x30,0x00,0x00,0x00,0x00};

/*****************************************************************************
* Globals
*****************************************************************************/

/* return param in signal handler */
nrf_radio_signal_callback_return_param_t g_signal_callback_return_param;


/* timeslot request EARLIEST. Used to send the first timeslot request */
static nrf_radio_request_t g_timeslot_req_earliest = 
			{NRF_RADIO_REQ_TYPE_EARLIEST, 
			.params.earliest = {
						HFCLK, 
						NRF_RADIO_PRIORITY_NORMAL, 
						TIMESLOT_INTERVAL_100MS,		
						10000}
			};

/* timeslot request NORMAL. Used to request a periodic timeslot, i.e. advertisement events */
static nrf_radio_request_t g_timeslot_req_normal =
			{NRF_RADIO_REQ_TYPE_NORMAL, 
			.params.normal = {
						HFCLK, 
						NRF_RADIO_PRIORITY_NORMAL, 
						TIMESLOT_INTERVAL_100MS,	
						TIMESLOT_INTERVAL_100MS}
			};




/*****************************************************************************
* Static Functions
*****************************************************************************/
			

/**
* Get next channel to use. Affected by the tsa adv channels.
*/			
static __INLINE void channel_iterate(void)
{
	while (((channel_map & (1 << (++channel - 37))) == 0) && channel < 40);	
}

/**
* Send initial time slot request to API. 
*/
static __INLINE void timeslot_req_initial(void)
{	
	DEBUG_PIN_POKE(7);
	/* send to sd: */
	uint8_t error_code = sd_radio_request(&g_timeslot_req_earliest);
	APP_ERROR_CHECK(error_code);
}

			
#if TS_SEND_SCAN_RSP

/**
* Extract the scanner address from the rx buffer
*/
static __INLINE void scan_addr_get(btle_address_type_t *addr_type, uint8_t* addr)
{
	*addr_type = ((ble_rx_buf[BLE_TYPE_OFFSET] & BLE_ADDR_TYPE_MASK) > 0 ? BTLE_ADDR_TYPE_RANDOM : BTLE_ADDR_TYPE_PUBLIC);
	
	memcpy((void*) addr, (void*) &ble_rx_buf[BLE_ADDR_OFFSET], BLE_ADDR_LEN);	
}

/**
* Get data from rx-buffer and dispatch scan req event
*/
static __INLINE void scan_req_evt_dispatch(void)
{
	/* prepare scan req report */
	nrf_report_t scan_req_report;
	
	/* packet counters */
	scan_req_report.valid_packets = packet_count_valid;
	scan_req_report.invalid_packets = packet_count_invalid;
	 
	/* event details */
	scan_req_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_SCAN_REQ_REPORT;
	scan_req_report.event.opcode			= BTLE_CMD_NONE;
	
	scan_addr_get(&scan_req_report.event.params.nrf_scan_req_report_event.address_type, 
									scan_req_report.event.params.nrf_scan_req_report_event.address);
	
	periph_radio_rssi_read(&(scan_req_report.event.params.nrf_scan_req_report_event.rssi));
	periph_radio_channel_get(&(scan_req_report.event.params.nrf_scan_req_report_event.channel));
	
	/* send scan req event to user space */
	nrf_report_disp_dispatch(&scan_req_report);
}

void generate_report(uint8_t flag,uint8_t * const pkt)
{
	nrf_report_t my_report;
	my_report.valid_packets=packet_count_valid;
	my_report.invalid_packets=packet_count_invalid;
	
	my_report.event.event_code=BTLE_EVENT_LE_ADVERTISING_REPORT;
	my_report.event.opcode=BTLE_CMD_NONE;
	
	if (pkt!=NULL)
	{
		memcpy(&my_report.event.params.le_advertising_report_event.report_data[1],pkt,30);
	}
	my_report.event.params.le_advertising_report_event.report_data[0]=flag;
	
	//btle_ev_param_le_advertising_report_t 
	nrf_report_disp_dispatch(&my_report);
}

/**
* Short check to verify that the incomming 
* message was a scan request for this unit
*/
static uint8_t is_central_req_sensor_rsq(void)
{	
	/* check CRC. Add to number of CRC faults if wrong */

	if (0 == NRF_RADIO->CRCSTATUS) 
	{
		++packet_count_invalid;
		return 0;
	}
	/* check included ADV addr, which must match own ADV addr BLE_ADDR_LEN) */
	if (memcmp(	(void*) GENERAL_SENSOR_ADDR, 
							(void*) &ble_rx_buf[BLE_PAYLOAD_OFFSET], 2) != 0)
	{
		++packet_count_invalid;
		return 0;
	}
	
	if ((0xc3 == ble_rx_buf[0]) && (0x0C == ble_rx_buf[1]))
	{
		++packet_count_valid;
		return 1;
	}
	// to be tested
	if ((0x44 == ble_rx_buf[0]) && (0x25 == ble_rx_buf[1]))
	{
		++packet_count_valid;
		return 2;
	}
	++packet_count_invalid;
		return 0;
}



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



static void deal_sensor_rsp_pkt(int8_t index)
{
	/* prepare scan rsp report */
	nrf_report_t sensor_rsp_report;
	
	/* packet counters */
	sensor_rsp_report.valid_packets = packet_count_valid;
	sensor_rsp_report.invalid_packets = packet_count_invalid;
	 
	// to be modified
	sensor_rsp_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_SCAN_REQ_REPORT;
	sensor_rsp_report.event.opcode			= BTLE_CMD_NONE;
	
	scan_addr_get(&sensor_rsp_report.event.params.nrf_scan_req_report_event.address_type, 
									sensor_rsp_report.event.params.nrf_scan_req_report_event.address);
	
	
	periph_radio_rssi_read(&(sensor_rsp_report.event.params.nrf_scan_req_report_event.rssi));
	periph_radio_channel_get(&(sensor_rsp_report.event.params.nrf_scan_req_report_event.channel));
	
	if (index>UNIQUE_INDEX)
	{
		ble_scan_rsp_data[BLE_PAYLOAD_OFFSET+index-1]=sensor_rsp_report.event.params.nrf_scan_req_report_event.rssi;
	}
	else if (index<UNIQUE_INDEX)
	{
		ble_scan_rsp_data[BLE_PAYLOAD_OFFSET+index]=sensor_rsp_report.event.params.nrf_scan_req_report_event.rssi;
	}

	/* send scan req event to user space */
	nrf_report_disp_dispatch(&sensor_rsp_report);

}
#endif



/**
* Request a new timeslot for the next advertisement event
* If the advertiser has been told to stop, the function will not 
* schedule any more, but just end the timeslot.
*/
static __INLINE void next_timeslot_schedule(void)
{
	if (sm_adv_run)
	{
		//g_timeslot_req_normal.params.normal.distance_us = ADV_INTERVAL_TRANSLATE(adv_int_min) + 1000 * ((rng_pool[pool_index++]) % (ADV_INTERVAL_TRANSLATE(adv_int_max - adv_int_min)));
		g_signal_callback_return_param.params.request.p_next = &g_timeslot_req_normal;
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
		//if(NRF_TIMER0->EVENTS_COMPARE[0]!=0)
		//	periph_timer_abort(0);
	}
	else
	{
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
	}
}

/**
* Doing the setup actions before the first adv_send state actions
*/
static __INLINE void adv_evt_setup(void)
{
	periph_radio_setup();
	
	/* set channel to first in sequence */
	channel = 36; /* will be iterated by channel_iterate() */
	channel_iterate();
}	
	





/******************************************
* Functions for start/end of adv_send state 
******************************************/
static void sm_enter_adv_send(void)
{
	//generate_report(0x0+START_FLAG*16,NULL);
	sm_state = STATE_ADV_SEND;
	periph_radio_ch_set(channel);
	
	/* trigger task early, the rest of the setup can be done in RXRU */
	PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_TXEN);
	
	periph_radio_packet_ptr_set(&adv_data_local[0]);
	
#if TS_SEND_SCAN_RSP
	periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
														RADIO_SHORTS_END_DISABLE_Msk |
														RADIO_SHORTS_DISABLED_RXEN_Msk);
#else
	periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
														RADIO_SHORTS_END_DISABLE_Msk);
#endif    
	
	periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
}

static void sm_exit_adv_send(void)
{
	//generate_report(0x1+START_FLAG*16,NULL);
	/* wipe events and interrupts triggered by this state */
	periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
}


/******************************************
* Functions for start/end of SCAN_REQ_RSP
******************************************/
#if TS_SEND_SCAN_RSP
static void sm_enter_scan_req_rsp(void)
{
	//generate_report(0x2+START_FLAG*16,NULL);
	sm_state = STATE_SCAN_REQ_RSP;
	periph_radio_ch_set(channel);
	periph_radio_packet_ptr_set(&ble_rx_buf[0]);
	
	periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
														RADIO_SHORTS_END_DISABLE_Msk |
														RADIO_SHORTS_DISABLED_TXEN_Msk |
														RADIO_SHORTS_ADDRESS_RSSISTART_Msk);
	
	periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
	
	/* change the tifs in order to be able to capture all packets */
	periph_radio_tifs_set(148);
	
	/* start the timer that aborts the RX if no address is received. */
	//periph_timer_start(0, 200, true);
	
	/* set PPI pipe to stop the timer as soon as an address is received */
	//periph_ppi_set(0, &(NRF_TIMER0->TASKS_STOP), &(NRF_RADIO->EVENTS_ADDRESS));
}

static void sm_exit_scan_req_rsp(void)
{
	//generate_report(0x3+START_FLAG*16,NULL);
	//periph_timer_abort(0);
	periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
	//periph_ppi_clear(0);
}
#endif

/*****************************************
* Functions for start/end of WAIT_FOR_IDLE
******************************************/
static void sm_enter_wait_for_idle(bool req_rx_accepted)
{
	sm_state = STATE_WAIT_FOR_IDLE;
	/* enable disabled interrupt to avoid race conditions */
	periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
	
	/* different behaviour depending on whether we actually 
	received a scan request or not */
	if (req_rx_accepted)
	{
#if TS_SEND_SCAN_RSP		
		// send_rsp_packet();
		uint32_t err_code = sd_radio_session_close ();
		APP_ERROR_CHECK(err_code);
#endif		
	}
	else
	{
		/* remove shorts and disable radio */
		periph_radio_shorts_set(0);
		PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_DISABLE);
	}
}

static void deal_sync_packet(void)
{
	//generate_report(0x30,NULL);
	START_FLAG=1;
#if TS_SEND_SCAN_RSP		
	//uint32_t err_code = sd_radio_session_close ();

	//APP_ERROR_CHECK(err_code);
#endif
}

static void send_rsp_packet(void)
{
	sm_state=STATE_SEND_RSP;
	/* enable disabled interrupt to avoid race conditions */
	periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);

#if TS_SEND_SCAN_RSP		
	/* need to answer request, set scan_rsp packet and 
	let radio continue to send */
	periph_radio_packet_ptr_set(&ble_scan_rsp_data[0]);
	periph_radio_shorts_set(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
	// to be tested
	periph_radio_tifs_set(100);
	//scan_req_evt_dispatch();
#endif
}

static void exit_send_rsp_state(void)
{
	periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);

	//channel_iterate();
	//return (channel > 39);
}

static bool sm_exit_wait_for_idle(void)
{
	periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
	
	channel_iterate();
	
	/* return whether the advertisement event is done */
	return (channel > 39);
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

void ctrl_init(void)
{
	/* set the contents of advertisement and scan response to something
	that is in line with BLE spec */
	
	/* erase package buffers */
	memset(&adv_data_local[0], 0, 40);
#if TS_SEND_SCAN_RSP	
	memset(&ble_scan_rsp_data[0], 0, 40);
#endif 
	
#if TS_SEND_SCAN_RSP	
	/* set message type to ADV_IND_DISC, RANDOM in type byte of adv data */
	adv_data_local[BLE_TYPE_OFFSET] = 0x46;
	/* set message type to SCAN_RSP, RANDOM in type byte of scan rsp data */
	ble_scan_rsp_data[BLE_TYPE_OFFSET] = 0x44;
#else
	/* set message type to ADV_IND_NONCONN, RANDOM in type byte of adv data */
	adv_data_local[BLE_TYPE_OFFSET] = 0x42;
#endif
	

	adv_data_local[BLE_ADFLAG_OFFSET]  = 0x00;
	//adv_data_local[BLE_ADFLAG_OFFSET] |= (1 << 1);  /* General discoverable mode */
  adv_data_local[BLE_ADFLAG_OFFSET] |= (1 << 2);    /* BR/EDR not supported      */

	/* set message length to only address */
	adv_data_local[BLE_SIZE_OFFSET] 			= 0x06;
#if TS_SEND_SCAN_RSP
	ble_scan_rsp_data[BLE_SIZE_OFFSET] 	= 0x06;
#endif	
	
	/* generate rng sequence */
	adv_rng_init(rng_pool);
	
}




__INLINE void ctrl_signal_handler(uint8_t sig)
{
	switch (sig)
	{
		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:	
			DEBUG_PIN_POKE(3);
			periph_timer_start(0,g_timeslot_req_normal.params.normal.distance_us-500,true);		
			adv_evt_setup();
			if(START_FLAG==0)
				sm_enter_adv_send();
			else
			{
				sm_enter_scan_req_rsp();
			}
			periph_timer_start(0, (uint16_t)g_timeslot_req_earliest.params.normal.distance_us, true);
			break;
		
		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
		{
			DEBUG_PIN_POKE(0);
			/* check state, and act accordingly */
			switch (sm_state)
			{
				// just send the adv packet -> scan for req
				case STATE_ADV_SEND:
					if (RADIO_EVENT(EVENTS_DISABLED))
					{
						sm_exit_adv_send();
#if TS_SEND_SCAN_RSP
						// scan for req package
						sm_enter_scan_req_rsp(); 
#else 
						sm_enter_wait_for_idle(false);
						PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_DISABLE);
#endif
					}
					break;
#if TS_SEND_SCAN_RSP	
				// scan for req rsp
				case STATE_SCAN_REQ_RSP:
					if (RADIO_EVENT(EVENTS_DISABLED))
					{
						sm_exit_scan_req_rsp();
						// received req for sync
						uint8_t for_me=is_central_req_sensor_rsq();
						// central sync packet
						if(for_me==1)
						{
							scan_req_evt_dispatch();
							generate_report(0x50,NULL);
							deal_sync_packet();
							send_rsp_packet();
						}
						// sensor rsp
						else if(for_me==2)
						{
							scan_req_evt_dispatch();
							int8_t rsp_sensor_index=get_packet_index(ble_rx_buf);
							generate_report(0x55+rsp_sensor_index,NULL);
							// other sensor rsp packet
							if((START_FLAG==1)&&(rsp_sensor_index!=-1))
							{
								deal_sensor_rsp_pkt(rsp_sensor_index);
							}
							sm_enter_scan_req_rsp();
						}
						else
							sm_enter_scan_req_rsp();
					}
					break;
#endif
				case STATE_SEND_RSP:
					//generate_report(0x52,NULL);
					exit_send_rsp_state();
					sm_enter_scan_req_rsp();
					break;
				
				// to be modified
				case STATE_WAIT_FOR_IDLE:
				/*
					if (RADIO_EVENT(EVENTS_DISABLED))
					{
						DEBUG_PIN_POKE(12);
						DEBUG_PIN_POKE(15);
						state exit function returns whether the adv event is complete
						bool adv_evt_done = sm_exit_wait_for_idle();
						
						if (adv_evt_done)
						{
							generate_report(1);
							next_timeslot_schedule();
						}
						else
						{
							sm_enter_adv_send();
						}
					}
				*/
					break;							
			
				default:
					/* Shouldn't happen */
					ASSERT(false);
				}
		}
			break;
		
#if TS_SEND_SCAN_RSP		
		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
			//(0x10+sm_state,NULL);
			if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
			{
				periph_timer_abort(0);
				next_timeslot_schedule();
				//periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk); ...
			}
			PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_DISABLE);
			break;
#endif		

		default:
			/* shouldn't happen in this advertiser. */

			DEBUG_PIN_SET(LED_0);
			DEBUG_PIN_POKE(13);		
			DEBUG_PIN_POKE(14);		
	}
	
}	


bool ctrl_adv_param_set(btle_cmd_param_le_write_advertising_parameters_t* adv_params)
{
	ASSERT(adv_params != NULL);
	/* Checks for error */

	/* channel map */
	if (0x00 == adv_params->channel_map || 0x07 < adv_params->channel_map)
	{
		return false;
	}

	/* address */
	if (NULL == adv_params->direct_address)
	{
		return false;
	}

	/* set channel map */
	channel_map = adv_params->channel_map;


	/* put address into advertisement packet buffer */
	//static uint8_t ble_addr[] = 000000;
	// copied from main.c
	//static uint8_t ble_addr[] = {0x4e, 0x6f, 0x72, 0x64, 0x69, 0x63};
	//DEFAULT_DEVICE_ADDRESS;
	memcpy((void*) &adv_data_local[BLE_ADDR_OFFSET],
				 (void*) &unique_ble_addr[0], BLE_ADDR_LEN);

#if TS_SEND_SCAN_RSP
	/* put address into scan response packet buffer */
	//memcpy((void*) &ble_scan_rsp_data[BLE_ADDR_OFFSET],(void*) &adv_params->direct_address[0], BLE_ADDR_LEN);
	memcpy((void*) &ble_scan_rsp_data[BLE_ADDR_OFFSET], 
					(void*) &unique_ble_addr[0], BLE_ADDR_LEN);
#endif
	
	/* address type */
	adv_data_local[BLE_TYPE_OFFSET] &= BLE_ADDR_TYPE_MASK;
	adv_data_local[BLE_TYPE_OFFSET] |= (adv_params->own_address_type) << 6;
#if TS_SEND_SCAN_RSP
	ble_scan_rsp_data[BLE_TYPE_OFFSET] &= BLE_ADDR_TYPE_MASK;
	ble_scan_rsp_data[BLE_TYPE_OFFSET] |= (adv_params->own_address_type) << 6;
#endif
	
	/* whitelist */
	if (BTLE_ADV_FILTER_ALLOW_ANY 		== adv_params->filter_policy || 
			BTLE_ADV_FILTER_ALLOW_LEVEL2 	== adv_params->filter_policy)
	{
		wl_disable();
	}
	else
	{
		wl_enable();
	}

	/* Advertisement interval */
	adv_int_min = adv_params->interval_min;
	adv_int_max = adv_params->interval_max;

#if TS_SEND_SCAN_RSP	
	/* adv type */
	adv_data_local[BLE_TYPE_OFFSET] &= ~BLE_TYPE_MASK;
	adv_data_local[BLE_TYPE_OFFSET] |= (ble_adv_type_raw[adv_params->type] & BLE_TYPE_MASK);

	/* scan rsp type */
	ble_scan_rsp_data[BLE_TYPE_OFFSET] &= ~BLE_TYPE_MASK;
	ble_scan_rsp_data[BLE_TYPE_OFFSET] |= 0x04;
#else
	/* adv type is locked to nonconn */
	adv_data_local[BLE_TYPE_OFFSET] &= ~BLE_TYPE_MASK;
	adv_data_local[BLE_TYPE_OFFSET] |= (ble_adv_type_raw[BTLE_ADV_TYPE_NONCONN_IND] & BLE_TYPE_MASK);
#endif
	return true;
}

void ctrl_timeslot_order(void)
{
	sm_adv_run = true;
	timeslot_req_initial();
}

void ctrl_timeslot_abort(void)
{
	sm_adv_run = false;
}

bool ctrl_adv_data_set(btle_cmd_param_le_write_advertising_data_t* adv_data)
{	
	ASSERT(adv_data != NULL);
	
	/* length cannot exceed 31 bytes */
	uint8_t len = ((adv_data->data_length <= BLE_PAYLOAD_MAXLEN)? 
									adv_data->data_length : 
									BLE_PAYLOAD_MAXLEN);

	/* put into packet buffer */
	memcpy((void*) &adv_data_local[BLE_PAYLOAD_OFFSET], 
					(void*) &adv_data->advertising_data[0], len);

	/* set length of packet in length byte. Account for 6 address bytes */
	adv_data_local[BLE_SIZE_OFFSET] &= 0x00;
	adv_data_local[BLE_SIZE_OFFSET] = (BLE_ADDR_LEN + len);

	return true;
}

bool ctrl_scan_data_set(btle_cmd_param_le_write_scan_response_data_t* data)
{
#if TS_SEND_SCAN_RSP	
	ASSERT(data != NULL);

	/* length cannot exceed 31 bytes */
	uint8_t len = ((data->data_length <= BLE_PAYLOAD_MAXLEN)? 
									data->data_length : 
									BLE_PAYLOAD_MAXLEN);

	/* put into packet buffer */
	memcpy((void*) &ble_scan_rsp_data[BLE_PAYLOAD_OFFSET], 
					(void*) &data->response_data[0], len);

	/* set length of packet in length byte. Account for 3 header bytes
	* and 6 address bytes */
	ble_scan_rsp_data[BLE_SIZE_OFFSET] = (BLE_ADDR_LEN + len);
#endif	

	return true;
}
