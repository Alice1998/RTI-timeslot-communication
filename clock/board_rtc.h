#ifndef _BOARD_RTC_H_
#define _BOARD_RTC_H_

/*********************************************************************
 * INCLUDES
 */
#include "common.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern volatile uint32_t g_timestamp;

/*********************************************************************
 * API FUNCTIONS
 */
void RTC_Init(void);
void RTC_SetTime(uint32_t timestampNow);
uint32_t RTC_GetTime(void);

#endif /* _BOARD_RTC_H_ */
