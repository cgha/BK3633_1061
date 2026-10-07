#ifndef _ZENO_RTC_H
#define _ZENO_RTC_H

#include "app.h"

typedef __packed struct
{
  unsigned char first_addr;
  unsigned char sec;
  unsigned char min;
  unsigned char hour;
  unsigned char day;
  unsigned char week;
  unsigned char month;
  unsigned char year; 
}RTC;

typedef void (*CR_CBack_SecUpdate) (void);    
typedef void (*CR_CBack_MinUpdate) (void);
typedef void (*CR_CBack_HourUpdate) (void);    
typedef void (*CR_CBack_DayUpdate) (void);

void RTC_AutoSetWeek(RTC *rtc_time);
void RTC_AutoSetWeek_ForHexDate(RTC *rtc_time);
void RTC_SetTime(RTC *rtc_time);
void RTC_SetTimeHex(RTC *rtc_time_hex);
void RTC_GetTime(RTC *rtc_time);
void RTC_init(void);
void RTC_Start(void);
void RTC_Stop(void);
uint8_t RTC_IsAlive(void);
void ZenoRtcAutoSetWeekForHex(RTC *rtc_time);
unsigned char cal_day(uint16_t year, unsigned char month);
void ZenoRtcGetTime_PowerSave(RTC *rtc_time);
void TH_AppTimerRTC(void * p_context);
#endif
