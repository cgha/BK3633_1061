#include <stdint.h>
#include <string.h>

#include "ZenoRtc.h"

#include "Watch.h"
#include "app_task.h"
#include "ke_timer.h"
#include "rtc.h"
#include "uart.h"


static volatile RTC real_time;
static volatile unsigned char second;
static unsigned char MaxDayInMonth;
static uint8_t TimeIsBeChanged = 1;
extern uint8_t rtc_notify;
static uint8_t rtc_alive;
extern uint8_t IsFastConnect;
extern uint8_t NowConnIntvSlow;
extern uint16_t disconnectCnt;

//闰年及每月天数计算
unsigned char cal_day(uint16_t year, unsigned char month)
{
	switch(month)
	{
		case 1: return(31);//break;
		case 3: return(31);//break;
		case 5: return(31);//break;
		case 7: return(31);//break;
		case 8: return(31);//break;
		case 10: return(31);//break;
		case 12: return(31);//break;
		case 4: return(30);//break;
		case 6: return(30);//break;
		case 9: return(30);//break;
		case 11: return(30);//break;
		case 2: 
			if((((year%4) ==0) && ((year%100) !=0)) || ((year%400) ==0 ))
				return(29);
			else
				return(28);
			//break;
        default:
            break;
	}
    return 30;
}

/***************************************************************************************************
Func Name:RTC_AutoSetWeek
Desc: auto set week from day
In:
    *rtc_time --- point to a RTC struct
Out:void
***************************************************************************************************/
void RTC_AutoSetWeek(RTC *rtc_time)
{
    unsigned char month,day;
    uint16_t year;
  

	year = rtc_time->year;
	month = rtc_time->month;
	day = rtc_time->day;
    year += 2000;
    
    if( month == 1 || month == 2 )  
    {
        month += 12;
        if( year> 0 )
          year--;
        else
          year = 4;
    }

    rtc_time->week =  ( day + 2*month + 3*(month+1)/5 + year + year/4 ) %7;
} 

void RTC_AutoSetWeek_ForHexDate(RTC *rtc_time)
{
   unsigned char month,day;
	uint16_t year;
  
    year = ((rtc_time->year >> 4) & 0x0F) * 10 + (rtc_time->year & 0x0F);
    month = ((rtc_time->month >> 4) & 0x0F) * 10 + (rtc_time->month & 0x0F);
    day = ((rtc_time->day >> 4) & 0x0F) * 10 + (rtc_time->day & 0x0F);

    year += 2000;

    if( month == 1 || month == 2 )  
    {
      month += 12;
      if( year> 0 )
          year--;
      else
          year = 4;
    }

    rtc_time->week =  ( day + 2*month + 3*(month+1)/5 + year + year/4 ) %7;
} 
/***************************************************************************************************
Func Name:RTC_SetTime
Desc: set the time
In:
    *rtc_time --- point to a RTC struct
Out:void
***************************************************************************************************/
void RTC_SetTime(RTC *rtc_time)  
{   
	RTC_DATE_DESC RTC_date_desc;
	
    real_time.year = ((rtc_time->year >> 4) & 0x0F) * 10 + (rtc_time->year & 0x0F);
    real_time.month = ((rtc_time->month >> 4) & 0x0F) * 10 + (rtc_time->month & 0x0F);
    real_time.day = ((rtc_time->day >> 4) & 0x0F) * 10 + (rtc_time->day & 0x0F);
    real_time.hour = ((rtc_time->hour >> 4) & 0x0F) * 10 + (rtc_time->hour & 0x0F);
    real_time.min = ((rtc_time->min >> 4) & 0x0F) * 10 + (rtc_time->min & 0x0F);
    real_time.sec = ((rtc_time->sec >> 4) & 0x0F) * 10 + (rtc_time->sec & 0x0F);
	
	RTC_AutoSetWeek((RTC *)&real_time);
	
	RTC_date_desc.hour = real_time.hour;
	RTC_date_desc.minute = real_time.min;
	RTC_date_desc.second = real_time.sec;
	RTC_date_desc.week_day = real_time.week;
	rtc_init(&RTC_date_desc);
	UART_PRINTF("rtc desc:%x,%x,%x,%x\r\n",real_time.hour,real_time.min,real_time.sec,real_time.week);
	rtc_alive = 1;
}   

/***************************************************************************************************
Func Name:RTC_SetTime
Desc: set the time
In:
    *rtc_time --- point to a RTC struct
Out:void
***************************************************************************************************/
void RTC_SetTimeHex(RTC *rtc_time_hex)  
{   
	RTC_DATE_DESC RTC_date_desc;
	
    real_time.year = rtc_time_hex->year;
    real_time.month = rtc_time_hex->month;
    real_time.day = rtc_time_hex->day;
    real_time.hour = rtc_time_hex->hour;
    real_time.min = rtc_time_hex->min;
    real_time.sec = rtc_time_hex->sec;
	
	RTC_AutoSetWeek((RTC *)&real_time);
	
	RTC_date_desc.hour = real_time.hour;
	RTC_date_desc.minute = real_time.min;
	RTC_date_desc.second = real_time.sec;
	RTC_date_desc.week_day = real_time.week;
	rtc_init(&RTC_date_desc);
	rtc_alive = 1;
}

/***************************************************************************************************
Func Name:RTC_GetTime
Desc: get the time
In:
    *rtc_time --- point to a RTC struct
Out:void
***************************************************************************************************/
void RTC_GetTime(RTC *rtc_time)  
{  
	RTC_DATE_DESC RTC_date_desc;
	
	//RTC_AutoSetWeek((RTC *)&real_time);
	rtc_time->year = ((unsigned char) (((real_time.year ) / 10) << 4)) | ((real_time.year % 10) & 0x0F);
	rtc_time->month = ((unsigned char) (((real_time.month ) / 10) << 4)) | ((real_time.month % 10) & 0x0F);	
	rtc_time->day = ((unsigned char) (((real_time.day ) / 10) << 4)) | ((real_time.day % 10) & 0x0F);	
	
	rtc_get_time(&RTC_date_desc);
	rtc_time->hour = ((unsigned char) (((RTC_date_desc.hour ) / 10) << 4)) | ((RTC_date_desc.hour % 10) & 0x0F);	
	rtc_time->min = ((unsigned char) (((RTC_date_desc.minute ) / 10) << 4)) | ((RTC_date_desc.minute % 10) & 0x0F);		
	rtc_time->sec = ((unsigned char) (((RTC_date_desc.second ) / 10) << 4)) | ((RTC_date_desc.second % 10) & 0x0F);	
	rtc_time->week = RTC_date_desc.week_day;
}  

/*******************************************************************
Function: ZenoRtcGetTime_PowerSave
Desc: ????RTC??,???????,????
Input: RTC *rtc_time --- point to a RTC struct
Output: 
*******************************************************************/
void ZenoRtcGetTime_PowerSave(RTC *rtc_time)  
{  
    rtc_time->sec = ((unsigned char) (((real_time.sec ) / 10) << 4));
    rtc_time->sec = rtc_time->sec | ((real_time.sec % 10) & 0x0F);
    
    if(rtc_time->sec == 0x00)
    {
      rtc_time->hour = ((unsigned char) (((real_time.hour ) / 10) << 4));
      rtc_time->hour = rtc_time->hour | ((real_time.hour % 10) & 0x0F);	
      rtc_time->min = ((unsigned char) (((real_time.min ) / 10) << 4));
      rtc_time->min = rtc_time->min | ((real_time.min % 10) & 0x0F);	    
    }
	
	if(TimeIsBeChanged)
    {
      TimeIsBeChanged = 0;
      RTC_AutoSetWeek((RTC *)&real_time);
      
      rtc_time->year = ((unsigned char) (((real_time.year ) / 10) << 4));
      rtc_time->year = rtc_time->year | ((real_time.year % 10) & 0x0f);
      rtc_time->month = ((unsigned char) (((real_time.month ) / 10) << 4));
      rtc_time->month = rtc_time->month | ((real_time.month % 10) & 0x0F);	
      rtc_time->day = ((unsigned char) (((real_time.day ) / 10) << 4));
      rtc_time->day = rtc_time->day | ((real_time.day % 10) & 0x0F);	
      rtc_time->hour = ((unsigned char) (((real_time.hour ) / 10) << 4));
      rtc_time->hour = rtc_time->hour | ((real_time.hour % 10) & 0x0F);	
      rtc_time->min = ((unsigned char) (((real_time.min ) / 10) << 4));
      rtc_time->min = rtc_time->min | ((real_time.min % 10) & 0x0F);	      
      rtc_time->week = real_time.week;
    }
}  

void TH_AppTimerRTC(void * p_context)
{
	RTC rtc_time;
	RTC_DATE_DESC RTC_date_desc;
	
	rtc_get_time(&RTC_date_desc);
	
	rtc_notify = 0;
	if(real_time.sec != RTC_date_desc.second)
	{
		rtc_notify = RTC_NOTIFY_SEC;
		real_time.sec = RTC_date_desc.second;
	}
	if(real_time.min != RTC_date_desc.minute)
	{
		rtc_notify |= RTC_NOTIFY_MIN;
		real_time.min = RTC_date_desc.minute;
	}
	if(real_time.hour != RTC_date_desc.hour)
	{
		rtc_notify |= RTC_NOTIFY_HOUR;
		real_time.hour = RTC_date_desc.hour;
	}
	
	if(rtc_notify)
	{
		//UART_PRINTF("RTC %x,%x,%x, %x,%x,%x\r\n",real_time.hour,real_time.min,real_time.sec,IsFastConnect,NowConnIntvSlow,disconnectCnt);
		
		//rtc_notify |= RTC_NOTIFY_SEC;
		if((real_time.sec % 5) == 0)
		{
			rtc_notify |= RTC_NOTIFY_5SEC;
		}
	}

}

/*******************************************************************
Function: ZenoRtcAutoSetWeekForHex
Desc: ?RTC??????,???????,???????(????)
Input: RTC *rtc_time --- point to a RTC struct
Output: void
*******************************************************************/
void ZenoRtcAutoSetWeekForHex(RTC *rtc_time)
{
    unsigned char month,day;
	uint16_t year;
  
    year = ((rtc_time->year >> 4) & 0x0F) * 10 + (rtc_time->year & 0x0F);
    month = ((rtc_time->month >> 4) & 0x0F) * 10 + (rtc_time->month & 0x0F);
    day = ((rtc_time->day >> 4) & 0x0F) * 10 + (rtc_time->day & 0x0F);

    year += 2000;

    if( month == 1 || month == 2 )  
    {
      month += 12;
      if( year> 0 )
          year--;
      else
          year = 4;
    }

    rtc_time->week =  ( day + 2*month + 3*(month+1)/5 + year + year/4 ) %7;
} 



void RTC_init(void)
{	
	uint32_t err_code;
			 // Create RTC Timer
	ke_timer_set(APP_PERIOD_TIMER, TASK_APP, 100);
}

void RTC_Start(void)
{	
	//StartTimer(TIMER_ID_RTC,1000);
}

void RTC_Stop(void)
{
	//StopTimer(TIMER_ID_RTC);
	rtc_disable();
	rtc_alive = 0;
}

uint8_t RTC_IsAlive(void)
{
	return rtc_alive;
}
