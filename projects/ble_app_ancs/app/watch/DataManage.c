/**************************************************************************************************
Filename:       DataManage.c
Revised Date: 2012-4-25
Revision: V1
Description:    This file contains G-Sensor Read and write
Creator: Zeno Zhang

Copyright: Signal electronic technology LTD.
Http://www.signal-etech.com
****************************************************************************************************/
#define DATA_MANAGE_INTERNAL  1
#include <string.h>
#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "flash.h"
#include "DataManage.h"
#include "uart.h"

extern SPORT_PERIOD SportPeriod;
extern SLEEP_PERIOD SleepPeriod;
extern uint16_t HR_LogStartMin,HR_LogEndMin;
extern uint16_t SleepStartMin,SleepEndMin;
const unsigned char EEPROM_FORMAT[6] = {0x55,0xAA,0x5A,0xAc,0x05,0x02/*0x58*/};
//const unsigned char BT_NAME[14] = {'J','o','i','n','t','-','S','t','y','l','e','-','C','H'};
const unsigned char BT_NAME[14] = {'E','V','E','R','L','A','S','T',' ',' ',' ',' ',' ',' '};
//const unsigned char BT_NAME[14] = {'B','l','u','e',' ','T','o','o','t','h',' ',' ',' ',' '};
const unsigned char PRODUCT_INFO[5] = {0x01,100,0x14,0x03,0x01};



//__align(4) DAY_RECORD_NODATA DayRecordNodata;

static uint32_t DayRecordMask;
static unsigned char CurDayIndex;
static SAVE_TIME LastSaveTime;



#define SETTING_DATA_PAGE_BASE	0
#define SPORT_DATA_PAGE_BASE	1

extern SPORT_PERIOD SportPeriod;

extern RTC RTC_CurTime;


extern uint8_t distanceMode;

uint8_t Setting_Is_Changed;


extern DAY_RECORD TodayRecord;
extern DAY_RECORD DayRecord;
extern SLEEP_DATA tempSleepData[96];
extern ALARM AlarmSave[ALARM_NUM_MAX];
extern DAY_RECORD SaveDayRecord[15];
extern PERSON PersonInfo;

__align(4) EE_SETTING_T eepromSettingRam;


//uint8_t pageBuff[1024];


#define PAGE_SIZE_BYTES 4096

#define EEPROM_BASE_PAGE	(320-16) //Total 320 page for main flash
#define EEPROM_BASE_ADDR	(0x43000) //0x42000~0x43000 is for bond info


#define IS_VALID_OP_ADDRESS(ADDR) (((ADDR) >= EEPROM_BASE_ADDR) && ((ADDR) <= (0x53000)))

extern void power_manage(void);

void RamSetting_Save(void * ramSettingBuf, void *srcBuf,uint8_t len)
{
	memcpy(ramSettingBuf,srcBuf,len);
	Setting_Is_Changed = 1;
}

uint8_t FlashSetting_Save(uint8_t *srcBuf, uint16_t byteCount)
{
	return 1;//EEPROM_W_Reg_Multi(SETTING_DATA_PAGE_BASE,0,byteCount,srcBuf);
}

uint8_t FlashSetting_Load(uint8_t *dstBuf, uint16_t byteCount)
{
	return 1;//EEPROM_R_Reg_Multi(SETTING_DATA_PAGE_BASE,0,byteCount,dstBuf);
}



void DM_SetProductInfo(unsigned char *pName,unsigned char Len)
{
	RamSetting_Save(&eepromSettingRam.productInfo,pName,Len);
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
}

void DM_GetProductInfo(unsigned char *pName,unsigned char Len)
{
	memcpy(pName, &eepromSettingRam.productInfo, Len);
}


/***************************************************************************************************
Func Name:DM_SetSportPeriod
Desc:Save the sport period to EEPROM
In: pSportPeriod -->½á¹¹Ö¸Õë
Out:void
***************************************************************************************************/
void DM_SetSleepPeriod(SLEEP_PERIOD *pSleepPeriod)
{

	RamSetting_Save(&eepromSettingRam.sleepPeriod ,pSleepPeriod,sizeof(SLEEP_PERIOD));
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam)); 
    SleepStartMin =(((((pSleepPeriod->StartHour & 0xf0) >> 4) & 0x0F) * 10) + (pSleepPeriod->StartHour & 0x0f)) * 60 + \
		(((((pSleepPeriod->StartMin & 0xf0) >> 4)) & 0x0F) * 10) + (pSleepPeriod->StartMin & 0x0f);

	SleepEndMin =(((((pSleepPeriod->EndHour & 0xf0) >> 4) & 0x0F) * 10) + (pSleepPeriod->EndHour & 0x0f)) * 60 + \
		(((((pSleepPeriod->EndMin & 0xf0) >> 4)) & 0x0F) * 10) + (pSleepPeriod->EndMin & 0x0f);     
}

void DM_FormatSleepPeriod(void)
{
	SleepPeriod.StartHour = 0x22;
	SleepPeriod.StartMin = 0x00;
	SleepPeriod.EndHour = 0x07;
	SleepPeriod.EndMin = 0x00;  

	DM_SetSleepPeriod(&SleepPeriod);
}

/***************************************************************************************************
Func Name:DM_GetSleepPeriod
Desc:Get the sleep period from EEPROM
In: pSleepPeriod -->½á¹¹Ö¸Õë
Out:void
***************************************************************************************************/
void DM_GetSleepPeriod(SLEEP_PERIOD *pSleepPeriod)
{ 
	memcpy(pSleepPeriod,&eepromSettingRam.sleepPeriod ,sizeof(SLEEP_PERIOD));
	SleepStartMin =(((((pSleepPeriod->StartHour & 0xf0) >> 4) & 0x0F) * 10) + (pSleepPeriod->StartHour & 0x0f)) * 60 + \
		(((((pSleepPeriod->StartMin & 0xf0) >> 4)) & 0x0F) * 10) + (pSleepPeriod->StartMin & 0x0f);
	SleepEndMin =(((((pSleepPeriod->EndHour & 0xf0) >> 4) & 0x0F) * 10) + (pSleepPeriod->EndHour & 0x0f)) * 60 + \
		(((((pSleepPeriod->EndMin & 0xf0) >> 4)) & 0x0F) * 10) + (pSleepPeriod->EndMin & 0x0f); 
}

/***************************************************************************************************
Func Name:DM_SetAlarm
Desc:Save the alarm info to EEPROM
In: ALARM *pAlarm -->½á¹¹Ö¸Õë£¬Num-->Alarm number
Out:void
***************************************************************************************************/
void DM_SetAlarm(ALARM *pAlarm,unsigned char Num)
{
	if(Num < ALARM_NUM_MAX)
	{
		RamSetting_Save(&eepromSettingRam.alarmInfo[Num],pAlarm,sizeof(ALARM));
		//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
	}
}

void DM_FormatAlarm(void)
{
	ALARM alarm;
	unsigned char i;

	alarm.IsEn = 0;
	alarm.Hour = 0x08;
	alarm.Min = 0x00;
	alarm.Week = 0x3E;

	for(i = 0; i < ALARM_NUM_MAX; i++)
	{
		alarm.Num = i;
		RamSetting_Save(&eepromSettingRam.alarmInfo[i],(unsigned char *)&alarm,sizeof(ALARM));
		//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
		//bros1 EEPROM_W_Reg_Multi(EEPROM_ADDR_ALARM_DATA + sizeof(ALARM)* i,sizeof(ALARM),(unsigned char *)&alarm);
	}

}

/***************************************************************************************************
Func Name:DM_GetAlarm
Desc:Get the alarm info from EEPROM
In: ALARM *pAlarm -->½á¹¹Ö¸Õë£¬Num-->Alarm number
Out:void
***************************************************************************************************/
void DM_GetAlarm(ALARM *pAlarm,unsigned char Num)
{ 

	if(Num < ALARM_NUM_MAX)
	{
		memcpy(pAlarm, &eepromSettingRam.alarmInfo[Num], sizeof(ALARM));
	}
}


/***************************************************************************************************
Func Name:DM_SetSportPeriod
Desc:Save the sport period to EEPROM
In: pSportPeriod -->½á¹¹Ö¸Õë
Out:void
***************************************************************************************************/
void DM_SetSportPeriod(SPORT_PERIOD *pSportPeriod)
{

	RamSetting_Save(&eepromSettingRam.sportPeriod ,pSportPeriod,sizeof(SPORT_PERIOD));
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam)); 
    
}

void DM_FormatSportPeriod(void)
{
	SPORT_PERIOD sport_period;

	sport_period.StartHour = 0x06;
	sport_period.StartMin = 0x00;
	sport_period.EndHour = 0x07;
	sport_period.EndMin = 0x00;  
	sport_period.WeekEn = 0x3f;
	sport_period.VibPeriod = 45;
	DM_SetSportPeriod(&sport_period);
}

/***************************************************************************************************
Func Name:DM_GetSportPeriod
Desc:Get the sport period from EEPROM
In: pSportPeriod -->½á¹¹Ö¸Õë
Out:void
***************************************************************************************************/
void DM_GetSportPeriod(SPORT_PERIOD *pSportPeriod)
{ 
	memcpy(pSportPeriod,&eepromSettingRam.sportPeriod ,sizeof(SPORT_PERIOD));    
}

/***************************************************************************************************
Func Name:DM_SetActiveCode
Desc:Save the active code to EEPROM
In: pAicticeCode--> point to active code buffer
len--> the length of active code
Out:void
***************************************************************************************************/
void DM_SetActiveCode(unsigned char * pAicticeCode,unsigned char len)
{
	RamSetting_Save(eepromSettingRam.activeCode,pAicticeCode,len);
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam)); 
}


/***************************************************************************************************
Func Name:DM_GetActiveCode
Desc:read out the active code from EEPROM
In: pAicticeCode--> point to active code buffer that read out
len--> the length of active code
Out:void
***************************************************************************************************/
void DM_GetActiveCode(unsigned char * pAicticeCode,unsigned char len)
{
	memcpy(pAicticeCode,eepromSettingRam.activeCode ,len);
}

/***************************************************************************************************
Func Name:DM_SetDevID
Desc:Save the Devicve ID to EEPROM
In: pDevID--> point to device ID buffer
len--> the length of device ID 
Out:void
***************************************************************************************************/
void DM_SetDevID(unsigned char * pDevID,unsigned char len)
{
	RamSetting_Save(eepromSettingRam.devID,pDevID,len);
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam)); 
}

/***************************************************************************************************
Func Name:DM_GetDevID
Desc:read out the Devicve ID from EEPROM
In: pDevID--> point to device ID buffer that read out
len--> the length of device ID 
Out:void
***************************************************************************************************/
void DM_GetDevID(unsigned char * pDevID,unsigned char len)
{
	memcpy(pDevID,eepromSettingRam.devID,len);
}

/***************************************************************************************************
Func Name:DM_SetPerson
Desc:set the person information
In: pPerson --> point to the person information you want to set
Out: void
***************************************************************************************************/
void DM_SetPerson(PERSON *pPerson)
{
	RamSetting_Save((uint8_t*)&eepromSettingRam.personInfo,(uint8_t*)pPerson,sizeof(PERSON));
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
}

/***************************************************************************************************
Func Name:DM_GetPerson
Desc:get the person information
In: pPerson --> point to the person information you want read out
Out: void
***************************************************************************************************/
void DM_GetPerson(PERSON *pPerson)
{
	memcpy((uint8_t*)pPerson,(uint8_t*)&eepromSettingRam.personInfo,sizeof(PERSON));
}



/***************************************************************************************************
Func Name:DM_IsFormat
Desc:Check the EEPROM is had formated
In: void
Out:
= 1 is Formated; = 0 is not
***************************************************************************************************/
unsigned char DM_IsFormat(void)
{
	unsigned char head[sizeof(EEPROM_FORMAT)],IsFormat,i;
	//unsigned char P0IEN_Save;

	IsFormat = 1;

	//  P0IEN_Save = P0IEN;
	// RTC_InterruptEn(0);
	//bros1 EEPROM_R_Reg_Multi(EEPROM_ADDR_HEAD,sizeof(head),head);
	// P0IEN = P0IEN_Save;

	memcpy(head,eepromSettingRam.head,sizeof(head));

	for(i = 0; i < sizeof(EEPROM_FORMAT); i++)
	{
		if(head[i] != EEPROM_FORMAT[i])
		{
			IsFormat = 0;
			break;
		}
	}

	return IsFormat;
}

/***************************************************************************************************
Func Name:DM_Format
Desc:format the EEPROM
In: void
Out: void
***************************************************************************************************/
void DM_Format(void)
{
	unsigned char i;
	PERSON person;
//	DAY_RECORD_HEAD Head;
	RTC rtc;
	ALARM alarm;

	//initi Day record data
	DayRecordMask = 0;
	eepromSettingRam.dayRecordMask = 0;
	//bros1 EEPROM_W_Reg_Multi(EEPROM_ADDR_DAY_IS_RECORD,sizeof(DayRecordMask),(unsigned char *)&DayRecordMask); 
	//initi day index
	CurDayIndex = 0;
	eepromSettingRam.curDayIndex = 0;
	//bros1 EEPROM_W_Reg_Byte(EEPROM_ADDR_CUR_DAY_INDEX,CurDayIndex);
	
	
	//initi the new day data
	/*for(i = 0; i < RECORD_NUM_MAX; i++)
	{
		Head.Year = 0x00;
		Head.Month = 0x01;
		Head.Day = (((i+1) / 10) << 4) & 0xF0; 
		Head.Day = Head.Day | ((i+1) % 10);
		//bros EEPROM_W_Reg_Multi(i * DATA_LENG_DAY + EEPROM_ADDR_DATA,sizeof(DAY_RECORD_HEAD),(unsigned char *)&Head);  
	}*/
   
	rtc.first_addr = 0x02;
	rtc.sec= 0;
	rtc.min =0x0;
	rtc.hour =0x0;
	rtc.day = 0x01;
	rtc.week = 2;
	rtc.month = 0x01;
	rtc.year = 0x19; 		
		//DM_SetSaveTime((unsigned char *)&rtc);

	//initi person info
	PersonInfo.Sex = 0;
	PersonInfo.Age = 26;
	PersonInfo.Stature = 175;
	PersonInfo.Weight = 70;
	PersonInfo.StepLength = 60;
	PersonInfo.TargetSteps = 10000;  
	PersonInfo.TargetType = 0;
	//person.TargetSteps = 20;  
	PersonInfo.DistUnit = 1;
	PersonInfo.TimeDispMode = 1; //0= 12 ,1 = 24
	PersonInfo.ANCS_Mask = 0x03;

	RamSetting_Save(&eepromSettingRam.personInfo,&PersonInfo,sizeof(PERSON));
	//RamSetting_Save(&eepromSettingRam.sportPeriod, &SportPeriod, sizeof(SportPeriod));

	SleepPeriod.en = 0x00;
	SleepPeriod.StartHour = 0x22;
	SleepPeriod.StartMin = 0x00;
	SleepPeriod.EndHour = 0x07;
	SleepPeriod.EndMin = 0x00;  
	memcpy(&eepromSettingRam.sleepPeriod ,&SleepPeriod,sizeof(SLEEP_PERIOD));
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam)); 

	SleepStartMin =((((((SleepPeriod.StartHour & 0xf0) >> 4)) & 0x0F) * 10) + (SleepPeriod.StartHour & 0x0f)) * 60 + \
		((((SleepPeriod.StartMin & 0xf0) >> 4) & 0x0F) * 10) + (SleepPeriod.StartMin & 0x0f);

	SleepEndMin =(((((SleepPeriod.EndHour & 0xf0) >> 4) & 0x0F) * 10) + (SleepPeriod.EndHour & 0x0f)) * 60 + \
		((((SleepPeriod.EndMin & 0xf0) >> 4) & 0x0F) * 10) + (SleepPeriod.EndMin & 0x0f);
	//RamSetting_Save(&eepromSettingRam.sleepPeriod ,&SleepPeriod,sizeof(SLEEP_PERIOD));

	//DM_SetPerson(&person);
	//DM_FormatSportPeriod();
	//DM_SetKarmaUnit(390);
	//DM_SetDispMode(1);
	//DM_FormatAlarm();
	alarm.IsEn = 0;
	alarm.Hour = 0x08;
	alarm.Min = 0x00;
	alarm.Week = 0x3E;
	for(i = 0; i < ALARM_NUM_MAX; i++)
	{
		alarm.Num = i;
		memcpy(&AlarmSave[i],(unsigned char *)&alarm,sizeof(ALARM));
		//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
		//bros1 EEPROM_W_Reg_Multi(EEPROM_ADDR_ALARM_DATA + sizeof(ALARM)* i,sizeof(ALARM),(unsigned char *)&alarm);
	}

	//  DM_GetBT_Name((unsigned char *)&BtName,14);
	//format head
	//bros1 EEPROM_W_Reg_Multi(EEPROM_ADDR_HEAD,sizeof(EEPROM_FORMAT),(unsigned char *)&EEPROM_FORMAT);  

	//RamSetting_Save(&eepromSettingRam.head[0],(uint8_t*)EEPROM_FORMAT,sizeof(EEPROM_FORMAT));
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
}


/***************************************************************************************************
Func Name:DM_Initi
Desc:data manage intiti
In: void
Out: void
***************************************************************************************************/
void DM_Initi(void)
{
//	unsigned char ResetCnt;
	DM_Format();	
}



void DM_SaveDataIniti(SAVE_TIME *pSaveTime)
{
	/*__align(4)*/ DAY_RECORD_NODATA DayRecordNodata;
	unsigned char i;
//	unsigned char *pAddr;
//	unsigned char temp[9];
	//unsigned char P0IEN_Save;
	//initi the new day data
	DayRecordNodata.Head.Year = pSaveTime->Year;
	DayRecordNodata.Head.Month = pSaveTime->Month;
	DayRecordNodata.Head.Day = pSaveTime->Day; 

	//DayRecordNodata.SportDataTotal.StepsTotal = 0;
	//DayRecordNodata.SportDataTotal.RunStepsTotal = 0;
	//DayRecordNodata.SportDataTotal.KllTotal = 0;
	//DayRecordNodata.SportDataTotal.DistTotal = 0;
	//DayRecordNodata.SportDataTotal.SportTimeMinTotal = 0;
	//DayRecordNodata.SportDataTotal.TargetCompleteRate = 0;
//	DayRecordNodata.SportDataTotal.Speed = 0;
//	DayRecordNodata.SportDataTotal.Ex = 0;  

	//for(i = 0; i < sizeof(DayRecordNodata.SportHead); i++)
	//	DayRecordNodata.SportHead[i] = 0;
	memset(DayRecordNodata.SportHead,0,sizeof(DayRecordNodata.SportHead));
	memset(&DayRecordNodata.SportDataTotal,0,sizeof(DayRecordNodata.SportDataTotal));

	//initi data update to EEPROM
	//bros1 EEPROM_W_Reg_Multi(CurDayIndex * DATA_LENG_DAY + EEPROM_ADDR_DATA,\
	sizeof(DayRecordNodata),\
	(unsigned char *)&DayRecordNodata);
	//EEPROM_W_Reg_Multi(CurDayIndex+SPORT_DATA_PAGE_BASE, 0, sizeof(DayRecordNodata),(unsigned char *)&DayRecordNodata);
	memcpy(&SaveDayRecord[CurDayIndex],(unsigned char *)&DayRecordNodata,sizeof(DAY_RECORD_NODATA));

}

/***************************************************************************************************
Func Name:DM_NewDay
Desc:New day data initi
In: DayOffset --> offset days with last save
pSaveTime --> point to current save time
Out:void
***************************************************************************************************/
void DM_NewDay(SAVE_TIME *pSaveTime)
{
	if(DM_IsThisDaySaved(0))
	{
		if(++CurDayIndex >= RECORD_NUM_MAX)
			CurDayIndex = 0;  
		DM_DelDayRecord(0);
		DM_SaveDataIniti(pSaveTime);     
	}

}

//add on 20181010
/******************************************************************
Func Name:DM_SaveDataOneDay
Desc:将一天的数据全部写入
In:
Out:void
*******************************************************************/
void DM_SaveDataOneDay(SAVE_TIME *pSaveTime,DAY_RECORD *pDayRecord,unsigned char *p_save_time)
{
//    uint16_t DataSaveAddr;
//    unsigned char i;
    unsigned char *pAddr;
    unsigned char temp[9];
    uint32_t BitMask; 
	//uint8_t dstPage;
	uint8_t saveDayIndex = CurDayIndex;
    
	//dstPage = CurDayIndex+SPORT_DATA_PAGE_BASE;
    //计算对应的存储地址
    //DataSaveAddr = EEPROM_ADDR_DATA + CurDayIndex * DATA_LENG_DAY;   
    //标记已存储日标志
    /*BitMask = 0x00000001;
    for(i = 0; i < CurDayIndex; i++)
    {
      BitMask <<= 1;
      BitMask &= 0x3FFFFFFE;
    }*/
	BitMask = 1<<CurDayIndex;
    DayRecordMask |= BitMask;      
    
    //保存时刻
     //save the time
    
    //开新的一天
    if(++CurDayIndex >= RECORD_NUM_MAX)
      CurDayIndex = 0;  
    DM_DelDayRecord(0);

    //将日标志，天索引，保存时间，写到设置数据缓冲区
    //pAddr = (unsigned char *)&DayRecordMask;
    //temp[0] = *pAddr++;
    //temp[1] = *pAddr++;
    //temp[2] = *pAddr++;
    //temp[3] = *pAddr++;  
    //temp[4] = CurDayIndex;  
    //temp[5] = LastSaveTime.Year;
    //temp[6] = LastSaveTime.Month;
    //temp[7] = LastSaveTime.Day;
    //temp[8] = LastSaveTime.QuarterIndex;
    //save the DayRecordMask;
    //EEPROM_W_Reg_Multi(EEPROM_ADDR_DAY_IS_RECORD,9,temp); 
	//memcpy((uint8_t*)&eepromSettingRam.dayRecordMask,temp,9);

    //将当前的时间写到设置缓冲区
    //DM_SetSaveTime(p_save_time);
    
    //将一天的运动数据，及数据缓冲区的数据，一并写入到Flash中保存
    //EEPROM_W_Block(DataSaveAddr,DATA_LENG_DAY,(unsigned char *)pDayRecord);
	memcpy(&SaveDayRecord[saveDayIndex],(unsigned char *)pDayRecord,sizeof(TodayRecord));
	UART_PRINTF("saveday record%x,%x,%x,curDayIndex:%x\r\n",TodayRecord.Head.Year,TodayRecord.Head.Month,TodayRecord.Head.Day,CurDayIndex);
	UART_PRINTF("saved record%x,%x,%x\r\n",SaveDayRecord[saveDayIndex].Head.Year,SaveDayRecord[saveDayIndex].Head.Month,SaveDayRecord[saveDayIndex].Head.Day);
}
//end add

void DM_SetLastSaveTime(unsigned char *save_time)
{
	RamSetting_Save((uint8_t*)&eepromSettingRam.lastSaveTime,save_time,sizeof(SAVE_TIME));
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
}

void DM_GetLastSaveTime(unsigned char *last_save_time)
{
	//bros1 EEPROM_R_Reg_Multi(EEPROM_ADDR_LAST_SAVE_TIME,sizeof(SAVE_TIME),last_save_time);
	memcpy(last_save_time,(uint8_t*)&eepromSettingRam.lastSaveTime,sizeof(SAVE_TIME));
}

void DM_SetSaveTime(unsigned char *save_time)
{
	//bros1 EEPROM_W_Reg_Multi(EEPROM_ADDR_SAVE_TIME,sizeof(RTC),save_time);
	RamSetting_Save((uint8_t*)&eepromSettingRam.saveTime,save_time,sizeof(RTC));
	//FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
}
void DM_GetSaveTime(unsigned char *save_time)
{
	//bros1 EEPROM_R_Reg_Multi(EEPROM_ADDR_SAVE_TIME,sizeof(RTC),save_time);
	memcpy(save_time,(uint8_t*)&eepromSettingRam.saveTime,sizeof(RTC));
}
/***************************************************************************************************
Func Name:DM_DelDayRecord
Desc: delete some day record
In: DayNum =0 is today, =1 is last day
Out: =1 is delete succeed; =0 is not
***************************************************************************************************/
unsigned char DM_DelDayRecord(unsigned char DayNum)
{
	unsigned char DayIndex;
	uint32_t BitMask = 0x00000001;

	if(DayNum < RECORD_NUM_MAX)
	{
		if(CurDayIndex >= DayNum)
			DayIndex = CurDayIndex - DayNum;
		else
			DayIndex = RECORD_NUM_MAX - (DayNum - CurDayIndex);

		/*for(i = 0; i < DayIndex; i++)
		{
			BitMask <<= 1;
			BitMask &= 0x3FFFFFFE;
		}*/
		BitMask = 1<< DayIndex;
		DayRecordMask &= ~BitMask;
		return 1;
	}
	else
		return 0;
}

/***************************************************************************************************
Func Name:DM_IsThisDaySaved
Desc: Check if thies day have record sport data
In: DayNum =0 is today, =1 is last day
Out: =1 is saved, =0 is not
***************************************************************************************************/
unsigned char DM_IsThisDaySaved(unsigned char DayNum)
{
	unsigned char DayIndex;
	uint32_t BitMask = 0x00000001;

	if(DayNum < RECORD_NUM_MAX)
	{
		if(CurDayIndex >= DayNum)
			DayIndex = CurDayIndex - DayNum;
		else
			DayIndex = RECORD_NUM_MAX - (DayNum - CurDayIndex);

		/*for(i = 0; i < DayIndex; i++)
		{
			BitMask <<= 1;
			BitMask &= 0x3FFFFFFE;
		}*/
		BitMask = 1<< DayIndex;

		if((DayRecordMask & BitMask) != 0)
			return 1;
		else
			return 0;
	}
	return 0;
}

/***************************************************************************************************
Func Name:DM_GetThisDaySportData
Desc:read out this day saport data from EEPROM
In:
DayNum =0 is today, =1 is last day

Out:void
***************************************************************************************************/
void DM_GetThisDaySportData(unsigned char DayNum, unsigned char *pDataRecord)
{    
	uint16_t DataLeng;
	unsigned char DayIndex;

	if(DayNum < RECORD_NUM_MAX)
	{
		DataLeng = DATA_LENG_DAY;
		if(CurDayIndex >= DayNum)
			DayIndex = CurDayIndex - DayNum;
		else
			DayIndex = RECORD_NUM_MAX - (DayNum - CurDayIndex);

		//EEPROM_R_Reg_Multi(DayIndex+SPORT_DATA_PAGE_BASE, 0,DataLeng,pDataRecord);
		memcpy(pDataRecord, &SaveDayRecord[DayIndex],DataLeng);
	}
}


void DM_SetThisDaySportData(unsigned char DayNum, unsigned char *pDataRecord)
{    
	uint16_t DataLeng;
	unsigned char DayIndex;

	if(DayNum < RECORD_NUM_MAX)
	{
		DataLeng = DATA_LENG_DAY;
		if(CurDayIndex >= DayNum)
			DayIndex = CurDayIndex - DayNum;
		else
			DayIndex = RECORD_NUM_MAX - (DayNum - CurDayIndex);

		//EEPROM_W_Reg_Multi(DayIndex+SPORT_DATA_PAGE_BASE, 0,DataLeng,pDataRecord);
		memcpy(&SaveDayRecord[DayIndex], pDataRecord, DataLeng);
	}
}

/***************************************************************************************************
Func Name:GetDayRecordMask
Desc:Get day record bit information
In: void
Out:DayRecordMask
***************************************************************************************************/
uint32_t GetDayRecordMask(void)
{
	uint32_t DayRecordMaskByTime,TempL;
	unsigned char i;

	DayRecordMaskByTime = 0;

	for(i = 0; i < RECORD_NUM_MAX; i++)
	{
		if(DM_IsThisDaySaved(i))
		{
			TempL = 0x00000001;
			TempL <<= i;
			DayRecordMaskByTime |= TempL;
		}
	}

	return DayRecordMaskByTime;
}



void DM_SaveSleepData(SLEEP_TIME* sleepEnterTime,SLEEP_TIME* sleepExitTime)
{
	uint8_t tempDayIndex = 0,j,k = 0,acrossDay = 0;
	uint8_t index = 0,offset = 0, bitMask = 0;
	
	if(sleepExitTime->day != sleepEnterTime->day)
		acrossDay = 1;
	
	uint8_t enterQuarterIndex = (((sleepEnterTime->hour >> 4) & 0x0F) * 10 + (sleepEnterTime->hour & 0x0F)) * 4 \
			+ ((((sleepEnterTime->min >> 4) & 0x0F) * 10 + (sleepEnterTime->min & 0x0F)) / 15);
		/*if(enterQuarterIndex == 0)
			enterQuarterIndex = 95;
		else
			enterQuarterIndex -= 1;*/
	uint8_t exitQuarterIndex = (((sleepExitTime->hour >> 4) & 0x0F) * 10 + (sleepExitTime->hour & 0x0F)) * 4 \
			+ ((((sleepExitTime->min >> 4) & 0x0F) * 10 + (sleepExitTime->min & 0x0F)) / 15);
		/*if(exitQuarterIndex == 0)
			exitQuarterIndex = 95;
		else
			exitQuarterIndex -= 1;*/
	
	if(acrossDay)
	{
		DM_GetThisDaySportData(1,(unsigned char *)&DayRecord);
		
		index = enterQuarterIndex / 8;
		offset = enterQuarterIndex % 8;	
		
		for(uint8_t i = enterQuarterIndex;i<96;i++)
		{
			bitMask = 1<<offset;
			DayRecord.SportHead[index] |= bitMask;
			offset++;
			if(offset>=8)
			{
				offset = 0;
				index++;
			}
			for(j= 0; j<sizeof(SLEEP_DATA); j++){
				DayRecord.SportData[i].sleepQ_Data[j] = tempSleepData[k].sleepData[j];
			}
			k++;
		}
		
		DM_SetThisDaySportData(1,(unsigned char *)&DayRecord);
		
		index = 0x00;
		offset = 0x00;
		
		//DM_GetThisDaySportData(0,(unsigned char *)&DayRecord);	
		//memcpy((unsigned char *)&DayRecord,(unsigned char *)&TodayRecord,sizeof(DayRecord));
		for(uint8_t i = 0;i<=exitQuarterIndex;i++)
		{
			bitMask = 1<<offset;
			TodayRecord.SportHead[index] |= bitMask;
			offset++;
			if(offset>=8)
			{
				offset = 0;
				index++;
			}
			for(j= 0; j<sizeof(SLEEP_DATA); j++){
				TodayRecord.SportData[i].sleepQ_Data[j] = tempSleepData[k].sleepData[j];
			}
			k++;
		}
		
		//DM_SetThisDaySportData(0,(unsigned char *)&DayRecord);
	}
	else{
		//DM_GetThisDaySportData(0,(unsigned char *)&DayRecord);
		//memcpy((unsigned char *)&DayRecord,(unsigned char *)&TodayRecord,sizeof(DayRecord));
		
		index = enterQuarterIndex / 8;
		offset = enterQuarterIndex % 8;
		
		for(uint8_t i = enterQuarterIndex;i<=exitQuarterIndex;i++)
		{
			bitMask = 1<<offset;
			TodayRecord.SportHead[index] |= bitMask;
			offset++;
			if(offset>=8)
			{
				offset = 0;
				index++;
			}
			for(j= 0; j<sizeof(SLEEP_DATA); j++){
				TodayRecord.SportData[i].sleepQ_Data[j] = tempSleepData[k].sleepData[j];
			}
			k++;
		}
		
		//DM_SetThisDaySportData(0,(unsigned char *)&DayRecord);
	}
}

/*******************************************************************
Func Name:EEPROM_Event
Desc: check the EEPROM if need to save each 5second
In: void
    
Out:  void
*******************************************************************/
void EEPROM_Setting_SaveEvent(void)
{
  if(Setting_Is_Changed)
  {
	  UART_PRINTF("write to flash\r\n");
      // EEPROM_SAVE();
	FlashSetting_Save((uint8_t*)&eepromSettingRam,sizeof(eepromSettingRam));
    Setting_Is_Changed = 0;
  }
}



static uint32_t address_of_page(uint16_t page_num)
{
    return (EEPROM_BASE_ADDR + (page_num * PAGE_SIZE_BYTES));
}

/*******************************************************************
Function: EEPROM_W_Reg_Multi
Desc: EEPROM write multi bytes
Input: addr --- the dst address offset in page,the addr must be 4 bytes aligned
leng --- the length of the bytes to be written
*data_buf --- the src buff that to be written
Output: 1(sucess) 0(fail)
Note: it will not read out the data content first, so assume the old data in page to be written has been copied out or it is useless
*******************************************************************/
uint8_t EEPROM_W_Reg_Multi(uint8_t pageNum, uint16_t addr ,uint16_t leng,unsigned char *data_buf) 
{
	//uint16_t need_erase = 0;
	//uint8_t alignedWordLen;

//	uint8_t ret;

	uint32_t writeAddr = address_of_page(pageNum)+addr;
	UART_PRINTF("wr addr %x, len:%x\r\n",writeAddr,leng);

	if(IS_VALID_OP_ADDRESS(writeAddr) /*&& leng < FLASH_PAGE_SIZE */){
		#if 0
		if(memcmp((uint8_t*)writeAddr,data_buf,leng) == 0) //same
		{
			return 1;
		}
		else
		#endif
		{
			UART_PRINTF("erase page addr%x\r\n",(uint32_t)address_of_page(pageNum)/4);

			flash_erase(flash_env.space_type,address_of_page(pageNum),FLASH_ERASE_SECTOR_SIZE,NULL);
			flash_write(flash_env.space_type,address_of_page(pageNum),leng,data_buf,NULL);
			
			return 1;
		}
	}
	else
	{
		if(leng >= FLASH_PAGE_SIZE)
			UART_PRINTF("len error\r\n");
		else
			UART_PRINTF("addr error\r\n");
	}
	return 0;	
}

/*******************************************************************
Func Name:EEPROM_R_Reg_Multi
Desc:EEPROM read multi byte
In:
addr --- need to be word aligned
leng --- byte len
*data_buf --- the dst buf ptr
Out:²Ù×÷³É¹¦·µ»Ø1£¬·ñÔò·µ»Ø0
*******************************************************************/
unsigned char EEPROM_R_Reg_Multi(uint8_t pageNum, uint16_t addr,uint16_t leng,unsigned char *data_buf) 
{
	uint32_t readFlashAddr;

	uint16_t i;
	readFlashAddr = (address_of_page(pageNum) + addr);
	UART_PRINTF("rd addr %x page num %x\r\n",readFlashAddr,pageNum);
	if(IS_VALID_OP_ADDRESS(readFlashAddr))
	{
		//readWordAddr = ((uint32_t)readFlashAddr/4);
		//UART_PRINTF("read addr = 0x%x\r\n",readWordAddr);
		flash_read(flash_env.space_type,readFlashAddr,leng,data_buf,NULL);
		return 1;
	}

	return 0;  
}


/*******************************************************************
Func Name:EEPROM_R_Reg_Byte
Desc: read our the data from a register
In:
addr --- adderess for the register

Out: 
the content of the register
*******************************************************************/
uint8_t EEPROM_R_Reg_Byte(uint8_t pageNum, uint16_t addr)  
{
	uint8_t data = 0;
	uint32_t readFlashAddr = (pageNum*FLASH_PAGE_SIZE+addr+EEPROM_BASE_ADDR);

	if(IS_VALID_OP_ADDRESS(readFlashAddr))
	{
		data = *(uint32_t*)readFlashAddr;
	}
	return data;
}

