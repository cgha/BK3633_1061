/******************************************************************************
File: Watch.c
Creator: Zeno Zhang
Desc:手表功能
******************************************************************************/
#include <string.h>

//#include "WatchCommon.h"
#include "Watch.h"

#include "DataManage.h"
#include "Watch_BLE_Command.h"
#include "app.h"
#include "app_task.h"              // Application Manager Task API
#include "app.h"                      // Application Manager Definition
#include "gapc_task.h"            // GAP Controller Task API
#include "gapm_task.h"          // GAP Manager Task API
#include "gattc_task.h"
#include "arch.h"                    // Platform Definitions
#include "app_fff0.h"
#include "ke_timer.h"
#include "spi.h"
#include "gpio.h"
#include "uart.h"
#include "icu.h"
#include "SPI_Transfer.h"
#include "ZenoRtc.h"
#include "rf.h"
#include "app_fff0.h"
#include "app_bond_db.h"
#ifdef I2C_HARDWARE
#include "i2c.h"
#else
#include "i2cs.h"
#endif
#include "hrs3300.h"
#include "wdt.h"


#define WATCH_SOFT_VERSION      0x24
//定义返回时间界面时间
#define DISP_TIME_STOP_WATCH    300
#define DISP_TIME_OTHER         10

//按键值定义
#define KEY_VAL_MODE    0x01
#define KEY_VAL_LIGHT   0x02
#define KEY_VAL_PRE    0x04
#define KEY_VAL_RES    0x08

#define WATCH_STATE_START_DISP_ON       0x00
#define WATCH_STATE_START_DISP_ALL      0x01
#define WATCH_STATE_START_DISP_VERSION  0x02
#define WATCH_STATE_NORMAL              0x03
#define WATCH_STATE_SHUT_DISP_OFF       0x04
#define WATCH_STATE_SHUT                0x05
#define WATCH_STATE_START_WAKE_ON       0x06
#define WATCH_STATE_RESTART             0x07
#define WATCH_STATE_HARDRESET           0x08
#define WATCH_STATE_WAKE_FROM_SHUTDOWN  0x09

#define HR_RECORD_MAX	30

//#include "icall_api.h"
#define CMD_LENG                      16

//#define TEST_DATA
/*
#pragma data_alignment=4096
#pragma location = ".mySnvSectors"
const uint8 NV_FLASH2[4096] = {0x55,0xAA,0xA5,0x5A};*/




const struct gapc_conn_param ConnParamSlow =
{
	// 24,        /* Minimum connection interval of 1000 ms */
	// 40,        /* Maximum connection interval of 1000 ms */
	// 19,          /* Slave latency */
	// 600        /* Supervision timeout of 5 seconds */

	144,        /* Minimum connection interval of 180 ms */
	158,        /* Maximum connection interval of 197.5 ms */
	9,          /* Slave latency */
	600        /* Supervision timeout of 5 seconds */

};

const struct gapc_conn_param ConnParamFast =
{
	// 16,//40,// 16,        /* Minimum connection interval of 1000 ms */
	// 32,//56,// 32,        /* Maximum connection interval of 1000 ms */
	// 2,//4,// 0,          /* Slave latency */
	// 400//400// 100        /* Supervision timeout of 5 seconds */
	26,// 16,        /* Minimum connection interval of 1000 ms */
	42,// 32,        /* Maximum connection interval of 1000 ms */
	4,// 0,          /* Slave latency */
	400// 100        /* Supervision timeout of 5 seconds */
};

typedef struct date {
    uint8_t year;  /* 年 */
    uint8_t month; /* 月 */
    uint8_t day;   /* 日 */
} DATE;

DATE day;
uint8_t dpm[13] = {0,31,28,31,30,31,30,31,31,30,31,30,31}; 

uint8_t IsFastConnect = 1,ConectParamUpdataTimer = 0;
extern uint8_t NowConnIntvSlow;

//static uint8_t send_data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//static uint8 TxBuf[16]; //BLE TX buffer
//static uint8 RxBuf[16]; //BLE RX Buffer
uint8_t DataSendToHost[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//,19};
uint8_t DataGetFromHost[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //,19};
static unsigned char AckBuf[CMD_LENG];
static unsigned char DayRecordTransIndex = 0,BLE_DataIsSending = 0;
ALARM AlarmSave[ALARM_NUM_MAX];
SPORT_PERIOD SportPeriod;
volatile uint32_t SportStartMin = 0,SportEndMin = 0;

SAVE_TIME SaveTime,Last_SaveTime,SleepMode_Last_SaveTime;
__align(4) DAY_RECORD DayRecord; //运动数据存储结构
__align(4) DAY_RECORD SaveDayRecord[RECORD_NUM_MAX]; 
static SPORT_DATA QuarterSportData;
//delete on 20181010
//static SPORT_DATA_TOTAL DaySportDataTotal;
//end delete
static uint32_t SportTime_s;
static uint32_t LastSaveSteps = 0,LastSaveKll= 0,LastSaveDist = 0;
//static float KLL_output;
static uint32_t RunSteps = 0;
static RTC RTC_CurTime;//,LCD_DispTime,LastDispTime;
static RTC RTC_SaveTime;
static unsigned char IsNewDay;//,SetTimeToNewDay = 0;
PERSON PersonInfo;
SLEEP_PERIOD SleepPeriod;
//static uint16_t IsTargetDisp = 0,IsTargetPassed = 0,IsTargetVibration = 0;
static volatile float kll,f_kll;
//static uint32_t SportSteps;
static unsigned char IsRealTimeMode = 0;
//static unsigned char RTC_Event = 0;//,RTC_TimeUpdateEvent = 0;
static unsigned char WatchMode;
static uint8_t IsUserSetMode = 0;
//static uint8_t StopWatchMin = 0,StopWatchSec = 0,StopWatch10ms = 0;
//static uint8_t IconDispBluetooth = 0,IconDispLowPower = 0,IconDispAlarmEn = 0, IconDispCamera = 1, IconDispTel = 1,IconDispMessage = 1;
//static uint8_t IsWDT_EN = 0;
static float Kll_Value = 0,Kll_Base = 0,Kll_Stature = 0,Kll_Weight = 0;
static volatile unsigned long StepsMin = 0,LastStepsMin = 0;
static uint8_t TimeBeSet = 0;
//static uint8_t IsBleCmd = 0;
//static uint8_t RTC_IntEvent = 0,CyTimerEvent = 0;
//static KEY_HOLD KeyHold;
static uint8_t IsRegistOk = 0;
//volatile uint8_t MacAddr[6];
static uint8_t RegistCode[6];
uint32_t ancs_notify = 0;
//static uint8_t IsTakePhoto = 0;

volatile unsigned char ToStopBluetooth = 0;

//static uint8_t KeyIsPresed = 0;
static uint8_t IconFlag = 0xFF;

static uint8_t WatchState = WATCH_STATE_HARDRESET;
static uint8_t WatchIsStart = 0;

static uint8_t IsWatchStarted = 0;
static uint32_t DispSteps = 0;

//add on 20181010
__align(4) DAY_RECORD TodayRecord;
//end add
//add on 20181010 for flash test
volatile uint32_t FlashEraseCycles = 0;
//end add
static uint8_t UiSpiSendBuf[SPI_CMD_LENG_MAX];
static uint8_t UiSpiRcvBuf[SPI_CMD_LENG_MAX];
static uint8_t UiSpiTaskId;

//uint8_t manualStopAdv;
volatile uint8_t rtc_notify;
volatile uint8_t rtc_notify_mask;


uint8_t gapCurState;
uint8_t delayAck;

uint8_t PowerOn;
uint8_t timer1sValid;
uint8_t sleepStage;
uint8_t hrChkFakeCnt;
uint8_t hrMeasureTick;
uint8_t staticCnt;
uint8_t onWrist;
uint16_t deepSleepMin = 0,notDeepSleepMin = 0,totalSleep = 0;
SLEEP_TIME sleepEnterTime,sleepExitTime;
uint16_t SleepStartMin;
uint16_t SleepEndMin;
uint8_t tempSleepDataIndex;
SLEEP_DATA tempSleepData[96];
uint8_t sleepQ[8];
extern hrs3300_results_t alg_results;
HR_RECORD HR_Data[HR_RECORD_MAX];
uint8_t HR_DataIndex;
uint8_t hrSendIndex;
extern uint8_t HeartRateBuf[20];
uint8_t HR_Max,HR_Min,HR_Average;
uint8_t poweringOff;
volatile uint8_t inHrMode;
extern volatile AxesRaw_t Axis;
extern uint8_t chipID;
extern bool hrs3300_power_up_flg;

#define TEMP_SLEEP_DATA_BUF_LEN	(sizeof(tempSleepData)/sizeof(SLEEP_DATA))

#ifdef USE_WATCHDOG_TIMER
static volatile uint8_t IsWatchdogReset = 0;
// Watchdog handle
Watchdog_Handle watchdogHandle;
#endif
//函数声明
void PED_DataIniti(void);
void Kll_Initi(void);

unsigned int GetAncsMask(void);


void WatchIniti(void);
void sendTakePhotoCmd(void);

void WatchReadySleep(void);
void WatchWakeOn(void);
void GetStepDist(void);
void Watch_PeriSleep(void);
void Watch_MusicPlayHandler(uint8_t event);
void app_gpio_int_cb(void);
void getCurHexTime(RTC * rtc_hex);
void spi_getMcuTime(void);
#ifdef TEST_DATA
DATE dateDelta (DATE* start, int8_t delta) ;
#endif
#ifdef USE_WATCHDOG_TIMER
/********************WATCHDOG FUNCTIONS********************/
static void watchdogCallback(uintptr_t unused);
#endif


/*******************************************************************
Function: Watch_GetCurIconFlag
Desc: SPI命令前准备BYTE4的数据
Input: void
Output: uint8_t Flag 返回标志，用于传输到SPI BYTE4
*******************************************************************/
uint8_t Watch_GetCurIconFlag(void)
{
	uint8_t Flag;

	Flag = 0;
	//BAT Icon
//  if(BatVolLevel == 0)
//    Flag |= 0x08;
	//Blue Icon
	if(gapCurState == BT_ADVERTISING)
		Flag |= 0x02;
	if(gapCurState == BT_CONNECTED)
		Flag |= 0x01;
	//时间模式，=1为24小时制
	if(PersonInfo.TimeDispMode != 0)
		Flag |= 0x10;
	//闹钟使能
	if(AlarmSave[0].IsEn != 0)
		Flag |= 0x04;
	//设置位选

	return Flag;
}

/*******************************************************************
Function: WatchModeIniti
Desc: Watch mode initi
Input: void
Output: void
*******************************************************************/
void WatchModeIniti(void)
{

}


/*******************************************************************
Function: StartBluetooth
Desc: start to 蓝牙广播
Input: void
Output: void
*******************************************************************/
void StartBluetooth(void)
{
	ToStopBluetooth = 0;
	if(gapCurState != BT_CONNECTED )
	{
		if(gapCurState != BT_ADVERTISING)
		{
			appm_start_advertising();
		}
	}
}


/*******************************************************************
Function: StopBlueScan
Desc: 关闭蓝牙广播
Input: void
Output: void
*******************************************************************/
void StopBleAdv(void)
{
	if( gapCurState == BT_ADVERTISING )
	{
		appm_stop_advertising();
		ToStopBluetooth = 0;
	}
}


/*******************************************************************
Function: StopBluetooth
Desc: 关闭蓝牙功能
Input: void
Output: void
*******************************************************************/
void StopBluetooth(void)
{
	if( gapCurState != BT_CONNECTED )
	{
		StopBleAdv();
	}
	else
	{
		appm_disconnect();
		//manualStopAdv = 1;
		UART_PRINTF("disconnect\r\n");
	}
	if(ke_timer_active(APP_ADV_TO_TIMER,TASK_APP))
	{
		ke_timer_clear(APP_ADV_TO_TIMER,TASK_APP);
	}
}



/*******************************************************************
Function: watchdogCallback
Desc:Watchdog callback for interrupt
Input: uintptr_t unuse
Output: void
*******************************************************************/
#ifdef USE_WATCHDOG_TIMER
/*
 *  ======== watchdogCallback ========
 *  Watchdog interrupt callback function.
 */
static void watchdogCallback(uintptr_t unused)
{
	/* Clear watchdog interrupt flag */
	//Watchdog_clear(watchdogHandle);
	IsWatchdogReset = 1;

}
#endif

/*******************************************************************
Function: WatchConnectParamUpdata
Desc: BLE 连接参数更改
Input: uint8_t IsFast = 1切换到快速模式，=0切换到低速模式
Output: void
*******************************************************************/
void WatchConnectParamUpdata(uint8_t IsFast)
{
	if(gapCurState == BT_CONNECTED)
	{
		if(IsFast)
		{
			appm_update_param((struct gapc_conn_param *)&ConnParamFast);
			IsFastConnect = 1;
		}
		else
		{
			appm_update_param((struct gapc_conn_param *)&ConnParamSlow);
			IsFastConnect = 0;
		}
	}
}

/*******************************************************************
Function: Watch_BLE_SendData
Desc: BLE命令传送
Input: void
Output: 发送成功返回0，否则返回-1
*******************************************************************/
uint8_t Watch_BLE_SendData(uint8_t* pData,uint8_t len)
{
	UART_PRINTF("SendData %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\r\n",\
		pData[0],pData[1],pData[2],pData[3],pData[4],pData[5],pData[6],pData[7],pData[8],pData[9],pData[10],pData[11],pData[12],pData[13],pData[14],pData[15]);
	app_fff1_send_lvl(pData, len);

	return 0;
}

/*******************************************************************
Function: SendDayRecord
Desc: 传送一天的运动数据（96条上传）
Input: void
Output: 发送成功返回0，否则返回-1
*******************************************************************/
unsigned char SendDayRecord(void)
{
	static unsigned char CheckSum;
	static unsigned char *pSend;
	unsigned char i,SportHeadIndex,BitOffset,BitMask;
	//add for ble speed
	unsigned char ret = 0;
	//end add
	SportHeadIndex = DayRecordTransIndex / 8;
	BitOffset = DayRecordTransIndex % 8;
	//BitMask = 0x01;
	//for(i = 0; i < BitOffset; i++)
	//	BitMask <<= 1;
	BitMask = 1<< BitOffset;

	DataSendToHost[0] = CMD_GET_THIS_DAY_RECORD;
	//CheckSum = DataSendToHost[0];
	DataSendToHost[1] = 0xF0;
	//CheckSum += DataSendToHost[1];
	DataSendToHost[2] = DayRecord.Head.Year;
	//CheckSum += DataSendToHost[2];
	DataSendToHost[3] = DayRecord.Head.Month;
	//CheckSum += DataSendToHost[3];
	DataSendToHost[4] = DayRecord.Head.Day;
	//CheckSum += DataSendToHost[4];
	DataSendToHost[5] = DayRecordTransIndex;
	//CheckSum += DataSendToHost[5];

	for(i = 6; i < (sizeof(DataSendToHost) - 1); i++)
		DataSendToHost[i] = 0;

	if((DayRecord.SportHead[SportHeadIndex] & BitMask) != 0)
	{
		pSend = (unsigned char *)&DayRecord.SportData[DayRecordTransIndex];

		for(i = 0; i < sizeof(SPORT_DATA); i++)
		{
			//CheckSum += *pSend;
			DataSendToHost[i+7] = *pSend; //here should take care of the offset if comment item of SPORT_DATA
			pSend++;
		}
	}
	else
	{
		//DataSendToHost[7] = 0;
		//DataSendToHost[8] = 0; //2 bytes kll
		//DataSendToHost[9] = 0;
		//DataSendToHost[10] = 0; //2 bytes steps
		DataSendToHost[11] = 0xFF;
		DataSendToHost[12] = 0xFF;
		DataSendToHost[13] = 0xFF;
		DataSendToHost[14] = 0xFF; //four bytes sleep data
	}
	CheckSum = 0;
	for(i = 0; i < (sizeof(DataSendToHost) - 1); i++)
	{
		CheckSum += DataSendToHost[i];
	}
	DataSendToHost[CMD_LENG - 1] = CheckSum;
	ret = Watch_BLE_SendData(DataSendToHost,CMD_LENG);

	if(ret == 0)
	{
		if(++DayRecordTransIndex >= 96)
		{
			DayRecordTransIndex = 0;
			BLE_DataIsSending = 0;
			return 0;
		}
	}
	return 0xff;
}

uint8_t calCheckSum(uint8_t *buf,uint8_t len)
{
	uint8_t checkSum = 0,i;
	for(i = 0; i< len; i++)
	{
		checkSum += buf[i];
	}
	return checkSum;
}

unsigned char SendHrRecord(void)
{
	static unsigned char CheckSum;
	static unsigned char *pSend;
	unsigned char i;
	//add for ble speed
	unsigned char ret = 0;
	//end add
	for(i = 0; i < (sizeof(DataSendToHost) - 1); i++)
		DataSendToHost[i] = 0;
	
	if(hrSendIndex)
		hrSendIndex--;
	else
		hrSendIndex = HR_RECORD_MAX-1;
	
	DataSendToHost[0] = CMD_GET_MANUAL_HR_RECORD;  
	DataSendToHost[1] = DayRecordTransIndex;
	if(HR_Data[hrSendIndex].AvarageHR){	
		DataSendToHost[2] = HR_Data[hrSendIndex].year;  
		DataSendToHost[3] = HR_Data[hrSendIndex].month; 
		DataSendToHost[4] = HR_Data[hrSendIndex].day; 
		DataSendToHost[5] = HR_Data[hrSendIndex].hour;
		DataSendToHost[6] = HR_Data[hrSendIndex].min;
		DataSendToHost[7] = HR_Data[hrSendIndex].AvarageHR;
	}

	if(hrSendIndex)
		hrSendIndex--;
	else
		hrSendIndex = HR_RECORD_MAX-1;
	
	if(HR_Data[hrSendIndex].AvarageHR){	
		DataSendToHost[8] = HR_Data[hrSendIndex].year;  
		DataSendToHost[9] = HR_Data[hrSendIndex].month; 
		DataSendToHost[10] = HR_Data[hrSendIndex].day; 
		DataSendToHost[11] = HR_Data[hrSendIndex].hour;
		DataSendToHost[12] = HR_Data[hrSendIndex].min;
		DataSendToHost[13] = HR_Data[hrSendIndex].AvarageHR;
	}

	DataSendToHost[CMD_LENG - 1] = calCheckSum(DataSendToHost,CMD_LENG - 1);
	ret = Watch_BLE_SendData(DataSendToHost,CMD_LENG);

	if(ret == 0)
	{
		if(++DayRecordTransIndex >= 15)
		{
			DayRecordTransIndex = 0;
			BLE_DataIsSending = 0; 
			hrSendIndex = 0;
			return 0;
		}
		else		
			return 0xFF;
	}
	else{
		DayRecordTransIndex = 0;
		BLE_DataIsSending = 0; 
		hrSendIndex = 0;
		return 0xff;
	}
	
}

/*******************************************************************
Function: Host_Ack
Desc: BLE命令应答
Input: void
Output: void
*******************************************************************/
void Host_Ack(void)
{
	unsigned char i,CheckSum,resendtimes;

	CheckSum = 0;
	for(i = 0; i < (sizeof(AckBuf) - 1); i++)
	{
		CheckSum += AckBuf[i];
		DataSendToHost[i] = AckBuf[i];
	}

	DataSendToHost[CMD_LENG - 1] = CheckSum;

	for(resendtimes = 0; resendtimes < 4; resendtimes++)
	{
		if(Watch_BLE_SendData(DataSendToHost,CMD_LENG) == 0)
			break;
	}

}

#ifdef TEST_DATA
void fillSportData(uint8_t day_arg)
{
	
	DATE date, lastDate;
	uint8_t i,sleepStartIndex,sleepEndIndex;
	
	DM_GetSleepPeriod(&SleepPeriod);
	sleepStartIndex = SleepStartMin/15;
	sleepEndIndex = SleepEndMin/15;
	
	
	if(day_arg == 1)
	{
		date.year = RTC_CurTime.year;
		date.month = RTC_CurTime.month;
		date.day = RTC_CurTime.day;
		
		lastDate = dateDelta(&date,(uint8_t)-1);
		
		DayRecord.Head.Year = lastDate.year;
		DayRecord.Head.Month = lastDate.month;
		DayRecord.Head.Day = lastDate.day;
	}
	else
	{
		DayRecord.Head.Year = RTC_CurTime.year;
		DayRecord.Head.Month = RTC_CurTime.month;
		DayRecord.Head.Day = RTC_CurTime.day;
	}
	
	#if 0
	if(sleepStartIndex < sleepEndIndex)
	{
		for(i = sleepStartIndex; i<sleepEndIndex; i++)
		{
			DayRecord.SportData[i].Kll = 0;
			DayRecord.SportData[i].Steps = 0;
			DayRecord.SportData[i].sleepQ_Data[0] = 0x00;
			DayRecord.SportData[i].sleepQ_Data[1] = 0x11;
			DayRecord.SportData[i].sleepQ_Data[2] = 0x22;
			DayRecord.SportData[i].sleepQ_Data[3] = 0x33;
		}
		for(i = 0; i<sleepStartIndex; i++)
		{
			DayRecord.SportData[i].Kll = 100;
			DayRecord.SportData[i].Steps = 500;
			DayRecord.SportData[i].sleepQ_Data[0] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[1] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[2] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[3] = 0xFF;
		}
		for(i = sleepEndIndex; i<96; i++)
		{
			DayRecord.SportData[i].Kll = 100;
			DayRecord.SportData[i].Steps = 500;
			DayRecord.SportData[i].sleepQ_Data[0] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[1] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[2] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[3] = 0xFF;
		}
	}
	else
	{
		/*for(i = sleepStartIndex; i<96; i++)
		{
			DayRecord.SportData[i].Kll = 0;
			DayRecord.SportData[i].Steps = 0;
			DayRecord.SportData[i].sleepQ_Data[0] = 0x00;
			DayRecord.SportData[i].sleepQ_Data[1] = 0x11;
			DayRecord.SportData[i].sleepQ_Data[2] = 0x22;
			DayRecord.SportData[i].sleepQ_Data[3] = 0x33;
		}
		for(i = 0; i<sleepEndIndex; i++)
		{
			DayRecord.SportData[i].Kll = 0;
			DayRecord.SportData[i].Steps = 0;
			DayRecord.SportData[i].sleepQ_Data[0] = 0x00;
			DayRecord.SportData[i].sleepQ_Data[1] = 0x11;
			DayRecord.SportData[i].sleepQ_Data[2] = 0x22;
			DayRecord.SportData[i].sleepQ_Data[3] = 0x33;
		}
		for(i = sleepEndIndex; i<sleepStartIndex; i++)
		{
			DayRecord.SportData[i].Kll = 100;
			DayRecord.SportData[i].Steps = 500;
			DayRecord.SportData[i].sleepQ_Data[0] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[1] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[2] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[3] = 0xFF;
		}
		for(i = 12; i<=18; i++) //2 hours test data
		{
			DayRecord.SportData[i].Kll = 100;
			DayRecord.SportData[i].Steps = 500;
			DayRecord.SportData[i].sleepQ_Data[0] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[1] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[2] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[3] = 0xFF;
		}*/
		for(i = 0; i<96; i++)
		{
			DayRecord.SportData[i].Kll = 100;
			DayRecord.SportData[i].Steps = 500;
			DayRecord.SportData[i].sleepQ_Data[0] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[1] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[2] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[3] = 0xFF;
		}
		for(i = 30; i<40; i++)
		{
			DayRecord.SportData[i].Kll = 0;
			DayRecord.SportData[i].Steps = 0;
			DayRecord.SportData[i].sleepQ_Data[0] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[1] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[2] = 0x10;
			DayRecord.SportData[i].sleepQ_Data[3] = 0x10;
		}
		for(i = 40; i<48; i++)
		{
			DayRecord.SportData[i].Kll = 0;
			DayRecord.SportData[i].Steps = 0;
			DayRecord.SportData[i].sleepQ_Data[0] = 0x00;
			DayRecord.SportData[i].sleepQ_Data[1] = 0x00;
			DayRecord.SportData[i].sleepQ_Data[2] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[3] = 0xFF;
		}
		for(i = 80; i<96; i++)
		{
			DayRecord.SportData[i].Kll = 0;
			DayRecord.SportData[i].Steps = 0;
			DayRecord.SportData[i].sleepQ_Data[0] = 0x11;
			DayRecord.SportData[i].sleepQ_Data[1] = 0x00;
			DayRecord.SportData[i].sleepQ_Data[2] = 0xFF;
			DayRecord.SportData[i].sleepQ_Data[3] = 0xFF;
		}
	}
	for(i = 0; i< 12; i++)
	{
		DayRecord.SportHead[i] = 0xFF;
	}
	
	memcpy((uint8_t*)&TodayRecord,(uint8_t*)&DayRecord,sizeof(DAY_RECORD));
	#endif

	extern uint8_t testSleepData[96][16];
	for(i = 0; i<96; i++)
	{
		DayRecord.SportData[i].Kll = testSleepData[i][8] << 8 | testSleepData[i][7];
		DayRecord.SportData[i].Steps = testSleepData[i][10] << 8 | testSleepData[i][9];
		DayRecord.SportData[i].sleepQ_Data[0] = testSleepData[i][11];
		DayRecord.SportData[i].sleepQ_Data[1] = testSleepData[i][12];
		DayRecord.SportData[i].sleepQ_Data[2] = testSleepData[i][13];
		DayRecord.SportData[i].sleepQ_Data[3] = testSleepData[i][14];
	}
	for(i = 0; i< 12; i++)
	{
		DayRecord.SportHead[i] = 0xFF;
	}
	memcpy((uint8_t*)&TodayRecord,(uint8_t*)&DayRecord,sizeof(DAY_RECORD));
}


uint8_t isLeap (uint8_t year) {
	uint16_t Year = 2000 + year;
    if( (Year%4==0 && Year%100!=0) || (Year%400==0))
		return 1;
	else
		return 0;
}

DATE dateDelta (DATE* start, int8_t delta) 
{
    DATE end;
	end.year = (start->year >> 4 & 0x0F) * 10 + (start->year & 0x0F);
	end.month = (start->month >> 4 & 0x0F) * 10 + (start->month & 0x0F);
	end.day = (start->day >> 4 & 0x0F) * 10 + (start->day &0x0F);
     
    if (delta>=0) { /* 日期start向后计算 */
        end.day = end.day + delta; /* start日加intervalDays天 */
        while (end.day > dpm[end.month]) { /* 不断减去整月天数，向月、年进位，直至其不超过当前月份天数 */
            end.day -= dpm[end.month];
            if (isLeap (end.year) && end.month==2) /* 闰年2月天数-1 */
                end.day--;
            end.month++; /* 月进位+1 */
            if (end.month >= 12) { /* 满1年 */
                end.year++; /* 年进位+1 */
                end.month = 1; /* 月归位至1月 */
            }
        }
    }
    else { /* 日期start向前计算 */
        end.day = end.day + delta; /* start日加intervalDays天 */
        while (end.day < 1) { /* 不断加整月天数，向月、年借位，直至其不小于1 */
            end.month--; /* 月借位-1 */
            if (end.month == 0) { /* 向年借位 */
                end.year--; /* 年借位-1 */
                end.month = 12; /* 月归位至12月 */
            }
            end.day += dpm[end.month];
            if (isLeap (end.year) && end.month==2) /* 闰年2月天数+1 */
                end.day++;
        }
    }
    
	end.year = ((end.year / 10) << 4) | ((end.year% 10) & 0x0F);
	end.month = ((end.month / 10) << 4) | ((end.month% 10) & 0x0F);
	end.day = ((end.day / 10) << 4) | ((end.day% 10) & 0x0F);
    return end;
}
#endif

/*******************************************************************
Function: Host_CmdAnalyze
Desc: BLE主机命令解释并执行
Input: void
Output: void
*******************************************************************/
void Host_CmdAnalyze(unsigned char *CmdBuf)
{
	unsigned char i,CheckSum;
// int ret;
	uint16_t Cmd;
	unsigned char *pAddr,DayIndex;
	uint32_t RecordMask,dist;
	unsigned char temp;
	ALARM AlarmInfo;
	RTC WatchTimeHex;
	//add on 20181010
	unsigned char *pSource,*pTarget;
	unsigned char IsFillData;
	uint16_t index16;
	uint32_t target;
	//end add

	//calculate command checksum
	CheckSum = 0;
	for(i = 0; i < (CMD_LENG - 1); i++)
		CheckSum += CmdBuf[i];
	//clear the ACK buffer
	for(i = 0; i < CMD_LENG; i++)
		AckBuf[i] = 0;
	AckBuf[0] = CmdBuf[0];
	if((CheckSum == CmdBuf[CMD_LENG - 1]) && (BLE_DataIsSending == 0))
	{
		/*BLE连接参数调节
		if(curConnectionParam.connIntvMin != hrmConnectionParamFast.connIntvMin && connUpdate!=2)
		{
		    connUpdate = 1;
		}*/

		//checksum is OK
		Cmd = CmdBuf[0];
		//command execute
		switch(Cmd)
		{
		case  CMD_NOTIFICATION:

			switch(CmdBuf[1])
			{
			case 0:
				if(GetAncsMask() & ANCS_MASK_CALL)
					ancs_notify = NOTIFY_CALL;
				break;

			case 1:
				if(GetAncsMask() & ANCS_MASK_MESSAGE)
					ancs_notify = NOTIFY_MSG;
				break;

			case 2:
				if(GetAncsMask() & ANCS_MASK_QQ)
					ancs_notify = NOTIFY_MQQ;
				break;

			case 3:
				if(GetAncsMask() & ANCS_MASK_WEIXIN)
					ancs_notify = NOTIFY_WECHAT;
				break;

			case 4:
				if(GetAncsMask() & ANCS_MASK_TWITTER)
					ancs_notify = NOTIFY_TWITTER;
				break;

			case 5:
				if(GetAncsMask() & ANCS_MASK_FACEBOOK)
					ancs_notify = NOTIFY_FACEBOOK;
				break;

			case 6:
				if(GetAncsMask() & ANCS_MASK_WHATSAPP)
					ancs_notify = NOTIFY_WHATSAPP;
				break;

			case 7:
				if(GetAncsMask() & ANCS_MASK_SKYPE)
					ancs_notify = NOTIFY_SKYPE;
				break;
			//以下为扩展消息	
			case 8:
				if(GetAncsMask() & ANCS_MASK_MAIL)
					ancs_notify = NOTIFY_MAIL;
				break;
			case 9:
				if(GetAncsMask() & ANCS_MASK_TIKTOK)
					ancs_notify = NOTIFY_TIKTOK;
				break;
			case 10:
				if(GetAncsMask() & ANCS_MASK_INSTAGRAM)
					ancs_notify = NOTIFY_INSTAGRAM;
				break;
			case 11:
				if(GetAncsMask() & ANCS_MASK_LINE)
					ancs_notify = NOTIFY_LINE;
				break;
			case 12:
				if(GetAncsMask() & ANCS_MASK_KAKAO)
					ancs_notify = NOTIFY_KAKAO;
				break;
			case 13:
				if(GetAncsMask() & ANCS_MASK_TELEGRAM)
					ancs_notify = NOTIFY_TELEGRAM;
				break;
			case 14:
				if(GetAncsMask() & ANCS_MASK_VIBER)
					ancs_notify = NOTIFY_VIBER;
				break;
			case 15:
				if(GetAncsMask() & ANCS_MASK_LINKEDIN)
					ancs_notify = NOTIFY_LINKEDIN;
				break;
			case 16:
				if(GetAncsMask() & ANCS_MASK_ALL_APP)
					ancs_notify = NOTIFY_OTHER_APP;
				break;
			}

			/*
			   if((ancs_call_notify != 0) || (ancs_message_notify != 0))
			   {
			     //投递提醒事件消息
			     AncsApp_enqueueMsg(WATCH_MSG_MESSAGE_REMIND_EVT, 0,0);
			   }*/
			
			break;

		case  CMD_SET_TIME:

			//  if((RTC_CurTime.day != CmdBuf[3]) || (RTC_CurTime.month !=  CmdBuf[2]) || (RTC_CurTime.year != CmdBuf[1]))
			//   SetTimeToNewDay = 1;
			RTC_CurTime.year = CmdBuf[1];
			RTC_CurTime.month = CmdBuf[2];
			RTC_CurTime.day = CmdBuf[3];
			RTC_CurTime.hour = CmdBuf[4];
			RTC_CurTime.min = CmdBuf[5];
			RTC_CurTime.sec = CmdBuf[6];
			//bros1 ZenoRtcSetTime(&RTC_CurTime);
			//RTC_SetTime(&RTC_CurTime);
			//bros1 WatchModeUpdate(0,0);
			getCurHexTime(&WatchTimeHex);
			spi_setBtTime(&WatchTimeHex);
			//RTC_GetTime(&RTC_CurTime);
			UART_PRINTF("SetTime %x,%x,%x,%x,%x,%x",RTC_CurTime.year,RTC_CurTime.month,RTC_CurTime.day,RTC_CurTime.hour,RTC_CurTime.min,RTC_CurTime.sec);
			TimeBeSet = 1;
			break;

		case  CMD_GET_TIME:
			/*RTC_CurTime.first_addr = 0x02;
			//bros1 ZenoRtcGetTime(&RTC_CurTime);
			RTC_GetTime(&RTC_CurTime);
			RTC_CurTime.sec &= 0x7f;
			RTC_CurTime.min &= 0x7f;
			RTC_CurTime.hour &= 0x3f;
			RTC_CurTime.day &= 0x3f;
			RTC_CurTime.month &= 0x1f;
			AckBuf[1] = RTC_CurTime.year;
			AckBuf[2] = RTC_CurTime.month;
			AckBuf[3] = RTC_CurTime.day;
			AckBuf[4] = RTC_CurTime.hour;
			AckBuf[5] = RTC_CurTime.min;
			AckBuf[6] = RTC_CurTime.sec;*/
			spi_getMcuTime();
			delayAck |= DELAY_ACK_CMD_0x41;
			UART_PRINTF("APP GET TIME\r\n");
			goto ChkSwitchToFast;
			
			//break;

		case CMD_SET_TARGET_STEPS:
			DM_GetPerson(&PersonInfo);
			target =  (CmdBuf[2] << 24) | (CmdBuf[3] <<16) | (CmdBuf[4] <<8) | CmdBuf[5];
			//if(target != PersonInfo.TargetSteps || PersonInfo.TargetType != CmdBuf[1])
			{
				pAddr = (unsigned char *)&PersonInfo.TargetSteps;
				pAddr[3] = CmdBuf[2];
				pAddr[2] = CmdBuf[3];
				pAddr[1] = CmdBuf[4];
				pAddr[0] = CmdBuf[5];
				PersonInfo.TargetType = CmdBuf[1];
				DM_SetPerson(&PersonInfo);
			}
			//IsTargetDisp = 0;IsTargetPassed = 0; //????
			break;

		case CMD_GET_TARGET_STEPS:
			AckBuf[1] = PersonInfo.TargetType;
			pAddr = (unsigned char *)&PersonInfo.TargetSteps;
			AckBuf[2] = pAddr[3];
			AckBuf[3] = pAddr[2];
			AckBuf[4] = pAddr[1];
			AckBuf[5] = pAddr[0];
			break;

		case  CMD_SET_PERSON:
			DM_GetPerson(&PersonInfo);
			PersonInfo.Sex = CmdBuf[1];
			PersonInfo.Age = CmdBuf[2];
			PersonInfo.Stature = CmdBuf[3];
			PersonInfo.Weight = CmdBuf[4];
			// PersonInfo.StepLength = CmdBuf[5];//自动设置步长

			Kll_Initi();
			DM_SetPerson(&PersonInfo);
			//spi_setTimeMode(PersonInfo.TimeDispMode);
			spi_setPerson(&PersonInfo);
			break;

		case  CMD_GET_PERSON:
			DM_GetPerson(&PersonInfo);
			AckBuf[1] = PersonInfo.Sex;
			AckBuf[2] = PersonInfo.Age;
			AckBuf[3] = PersonInfo.Stature;
			AckBuf[4] = PersonInfo.Weight;
			AckBuf[5] = PersonInfo.StepLength;
			//DM_GetDevID((unsigned char *) &AckBuf[6],DEVICE_ID_LENG);
			break;

		case  CMD_SET_SLEEP_PERIOD:
			SleepPeriod.en = CmdBuf[1];
			SleepPeriod.StartHour = CmdBuf[2];
			SleepPeriod.StartMin = CmdBuf[3];
			SleepPeriod.EndHour = CmdBuf[4];
			SleepPeriod.EndMin = CmdBuf[5];

			DM_SetSleepPeriod(&SleepPeriod);
			spi_setSleepPeriod(&SleepPeriod);
			break;

		case  CMD_GET_SLEEP_PERIOD:
			DM_GetSleepPeriod(&SleepPeriod);
			AckBuf[1] = SleepPeriod.en;
			AckBuf[2] = SleepPeriod.StartHour;
			AckBuf[3] = SleepPeriod.StartMin;
			AckBuf[4] = SleepPeriod.EndHour;
			AckBuf[5] = SleepPeriod.EndMin;

			break;

		case  CMD_GET_THIS_DAY_RECORD:
//			GetStepDist();

			/*if(CmdBuf[1] ==1){
				DayRecord.Head.Year = 0x19;
				DayRecord.Head.Month = 0x06;
				DayRecord.Head.Day = 0x13;
				for(i = 0; i<12; i++){
					DayRecord.SportHead[i] = 0;
				}
				DayRecordTransIndex = 0;
				SendDayRecord();
				BLE_DataIsSending = 1;
				ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,5);
				goto ChkSwitchToFast;
			}
			else if(CmdBuf[1] ==2){
				DayRecord.Head.Year = 0x18;
				DayRecord.Head.Month = 0x12;
				DayRecord.Head.Day = 0x31;
				for(i = 0; i<12; i++){
					DayRecord.SportHead[i] = 0;
				}
				DayRecordTransIndex = 0;
				SendDayRecord();
				BLE_DataIsSending = 1;
				ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,5);
				goto ChkSwitchToFast;
			}
			else{
				//无数据
				AckBuf[1] = 0xFF;
				AckBuf[2] = 0x00;
				AckBuf[3] = 0x01;
				AckBuf[4] = 0x01;
			}
			break;	*/
			#ifdef TEST_DATA
			fillSportData(CmdBuf[1]); //for test
			if(CmdBuf[1] <= 1)
			//modify on 20181010
			#else
			if(CmdBuf[1] == 0)
			#endif
			{
				//今天
				IsFillData = 0;
				for(index16 = 0; index16 < 12; index16++)
				{
					if(TodayRecord.SportHead[index16] != 0)
					{
						IsFillData = 1;
						break;
					}
				}
				if(IsFillData != 0)
				{
					//有数据
					pSource = (unsigned char *)&TodayRecord;
					pTarget = (unsigned char *)&DayRecord;

					for(index16 = 0; index16 < sizeof(DayRecord); index16++)
					{
						*pTarget = *pSource;
						pTarget++;
						pSource++;
					}
					DayRecordTransIndex = 0;
					SendDayRecord();
					BLE_DataIsSending = 1;
					ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,5);
					UART_PRINTF("0x43 has data\r\n");
					goto ChkSwitchToFast;
				}
				else
				{
					//无数据
					AckBuf[1] = 0xFF;
					AckBuf[2] = 0x00;
					AckBuf[3] = 0x01;
					AckBuf[4] = 0x01;
					UART_PRINTF("0x43 No data,%x\r\n",delayAck);
				}
			}
			#ifndef TEST_DATA
			else
			{
				DM_GetThisDaySportData(CmdBuf[1],(unsigned char *)&DayRecord);

				if(DM_IsThisDaySaved(CmdBuf[1]) != 0)
				{
					DayRecordTransIndex = 0;
					SendDayRecord();
					BLE_DataIsSending = 1;
					//Util_startClock(&ClockTimer_BleSend);
					ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,5);
					goto ChkSwitchToFast;
				}
				else
				{
					AckBuf[1] = 0xFF;
					AckBuf[2] = 0x00;
					AckBuf[3] = 0x01;
					AckBuf[4] = 0x01;
				}
			}
			#else
			else{
				AckBuf[1] = 0xFF;
				AckBuf[2] = 0x00;
				AckBuf[3] = 0x01;
				AckBuf[4] = 0x01;
			}
			#endif
			//end modify


			break;

		case  CMD_DEL_THIS_DAY_RECORD:
			if(DM_DelDayRecord(CmdBuf[1]) == 0)
				AckBuf[0] |= 0x80;
			if(CmdBuf[1] == 0)
			{
				// if(WorkMode == WORK_MODE_SLEEP_MONITOR)
				//   SleepQualityModeIniti();
				// else
				PED_DataIniti();
			}
			break;

		/*case  CMD_SET_DEVICE_ID:
			DM_SetDevID((unsigned char *) &CmdBuf[1],DEVICE_ID_LENG);
			break;

		case  CMD_GET_DEVICE_ID:
			DM_GetDevID((unsigned char *) &AckBuf[1],DEVICE_ID_LENG);
			break;

		case  CMD_ACTIVE_DEVICE:
			if(IsRegistOk == 0)
			{
				RegistCode[0] = CmdBuf[1];
				RegistCode[1] = CmdBuf[2];
				RegistCode[2] = CmdBuf[3];
				RegistCode[3] = CmdBuf[4];
				RegistCode[4] = CmdBuf[5];
				RegistCode[5] = CmdBuf[6];
				DM_SetActiveCode((unsigned char *)&RegistCode,sizeof(RegistCode));
				//IsRegistOk = ChkRegistCode();
				if(IsRegistOk)
				{
					//激活码注册成功

				}
			}
			break;*/

		case  CMD_IS_THIS_DAY_RECORD:
			RecordMask = GetDayRecordMask();
			//add on 20181010
			RecordMask |= 0x00000001; //因现在是一天一存，没有15分钟一存了，所以今天的数据必然使能
			//end add
			pAddr = (unsigned char *)&RecordMask;
			AckBuf[1] = pAddr[3];
			AckBuf[2] = pAddr[2];
			AckBuf[3] = pAddr[1];
			AckBuf[4] = pAddr[0];
			break;

		case  CMD_GET_THIS_DAY_RECORD_TOTAL:
			//GetStepDist();
			DayIndex = CmdBuf[1];

			if(DayIndex == 0)
			{
				//		UpdateTime();
				/*GetStepDist();
				AckBuf[1] = 0;  //responde index = 0
				AckBuf[2] = DayIndex;
				AckBuf[3] = RTC_CurTime.year;
				AckBuf[4] = RTC_CurTime.month;
				AckBuf[5] = RTC_CurTime.day;
				pAddr = (unsigned char *)&TodayRecord.SportDataTotal.StepsTotal;
				AckBuf[6] = pAddr[2];
				AckBuf[7] = pAddr[1];
				AckBuf[8] = pAddr[0];
				pAddr = (unsigned char *)&TodayRecord.SportDataTotal.RunStepsTotal;
				AckBuf[9] = pAddr[2];
				AckBuf[10] = pAddr[1];
				AckBuf[11] = pAddr[0];
				pAddr = (unsigned char *)&TodayRecord.SportDataTotal.KllTotal;
				AckBuf[12] = pAddr[2];
				AckBuf[13] = pAddr[1];
				AckBuf[14] = pAddr[0];
				Host_Ack();

				// BLE_Send_Delay();

				//Secoend responde
				AckBuf[1] = 1;  //responde index = 1
				AckBuf[2] = DayIndex;
				AckBuf[3] = RTC_CurTime.year;
				AckBuf[4] = RTC_CurTime.month;
				AckBuf[5] = RTC_CurTime.day;

				pAddr = (unsigned char *)&TodayRecord.SportDataTotal.DistTotal;
				AckBuf[6] = pAddr[2];
				AckBuf[7] = pAddr[1];
				AckBuf[8] = pAddr[0];
				pAddr = (unsigned char *)&TodayRecord.SportDataTotal.SportTimeMinTotal;
				AckBuf[9] = pAddr[1];
				AckBuf[10] = pAddr[0];*/
				spi_getTotalRecord();
				delayAck |= DELAY_ACK_CMD_0x07;
				goto ChkSwitchToFast;
			}
			else
			{
				if(DM_IsThisDaySaved(DayIndex) != 0)
				{
					//first responde
					DM_GetThisDaySportData(CmdBuf[1],(unsigned char *)&DayRecord);
					AckBuf[1] = 0;  //responde index = 0
					AckBuf[2] = DayIndex;
					AckBuf[3] = DayRecord.Head.Year;
					AckBuf[4] = DayRecord.Head.Month;
					AckBuf[5] = DayRecord.Head.Day;
					pAddr = (unsigned char *)&DayRecord.SportDataTotal.StepsTotal;
					AckBuf[6] = pAddr[2];
					AckBuf[7] = pAddr[1];
					AckBuf[8] = pAddr[0];
					pAddr = (unsigned char *)&DayRecord.SportDataTotal.RunStepsTotal;
					AckBuf[9] = pAddr[2];
					AckBuf[10] = pAddr[1];
					AckBuf[11] = pAddr[0];
					pAddr = (unsigned char *)&DayRecord.SportDataTotal.KllTotal;
					AckBuf[12] = pAddr[2];
					AckBuf[13] = pAddr[1];
					AckBuf[14] = pAddr[0];
					Host_Ack();

					//Secoend responde
					AckBuf[1] = 1;  //responde index = 1
					AckBuf[2] = DayIndex;
					AckBuf[3] = DayRecord.Head.Year;
					AckBuf[4] = DayRecord.Head.Month;
					AckBuf[5] = DayRecord.Head.Day;
					pAddr = (unsigned char *)&DayRecord.SportDataTotal.DistTotal;
					AckBuf[6] = pAddr[2];
					AckBuf[7] = pAddr[1];
					AckBuf[8] = pAddr[0];
					pAddr = (unsigned char *)&DayRecord.SportDataTotal.SportTimeMinTotal;
					AckBuf[9] = pAddr[1];
					AckBuf[10] = pAddr[0];
				}
				else
				{
					//first responde
					// DM_GetThisDaySportData(DayIndex,(unsigned char *)&DayRecord);
					AckBuf[1] = 0;  //responde index = 0
					AckBuf[2] = DayIndex;
					AckBuf[3] = 0;
					AckBuf[4] = 0;
					AckBuf[5] = 0;
					AckBuf[6] = 0;
					AckBuf[7] = 0;
					AckBuf[8] = 0;
					AckBuf[9] = 0;
					AckBuf[10] = 0;
					AckBuf[11] = 0;
					AckBuf[12] = 0;
					AckBuf[13] = 0;
					AckBuf[14] = 0;
					Host_Ack();

					//Secoend responde
					AckBuf[1] = 1;  //responde index = 1
					AckBuf[2] = DayIndex;
					AckBuf[3] = 0;
					AckBuf[4] = 0;
					AckBuf[5] = 0;
					AckBuf[6] = 0;
					AckBuf[7] = 0;
					AckBuf[8] = 0;
					AckBuf[9] = 0;
					AckBuf[10] = 0;
					AckBuf[11] = 0;
					AckBuf[12] = 0;
					AckBuf[13] = 0;
					AckBuf[14] = 0;
				}
			}
			break;

		case  CMD_GET_THIS_DAY_TARGRT:
			//GetStepDist();
			DayIndex = CmdBuf[1];

			//modify on 20181011
			if(DayIndex == 0)
			{
				/*GetStepDist();
				AckBuf[1] = DayIndex;
				AckBuf[2] = TodayRecord.Head.Year;
				AckBuf[3] = TodayRecord.Head.Month;
				AckBuf[4] = TodayRecord.Head.Day;

				AckBuf[5] = TodayRecord.SportDataTotal.TargetCompleteRate;
				//pAddr = (unsigned char *)&TodayRecord.SportDataTotal.Speed;
				//AckBuf[6] = pAddr[1];
				//AckBuf[7] = pAddr[0];
				//pAddr = (unsigned char *)&TodayRecord.SportDataTotal.Ex;
				//AckBuf[8] = pAddr[2];
				//AckBuf[9] = pAddr[1];
				//AckBuf[10] = pAddr[0];
				pAddr = (unsigned char *)&TodayRecord.SportDataTotal.TargetCompleteRate;
				AckBuf[11] = pAddr[1];
				AckBuf[12] = pAddr[0];
				AckBuf[13] = 0;
				AckBuf[14] = 0;*/
				spi_getTotalRecord();
				delayAck |= DELAY_ACK_CMD_0x08;
				goto ChkSwitchToFast;
			}
			else
			{
				if(DM_IsThisDaySaved(DayIndex) != 0)
				{
					DM_GetThisDaySportData(DayIndex,(unsigned char *)&DayRecord);
					AckBuf[1] = DayIndex;
					AckBuf[2] = DayRecord.Head.Year;
					AckBuf[3] = DayRecord.Head.Month;
					AckBuf[4] = DayRecord.Head.Day;

					AckBuf[5] = DayRecord.SportDataTotal.TargetCompleteRate;
					//pAddr = (unsigned char *)&DayRecord.SportDataTotal.Speed;
					//AckBuf[6] = pAddr[1];
					//AckBuf[7] = pAddr[0];
					//pAddr = (unsigned char *)&DayRecord.SportDataTotal.Ex;
					//AckBuf[8] = pAddr[2];
					//AckBuf[9] = pAddr[1];
					//AckBuf[10] = pAddr[0];
					pAddr = (unsigned char *)&DayRecord.SportDataTotal.TargetCompleteRate;
					AckBuf[11] = pAddr[1];
					AckBuf[12] = pAddr[0];
					AckBuf[13] = 0;
					AckBuf[14] = 0;
				}
				else
				{
					AckBuf[1] = DayIndex;
					AckBuf[2] = 0;
					AckBuf[3] = 0;
					AckBuf[4] = 0;
					AckBuf[5] = 0;
					AckBuf[6] = 0;
					AckBuf[7] = 0;
					AckBuf[8] = 0;
					AckBuf[9] = 0;
					AckBuf[10] = 0;
					AckBuf[11] = 0;
					AckBuf[12] = 0;
					AckBuf[13] = 0;
					AckBuf[14] = 0;
				}
			}
			break;

		case  CMD_START_REALTIME_MODE:
			IsRealTimeMode = 1;
			// ZenoRtcNotifyEnableAdd(RTC_NOTIFY_SEC); //实时模式，使能RTC秒通知
			break;

		case  CMD_STOP_REALTIME_MODE:
			IsRealTimeMode = 0;
			break;

		case CMD_BONDING:

			break;

		case CMD_SET_ALARM:
			//AlarmInfo.Num = CmdBuf[1];
			if(CmdBuf[1] < ALARM_NUM_MAX)
			{
				AlarmSave[CmdBuf[1]].IsEn = CmdBuf[2];
				AlarmSave[CmdBuf[1]].Hour = CmdBuf[3];
				AlarmSave[CmdBuf[1]].Min = CmdBuf[4];
				AlarmSave[CmdBuf[1]].IsPassed = 0;

				temp = 0x00;
				for(i = 11; i > 4; i--)
				{
					temp <<= 1;
					if(CmdBuf[i] != 0)
						temp |= 0x01;

				}
				AlarmSave[CmdBuf[1]].Week = temp;
				DM_SetAlarm(&AlarmSave[CmdBuf[1]],CmdBuf[1]);
				spi_setAlarm();
			}
			else
				AckBuf[0] |= 0x80;
			//bros1 WatchModeUpdate(0,0);
			//spi_setAlarm();
			break;

		case CMD_GET_ALARM:
			DM_GetAlarm(&AlarmInfo,CmdBuf[1]);
			AckBuf[1] = AlarmInfo.Num ;
			AckBuf[2] = AlarmInfo.IsEn;
			AckBuf[3] = AlarmInfo.Hour;
			AckBuf[4] = AlarmInfo.Min;

			temp = AlarmInfo.Week;
			for(i = 5; i < 12; i++)
			{
				if((temp & 0x01) != 0)
					AckBuf[i] = 1;
				else
					AckBuf[i] = 0;
				temp >>= 1;
			}
			break;

		case  CMD_SET_SPORT_PERIOD:
			SportPeriod.StartHour = CmdBuf[1];
			SportPeriod.StartMin = CmdBuf[2];
			SportPeriod.EndHour = CmdBuf[3];
			SportPeriod.EndMin = CmdBuf[4];
			SportPeriod.WeekEn = CmdBuf[5];
			SportPeriod.VibPeriod = CmdBuf[6];
			DM_SetSportPeriod(&SportPeriod);
			break;

		case  CMD_GET_SPORT_PERIOD:
			DM_GetSportPeriod(&SportPeriod);
			AckBuf[1] = SportPeriod.StartHour;
			AckBuf[2] = SportPeriod.StartMin;
			AckBuf[3] = SportPeriod.EndHour;
			AckBuf[4] = SportPeriod.EndMin;
			AckBuf[5] = SportPeriod.WeekEn;
			AckBuf[6] = SportPeriod.VibPeriod;
			break;

		case CMD_FACTORY_RESET:
			DM_Format();
			PED_DataIniti();
			Kll_Initi();

			break;
		case  CMD_GET_MAC:
			/* AckBuf[1] = MacAddr.bdAddr[0];
			 AckBuf[2] = MacAddr.bdAddr[1];
			 AckBuf[3] = MacAddr.bdAddr[2];
			 AckBuf[4] = MacAddr.bdAddr[3];
			 AckBuf[5] = MacAddr.bdAddr[4];
			 AckBuf[6] = MacAddr.bdAddr[5];*/
			break;

		case  CMD_SET_DIST_UNIT:
			if(CmdBuf[1] <=1)
			{
				if(PersonInfo.DistUnit != CmdBuf[1])
				{
					PersonInfo.DistUnit = CmdBuf[1];
					DM_SetPerson(&PersonInfo);
					//WatchModeUpdate(0,0);
					spi_setPerson(&PersonInfo);
				}
			}
			break;

		case CMD_GET_DIST_UNIT:
			AckBuf[1] = PersonInfo.DistUnit;
			break;

		case CMD_SET_KARMA_UNIT:

			break;

		case CMD_GET_KARMA_UNIT:

			break;


			/*case CMD_GET_VERSION:
			    AckBuf[1] = SOFT_VERSION[0];
			    AckBuf[2] = SOFT_VERSION[1];
			    AckBuf[3] = SOFT_VERSION[2];
			    AckBuf[4] = SOFT_VERSION[3];
			    AckBuf[5] = SOFT_VERSION[4];

			    break;    */
		case CMD_MCU_SOFT_RESET:

			Host_Ack();
			//硬件复位
			//HAL_SYSTEM_RESET();
			//SaveTimeBeforReset();
			//DM_SetSaveTime((unsigned char *)&RTC_CurTime);
			//NVIC_SystemReset();
			platform_reset(RESET_NO_ERROR);
			//while(1);
			break;

		case CMD_GET_BAT_LEVEL:
			//AckBuf[1] = BatVolLevel;
			break;

		case CMD_VABRATION_TEST:

			break;

		case CMD_SET_BT_NAME:
			/*for(i = 1; i <15; i++)
			{
			  if((CmdBuf[i] < 32) || (CmdBuf[i] > 127))
			    CmdBuf[i] = 32;
			}
			DM_SetBT_Name((unsigned char *)&CmdBuf[1],14);*/
			break;

		case CMD_GET_BT_NAME:
			//DM_GetBT_Name((unsigned char *)&AckBuf[1],14);
			break;

		case CMD_SET_PRODUCT_INFO:
			DM_SetProductInfo((unsigned char *)&CmdBuf[1],5);
			break;

		case CMD_GET_PRODUCT_INFO:
			DM_GetProductInfo((unsigned char *)&AckBuf[1],5);
			break;

		/*case CMD_DEBUG:
			//add on 20181010 for flash test
			AckBuf[1] = FlashEraseCycles >> 24;
			AckBuf[2] = FlashEraseCycles >> 16;
			AckBuf[3] = FlashEraseCycles >> 8;
			AckBuf[4] = FlashEraseCycles & 0x000000FF;
			//end add
			break;*/

		case CMD_SET_TIME_MODE:
			DM_GetPerson(&PersonInfo);
			if(PersonInfo.TimeDispMode != CmdBuf[1])
			{
				PersonInfo.TimeDispMode = CmdBuf[1];
				DM_SetPerson(&PersonInfo);
				//bros1 WatchModeUpdate(0,0);
				spi_setTimeMode(CmdBuf[1]);
			}
			break;

		case CMD_GET_TIME_MODE:
			AckBuf[1] = PersonInfo.TimeDispMode;
			break;

		case CMD_SET_ANCS_MASK:
			DM_GetPerson(&PersonInfo);
			PersonInfo.ANCS_Mask = CmdBuf[3]<<16 | CmdBuf[2]<<8 | CmdBuf[1];
			DM_SetPerson(&PersonInfo);
			UART_PRINTF("ANCS_MASK%x",PersonInfo.ANCS_Mask);
			break;

		case CMD_GET_ANCS_MASK:
			AckBuf[1] = PersonInfo.ANCS_Mask & 0xFF;
			AckBuf[2] = (PersonInfo.ANCS_Mask >> 8) & 0xFF;
			AckBuf[3] = (PersonInfo.ANCS_Mask >> 16) & 0xFF;
			UART_PRINTF("ANCS_MASK1:%x",PersonInfo.ANCS_Mask);
			break;

		case CMD_UPDATE_FW:
			if((CmdBuf[1] == 0x55) && (CmdBuf[2] == 0xAA) && (CmdBuf[3] == 0x5A) && (CmdBuf[4] == 0xA5))
			{

			}
			break;

		case CMD_GET_CUR_SPORT_INFO:
			GetStepDist();
			pAddr = (unsigned char *)&TodayRecord.SportDataTotal.StepsTotal;
			AckBuf[1] = pAddr[2];
			AckBuf[2] = pAddr[1];
			AckBuf[3] = pAddr[0];
			pAddr = (unsigned char *)&RunSteps;
			AckBuf[4] = pAddr[2];
			AckBuf[5] = pAddr[1];
			AckBuf[6] = pAddr[0];
			pAddr = (unsigned char *)&TodayRecord.SportDataTotal.KllTotal;
			AckBuf[7] = pAddr[2];
			AckBuf[8] = pAddr[1];
			AckBuf[9] = pAddr[0];

			dist = (TodayRecord.SportDataTotal.StepsTotal * PersonInfo.StepLength) / 1000;
			pAddr = (unsigned char *)&dist;
			AckBuf[10] = pAddr[2];
			AckBuf[11] = pAddr[1];
			AckBuf[12] = pAddr[0];

//        TodayRecord.SportDataTotal.SportTimeMinTotal = (uint32_t)(SportTime_s/60);
			pAddr = (unsigned char *)&TodayRecord.SportDataTotal.SportTimeMinTotal;
			AckBuf[13] = pAddr[1];
			AckBuf[14] = pAddr[0];


			break;

		case CMD_GET_PRODUCT_TYPE:
			AckBuf[1] = 0x03;
			AckBuf[5] = 0x01;
			AckBuf[6] = 0x04;
			break;


		case CMD_ENTER_EXIT_SLEEP:

			break;

		case CMD_START_ECG:


			break;

		case CMD_STOP_ECG:

			break;
		
		case CMD_GET_SLEEP_STAGE:
			AckBuf[1] = sleepStage;
			AckBuf[2] = staticCnt;
			AckBuf[3] = sleepEnterTime.day;
			AckBuf[4] = sleepEnterTime.hour;
			AckBuf[5] = sleepEnterTime.min;
			AckBuf[6] = sleepExitTime.day;
			AckBuf[7] = sleepExitTime.hour;
			AckBuf[8] = sleepExitTime.min;
			break;
		
		case CMD_GET_HR_DAY_RECORD:
			//无数据
			AckBuf[1] = 0xFF;
			AckBuf[2] = 0x00;
			AckBuf[3] = 0x01;
			AckBuf[4] = 0x01;     
			break;
		case CMD_APP_TRIG_HR:
			if(hrMeasureing == HR_MEASURE_IDLE)
				spi_send_app_trig_hr();
			break;
		case CMD_GET_MANUAL_HR_RECORD:
			hrSendIndex = HR_DataIndex; //SendHrRecord里面hrSendIndex会先减1，因为HR_DataIndex在存的时候是存完后++的
			BLE_DataIsSending = 2;
			DayRecordTransIndex = 0;
			
			SendHrRecord();
			ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,5);
			goto ChkSwitchToFast;
		}
	} 
	else
	{
		AckBuf[0] |= 0x80;//如果当前ble is sending, 再发命令回复命令错误
	}

	Host_Ack();
	
	ChkSwitchToFast:
	if(Cmd != CMD_NOTIFICATION)
	{
		ConectParamUpdataTimer = 0;
		// if(IsFirstLinkStable != 0)
		{
			if(IsFastConnect == 0)
			{
				UART_PRINTF("switch to fast\r\n");
				WatchConnectParamUpdata(1);
			}
		}
	}
}

/*******************************************************************
Function: Watch_RealStepsSend
Desc: 实时步数命令传送
Input: void
Output: void
*******************************************************************/
void Watch_RealStepsSend(void)
{
	static uint8_t *pAddr,i;
	static uint32_t dist;

	if(gapCurState == BT_CONNECTED)
	{
		GetStepDist();
		DataSendToHost[0] = CMD_START_REALTIME_MODE;
		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.StepsTotal;
		DataSendToHost[1] = pAddr[2];
		DataSendToHost[2] = pAddr[1];
		DataSendToHost[3] = pAddr[0];
		//pAddr = (unsigned char *)&RunSteps;
		//DataSendToHost[4] = pAddr[2];
		//DataSendToHost[5] = pAddr[1];
		//DataSendToHost[6] = pAddr[0];
		if(Hrs3300_Present() && alg_results.alg_status == MSG_HR_READY)
			DataSendToHost[6] = alg_results.hr_result;
		else
			DataSendToHost[6] = 0;
		
		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.KllTotal;
		DataSendToHost[7] = pAddr[2];
		DataSendToHost[8] = pAddr[1];
		DataSendToHost[9] = pAddr[0];

		dist = (TodayRecord.SportDataTotal.StepsTotal * PersonInfo.StepLength) / 1000;
		pAddr = (unsigned char *)&dist;
		DataSendToHost[10] = pAddr[2];
		DataSendToHost[11] = pAddr[1];
		DataSendToHost[12] = pAddr[0];

		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.SportTimeMinTotal;
		DataSendToHost[13] = pAddr[1];
		DataSendToHost[14] = pAddr[0];

		DataSendToHost[15] = 0;
		for(i = 0; i < (sizeof(DataSendToHost) - 1); i++)
			DataSendToHost[15] += DataSendToHost[i];

		if(BLE_DataIsSending == 0)
			Watch_BLE_SendData(DataSendToHost,CMD_LENG);
	}
	else
	{
		IsRealTimeMode = 0; //蓝牙未连接，退出实时模式
	}
}

/*******************************************************************
Function: sendTakePhotoCmd
Desc: 传送拍照指令
Input: void
Output: void
*******************************************************************/
void sendTakePhotoCmd(void)
{
	uint8_t i;
	AckBuf[0] = CMD_TAKE_PHOTO;
	AckBuf[1] = 0x01;
	for(i=2 ; i<sizeof(AckBuf)-1; i++)
	{
		AckBuf[i] = 0;
	}
	Host_Ack();
}

/*******************************************************************
Function: BLE_SendTimerEvent
Desc: BLE定时传送事件
Input: void
Output: void
*******************************************************************/
void BLE_SendTimerEvent(void)
{
	unsigned char i,ret = 0xFF;

	if(BLE_DataIsSending == 1)
	{
		if(gapCurState == BT_CONNECTED)
		{
			//if(NowConnIntvSlow == 0)
			{
				for(i = 0; i < 4; i++)
				{
					ret = SendDayRecord();
					if(ret == 0)
						break;
				}
			}

			if(ret == 0)//send day record to host every 40ms
				BLE_DataIsSending = 0;
			else
			{
				//Util_startClock(&ClockTimer_BleSend);
				if(NowConnIntvSlow){
					ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,20);//according to 
				}
				else{
					ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,5);
				}
			}
		}
		else
			BLE_DataIsSending = 0;
	}
	else if(BLE_DataIsSending == 2)
	{
		if(gapCurState == BT_CONNECTED)
		{
			//if(NowConnIntvSlow == 0)
			{
				for(i = 0; i < 4; i++)
				{
					ret = SendHrRecord();
					if(ret == 0)
						break;
				}
			}

			if(ret == 0)//send day record to host every 40ms
				BLE_DataIsSending = 0;
			else
			{
				//Util_startClock(&ClockTimer_BleSend);
				if(NowConnIntvSlow){
					ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,20);//according to 
				}
				else{
					ke_timer_set(APP_BLE_SEND_TIMER,TASK_APP,5);
				}
			}
		}
		else
			BLE_DataIsSending = 0;
	}
}

/*******************************************************************
Function: GetAncsMask
Desc: 获取ANCS使能
Input: void
Output: void
*******************************************************************/
unsigned int GetAncsMask(void)
{
	return PersonInfo.ANCS_Mask;
}
#if 0
void Timer1S_enable(bool enable)
{
	if(enable)
	{
		if(timer1sValid)
			return;
		ke_timer_set(APP_1S_TIMER,TASK_APP,100);
		timer1sValid = 1;
	}
	else
	{
		if(hrMeasureing == 0 && gapCurState == BT_IDLE)
		{
			ke_timer_clear(APP_1S_TIMER,TASK_APP);
			timer1sValid = 0;
		}
	}
}
#endif

/*******************************************************************
Function: Kll_Initi
Desc: 卡路里算法初始化
Input: void
Output: void
*******************************************************************/
void Kll_Initi(void)
{
	Kll_Weight = (float)2.20462; // 165.3456
	Kll_Weight = Kll_Weight * (float)PersonInfo.Weight;
	Kll_Stature = (float)0.3937008;
	Kll_Stature = Kll_Stature * (float)PersonInfo.Stature;

	if(PersonInfo.Sex)
	{
		//男
		Kll_Base = (float)(66.00) + ((float)(6.23) * (float)Kll_Weight) + ((float)(12.70) * Kll_Stature) + ((float)(6.80) * (float)PersonInfo.Age);
		PersonInfo.StepLength = (unsigned long )(PersonInfo.Stature * (float)(0.415));
	}
	else
	{
		//女
		Kll_Base = (float)(655.00) + ((float)(4.35) * (float)Kll_Weight) + ((float)(4.70) * Kll_Stature) + ((float)(4.70) * (float)PersonInfo.Age);
		PersonInfo.StepLength = (unsigned long )(PersonInfo.Stature * (float)(0.413));
	}
	Kll_Base = Kll_Base / (float)1440.00;//1分钟的基础卡路里
	Kll_Base = Kll_Base / (float)3.00;//20Sec的基础卡路里
	UART_PRINTF("KllParam:%x,%x,%x,%x",PersonInfo.Weight,PersonInfo.Stature,PersonInfo.Sex,PersonInfo.Age);
	UART_PRINTF("KllBase:%x",(uint32_t)(Kll_Base*100));
}

/*******************************************************************
Function: GetStepDist
Desc: 根据步数算出距离及完成百分比
Input: void
Output: void
*******************************************************************/
void GetStepDist(void)
{
	//有产生步数，计算距离和完成百分比
	TodayRecord.SportDataTotal.DistTotal = (TodayRecord.SportDataTotal.StepsTotal * PersonInfo.StepLength) / 1000;
	//TargetCompleteRateLast = TodayRecord.SportDataTotal.TargetCompleteRate;
	TodayRecord.SportDataTotal.TargetCompleteRate =(TodayRecord.SportDataTotal.StepsTotal * 100) / PersonInfo.TargetSteps;
	if(TodayRecord.SportDataTotal.TargetCompleteRate > 100)
		TodayRecord.SportDataTotal.TargetCompleteRate = 100;
}

/*******************************************************************
Function: GetKll
Desc: 卡路里计数,RTC计数每10S钟计数一次
Input: void
Output: void
*******************************************************************/
void GetKll(void)
{
	// static unsigned char TimerSec = 0;

	//if(++TimerSec >= 20)
	//if(++TimerSec >= 2)
	//{
	//  TimerSec = 0;
	if(LastStepsMin != TodayRecord.SportDataTotal.StepsTotal)
	{
		GetStepDist();
	}

	if(LastStepsMin < TodayRecord.SportDataTotal.StepsTotal)
		StepsMin = TodayRecord.SportDataTotal.StepsTotal - LastStepsMin;
	else
		StepsMin = 0;
	LastStepsMin = TodayRecord.SportDataTotal.StepsTotal;

	if(StepsMin >= 10)
	{
		//20秒内最少走10步才算运动时间
		SportTime_s += 20;
		TodayRecord.SportDataTotal.SportTimeMinTotal = SportTime_s / 60;
	}
	//计算运动状态并加分
	//      放松状态(不记录活动分数)	0-16
	//      走路状态(不记录活动分数)	17-33
	//      活跃状态	34-50
	//      非常活跃状态	51-80
	//      超级活跃状态	81-999
	if(StepsMin > 0)
	{
		if(StepsMin <= 16)
		{
			Kll_Value = Kll_Value + Kll_Base;
		}
		else
		{
			if(StepsMin <= 33)
			{
				Kll_Value = Kll_Value + Kll_Base * (float)(1.20);
			}
			else
			{
				if(StepsMin <= 50)
				{
					Kll_Value = Kll_Value + Kll_Base * (float)(1.375);
				}
				else
				{
					if(StepsMin <= 80)
					{
						Kll_Value = Kll_Value + Kll_Base * (float)(1.55);
					}
					else
					{
						Kll_Value = Kll_Value + Kll_Base * (float)(1.725);
					}
				}
			}
		}
	}
	TodayRecord.SportDataTotal.KllTotal = (uint32_t)(Kll_Value * 100);
	//}
}

/*******************************************************************
Function: SportDataSave
Desc: 运动数据存储
Input: void
Output: void
*******************************************************************/
void SportDataSave(uint8_t timeSet)
{
	unsigned char i,SportHeadIndex,BitOffset,BitMask;
	//return;
	if(BLE_DataIsSending == 0)
	{
		//计算Quarter Index
		/*if(timeSet)
		{
			SaveTime.Year = RTC_CurTime.year;
			SaveTime.Month = RTC_CurTime.month;
			SaveTime.Day = RTC_CurTime.day;
			SaveTime.QuarterIndex = (((RTC_CurTime.hour >> 4) & 0x0F) * 10 + (RTC_CurTime.hour & 0x0F)) * 4 \
									+ ((((RTC_CurTime.min >> 4) & 0x0F) * 10 + (RTC_CurTime.min & 0x0F)) / 15);
			if(SaveTime.QuarterIndex == 0)
				SaveTime.QuarterIndex = 95;
			else
				SaveTime.QuarterIndex -= 1; 
		}
		else//SaveTime is from spi cmd*/
		{
			TodayRecord.Head.Year = SaveTime.Year;
			TodayRecord.Head.Month = SaveTime.Month;
			TodayRecord.Head.Day = SaveTime.Day;
			spi_getMcuTime();
			/*RTC_CurTime.year = SaveTime.Year;
			RTC_CurTime.month = SaveTime.Month;
			RTC_CurTime.day = SaveTime.Day ;
			
			if(SaveTime.QuarterIndex == 95)
				i = 0;
			else
				i = SaveTime.QuarterIndex+1;*/
			
			/*RTC_CurTime.hour = i/4;
			RTC_CurTime.hour = (RTC_CurTime.hour /10) << 4 | (RTC_CurTime.hour %10 &0x0F);
			RTC_CurTime.min = (i %4)*15;
			RTC_CurTime.min = (RTC_CurTime.min /10) << 4 | (RTC_CurTime.min %10 &0x0F);
			RTC_CurTime.sec = 0;
			//RTC_SetTimeHex(&rtcHex);
			RTC_SetTime(&RTC_CurTime);
			//RTC_GetTime(&RTC_CurTime);*/
				
		}
		
		//检查是否新的一天
		//if((SaveTime.Day != Last_SaveTime.Day) || (SaveTime.Month != Last_SaveTime.Month) || (SaveTime.Year != Last_SaveTime.Year))
		if(SaveTime.QuarterIndex == 95)
		{
			IsNewDay = 1;
			UART_PRINTF("is new day = 1\r\n");
			//spi_getMcuTime();
			//读取是否达到自动开关蓝牙时间
		}
		else
			IsNewDay = 0;
		

		//检查是否需要存储
		if((IsNewDay == 1) || (SaveTime.QuarterIndex != Last_SaveTime.QuarterIndex))
		{

			//TodayRecord.SportDataTotal.RunStepsTotal = 0;
			//TodayRecord.SportDataTotal.Speed = 0;
			//TodayRecord.SportDataTotal.Ex = 0;
			//TodayRecord.SportDataTotal.SportTimeMinTotal = 0; //运动时间要增加进来，请修改程序

			/*if(TodayRecord.SportDataTotal.KllTotal > 9999999)
				TodayRecord.SportDataTotal.KllTotal = 9999999;
			if(TodayRecord.SportDataTotal.StepsTotal > 9999999)
				TodayRecord.SportDataTotal.StepsTotal = 9999999;

			//QuarterSportData.IsSleepData = 0;
			if(TodayRecord.SportDataTotal.StepsTotal > LastSaveSteps)
				QuarterSportData.Steps = TodayRecord.SportDataTotal.StepsTotal - LastSaveSteps;
			else
				QuarterSportData.Steps = 0;*/

			/*if(TodayRecord.SportDataTotal.KllTotal > LastSaveKll)
			  QuarterSportData.Kll = TodayRecord.SportDataTotal.KllTotal - LastSaveKll;
			else
			  QuarterSportData.Kll = 0;*/

			/*if(TodayRecord.SportDataTotal.DistTotal > LastSaveDist)
			  QuarterSportData.Dist = TodayRecord.SportDataTotal.DistTotal - LastSaveDist;
			else
			  QuarterSportData.Dist = 0;
			QuarterSportData.RunSteps = 0;*/

			//add on 20181010
			if(SaveTime.QuarterIndex < 96)
			{
				UART_PRINTF("saveIndex%x\r\n",SaveTime.QuarterIndex);
				SportHeadIndex = SaveTime.QuarterIndex / 8;
				BitOffset = SaveTime.QuarterIndex % 8;
				//BitMask = 0x01;
				//for(i = 0; i < BitOffset; i++)
				//	BitMask <<= 1;
				BitMask = 1<< BitOffset;

				//read and modify the sport head
				if(SportHeadIndex < sizeof(TodayRecord.SportHead))
				{
					if(QuarterSportData.Steps)
					TodayRecord.SportHead[SportHeadIndex] |= BitMask;
				}

				//save the sport data
				TodayRecord.SportData[SaveTime.QuarterIndex].Steps = QuarterSportData.Steps;
				TodayRecord.SportData[SaveTime.QuarterIndex].sleepQ_Data[0] = 0xFF;//QuarterSportData.sleepQ_Data[0];
				TodayRecord.SportData[SaveTime.QuarterIndex].sleepQ_Data[1] = 0xFF;//QuarterSportData.sleepQ_Data[1];
				TodayRecord.SportData[SaveTime.QuarterIndex].sleepQ_Data[2] = 0xFF;//QuarterSportData.sleepQ_Data[2];
				TodayRecord.SportData[SaveTime.QuarterIndex].sleepQ_Data[3] = 0xFF;//QuarterSportData.sleepQ_Data[3];
			}

			if(IsNewDay == 1)
			{		
				UART_PRINTF("save Time%x,%x,%x",TodayRecord.Head.Year,TodayRecord.Head.Month,TodayRecord.Head.Day);
				Last_SaveTime.QuarterIndex = SaveTime.QuarterIndex;
				DM_SaveDataOneDay(&Last_SaveTime,&TodayRecord,(unsigned char *)&RTC_CurTime);
				/*TodayRecord.Head.Year = RTC_CurTime.year;
				TodayRecord.Head.Month = RTC_CurTime.month;
				TodayRecord.Head.Day = RTC_CurTime.day;
				UART_PRINTF("now Time%x,%x,%x",TodayRecord.Head.Year,TodayRecord.Head.Month,TodayRecord.Head.Day);*/
			}
			//end add
			//delete on 20181010
			/*
			  if(IsNewDay == 1)
			  {
			      if(DM_IsThisDaySaved(0) != 0)
			      {
			          SportHeadIndex = SaveTime.QuarterIndex / 8;
			          BitOffset = SaveTime.QuarterIndex % 8;
			          BitMask = 0x01;
			          for(i = 0; i < BitOffset; i++)
			            BitMask <<= 1;

			          DM_GetThisDaySportData(0,(unsigned char *)&DayRecord);
			          if((DayRecord.SportHead[SportHeadIndex] & BitMask) != 0)
			          {
			            //QuarterSportData.Kll  = QuarterSportData.Kll + DayRecord.SportData[SaveTime.QuarterIndex].Kll;
			            QuarterSportData.Steps  = QuarterSportData.Steps + DayRecord.SportData[SaveTime.QuarterIndex].Steps;
			            //QuarterSportData.Dist  = QuarterSportData.Dist + DayRecord.SportData[SaveTime.QuarterIndex].Dist;
			            //QuarterSportData.RunSteps  = QuarterSportData.RunSteps + DayRecord.SportData[SaveTime.QuarterIndex].RunSteps;
			          }
			      }

			      Last_SaveTime.QuarterIndex = SaveTime.QuarterIndex;
			      if((QuarterSportData.Steps != 0))
			      {
			          DM_SaveSportData(&Last_SaveTime,&QuarterSportData,&TodayRecord.SportDataTotal,0,IsNewDay);
			          DM_SetSaveTime((unsigned char *)&RTC_CurTime);
			      }

			      DM_NewDay(&SaveTime);
			  }
			  else
			  {
			      if((QuarterSportData.Steps != 0))
			      {
			          DM_SaveSportData(&SaveTime,&QuarterSportData,&TodayRecord.SportDataTotal,0,IsNewDay);
			          DM_SetSaveTime((unsigned char *)&RTC_CurTime);
			      }
			    // DM_GetThisDaySportData(0,(unsigned char *)&DayRecord);
			  }*/
			//end delete

			//LastSaveSteps = TodayRecord.SportDataTotal.StepsTotal;
			//LastSaveKll = TodayRecord.SportDataTotal.KllTotal;
			//LastSaveDist = TodayRecord.SportDataTotal.DistTotal;

			Last_SaveTime.Year = SaveTime.Year;
			Last_SaveTime.Month = SaveTime.Month;
			Last_SaveTime.Day = SaveTime.Day;
			Last_SaveTime.QuarterIndex = SaveTime.QuarterIndex;

			if(IsNewDay == 1)
			{
				PED_DataIniti();
				//IsTargetDisp = 0;IsTargetPassed = 0;//IsTargetVibration = 0;
				IsNewDay = 0;
				//IconFlag = Watch_GetCurIconFlag();
				//bros1 UI_NotifySend(&RTC_CurTime,UI_EVENT_ID_UPDATE_WEEK,0,IconFlag);
			}
		}
	}
}

void SaveSleepData(uint8_t index)
{
	uint8_t fakeSleep = 0;
	sleepExitTime.day = RTC_SaveTime.day;
	sleepExitTime.hour = RTC_SaveTime.hour;
	if(index <= 7)
		sleepExitTime.min = RTC_SaveTime.min+2*index;
	
	if(notDeepSleepMin<=3){
		fakeSleep = 1;
	}
	if(totalSleep >= 720)
	{
		fakeSleep = 1;
	}
	if(totalSleep >= 120 && notDeepSleepMin*60 < deepSleepMin)
	{
		fakeSleep = 1;
	}	
	
	if(totalSleep >15 && fakeSleep == 0){ //test
		DM_SaveSleepData(&sleepEnterTime,&sleepExitTime);
	}
	memset(tempSleepData,0xFF,sizeof(tempSleepData));
	tempSleepDataIndex = 0;
	deepSleepMin = 0;
	notDeepSleepMin = 0;
	totalSleep = 0;
}

void checkSleepStage(uint8_t sleepData, uint8_t index)
{
	uint8_t hold = 0;
	//UART_PRINTF("cur sleep state %x %x %x\r\n",sleepStage, sleepData, index);
	switch (sleepStage) 
	{
		case SLEEP_STAGE_IDLE:
			//if(sleepWakeInQuater[sleepDataOffset] >=1 && sleepWakeInQuater[sleepDataOffset] <=21)//>=1为了避免重复心率监测
			if(sleepData <= NORMAL_SLEEP)
			{
				sleepStage = SLEEP_STAGE_ENTERING;
				staticCnt = 0;
				hrChkFakeCnt = 0;
				onWrist = 0;
			}
			break;
		case SLEEP_STAGE_ENTERING:
			if(sleepData <= NORMAL_SLEEP)//falling asleep?, now we check if it is a fake sleep
			{
				//if(staticCnt++>=4)
				if(staticCnt++>=2) //now it is
				{
					/*sleepStage = SLEEP_STAGE_CHK_FAKE;
					hrChkFakeCnt = 0;
					if( hrMeasureing == HR_MEASURE_IDLE)
					{
						UART_PRINTF("start check fake sleep\r\n");
						HR_StartMeasurement(HR_MEASURE_CHK_SLEEP); //trigger heartrate start measurement to check if watch is on wrist
						hrMeasureTick = 1;
						ke_timer_set(APP_1S_TIMER,TASK_APP,100);
					}*/
					sleepStage = SLEEP_STAGE_CHK_FAKE;
					onWrist = 1; //get rid of the hr sleep check
				}
			}
			else{
				staticCnt = 0;
				if(sleepData >= AWAKE ){
					sleepStage = SLEEP_STAGE_IDLE;
				}
			}
			break;
		case SLEEP_STAGE_CHK_FAKE:
			if(onWrist)//onWrist只有在确认的ready状态才会置1
			{
				onWrist = 0;
				deepSleepMin = 0;
				notDeepSleepMin = 0;
				totalSleep = 0;
				tempSleepDataIndex = 0;
				sleepStage = SLEEP_STAGE_ENTERED;
				sleepEnterTime.day = RTC_SaveTime.day;
				sleepEnterTime.hour = RTC_SaveTime.hour;
				
				sleepEnterTime.min = RTC_SaveTime.min+index*2;

				tempSleepData[tempSleepDataIndex].sleepData[0] = QuarterSportData.sleepQ_Data[0];
				tempSleepData[tempSleepDataIndex].sleepData[1] = QuarterSportData.sleepQ_Data[1];
				tempSleepData[tempSleepDataIndex].sleepData[2] = QuarterSportData.sleepQ_Data[2];
				tempSleepData[tempSleepDataIndex].sleepData[3] = QuarterSportData.sleepQ_Data[3];
				tempSleepDataIndex++;
					
				//ke_timer_clear(APP_1S_TIMER,TASK_APP);
			}
			else{
				/*if(hrChkFakeCnt++<3){
					if( hrMeasureing == HR_MEASURE_IDLE)
					{
						HR_StartMeasurement(HR_MEASURE_CHK_SLEEP); //trigger heartrate start measurement to check if watch is on wrist
						hrMeasureTick = 1;
						ke_timer_set(APP_1S_TIMER,TASK_APP,100);
					}
				}
				else*/{
					hrChkFakeCnt = 0;
					staticCnt = 0;
					sleepStage = SLEEP_STAGE_ON_TABLE;
				}
			}
			break;				
		case SLEEP_STAGE_ENTERED:						
			if(sleepData >= AWAKE)
			{
				sleepStage = SLEEP_STAGE_IDLE;
				hrChkFakeCnt = 0;
				staticCnt = 0;
				SaveSleepData(index);		
				//calSleepQ_Data();
			}
			else {
				if(sleepData == DEEP_SLEEP){
					deepSleepMin++;
					if(index < 7)
					deepSleepMin++;
				}
				else //normal sleep & light sleep
				{
					notDeepSleepMin++;	
					if(index < 7)
						notDeepSleepMin++;	
				}
				totalSleep++;
				if(index <7)
					totalSleep++;
					
				if(tempSleepDataIndex < TEMP_SLEEP_DATA_BUF_LEN)
				{
					tempSleepData[tempSleepDataIndex].sleepData[0] = QuarterSportData.sleepQ_Data[0];
					tempSleepData[tempSleepDataIndex].sleepData[1] = QuarterSportData.sleepQ_Data[1];
					tempSleepData[tempSleepDataIndex].sleepData[2] = QuarterSportData.sleepQ_Data[2];
					tempSleepData[tempSleepDataIndex].sleepData[3] = QuarterSportData.sleepQ_Data[3];
					tempSleepDataIndex++;
				}
			}
			
			if(totalSleep >= 720)
			{//too long sleep? fake?
				sleepStage = SLEEP_STAGE_IDLE;
				//sleepExitTime.day = RTC_CurTime.day;
				//sleepExitTime.hour = RTC_CurTime.hour;
				//sleepExitTime.min = RTC_CurTime.min;
				tempSleepDataIndex = 0;	
				memset(tempSleepData,0xFF,sizeof(tempSleepData));	
			}
			break;
		case SLEEP_STAGE_ON_TABLE:
			//if(sleepData >= AWAKE){
			if(sleepData >= NORMAL_SLEEP){
				sleepStage = SLEEP_STAGE_IDLE;
			}
			break;
	}
}

void AnalyzeSleepData(void)
{
	uint8_t i;
	static uint8_t holdIndex = 0;
	for(i=holdIndex ; i<8; i++)
	{
		//if(hrMeasureTick == 0){ 
			checkSleepStage(sleepQ[i],i);
		//}
		//if(hrMeasureTick){//checking fake sleep by hr sensor
		//	holdIndex = i;
		//	return ;
		//}
	}
	holdIndex = 0;
}

/*******************************************************************
Function: GetData_FirstPowerOn
Desc: 闹钟检查
Input: void
Output: void
*******************************************************************/
void GetData_FirstPowerOn(void)
{
	if(DM_IsThisDaySaved(0) != 0)
	{
		//first responde
		//modify on 20181010
		/*
		DM_GetThisDaySportData(0,(unsigned char *)&DayRecord);
		TodayRecord.SportDataTotal.StepsTotal = DayRecord.SportDataTotal.StepsTotal;
		TodayRecord.SportDataTotal.RunStepsTotal = DayRecord.SportDataTotal.RunStepsTotal;
		TodayRecord.SportDataTotal.KllTotal = DayRecord.SportDataTotal.KllTotal;
		TodayRecord.SportDataTotal.DistTotal = DayRecord.SportDataTotal.DistTotal;
		TodayRecord.SportDataTotal.SportTimeMinTotal = DayRecord.SportDataTotal.SportTimeMinTotal;  */
		DM_GetThisDaySportData(0,(unsigned char *)&TodayRecord);
		UART_PRINTF("powerPoSteps:%x\r\n",TodayRecord.SportDataTotal.StepsTotal);
		//end modify

		//SportTime_s =  60;
		//SportTime_s = SportTime_s * TodayRecord.SportDataTotal.SportTimeMinTotal;
		//LastSaveSteps = TodayRecord.SportDataTotal.StepsTotal;
		//LastSaveKll = TodayRecord.SportDataTotal.KllTotal;
		//LastSaveDist = TodayRecord.SportDataTotal.DistTotal;

		//Step_output = TodayRecord.SportDataTotal.StepsTotal;
		Kll_Value = LastSaveKll = TodayRecord.SportDataTotal.KllTotal / 100;
	}
}
/*******************************************************************
Function: PED_DataIniti
Desc: 计步初始化各变量，启动后或跨天时调用
Input: void
Output: void
*******************************************************************/
void PED_DataIniti(void)
{
	//add 20181010
	uint8_t i ,j;

	for(i = 0; i < 12; i++)
		TodayRecord.SportHead[i] = 0;
	for(i = 0; i < 96; i++)
	{
		TodayRecord.SportData[i].Steps = 0;
		for(j = 0; j<4; j++)
			TodayRecord.SportData[i].sleepQ_Data[j] = 0xFF;
	}
	//end add

	f_kll = 0;
	TodayRecord.SportDataTotal.StepsTotal = 0;
	//SportSteps = 0;
	//StepsCur = StepsLast = 0;
	TodayRecord.SportDataTotal.RunStepsTotal = 0;
	TodayRecord.SportDataTotal.KllTotal = 0;
	TodayRecord.SportDataTotal.DistTotal = 0;
	TodayRecord.SportDataTotal.SportTimeMinTotal = 0;
	TodayRecord.SportDataTotal.TargetCompleteRate = 0;
//  TodayRecord.SportDataTotal.Speed = 0;
//  TodayRecord.SportDataTotal.Ex = 0;
	SportTime_s = 0;

	QuarterSportData.Steps = 0;
	/*for(j = 0; i<4; i++)
	{
		QuarterSportData.sleepQ_Data[j] = 0;
	}*/
	
	//QuarterSportData.Kll = 0;
	//QuarterSportData.Dist = 0;

	//LastSaveSteps = 0;
	//LastSaveKll = 0;
	//LastSaveDist = 0;
	//LastStepsMin = 0;
	//StepsMin = 0;
	//Kll_Value = 0;
}

void getCurHexTime(RTC * rtc_hex)
{
	rtc_hex->year = (RTC_CurTime.year >> 4 &0x0F)*10 + (RTC_CurTime.year &0x0F);
	rtc_hex->month = (RTC_CurTime.month >> 4 &0x0F)*10 + (RTC_CurTime.month &0x0F);
	rtc_hex->day = (RTC_CurTime.day >> 4 &0x0F)*10 + (RTC_CurTime.day &0x0F);
	rtc_hex->hour = (RTC_CurTime.hour >> 4 &0x0F)*10 + (RTC_CurTime.hour &0x0F);
	rtc_hex->min = (RTC_CurTime.min >> 4 &0x0F)*10 + (RTC_CurTime.min &0x0F);
	rtc_hex->sec = (RTC_CurTime.sec >> 4 &0x0F)*10 + (RTC_CurTime.sec &0x0F);
}

void powerOn(void)
{
	wdt_enable(10);
	while(1);
}

void powerOff(void)
{
	UART_PRINTF("power off cmd\r\n");
	UiSpiSendBuf[0] = SPI_CMD_READY_POWER_OFF;
	UiSpiSendBuf[1] = 0x1; //len
	UiSpiSendBuf[2] = WATCH_SOFT_VERSION;
	for(uint8_t i = 3; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_setBtVersion(void)
{
	UiSpiSendBuf[0] = SPI_CMD_SET_VERSION;
	UiSpiSendBuf[1] = WATCH_SOFT_VERSION;
	if(chipID == 0x21)
		UiSpiSendBuf[2] = 1;
	else
		UiSpiSendBuf[2] = 0;
	//for(uint8_t i = 2; i< SPI_CMD_LENG_MAX; i++)
	for(uint8_t i = 3; i< SPI_CMD_LENG_MAX; i++) //2022.5.12, add HR fpc test
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
	#if 0
	if(hrs3300_power_up_flg){
		hrs3300_power_up_flg = 0;
		Hrs3300_chip_disable(); //turn off 2S later to test the hr fpc, 2022.5.22
		i2cs_deinit();
	}
	#endif
}

void spi_setBtState(uint8_t state)
{
	UiSpiSendBuf[0] = SPI_CMD_SET_BT_STATE;
	UiSpiSendBuf[1] = state;
	for(uint8_t i = 2; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_setTimeMode(uint8_t timeMode)
{
	UiSpiSendBuf[0] = SPI_CMD_SET_TIMEMODE;
	UiSpiSendBuf[1] = PersonInfo.TimeDispMode;
	for(uint8_t i = 2; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_setBtTime(RTC * rtc_time)
{
	UiSpiSendBuf[0] = SPI_CMD_SET_TIME;
	UiSpiSendBuf[1] = rtc_time->year;
	UiSpiSendBuf[2] = rtc_time->month;
	UiSpiSendBuf[3] = rtc_time->day;
	UiSpiSendBuf[4] = rtc_time->hour;
	UiSpiSendBuf[5] = rtc_time->min;
	UiSpiSendBuf[6] = rtc_time->sec;
	for(uint8_t i = 7; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
	UART_PRINTF("BT TIME:%x,%x,%x,%x,%x",rtc_time->year,rtc_time->month,rtc_time->day,rtc_time->hour,rtc_time->min);
}

void spi_getMcuTime(void)
{
	UiSpiSendBuf[0] = SPI_CMD_GET_MCU_TIME;
	UiSpiSendBuf[1] = 0;
	for(uint8_t i = 2; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_setPerson(PERSON * person)
{
	uint32_t Kll_Base_Uint32 = (uint32_t)(Kll_Base*100);
	UiSpiSendBuf[0] = SPI_CMD_SET_PERSON;
	UiSpiSendBuf[1] = PersonInfo.StepLength;
	UiSpiSendBuf[2] = PersonInfo.DistUnit;
	UiSpiSendBuf[3] = Kll_Base_Uint32 >>24 &0xFF;
	UiSpiSendBuf[4] = Kll_Base_Uint32 >>16 &0xFF;
	UiSpiSendBuf[5] = Kll_Base_Uint32 >>8 &0xFF;
	UiSpiSendBuf[6] = Kll_Base_Uint32 & 0xFF;
	for(uint8_t i = 7; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_setAlarm(void)
{
	UART_PRINTF("set5");
	UiSpiSendBuf[0] = SPI_CMD_SET_ALARM;
	UiSpiSendBuf[1] = AlarmSave[0].IsEn;
	UiSpiSendBuf[2] = (AlarmSave[0].Hour >> 4 &0x0F) *10 + (AlarmSave[0].Hour&0x0F);
	UiSpiSendBuf[3] = (AlarmSave[0].Min >> 4 &0x0F) *10 + (AlarmSave[0].Min&0x0F);
	UiSpiSendBuf[4] = AlarmSave[0].Week;
	UiSpiSendBuf[5] = AlarmSave[1].IsEn;
	UiSpiSendBuf[6] = (AlarmSave[1].Hour >> 4 &0x0F) *10 + (AlarmSave[1].Hour&0x0F);
	UiSpiSendBuf[7] = (AlarmSave[1].Min >> 4 &0x0F) *10 + (AlarmSave[1].Min&0x0F);
	UiSpiSendBuf[8] = AlarmSave[1].Week;
	UiSpiSendBuf[9] = AlarmSave[2].IsEn;
	UiSpiSendBuf[10] = (AlarmSave[2].Hour >> 4 &0x0F) *10 + (AlarmSave[2].Hour&0x0F);
	UiSpiSendBuf[11] = (AlarmSave[2].Min >> 4 &0x0F) *10 + (AlarmSave[2].Min&0x0F);
	UiSpiSendBuf[12] = AlarmSave[2].Week;
	
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_setSleepPeriod(SLEEP_PERIOD * sleepPeriod)
{
	UART_PRINTF("set6");
	UiSpiSendBuf[0] = SPI_CMD_SET_SLEEP_PERIOD;
	UiSpiSendBuf[1] = sleepPeriod->en;
	UiSpiSendBuf[2] = sleepPeriod->StartHour;
	UiSpiSendBuf[3] = sleepPeriod->StartMin;
	UiSpiSendBuf[4] = sleepPeriod->EndHour;
	UiSpiSendBuf[5] = sleepPeriod->EndMin;
	for(uint8_t i = 6; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_notification(uint8_t notification)
{
	UART_PRINTF("set7");
	UiSpiSendBuf[0] = SPI_CMD_NOTIFICATION;
	UiSpiSendBuf[1] = notification;
	for(uint8_t i = 2; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}

	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_send_hr_state(hrs3300_results_t * p_hrs3300_result)
{
	UiSpiSendBuf[0] = SPI_CMD_SET_HR_STATE;
	switch (p_hrs3300_result->alg_status)
	{
		case MSG_NO_TOUCH:
			UiSpiSendBuf[1] = 0xFF;
			break;
		case MSG_PPG_LEN_TOO_SHORT:
			UiSpiSendBuf[1] = 0;
			break;
		case MSG_HR_READY:
			UiSpiSendBuf[1] = p_hrs3300_result->hr_result;
			break;
		default:
		//	UiSpiSendBuf[1] = p_hrs3300_result->hr_result; // SETTLE OR TIMEOUT condition
			break;
	}
	for(uint8_t i = 2; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void spi_send_app_trig_hr(void)
{
	UiSpiSendBuf[0] = SPI_CMD_APP_TRIG_HR;
	UiSpiSendBuf[1] = 0xFF;
		
	for(uint8_t i = 2; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}
	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}


void spi_getTotalRecord(void)
{
	UiSpiSendBuf[0] = SPI_CMD_GET_TOTAL_RECORD;
	UiSpiSendBuf[1] = 0xFF;
	for(uint8_t i = 2; i< SPI_CMD_LENG_MAX; i++)
	{
		UiSpiSendBuf[i] = 0xFF;
	}

	SPI_TransferNew(UiSpiTaskId++,(uint8_t *)UiSpiSendBuf,(uint8_t *)UiSpiRcvBuf,SPI_CMD_LENG_MAX);
}

void SPI_InitMcu(void) //SPI_InitMcu is called ahead of the start_advertising
{
	RTC rtcHex;
	//getCurHexTime(&rtcHex);
	rtcHex.year = 19;
	rtcHex.month = 1;
	rtcHex.day = 1;
	rtcHex.hour = 0;
	rtcHex.min = 0;
	rtcHex.sec = 0;
	UART_PRINTF("spi_initMcu,%x,%x\r\n",rtcHex.year,RTC_CurTime.year);
	spi_setBtVersion();
//	spi_setBtState(BT_ADVERTISING);
	spi_setTimeMode(PersonInfo.TimeDispMode);
	spi_setBtTime(&rtcHex);
	spi_setPerson(&PersonInfo);
	spi_setAlarm();
	//spi_setSleepPeriod(&SleepPeriod);
}

void spi_parse_getTime(uint8_t * cmdBuf)
{
	RTC rtc_hex;
	rtc_hex.year = cmdBuf[1];
	rtc_hex.month = cmdBuf[2];
	rtc_hex.day = cmdBuf[3];
	rtc_hex.hour = cmdBuf[4];
	rtc_hex.min = cmdBuf[5];
	rtc_hex.sec = cmdBuf[6];
	if(PersonInfo.TimeDispMode != cmdBuf[7])
	{
		PersonInfo.TimeDispMode = cmdBuf[7];
		DM_SetPerson(&PersonInfo);
	}
	
	RTC_SetTimeHex(&rtc_hex);
	RTC_GetTime(&RTC_CurTime);
	
	if(delayAck & DELAY_ACK_CMD_0x41)
	{
		delayAck &= ~DELAY_ACK_CMD_0x41;
		AckBuf[0] = CMD_GET_TIME;
		AckBuf[1] = RTC_CurTime.year;
		AckBuf[2] = RTC_CurTime.month;
		AckBuf[3] = RTC_CurTime.day;
		AckBuf[4] = RTC_CurTime.hour;
		AckBuf[5] = RTC_CurTime.min;
		AckBuf[6] = RTC_CurTime.sec;
		Host_Ack();
	}
	/*else{
		if(PowerOn ==0){
			PowerOn = 1;
		}
		else{
			if(RTC_CurTime.year == 0x19 && RTC_CurTime.month == 0x01 && RTC_CurTime.day == 0x01)
			{}
			else{
				if(RTC_CurTime.year >=0x19)
				ChkBlueAutoOnOff();
			}
		}
	}*/
	UART_PRINTF("getTimeCmd:%x,%x,%x,%x,%x,%x\r\n",RTC_CurTime.year,RTC_CurTime.month,RTC_CurTime.day,RTC_CurTime.hour,RTC_CurTime.min,RTC_CurTime.sec);
}

void spi_parse_getAlarm(uint8_t * cmdBuf)
{
	AlarmSave[0].IsEn = cmdBuf[1];
	AlarmSave[0].Hour = ((cmdBuf[2]/ 10) << 4) | ((cmdBuf[2] % 10) & 0x0F);
	AlarmSave[0].Min = ((cmdBuf[3]/ 10) << 4) | ((cmdBuf[3] % 10) & 0x0F);
	AlarmSave[0].Week = cmdBuf[4];
	DM_SetAlarm(&AlarmSave[0],0);
}

void spi_parse_togBtState(uint8_t * cmdBuf)
{
	if((gapCurState == BT_CONNECTED || gapCurState == BT_ADVERTISING))
	{
		if(cmdBuf[1] == 0){
			ToStopBluetooth = 1;
			StopBluetooth();
		}
	}
	else
	{
		if(cmdBuf[1] == 1){
			StartBluetooth();
		}
	}	
}

void spi_parse_takePhoto(void)
{
	sendTakePhotoCmd();
}

void spi_parse_new15MinData(uint8_t * cmdBuf)
{
	uint8_t saveTimeIndex, isInSleepPeriod = 0;
	uint16_t saveTimeMin;
	
	SaveTime.Year = ((cmdBuf[1]/ 10) << 4) | ((cmdBuf[1] % 10) & 0x0F);
	SaveTime.Month = ((cmdBuf[2]/ 10) << 4) | ((cmdBuf[2] % 10) & 0x0F);
	SaveTime.Day = ((cmdBuf[3]/ 10) << 4) | ((cmdBuf[3] % 10) & 0x0F);
	
	SaveTime.QuarterIndex = cmdBuf[4];
	
	RTC_SaveTime.year = SaveTime.Year;
	RTC_SaveTime.month = SaveTime.Month;
	RTC_SaveTime.day = SaveTime.Day ;
	
	//if(SaveTime.QuarterIndex == 95)
	//	saveTimeIndex = 0;
	//else
	//	saveTimeIndex = SaveTime.QuarterIndex+1;
	
	RTC_SaveTime.hour = SaveTime.QuarterIndex/4;
	//RTC_SaveTime.hour = (RTC_CurTime.hour /10) << 4 | (RTC_CurTime.hour %10 &0x0F);
	RTC_SaveTime.min = (SaveTime.QuarterIndex %4)*15;
	//RTC_SaveTime.min = (RTC_CurTime.min /10) << 4 | (RTC_CurTime.min %10 &0x0F);
	RTC_SaveTime.sec = 0;
	
	QuarterSportData.Steps = (uint16_t)cmdBuf[5]<<8 | (uint16_t)cmdBuf[6];
	QuarterSportData.sleepQ_Data[0] = cmdBuf[7];
	QuarterSportData.sleepQ_Data[1] = cmdBuf[8];
	QuarterSportData.sleepQ_Data[2] = cmdBuf[9];
	QuarterSportData.sleepQ_Data[3] = cmdBuf[10];
	sleepQ[0] = cmdBuf[7] >> 4;
	sleepQ[1] = cmdBuf[7] &0x0F;
	sleepQ[2] = cmdBuf[8] >> 4;
	sleepQ[3] = cmdBuf[8] &0x0F;
	sleepQ[4] = cmdBuf[9] >> 4;
	sleepQ[5] = cmdBuf[9] &0x0F;
	sleepQ[6] = cmdBuf[10] >> 4;
	sleepQ[7] = cmdBuf[10] &0x0F;
	
	SportDataSave(0);
	
	saveTimeMin =  RTC_SaveTime.hour * 60 + RTC_SaveTime.min;
	
	if(SleepStartMin > SleepEndMin)
	{
		if(saveTimeMin < 1440 && saveTimeMin > SleepStartMin)
			isInSleepPeriod =  1;
		if(/*curAbsMin >= 0 && */saveTimeMin < SleepEndMin)
			isInSleepPeriod = 1;
	}
	else if(SleepStartMin < SleepEndMin)
	{
		if(saveTimeMin > SleepStartMin && saveTimeMin < SleepEndMin)
		{
			isInSleepPeriod = 1;
		}
	}
	
	if(isInSleepPeriod) 
		AnalyzeSleepData();
	UART_PRINTF("save sport data\r\n");
}

void spi_parse_totalRecord(uint8_t * cmdBuf)
{
	RTC rtc;
	unsigned char *pAddr;
	rtc.year = ((cmdBuf[1]/10) << 4) | (cmdBuf[1] %10);
	rtc.month = ((cmdBuf[2]/10) << 4) | (cmdBuf[2] %10);
	rtc.day = ((cmdBuf[3]/10) << 4) | (cmdBuf[3] %10);
	TodayRecord.SportDataTotal.StepsTotal = ((uint32_t)cmdBuf[4]<<16) | ((uint32_t)cmdBuf[5]<<8) |(uint32_t)cmdBuf[6];
	TodayRecord.SportDataTotal.DistTotal = ((uint32_t)cmdBuf[7]<<16) | ((uint32_t)cmdBuf[8]<<8) |(uint32_t)cmdBuf[9];
	TodayRecord.SportDataTotal.KllTotal = ((uint32_t)cmdBuf[10]<<16) | ((uint32_t)cmdBuf[11]<<8) |(uint32_t)cmdBuf[12];
	if(delayAck & DELAY_ACK_CMD_0x07)
	{
		delayAck &= ~DELAY_ACK_CMD_0x07;
		GetStepDist();
		AckBuf[0] = CMD_GET_THIS_DAY_RECORD_TOTAL;
		AckBuf[1] = 0;  //responde index = 0
		AckBuf[2] = 0;
		AckBuf[3] = rtc.year;
		AckBuf[4] = rtc.month;
		AckBuf[5] = rtc.day;
		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.StepsTotal;
		AckBuf[6] = pAddr[2];
		AckBuf[7] = pAddr[1];
		AckBuf[8] = pAddr[0];
		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.RunStepsTotal;
		AckBuf[9] = pAddr[2];
		AckBuf[10] = pAddr[1];
		AckBuf[11] = pAddr[0];
		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.KllTotal;
		AckBuf[12] = pAddr[2];
		AckBuf[13] = pAddr[1];
		AckBuf[14] = pAddr[0];
		Host_Ack();

		// BLE_Send_Delay();

		//Secoend responde
		AckBuf[1] = 1;  //responde index = 1
		AckBuf[2] = 0;
		AckBuf[3] = rtc.year;
		AckBuf[4] = rtc.month;
		AckBuf[5] = rtc.day;

		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.DistTotal;
		AckBuf[6] = pAddr[2];
		AckBuf[7] = pAddr[1];
		AckBuf[8] = pAddr[0];
		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.SportTimeMinTotal;
		AckBuf[9] = pAddr[1];
		AckBuf[10] = pAddr[0];
		Host_Ack();
	}
	if(delayAck & DELAY_ACK_CMD_0x08)
	{
		delayAck &= ~DELAY_ACK_CMD_0x08;
		GetStepDist();
		AckBuf[0] = CMD_GET_THIS_DAY_TARGRT;
		AckBuf[1] = 0;
		AckBuf[2] = TodayRecord.Head.Year;
		AckBuf[3] = TodayRecord.Head.Month;
		AckBuf[4] = TodayRecord.Head.Day;

		AckBuf[5] = TodayRecord.SportDataTotal.TargetCompleteRate;
		//pAddr = (unsigned char *)&TodayRecord.SportDataTotal.Speed;
		//AckBuf[6] = pAddr[1];
		//AckBuf[7] = pAddr[0];
		//pAddr = (unsigned char *)&TodayRecord.SportDataTotal.Ex;
		//AckBuf[8] = pAddr[2];
		//AckBuf[9] = pAddr[1];
		//AckBuf[10] = pAddr[0];
		pAddr = (unsigned char *)&TodayRecord.SportDataTotal.TargetCompleteRate;
		AckBuf[11] = pAddr[1];
		AckBuf[12] = pAddr[0];
		AckBuf[13] = 0;
		AckBuf[14] = 0;
		Host_Ack();
	}
}

void spi_parse_power_on_off(uint8_t * cmdBuf)
{
	if(cmdBuf[1])
		powerOn();
	else{
		if(gapCurState != BT_IDLE)
		{
			poweringOff = 1; //now turn off ble first
			ToStopBluetooth = 1;
			StopBluetooth();
		}
		else
		{
			powerOff();
		}	
	}
}

void spi_hr_mode_in_out(uint8_t * cmdBuf)
{
	//uint8_t i,HR_Max,HR_Min,validNum;
	uint8_t hr;
	uint16_t sum = 0;
	if(cmdBuf[1])
	{
		inHrMode = 1;
		i2cs_init(); //为了尽快拉高SDA和SCL线，因为他们初始状态是输入下拉的
		ke_timer_set(APP_1S_TIMER, TASK_APP, 100);
		//HR_StartMeasurement(HR_MEASURE_MANUAL);
		//Timer1S_enable(true);
	}
	else{
		inHrMode = 0;
		hr = get_last_hr();
		if(hr >=25 && hr <= 220){
			HR_Data[HR_DataIndex].AvarageHR = hr;
			HR_Data[HR_DataIndex].year = ((cmdBuf[2]/ 10) << 4) | ((cmdBuf[2] % 10) & 0x0F);
			HR_Data[HR_DataIndex].month = ((cmdBuf[3]/ 10) << 4) | ((cmdBuf[3] % 10) & 0x0F);
			HR_Data[HR_DataIndex].day = ((cmdBuf[4]/ 10) << 4) | ((cmdBuf[4] % 10) & 0x0F);
			HR_Data[HR_DataIndex].hour = ((cmdBuf[5]/ 10) << 4) | ((cmdBuf[5] % 10) & 0x0F);
			HR_Data[HR_DataIndex].min = ((cmdBuf[6]/ 10) << 4) | ((cmdBuf[6] % 10) & 0x0F);
			HR_DataIndex++;
			if(HR_DataIndex >= HR_RECORD_MAX)
				HR_DataIndex = 0;
		}
		HR_StopMeasurement(HR_MEASURE_MANUAL);
		i2cs_deinit(); //模块不插的话上面也会初始化I2C口线，需要重新设为输入下拉
		//Timer1S_enable(false);
		#if 0
		HR_Max = 0;
		HR_Min = 0xFF;
		validNum = 0;
		for(i = 0; i< sizeof(HeartRateBuf); i++)
		{
			if(HeartRateBuf[i] >=25 && HeartRateBuf[i] <= 220)
			{
				if(HeartRateBuf[i] < HR_Min)
					HR_Min = HeartRateBuf[i];
				if(HeartRateBuf[i] > HR_Max)
					HR_Max = HeartRateBuf[i];
				sum += HeartRateBuf[i];	
				validNum++;
			}
			HeartRateBuf[i] = 0;
		}
		if(validNum)
		{
			HR_Average = sum / validNum;

			HR_Data[HR_DataIndex].AvarageHR = HR_Average;
			HR_Data[HR_DataIndex].MinHR = HR_Min;
			HR_Data[HR_DataIndex].MaxHR = HR_Max;
			HR_DataIndex++;
			//fix me, should store it to?
				
			//move to HR_SaveDayRecord,prepare for record cmd acquire(today record query);
			//HR_SaveDayRecord.Head.Year = RTC_CurTime.year;
			//HR_SaveDayRecord.Head.Month = RTC_CurTime.month;
			//HR_SaveDayRecord.Head.Day = RTC_CurTime.day;
			//memcpy(&HR_SaveDayRecord.HrData[saveTimeIndex40Min*8],(unsigned char *)HR_Data,sizeof(HR_DATA)*8);
			//HR_SaveDayRecord.HrBitMask[saveTimeIndex40Min] = HrMask8bit;
		}
		else{
			HR_Data[HR_DataIndex].AvarageHR = 0;
			HR_Data[HR_DataIndex].MinHR = 0;
			HR_Data[HR_DataIndex].MaxHR = 0;	
			//HrMask8bit &= ~(1<<(HR_DataIndex));
		}
		#endif
		
	}
}

extern volatile AxesRaw_t gsen_buf;

void spi_hr_gs_data_refresh(uint8_t * cmdBuf)
{
	gsen_buf.AXIS_X = ((int8_t)cmdBuf[1])*64;
	gsen_buf.AXIS_Y = ((int8_t)cmdBuf[2])*64;
	gsen_buf.AXIS_Z = ((int8_t)cmdBuf[3])*64;
	UART_PRINTF("x= %d,y= %d,z= %d\r\n",gsen_buf.AXIS_X,gsen_buf.AXIS_Y,gsen_buf.AXIS_Z);
}


/*******************************************************************
Function: WatchIniti
Desc: 手表初始化
Input: void
Output: void
*******************************************************************/
void WatchIniti(void)
{
	RTC Rtc,save_rtc;
	SAVE_TIME LastTime;

	//初始化按键
	//delete on V20 20180925
	//ZenoKeyInit(Watch_keyChangeHandler);
	//end delete
	//初始化RTC,使能每10S和每分回调
//	RTC_init();


	//初始化UI界面
	//UI_initi(Watch_SpiTransferHandler,Watch_SpiTransferHandler1);
	//初始化计步Sensor，注册回调
	//StepsSenInit(Watch_StepsSenHandler);
	//存储管理初始化
	
	bond_db_init();
	
	DM_Initi();

	//DM_GetSaveTime((unsigned char *)&save_rtc);
	//DM_GetLastSaveTime((unsigned char *)&LastTime);

	/*if(++LastTime.QuarterIndex > 95)
		LastTime.QuarterIndex = 0;

	Rtc.first_addr = 0x02;
	Rtc.sec = save_rtc.sec;
	Rtc.min = save_rtc.min;
	Rtc.hour = save_rtc.hour;
	Rtc.day = save_rtc.day;
	Rtc.month = save_rtc.month;
	Rtc.year = save_rtc.year;

	RTC_SetTime(&Rtc);
	RTC_GetTime(&RTC_CurTime);

	if(LastTime.QuarterIndex == 0)
		LastTime.QuarterIndex = 95;
	else
		LastTime.QuarterIndex -= 1;

	SaveTime.Year = Last_SaveTime.Year = SleepMode_Last_SaveTime.Year = LastTime.Year;
	SaveTime.Month = Last_SaveTime.Month = SleepMode_Last_SaveTime.Month = LastTime.Month;
	SaveTime.Day = Last_SaveTime.Day = SleepMode_Last_SaveTime.Day = LastTime.Day;
	SaveTime.QuarterIndex = Last_SaveTime.QuarterIndex = SleepMode_Last_SaveTime.QuarterIndex = LastTime.QuarterIndex;

	IsRegistOk = 1;

	DM_GetPerson(&PersonInfo);

	DM_GetAlarm(&AlarmSave[0],0);
	DM_GetAlarm(&AlarmSave[1],1);
	DM_GetAlarm(&AlarmSave[2],2);*/
	PED_DataIniti();
	//add on 20181010
	TodayRecord.Head.Year = RTC_CurTime.year;
	TodayRecord.Head.Month = RTC_CurTime.month;
	TodayRecord.Head.Day = RTC_CurTime.day;
	UART_PRINTF("today%x,%x,%x",RTC_CurTime.year,RTC_CurTime.month,RTC_CurTime.day);
	//end add

	//KLL算法初始化
	Kll_Initi();
	//GetData_FirstPowerOn();
	
	//i2c_init(I2C_DEFAULT_SLAVE_ADDRESS, 0);
	//Delay_ms(5);
	//Hrs3300_chip_init();
	//Hrs3300_chip_enable();
	//Hrs3300_chip_disable();
	#ifdef I2C_HARDware
	i2c_deinit();
	#else
	i2cs_init();
	Hrs3300_chip_init();
	Hrs3300_chip_enable();
	ke_timer_set(APP_HR_TEST_TO_TIMER, TASK_APP,280);
	//Hrs3300_chip_disable(); //turn off 2S later to test the hr fpc, 2022.5.22
	//i2cs_deinit();
	#endif
	#ifndef POWERON_SYNC	
	SPI_Transfer_init();
	SPI_InitMcu();
	#else
	gpio_set(SPI_SYNC_PIN,1);
	gpio_config(SPI_SYNC_PIN, OUTPUT, PULL_NONE);
	gpio_config(SPI_IRQ_PIN, INPUT, PULL_NONE);
	gpio_config(GPIO_SPI_NSS_PIN, INPUT, PULL_NONE);
	ke_timer_set(APP_POWERON_SYNC_TIMER, TASK_APP,80);
	UART_PRINTF("init finish\r\n");
	#endif
}

/*******************************************************************
Function: Watch_RtcUpdateEvt
Desc: 手表RTC更新事件处理
Input: uint8_t rtc_notify
Output: void
*******************************************************************/
void Watch_RtcUpdateEvt(void)
{
	static uint8_t Rtc5S_Cnt = 0;
	uint8_t notify = 0;
//	static uint8_t MinCnt = 0;
// uint8_t flag;

	//更新当前时间
	//ZenoRtcGetTime_PowerSave(&RTC_CurTime);


	//if(ToStopBluetooth != 0)
	//	StopBluetooth();


	//RTC秒更新通知
	

	if(gapCurState == BT_CONNECTED)
	{
		if(IsRealTimeMode)
			Watch_RealStepsSend();    //实时模式下运动数据每秒自动上传到主机
		if(ancs_notify != 0)
		{
			//WatchVibrationAction(2, 80, 0);
			//UI_NotifySend(&RTC_CurTime,UI_EVENT_ID_TEL,0,IconFlag);
			//add by zeno 20180426
			//BeepPlayStart(BEEP_NUM_4);
			//end add
			if(ancs_notify > 0xFF)
				notify = NOTIFY_MSG; //扩展通知统一使用MSG通知
			else
				notify = ancs_notify & 0xFF;
			spi_notification(notify);
			ancs_notify = 0;
		}
		if(IsFastConnect != 0)
		{
			if(++ConectParamUpdataTimer >= 10)
			{
				if(BLE_DataIsSending == 0)
				{
					ConectParamUpdataTimer = 0;
					WatchConnectParamUpdata(0);
				}
				else
				{
					ConectParamUpdataTimer = 10;
				}
			}
		}	
	}


	/*if(rtc_notify & RTC_NOTIFY_MIN)
	{
		//RTC 分钟更新通知
		RTC_GetTime(&RTC_CurTime);
		ChkBlueAutoOnOff();

		// 检查闹钟
		//CheckAlarm(&RTC_CurTime);
		//时间更新，检查存储
		//SportDataSave();

	}*/

}

/*******************************************************************
Function: Watch_BleStateChangeEvt
Desc: 手表蓝牙状态改变事件处理
Input: uint8_t state 当前蓝牙状态
Output: void
*******************************************************************/
void Watch_BleStateChangeEvt(uint8_t state)
{

	if(poweringOff == 0){
	if(state != gapCurState)
	{
		gapCurState = state;
		if(state == BT_CONNECTED)
		{
			ConectParamUpdataTimer = 0;
			IsFastConnect = 1; //初始连接就按手机的参数
			NowConnIntvSlow = 0;
			rtc_notify_mask = 1;
			if(RTC_IsAlive() == 0){
				spi_getMcuTime();
			}
//			ke_timer_set(APP_1S_TIMER,TASK_APP, 100);
		} 
		else
		{
			ConectParamUpdataTimer = 0;
			IsFastConnect = 0;
			NowConnIntvSlow = 0;
			delayAck = 0;
			rtc_notify_mask = 0;
			//RTC_Stop();
				if(state == BT_IDLE)
				{
					ToStopBluetooth = 0;
				}
			}
			if(state <= BT_CONNECTED)
				spi_setBtState(state);
		}
	}
	else{
		//1. if connected when power off, disconnect callback will trigger idle state change
		//2. if advertising when power off, stopBluetooth call in cmd parse will stop adv,the stop adv callback then will call idle state change
		//3. if idle when power off, will call powerOffAck in cmd parse
		if(state == BT_IDLE){ 	
			powerOff() ;
		}
	}
}

uint8_t getCurBleState(void)
{
	return gapCurState;
}

/*******************************************************************
ANCS App ID list
*******************************************************************/
const unsigned char APP_NAME_CALL[21] = "com.apple.mobilephone";
const unsigned char APP_NAME_MESSAGE[19] = "com.apple.MobileSMS";
const unsigned char APP_NAME_QQ[15] = "com.tencent.mqq";
const unsigned char APP_NAME_QQ_TIM[15] = "com.tencent.tim";
const unsigned char APP_NAME_WECHAT[15] = "com.tencent.xin";
const unsigned char APP_NAME_FACEBOOK[12] = "com.facebook";
//const unsigned char APP_NAME_FACEBOOK[] = "com.facebook.Facebook";
//const unsigned char APP_NAME_MESSENGER[] = "com.facebook.Messenger";
const unsigned char APP_NAME_TWITTER[19] = "com.atebits.Tweetie";
//const unsigned char APP_NAME_SKYPE[] = "com.skype.tomskype";
const unsigned char APP_NAME_SKYPE[9] = "com.skype";
//const unsigned char APP_NAME_WHATSAPP[] = "net.whatsapp.WhatsApp";
const unsigned char APP_NAME_WHATSAPP[12] = "net.whatsapp";
const unsigned char APP_NAME_INSTAGRAM[] = "com.burbn.instagram";
const unsigned char APP_NAME_TELEGRAM[] = "ph.telegra.Telegraph";
const unsigned char APP_NAME_VIBER[] = "com.viber";
const unsigned char APP_NAME_LINE[] = "jp.naver.line";
const unsigned char APP_NAME_KAKAO[] = "com.iwilab.KakaoTalk";
const unsigned char APP_NAME_LINKEDIN[] = "com.linkedin";//com.linkedin.LinkedIn";com.linkedin.Zephyr
const unsigned char APP_NAME_TIKTOK[] = "com.zhiliaoapp.musically";
const unsigned char APP_NAME_DOUYIN[] = "com.ss.iphone.ugc.Aweme";
const unsigned char APP_NAME_MAIL1[] = "mail";
const unsigned char APP_NAME_MAIL2[] = "Mail";
const unsigned char APP_NAME_YAHOO_MAIL[] = "com.yahoo.Aerogram";
//com.tencent.qqmail ,com.netease.mailmaster
//const unsigned char APP_NAME_GMAIL[] = "com.google.Gmail"; //contain 'mail'
const unsigned char APP_NAME_OUTLOOK[] = "com.microsoft.Office.Outlook";
//const unsigned char APP_NAME_APPLEMAIL[] = "com.apple.mobilemail"; //contain 'mail'
/*******************************************************************
Function: IsMatchStr
Desc: APP ID 匹配检查
Input: uint8_t * pAncsChar,const unsigned char * pStr, unsigned char len_max
Output: uint8 = 1匹配OK， = 0不匹配
*******************************************************************/
uint8_t IsMatchStr(uint8_t * pAncsChar,const unsigned char * pStr, unsigned char len_max)
{
	unsigned char i;
	uint8_t IsMatch;

	IsMatch = 1;
	for(i = 0; i < len_max; i++)
	{
		if(pAncsChar[i] != pStr[i])
		{
			IsMatch = 0;
			break;
		}
	}

	return IsMatch;
}


/*******************************************************************
Function: Watch_AncsNotifyChk
Desc: 检查ANCS通知是否已使能并匹配
Input: uint8_t *pAncsChar --> App ID string
Output: void
*******************************************************************/
void Watch_AncsNotifyChk(uint8_t *pAncsChar)
{
	//UART_PRINTF("Notify check\r\n");
	//Call
	if(IsMatchStr(pAncsChar,APP_NAME_CALL,sizeof(APP_NAME_CALL)) != 0)
	{
		UART_PRINTF("call RCV\r\n");
		if(GetAncsMask() & ANCS_MASK_CALL)
		{
			ancs_notify = NOTIFY_CALL;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_MESSAGE,sizeof(APP_NAME_MESSAGE)) != 0)//Message
	{
		UART_PRINTF("MSG RCV\r\n");
			
		if(GetAncsMask() & ANCS_MASK_MESSAGE)
		{
			ancs_notify = NOTIFY_MSG;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_TWITTER,sizeof(APP_NAME_TWITTER)) != 0)
	{
		//Twitter

			UART_PRINTF("twitter RCV\r\n");
			if(GetAncsMask() & ANCS_MASK_TWITTER)
			{
				ancs_notify = NOTIFY_TWITTER;
			}
	}
	else if((IsMatchStr(pAncsChar,APP_NAME_QQ,sizeof(APP_NAME_QQ)) != 0) || (IsMatchStr(pAncsChar,APP_NAME_QQ_TIM,sizeof(APP_NAME_QQ_TIM))!= 0)) //QQ
	{
			 
		UART_PRINTF("QQ RCV\r\n");
		if(GetAncsMask() & ANCS_MASK_QQ)
		{
			ancs_notify = NOTIFY_MQQ;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_WECHAT,sizeof(APP_NAME_WECHAT)) != 0) //Wechat			
	{
		UART_PRINTF("WX RCV\r\n");
		if(GetAncsMask() & ANCS_MASK_WEIXIN)
		{
			ancs_notify = NOTIFY_WECHAT;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_FACEBOOK,sizeof(APP_NAME_FACEBOOK)) != 0)
	{
		//Facebook
		
		if(GetAncsMask() & ANCS_MASK_FACEBOOK)
		{
			ancs_notify = NOTIFY_FACEBOOK;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_WHATSAPP,sizeof(APP_NAME_WHATSAPP)) != 0)
	{
		//WhatsApp
		
		if(GetAncsMask() & ANCS_MASK_WHATSAPP)
		{
			ancs_notify = NOTIFY_WHATSAPP;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_SKYPE,sizeof(APP_NAME_SKYPE)) != 0)
	{
		//Skype
		
		if(GetAncsMask() & ANCS_MASK_SKYPE)
		{
			ancs_notify = NOTIFY_SKYPE;
		}
	}

	else if(IsMatchStr(pAncsChar,APP_NAME_INSTAGRAM,sizeof(APP_NAME_INSTAGRAM)-1) != 0)
	{
		UART_PRINTF("Instagram notify,%x",GetAncsMask());
		if(GetAncsMask() & ANCS_MASK_INSTAGRAM)
		{		
			ancs_notify = NOTIFY_INSTAGRAM;
			UART_PRINTF("Instagram notify1,%x",ancs_notify);
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_TELEGRAM,sizeof(APP_NAME_TELEGRAM)-1) != 0)
	{
		if(GetAncsMask() & ANCS_MASK_TELEGRAM)
		{
			ancs_notify = NOTIFY_TELEGRAM;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_VIBER,sizeof(APP_NAME_VIBER)-1) != 0)
	{
		if(GetAncsMask() & ANCS_MASK_VIBER)
		{
			ancs_notify = NOTIFY_VIBER;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_LINE,sizeof(APP_NAME_LINE)-1) != 0)
	{
		if(GetAncsMask() & ANCS_MASK_LINE)
		{
			ancs_notify = NOTIFY_LINE;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_KAKAO,sizeof(APP_NAME_KAKAO)-1) != 0)
	{
		if(GetAncsMask() & ANCS_MASK_KAKAO)
		{
			ancs_notify = NOTIFY_KAKAO;
		}
	}
	else if(IsMatchStr(pAncsChar,APP_NAME_LINKEDIN,sizeof(APP_NAME_LINKEDIN)-1) != 0)
	{
		if(GetAncsMask() & ANCS_MASK_LINKEDIN)
		{
			ancs_notify = NOTIFY_LINKEDIN;
		}
	}
	else if((IsMatchStr(pAncsChar,APP_NAME_TIKTOK,sizeof(APP_NAME_TIKTOK)-1) != 0) || \
		 (IsMatchStr(pAncsChar,APP_NAME_DOUYIN,sizeof(APP_NAME_DOUYIN)-1) != 0))
	{
		if(GetAncsMask() & ANCS_MASK_TIKTOK)
		{
			ancs_notify = NOTIFY_TIKTOK;
		}
	}
	
	else if(strstr((const char *)pAncsChar,(const char *)APP_NAME_MAIL1) != NULL || 
			strstr((const char *)pAncsChar,(const char *)APP_NAME_MAIL2) != NULL || \
		IsMatchStr(pAncsChar,APP_NAME_YAHOO_MAIL,sizeof(APP_NAME_YAHOO_MAIL)-1) != 0 || \
		 IsMatchStr(pAncsChar,APP_NAME_OUTLOOK,sizeof(APP_NAME_OUTLOOK)-1) != 0)
			 
	{
		if(GetAncsMask() & ANCS_MASK_MAIL)
		{
			ancs_notify = NOTIFY_MAIL;
		}
	}
	else{
		if(GetAncsMask() & ANCS_MASK_OTHER_SOCIAL)
		{
			ancs_notify = NOTIFY_MSG;
		}
	}
	
	
	/*
	    if((ancs_call_notify != 0) || (ancs_message_notify != 0))
	    {
	      //投递提醒事件消息
	      AncsApp_enqueueMsg(WATCH_MSG_MESSAGE_REMIND_EVT, 0,0);
	    }*/
	//WatchVibrationAction(1, 80, 0);
}

void setAncsNotify(uint32_t notify)
{
	ancs_notify = notify;
}

void spi_cmd_parse(void)
{
	switch (spi_cmd_RxBuf[0])
	{
		case SPI_CMD_MCU_TIME:
			spi_parse_getTime(spi_cmd_RxBuf);
			break;
		case SPI_CMD_MCU_SET_BT_STATE:
			spi_parse_togBtState(spi_cmd_RxBuf);
				//SPI_InitMcu();
			break;
		case SPI_CMD_MCU_15MIN_RECORD:
			spi_parse_new15MinData(spi_cmd_RxBuf);
			break;
		case SPI_CMD_MCU_TOTAL_RECORD:
			spi_parse_totalRecord(spi_cmd_RxBuf);
			break;
		case SPI_CMD_MCU_TAKE_PHOTO:
			spi_parse_takePhoto();
			UART_PRINTF("take photo\r\n");
			break;
		case SPI_CMD_MCU_ALARM:
			spi_parse_getAlarm(spi_cmd_RxBuf);
			break;
		case SPI_CMD_MCU_POWER_ON_OFF:
			spi_parse_power_on_off(spi_cmd_RxBuf);
			break;
		case SPI_CMD_MCU_HR_MODE:
			spi_hr_mode_in_out(spi_cmd_RxBuf);
			break;
		case SPI_CMD_MCU_GS_DATA:
			spi_hr_gs_data_refresh(spi_cmd_RxBuf);
			break;
		//case CMD_SAVE_RECORD:
		//	break;
	}
}



