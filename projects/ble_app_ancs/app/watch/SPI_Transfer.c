#include <string.h>
#include "compiler.h"
//#include "nrf_drv_spis.h"

#include "arch.h"
#include "SPI_Transfer.h"
#include "gpio.h"
#include "ke_timer.h"
#include "app_task.h"
#include "rf.h"
#include "uart.h"
#include "Watch.h"
//#include "common.h"


//SPI延迟时间
#define SPI_IRQ_DELAY_TIME	 3 /* 200us*/
#define SPI_IRQ_TRAN_TIMEOUT_TIME	 10000/* 10000*100US = 1000MS = 1Sec*/



//SPI任务最大数量
#define SPI_TASK_MAX		   10

//定义SPI状态
#define SPI_STATE_INIT		   0
#define SPI_STATE_FREE		   1
#define SPI_STATE_SEND_CMD		  2
#define SPI_STATE_SEND_RECOVER	3
#define SPI_STATE_IRQ_OFF	   3


typedef  struct
{
	uint8_t SendBuffer[SPI_CMD_LENG_MAX];
	uint8_t *pRec;
	uint8_t TranLeng;
	uint8_t TaskId;
} SPI_TRAN_TASK ;

typedef  struct
{
	uint8_t RecvBuffer[SPI_CMD_LENG_MAX];
	//uint8_t *pRec;
	//uint8_t TranLeng;
	uint8_t TaskId;
} SPI_RECV_TASK ;


static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

static SPI_TRAN_TASK SpiTranTask[SPI_TASK_MAX];
uint8_t SPI_TxBuf[SPI_CMD_LENG_MAX] = {0xAA, 0x81, 0x07,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8};
uint8_t SPI_RxBuf[SPI_CMD_LENG_MAX];
uint8_t spi_cmd_RxBuf[SPI_CMD_LENG_MAX];
uint8_t SPI_ReTxBuf[SPI_CMD_LENG_MAX];
static uint8_t SpiState = SPI_STATE_INIT;
static uint8_t SpiTaskIndex = 0;
uint8_t SpiTxIndex;
//static uint8_t RecBuffer[SPI_CMD_LENG_MAX];
static uint8_t TaskId_Cur;
uint8_t spiResendCnt;

static uint8_t *pSend = NULL, *pRec = NULL, SendLeng = 0;
//static uint8_t testbuf[SPI_TASK_MAX] = {0,1,2,3,4,5};
//static uint8_t CurTaskIsRunning = 0;
static uint16_t TaskNum_Start = 0,TaskNum_Finish = 0;
static uint8_t SpiIsOn = 0;


// Pointer to application callback
static WatchSpiCB_t appSpiHandler = NULL,appSpiHandler1 = NULL;

uint8_t spi_bt2mcu;
uint8_t spi_mcu2bt;
//volatile uint8_t syncPinLow;
// Key debounce clock

uint8_t SpiTranTaskNext(void);
void app_spi_cmd_transfered_handler(void);
/*
static void SpiIrqTimerHandler(void * p_context)
{
  if(appSpiHandler != NULL)
	appSpiHandler(0,0,NULL);
  timerStarted &= ~(uint16_t)(1 << TIMER_ID_SPI_IRQ);
}

static void SpiTimeoutTimerHandler(void * p_context)
{
  if(appSpiHandler != NULL)
	appSpiHandler(0,0,NULL);
  timerStarted &= ~(uint16_t)(1 << TIMER_ID_SPI_TIMEOUT);
}
*/

/*******************************************************************
Function: SPI_Transfer_IRQ_On
Desc: Enable CS of SPI line
Input: void
Output: void
*******************************************************************/
void SPI_Transfer_IRQ_On(void)
{
	gpio_set(SPI_SYNC_PIN, 0);
//	syncPinLow = 1;
	//nrf_gpio_pin_set(SPI_IRQ_PIN);
}

/*******************************************************************
Function: SPI_Transfer_IRQ_Off
Desc: Disnable CS of SPI line
Input: void
Output: void
*******************************************************************/
void SPI_Transfer_IRQ_Off(void)
{
	//UART_PRINTF("irq off");
	gpio_set(SPI_SYNC_PIN, 1);
//	syncPinLow = 0;
	//nrf_gpio_pin_clear(SPI_IRQ_PIN);
}

/*******************************************************************
Function: SPI_Transfer_init
Desc: SPI模块初始化
Input: void
Output: void
*******************************************************************/
void SPI_Transfer_init(void)
{
	SpiState = SPI_STATE_INIT;
	//appSpiHandler = appCB;
	//appSpiHandler1 = appCB1;

	spi_slave_int_init();

	
	//Initialize state
	pRec = NULL;
	pSend = NULL;
	SpiState = SPI_STATE_FREE;
	SpiTaskIndex = 0;
	SpiTxIndex = 0;
	SpiIsOn = 1;

}

/*******************************************************************
Function: SPI_Transfer_Stop
Desc: SPI模块关闭，省电
Input: void
Output: void
*******************************************************************/
void SPI_Transfer_Stop()
{
	SpiIsOn = 0;
	//关闭SPI外设
	//bros1 SPI_close(spi);

	spi_slave_reset();
	//将4个IO设成GPIO

	//所有IO设成0
	/*PIN_setOutputValue(hAsGpio_Pin, PIN_GPIO_MOSI, 0);
	PIN_setOutputValue(hAsGpio_Pin, PIN_GPIO_SCK, 0);
	PIN_setOutputValue(hAsGpio_Pin, PIN_GPIO_CS, 0);
	PIN_setOutputValue(hAsGpio_Pin, PIN_GPIO_MISO, 0);*/
	//SPI_Transfer_IRQ_Off();
	UART_PRINTF("IRQ HIGH 2");
}



/*******************************************************************
Function: SPI_Send
Desc: 发送SPI命令
Input:
	  uint8_t *pData	发送数据指针
	  uint8_t leng	  发送数据长度
Output: =true表示成功，否则为失败
*******************************************************************/
bool SPI_Send(uint8_t *pData,uint8_t leng)
{
	bool  transferOK = 0;

	if(leng <= SPI_CMD_LENG_MAX)
	{
		//spiTransaction.count = leng;
		//spiTransaction.txBuf = pData;
		//spiTransaction.rxBuf = receiveBuffer;

		//transferOK = SPI_transfer(spi, &spiTransaction);
		//nrf_drv_spi_transfer(	&spi, pData,leng,receiveBuffer,leng);
		for(uint8_t i = 0; i <= leng; i++) //need copy here, otherwise the txPtr content may be overritten by SPI_transferNew
		{
			SPI_TxBuf[i] = *(pData+i);
		}
		if(SPI_TxBuf[0] == SPI_CMD_READY_POWER_OFF){
			poweringOff = 2;
		}
		pSend = SPI_TxBuf;
		//nrf_drv_spis_buffers_set(&spi, SPI_TxBuf, leng+1, receiveBuffer,leng);
	}

	return transferOK;
}


/*******************************************************************
Function: SpiTranTaskNext
Desc: 顺序执行任务栈的任务
Input: void
Output: uint8_t; = 0表示所有任务已完成，=1表示任务还在执行
*******************************************************************/
uint8_t SpiTranTaskNext(void)
{
	uint8_t IsBusy = 0;
	if(SpiTaskIndex != 0 && SpiTxIndex < SpiTaskIndex)
	{
		//if(is_spi_busy()== 0)
		if(is_spi_on()==0 /*&& spiRecover == 0*/)
		{
			GLOBAL_INT_DISABLE();
			//SpiTaskIndex--;

			//if(SpiTaskIndex >= SPI_TASK_MAX)
			//	SpiTaskIndex = SPI_TASK_MAX - 1;

			//load Send data
			pSend = (uint8_t  *)&SpiTranTask[SpiTxIndex].SendBuffer;
			//ready the rec buffer point
			pRec = SpiTranTask[SpiTxIndex].pRec;
			//load the transfer data leng
			SendLeng = SpiTranTask[SpiTxIndex].TranLeng;
			//load the task ID
			TaskId_Cur = SpiTranTask[SpiTxIndex].TaskId;
			//Prepare load wait for SPI master
			//CurTaskIsRunning = 1;
			TaskNum_Start = TaskNum_Finish;//before transmit new cmd, should clear the old cmd timeout timer
			SPI_Send(pSend,SendLeng);
			UART_PRINTF("btToMcuS%x,%x,%x,%x,%x,%x\r\n",SPI_TxBuf[0],SPI_TxBuf[1],SPI_TxBuf[2],SPI_TxBuf[3],SPI_TxBuf[4],SPI_TxBuf[5]);
			spi_bt2mcu = 1;
			spi_slave_init(); //should be put behind the SPI_Send since spi_slave_init will fill the txBuf
			
			//IRQ Enable
			SPI_Transfer_IRQ_On();
			//SPI State update
			SpiState = SPI_STATE_SEND_CMD;
			//ke_timer_set(APP_SPI_TIMER,TASK_APP,100);
			GLOBAL_INT_RESTORE();
			
			//IRQ Delay setting
			//StartTimer(TIMER_ID_SPI_IRQ,1);

			IsBusy = 1;

			
		}
	}
	else
	{
		SpiState = SPI_STATE_FREE;
		IsBusy = 0;
	}

	return IsBusy;
}


/*******************************************************************
Function: SPI_TransferNew
Desc: SPI传输任务回调
Input:
  uint8_t *pSend 指向待发送的数据区
  uint8_t *pRec
  uint8_t leng
Output: void
*******************************************************************/
void SPI_TransferNew(uint8_t TaskId,uint8_t *pSendData,uint8_t *pRecData,uint8_t leng)
{
	uint8_t i,sameCmd = 0;
	uint8_t *ptr = pSendData;
	
	if((leng == 0) || (leng > SPI_CMD_LENG_MAX) || SpiIsOn == 0)
		return;

	if(SpiTaskIndex < SPI_TASK_MAX)
	{
		for(i=0; i<SpiTaskIndex; i++)
		{
			if(SpiTranTask[i].SendBuffer[0] == pSendData[0]) //the same cmd, replace the old one
			{
				sameCmd = 1;
				for(uint8_t j = 0 ; j< leng; j++)
					SpiTranTask[i].SendBuffer[j] = *ptr++;
				break;
			}
		}
		if(sameCmd == 0)
		{
			SpiTranTask[SpiTaskIndex].TaskId = TaskId;
			SpiTranTask[SpiTaskIndex].pRec = pRecData;
			SpiTranTask[SpiTaskIndex].TranLeng = leng;
			for(i = 0; i < leng; i++)
				SpiTranTask[SpiTaskIndex].SendBuffer[i] = *pSendData++;
			UART_PRINTF("TX%x,%x\r\n",SpiTaskIndex,SpiTranTask[SpiTaskIndex].SendBuffer[0]);
			SpiTaskIndex++;	
		}
	}

	if(SpiState == SPI_STATE_FREE)
	{
		UART_PRINTF("new transfer");
		SpiTranTaskNext();
	}
}

/*******************************************************************
Function: SPI_TransferTask
Desc: SPI??????
Input: void
Output: void
*******************************************************************/
void SPI_TransferTask(void)
{
	if(SpiIsOn != 0)
	{
		if(TaskNum_Start == TaskNum_Finish)
		{
			UART_PRINTF("S%d,F%d",TaskNum_Start,TaskNum_Finish);

			memset(SPI_TxBuf,0xFF,sizeof(SPI_TxBuf));
			UART_PRINTF("spi recover\r\n");	//recover packet, do not fill the SPI FIFO

			if(is_spi_on() == 0)//has been reset
			{
				if(spi_slave_init() == 0){

					SPI_Transfer_IRQ_On();
					//IRQ Delay setting
					//ke_timer_set(APP_SPI_TIMER,TASK_APP,100);
					//SPI State update
					//SpiState = SPI_STATE_BUSY;
				}
			}	
		}
		else
		{
			UART_PRINTF("Heihei");
			UART_PRINTF("S%d,F%d",TaskNum_Start,TaskNum_Finish);
			//SPI_Transfer_IRQ_Off();
		}
	}
}

void SPI_TransferTask1(void)
{
	if(SpiIsOn != 0)
	{
		//下一个任务
		pRec = NULL;
		pSend = NULL;
		//CurTaskIsRunning = 0;
		TaskNum_Finish++;
		//Next transfer task
		SpiTranTaskNext();
		//UART_PRINTF("next\r\n");
	}
}

void app_spi_transfer_err_handler(void)
{
	if(SpiState == SPI_STATE_SEND_CMD)
	{
		if(spi_bt2mcu){
			if(ke_timer_active(APP_SPI_TIMER,TASK_APP) == false)
			{
				SpiState = SPI_STATE_SEND_RECOVER;
				UART_PRINTF("SPI tmr 1");
				ke_timer_set(APP_SPI_TIMER,TASK_APP,20);//wait for 200ms(spi_sync_pin remains high)
				//SPI_TransferTask();
			}
		}
	}
	else if(SpiState == SPI_STATE_SEND_RECOVER)
	{
		spiResendCnt++;
		if(spiResendCnt < 3)
		{
			//if(spi_bt2mcu){
				if(ke_timer_active(APP_SPI_TIMER,TASK_APP) == false)
				{
					//SpiState = SPI_STATE_SEND_RECOVER;
					UART_PRINTF("SPI tmr 2");
					ke_timer_set(APP_SPI_TIMER,TASK_APP,20);//wait for 200ms(spi_sync_pin remains high)
					//SPI_TransferTask();
				}
			//}
		}
		else
		{
			UART_PRINTF("spi master no rsp\r\n");
			SpiState = SPI_STATE_FREE;
			SpiTaskIndex = 0; //reset spi buffer
			SpiTxIndex = 0;
			spiResendCnt = 0;
			if(spi_bt2mcu)
			{
				spi_bt2mcu = 0;
			}
		}
	}
}

void SPI_Transfer(void)
{
	if(gpio_get_input(SPI_IRQ_PIN) == 0){
		UART_PRINTF("IRQ LOW");
		if(SPI_BlockTransfer()){
			if(SpiState == SPI_STATE_SEND_RECOVER)
			{
				spiResendCnt = 0;
				if(ke_timer_active(APP_SPI_TIMER,TASK_APP))
				{
					ke_timer_clear(APP_SPI_TIMER,TASK_APP);
				}
			}
			app_spi_cmd_transfered_handler();				
		}
		else
		{ //transfer error
			app_spi_transfer_err_handler();
		}
	}
}

void CheckSpiMasterBlock(void)
{
	if(gpio_get_input(SPI_IRQ_PIN) && is_spi_on() && poweringOff<=2)
	{ //if mcu does not respond
				
		if(spiSendTime++ >15000){
			UART_PRINTF("IRQ timeout\r\n");
//					gpio_set(SPI_READY_PIN, 1);
			spi_slave_reset();//spi_sync_pin should be 1
			app_spi_transfer_err_handler();
		}
	}
}

void app_spi_cmd_transfered_handler(void)
{
	//UART_PRINTF("%s\r\n", __func__);
	UART_PRINTF("Rcv %x,%x,%x,%x,%x ,,%x\r\n",SPI_RxBuf[0],SPI_RxBuf[1],SPI_RxBuf[2],SPI_RxBuf[3],SPI_RxBuf[4]/*,SPI_RxBuf[5],SPI_RxBuf[6],SPI_RxBuf[7],SPI_RxBuf[8],SPI_RxBuf[9],SPI_RxBuf[10],SPI_RxBuf[11]*/,spi_bt2mcu);
	for(uint8_t i=0; i<SPI_CMD_LENG_MAX;i++)
	{
		spi_cmd_RxBuf[i] = SPI_RxBuf[i];
	}
	if(spi_bt2mcu)
	{
		spi_bt2mcu = 0;
		//if(SpiState != SPI_STATE_SEND_RECOVER) //skip the error cmd?
		SpiTxIndex ++;
		if(SpiTxIndex >= SpiTaskIndex)
		{
			SpiTxIndex = 0;
			SpiTaskIndex = 0;
		}
		if(poweringOff == 2)
		{
			UART_PRINTF("poweroff=3\r\n");
			poweringOff = 3;
			//wkupct_disable_irq();
		}
	}
	
	if((spi_cmd_RxBuf[0] & 0x80) ==0)
	{	
		UART_PRINTF("mcu2bt\r\n");
		
		spi_cmd_parse();
	}
	if(/*spiRecover == 0 && */poweringOff <= 2){
		SPI_TransferTask1();
	}
	
}



