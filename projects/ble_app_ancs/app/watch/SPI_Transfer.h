/******************************************************************************
File: SPI_Transfer.h
Creator: Zeno Zhang
Desc:SPI数据传输接口
******************************************************************************/

#ifndef SPI_TRANSFER_H
#define SPI_TRANSFER_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes */
#include "spi.h"


enum{
SPI_NOTBUSY,
SPI_BUSY_MCU2BT = 0x01,
SPI_BUSY_BT2MCU = 0x02	
};

#define PIN_SPI_IRQ             IOID_7
#define PIN_GPIO_MOSI           IOID_11
#define PIN_GPIO_SCK            IOID_10
#define PIN_GPIO_CS             IOID_9
#define PIN_GPIO_MISO           IOID_8
  
  
//SPI最大命令长度
#define SPI_CMD_LENG_MAX        13 

#define SPI_RECOVER	0
#define SPI_RESEND  1

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/
extern uint8_t spi_bt2mcu;
extern uint8_t SPI_TxBuf[SPI_CMD_LENG_MAX];
extern uint8_t SPI_RxBuf[SPI_CMD_LENG_MAX];
extern uint8_t spi_cmd_RxBuf[SPI_CMD_LENG_MAX];
/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */
typedef void (*WatchSpiCB_t)(uint8_t notify,uint8_t event_id,uint8_t *pSpiReadoutBuf);  

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*******************************************************************
Function: SPI_Transfer_init
Desc: SPI模块初始化
Input: void
Output: void
*******************************************************************/ 
//extern void SPI_Transfer_init(WatchSpiCB_t appCB);
extern void SPI_Transfer_init(void);
extern void SPI_Transfer_Stop(void);

/*******************************************************************
Function: SPI_Send
Desc: 发送SPI命令
Input: 
      uint8_t *pData    发送数据指针
      uint8_t leng      发送数据长度
Output: =true表示成功，否则为失败
*******************************************************************/ 
extern bool SPI_Send(uint8_t *pData,uint8_t leng);

/*******************************************************************
Function: SPI_TransferNew
Desc: SPI传输任务回调
Input: 
  uint8_t *pSend 指向待发送的数据区
  uint8_t *pRec
  uint8_t leng
Output: void
*******************************************************************/
extern void SPI_TransferNew(uint8_t TaskId,uint8_t *pSendData,uint8_t *pRecData,uint8_t leng);

/*******************************************************************
Function: SPI_TransferTask
Desc: SPI传输任务回调
Input: void
Output: void
*******************************************************************/
extern void SPI_TransferTask(void);
extern void SPI_TransferTask1(void);
extern void SPI_Transfer(void);
extern void SPI_Transfer_IRQ_On(void);
extern void CheckSpiMasterBlock(void);
/*******************************************************************
Function: SPI_Transfer_IRQ_Off
Desc: Disnable CS of SPI line
Input: void
Output: void
*******************************************************************/
extern  void SPI_Transfer_IRQ_Off(void);

/*********************************************************************
*********************************************************************/  
/*
#ifdef __cplusplus
}
#endif*/

#endif /* BOARD_KEY_H */
