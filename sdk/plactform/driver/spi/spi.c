
/**
 ****************************************************************************************
 *
 * @file spi.c
 *
 * @brief spi driver
 *
 * Copyright (C) BeKen 2009-2017
 *
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>     // standard definition
#include <stdarg.h>
#include <stdio.h>
#include "spi.h"       // uart definition
#include "BK3435_reg.h"
#include "reg_intc.h"
#include "intc.h"
#include "rwip.h"       // SW interface
#include "uart.h"
#include "gpio.h"
#include "app_task.h"
#include "ke_timer.h"
#include "icu.h"
#include "wdt.h"
#include "SPI_Transfer.h"
#include "rf.h"


//extern uint8_t syncPinLow;
volatile uint8_t irqPinLow;
uint32_t spiSendTime;

uint8_t is_spi_busy(void);



//SPI CLK = (Mclk/2)/(freq_div+1)  //Mclk is 16M 
//exapmle:freq_div:3   ->   SPI CLK  2M
//             freq_div:0   ->   SPI CLK  8M
void spi_init(uint8_t freq_div)
{
	
	 // SPI clock enable
	 REG_AHB0_ICU_SPICLKCON = 0x00;     

	 // Close JTAG to release GPIO to normal function
     	//CLOSE_JTAG_MODE();

	 // Enable GPIO P0.4, P0.5, P0.6, P0.7 peripheral function for spi
     	GPIO_SPI_CFG &= ~((0x1<<GPIO_SPI_SCK_PIN)  | (0x1<<GPIO_SPI_MOSI_PIN)
                    | (0x1<<GPIO_SPI_MISO_PIN) | (0x1<<GPIO_SPI_NSS_PIN ));
	
      		UART_PRINTF("SPI_CTRL 0 = %x\r\n",REG_APB2_SPI_CTRL);
		
		REG_APB2_SPI_CTRL = (0x01UL << BIT_SPI_CTRL_SPIEN)
					|( 0x1 << BIT_SPI_CTRL_MSTEN)           //master mode 
					| (CKPHA_CLK1 << BIT_SPI_CTRL_CKPHA)
					| (CKPOL_L << BIT_SPI_CTRL_CKPOL)
					| (SPI_CHARFORMAT_8BIT << BIT_SPI_CTRL_BIT_WIDTH)
					| (0x03UL << BIT_SPI_CTRL_NSSMD)
					| (freq_div << BIT_SPI_CTRL_SPI_CKR) 
					//| (8 << BIT_SPI_CTRL_SPI_CKR)
					| (0x01UL << BIT_SPI_CTRL_RXINT_EN)
					| (0x00UL << BIT_SPI_CTRL_TXINT_EN)
					| (0x00UL << BIT_SPI_CTRL_RXOVR_EN)
					| (0x00UL << BIT_SPI_CTRL_TXOVR_EN)
					| (0x03UL << BIT_SPI_CTRL_RXINT_MODE)
					| (0x00UL << BIT_SPI_CTRL_TXINT_MODE);
	
		UART_PRINTF("SPI_CTRL 1 = %x\r\n",REG_APB2_SPI_CTRL);
}	


static void spi_tx_start(void)
{
	uint32_t reg ;
	reg = REG_APB2_SPI_CTRL;
	
	reg |= (0x03ul << BIT_SPI_CTRL_NSSMD) ;
	REG_APB2_SPI_CTRL = reg; 	
	
	reg &= ~(0x03ul << BIT_SPI_CTRL_NSSMD) ;
  	reg |= (0x02ul << BIT_SPI_CTRL_NSSMD) ;
	
  	REG_APB2_SPI_CTRL = reg; 

	UART_PRINTF("SPI_CTRL 2 = %x\r\n",REG_APB2_SPI_CTRL);
}

void spi_tx_end(void)
{
	uint32_t reg ;
	uint32_t state;
	reg  = REG_APB2_SPI_CTRL;

	reg |=(0x01UL << BIT_SPI_CTRL_TXINT_EN);	
	
	REG_APB2_SPI_CTRL = reg;

	state = REG_APB2_SPI_STAT;
	
	while(!(state & (0x01 << BIT_SPI_STAT_TXINT)))
	{
		state = REG_APB2_SPI_STAT;
	}		
	
	reg &=~(0x01UL << BIT_SPI_CTRL_TXINT_EN);	
	
  	REG_APB2_SPI_CTRL = reg;
	
	REG_APB2_SPI_STAT = state;

	reg |= (0x03UL << BIT_SPI_CTRL_NSSMD);	
	
	REG_APB2_SPI_CTRL = reg;

	UART_PRINTF("SPI_CTRL 3 = %x\r\n",REG_APB2_SPI_CTRL);
}


static void spi_write_byte(uint8_t data)
{
	while(spi_is_busy());
	
	while(SPI_TX_FIFO_FULL);	
	
	REG_APB2_SPI_DAT = data;
}

static uint8_t spi_read_byte(void)
{
	while(spi_is_busy());
	
	while(SPI_RX_FIFO_EMPTY);	
	
	return (REG_APB2_SPI_DAT & 0xff);
}


void spi_write_read(uint8_t *wbuf, uint32_t w_size, uint8_t *rbuf, uint32_t r_size,uint32_t r_off)
{
	uint32_t max_len;
	
	max_len = (w_size > (r_size + r_off)) ? w_size : (r_size + r_off);

	spi_tx_start();

	while(max_len--)
	{
		if(w_size > 0)
		{
			spi_write_byte(*wbuf);
			wbuf++;
			w_size--;
		}
		else
		{
			spi_write_byte(0x00);
		}
		if(r_off > 0)
		{	
			spi_read_byte();
			r_off--;
		}
		else if(r_size > 0)
		{
			*rbuf = spi_read_byte();
			rbuf++;
			r_size--;
		}else
		{		
			spi_read_byte();
		}
	}
	spi_tx_end();
}


uint8_t spi_is_busy(void)
{
  	return REG_APB2_SPI_STAT & (0x01<<BIT_SPI_STAT_SPI_BUSY);
}

/*************************************************************************
SPI Slave 
*************************************************************************/

extern uint8_t SPI_TxBuf[SPI_CMD_LENG_MAX],SPI_RxBuf[SPI_CMD_LENG_MAX];
//uint8_t tx_length = SPI_BUF_LEN;
uint8_t spi_status = 0;
extern uint16_t recvCnt;


volatile uint8_t packetLen = 0;
volatile uint8_t spiIsOn;

unsigned char    *p_spi_txdata;
unsigned char    *p_spi_rxdata;


uint8_t spi_slave_init(void)
{
	UART_PRINTF("spi init%x\r\n",packetLen);
	
	memset(SPI_RxBuf,sizeof(SPI_RxBuf),0);
	p_spi_txdata = SPI_TxBuf;
    p_spi_rxdata = SPI_RxBuf;
	packetLen = 13;
	spiSendTime = 0;
//	gpio_set(SPI_READY_PIN, 0);
	
    REG_AHB0_ICU_SPICLKCON    = 0x0 ;                     //SPI clock enable
    //REG_AHB0_ICU_INT_ENABLE  |= INT_STATUS_SPI_bit;      //IRQ UART interrupt enable
    //REG_APB5_GPIOA_CFG &= ~(0xF0<<BIT_GPIO_NORMAL);// Enable GPIO-A[7:4] peripheral functions
	Delay_us(1);
	REG_APB2_SPI_CTRL = 0;
	Delay_us(1);
    REG_APB2_SPI_CTRL = //(0x01UL << BIT_SPI_CTRL_SPIEN)
					//|( 0x1 << BIT_SPI_CTRL_MSTEN)           //master mode //set slave mode 
					(CKPHA_CLK1 << BIT_SPI_CTRL_CKPHA)
					| (CKPOL_L << BIT_SPI_CTRL_CKPOL)
					| (SPI_CHARFORMAT_8BIT << BIT_SPI_CTRL_BIT_WIDTH)
					| (0x01UL << BIT_SPI_CTRL_NSSMD)
					//| (0x01UL << BIT_SPI_CTRL_NSSMD)  //
					//| (freq_div << BIT_SPI_CTRL_SPI_CKR)  //clk div
					//| (0x00UL << BIT_SPI_CTRL_RXINT_EN) 
					//| (0x01UL << BIT_SPI_CTRL_RXINT_EN)  // enable rxint
					//| (0x00UL << BIT_SPI_CTRL_TXINT_EN)  
					//| (0x00UL << BIT_SPI_CTRL_RXOVR_EN)
					//| (0x00UL << BIT_SPI_CTRL_TXOVR_EN)
					//| (0x00UL << BIT_SPI_CTRL_RXINT_MODE)
					| (0x03UL << BIT_SPI_CTRL_RXINT_MODE)  //rx 
					| (0x03UL << BIT_SPI_CTRL_TXINT_MODE);
	
	//put behind the REG_APB2_SPI_CTRL init, otherwise will have jitter on cs line
	GPIO_SPI_CFG &= ~((0x1<<GPIO_SPI_SCK_PIN)  | (0x1<<GPIO_SPI_MOSI_PIN)\
                    | (0x1<<GPIO_SPI_MISO_PIN) | (0x1<<GPIO_SPI_NSS_PIN ));
					
	REG_APB2_SPI_CTRL |= (0x01UL << BIT_SPI_CTRL_SPIEN);
	
	//REG_APB2_SPI_STAT = 0;//0x0F05;
	Delay_us(1);
	if(REG_APB2_SPI_STAT != 0x05)
		return 1;
	
	UART_PRINTF("spi stat:%x\r\n",REG_APB2_SPI_STAT);
    if(spi_bt2mcu)
	{
		#ifdef	USE_SPI_BUF
		for(uint8_t i = 0; i< 13; i++)
		{
			REG_APB2_SPI_DAT = SPI_TxBuf[i];
		}
		#else
		REG_APB2_SPI_DAT = *p_spi_txdata++;
		#endif
	}
	else{
		#ifdef	USE_SPI_BUF
		for(uint8_t i = 0; i< 13; i++)
		{
			REG_APB2_SPI_DAT = 0xFF;
		}
		#else
		memset(SPI_TxBuf,0xFF,sizeof(SPI_TxBuf));
		REG_APB2_SPI_DAT = *p_spi_txdata++;
		#endif
	}
	UART_PRINTF("SpiTxBuf:%x %x %x %x %x %x\r\n",SPI_TxBuf[0],SPI_TxBuf[1],SPI_TxBuf[2],SPI_TxBuf[3],SPI_TxBuf[4],SPI_TxBuf[5]);
	//icu_set_sleep_mode(1);
	
	
	spiIsOn = 1;
	return 0;	
}

void spi_slave_start(void)
{
	REG_APB2_SPI_CTRL |= 0x20;  //open rxint_en
}

void spi_slave_stop()
{
	gpio_config(GPIO_SPI_NSS_PIN, INPUT, PULL_NONE);
	gpio_config(GPIO_SPI_SCK_PIN, INPUT, PULL_NONE);
	gpio_config(GPIO_SPI_MOSI_PIN, INPUT, PULL_NONE);
	
	gpio_config(GPIO_SPI_MISO_PIN, OUTPUT, PULL_NONE); 
	
	gpio_set(GPIO_SPI_MISO_PIN,1);
	
	GPIO_SPI_CFG |= ((0x1<<GPIO_SPI_SCK_PIN)  | (0x1<<GPIO_SPI_MOSI_PIN)\
                    | (0x1<<GPIO_SPI_MISO_PIN) | (0x1<<GPIO_SPI_NSS_PIN ));
	
	
	REG_APB2_SPI_CTRL = 0;
	//REG_APB2_SPI_CTRL &= ~(0x01UL << BIT_SPI_CTRL_SPIEN);
	REG_APB2_SPI_STAT = 0x0F05;
	REG_AHB0_ICU_SPICLKCON    = 0x1 ;                     //SPI clock disable
    //REG_AHB0_ICU_INT_ENABLE  &= ~INT_STATUS_SPI_bit;      //IRQ UART interrupt enable
		
	
	spiIsOn = 0;
}

bool spi_slave_getSLVSEL() //bit 14
{
	return(REG_APB2_SPI_STAT & 0x4000);
}


uint8_t spi_get_status(void)
{
	return spi_status;
}

uint8_t is_spi_on(void)
{
	return spiIsOn;
}

void spi_slave_reset(void)
{
	spi_slave_stop();
	packetLen = 0;
	spiSendTime = 0;
	//memset(SPI_RxBuf,0,SPI_BUF_LEN);
	spi_status = SPI_IDLE;
	gpio_set(SPI_SYNC_PIN, 1);
//	gpio_set(SPI_READY_PIN,1);
//	if(RC_Calibrating == 0)
//	icu_set_sleep_mode(0); //has skipped sleep in main loop before sleep
	UART_PRINTF("spi reset\r\n");
}
#if 0
void spi_slave_isr(void) //2019 0227更改，中断接收方式改为查询方式
{
	unsigned int            rxint;
	unsigned char           rxfifo_empty = 0;
	
	
	rxint = REG_APB2_SPI_STAT & 0x0200; 
	uint8_t spiData;
	
	if(rxint)
	{
		rxfifo_empty = REG_APB2_SPI_STAT & 0x0004;
		while(!rxfifo_empty)
		{		
			//REG_APB2_SPI_DAT = *p_spi_txdata;  
            //p_spi_txdata ++;
			
			spiData = REG_APB2_SPI_DAT;

			*(p_spi_rxdata++) = spiData;
			if(packetLen)
				packetLen --;
			if(packetLen == 0)
			{
				SPI_Transfer_IRQ_Off();
				ke_msg_send_basic(APP_SPI_PACKET_RCVED,TASK_APP,TASK_APP);
				icu_set_sleep_mode(0);		
			}

			//*(p_test_buf++) = spiData;
			rxfifo_empty = REG_APB2_SPI_STAT & 0x0004;
			
		}

        REG_APB2_SPI_STAT |= 0x0200; //clear rxint
	}
	else{
		UART_PRINTF("Er%x %x,",REG_APB2_SPI_STAT,REG_APB2_SPI_CTRL);
	}
}
#endif

uint8_t SPI_BlockTransfer(void)
{
	uint8_t rxfifo_empty;
	uint8_t spiData;
	uint8_t newPacket = 0;
	uint8_t pin_toggle = 0;
	//uint16_t timeout = 0;


	//if( is_spi_busy() == SPI_BUSY_MCU2BT )
	if(is_spi_on() == 0)
	{	
		spi_slave_init();
		SPI_Transfer_IRQ_On();
		
	}

	if(packetLen ==13 && spiIsOn)
	{
		//GLOBAL_INT_DISABLE();
		while(packetLen && spiSendTime <=3000)
		{
			spiSendTime++;
			if(gpio_get_input(SPI_IRQ_PIN))
				break;
			rxfifo_empty = REG_APB2_SPI_STAT & 0x0004;
			while(!rxfifo_empty)
			{	
				#ifndef USE_SPI_BUF
				if(spi_bt2mcu ){//只有在发送命令时填缓冲区，防止发生错位后恢复不了，加入判断后理论上能恢复
					REG_APB2_SPI_DAT = *p_spi_txdata;  
					p_spi_txdata ++;
				}
				#endif
				
				spiData = REG_APB2_SPI_DAT;

				*(p_spi_rxdata++) = spiData;
				if(packetLen){
					packetLen --;
					if(pin_toggle ==0){
						pin_toggle = 1;
						gpio_set(SPI_SYNC_PIN, 1);
					}
					else{
						pin_toggle = 0;
						gpio_set(SPI_SYNC_PIN, 0);
					}
				}
				if(packetLen == 0)
				{
					newPacket = 1;
					break;					
				}

				//*(p_test_buf++) = spiData;
				rxfifo_empty = REG_APB2_SPI_STAT & 0x0004;
				
			}
		}
		//GLOBAL_INT_RESTORE();
	}
	else{
		UART_PRINTF("pktLen Err%d,%x\r\n",packetLen,spiIsOn);
		//ret = 0;
	}
	
	SPI_Transfer_IRQ_Off();
	
	while(/*spi_is_busy() || */(gpio_get_input(SPI_IRQ_PIN)==0))
	{
		if(spiSendTime <=5000)
			spiSendTime++;
		else
			break;
	}
	
	if(newPacket)
	{
		if(spiSendTime>5000)
			UART_PRINTF("packet Timeout\r\n");
		else{
			UART_PRINTF("packet OK\r\n");
		}
		//ret = 1;
	}
	else
	{
		UART_PRINTF("corrupt packet\r\n");
		//ret = 0;
	}
	while(REG_APB2_SPI_STAT & 1<<BIT_SPI_STAT_SLVSEL)
	{
		if(spiSendTime <=5000)
			spiSendTime++;
		else
			break;
	}
	spi_slave_reset();
	//while(gpio_get_input(SPI_IRQ_PIN)==0); 
	return newPacket;
}

void app_gpio_int_cb(void)
{
	irqPinLow = 1;
}

void spi_slave_int_init(void)
{
	SPI_Transfer_IRQ_Off();
	gpio_config(SPI_SYNC_PIN, OUTPUT, PULL_NONE);
	gpio_config(SPI_READY_PIN, OUTPUT, PULL_NONE);
	gpio_set(SPI_READY_PIN,1);
	
	
	gpio_config(GPIO_SPI_NSS_PIN, INPUT, PULL_NONE);
	gpio_config(GPIO_SPI_SCK_PIN, INPUT, PULL_NONE);
	gpio_config(GPIO_SPI_MOSI_PIN, INPUT, PULL_NONE);
	
	gpio_config(GPIO_SPI_MISO_PIN, OUTPUT, PULL_NONE);
	
	gpio_set(GPIO_SPI_MISO_PIN,1);
	
	
	gpio_cb_register(app_gpio_int_cb);
	
	gpio_config(SPI_IRQ_PIN, INPUT, PULL_NONE);
	
	REG_APB5_GPIO_WUATOD_TYPE |= 1<<(8*(SPI_IRQ_PIN>>4)+(SPI_IRQ_PIN&0x0f)); //falling edge int
	REG_APB5_GPIO_WUATOD_STAT |= 1<<(8*(SPI_IRQ_PIN>>4)+(SPI_IRQ_PIN&0x0f)); //cleat int status
	Delay_ms(2);
	REG_APB5_GPIO_WUATOD_ENABLE |= 1<<(8*(SPI_IRQ_PIN>>4)+(SPI_IRQ_PIN&0x0f)); //
	REG_AHB0_ICU_DEEP_SLEEP0 |= 1<<(8*(SPI_IRQ_PIN>>4)+(SPI_IRQ_PIN&0x0f));	
	
	REG_AHB0_ICU_INT_ENABLE |= INT_STATUS_GPIO_bit;
	
}





/// @} SPI DRIVER
