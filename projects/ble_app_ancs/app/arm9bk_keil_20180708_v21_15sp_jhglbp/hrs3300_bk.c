#include <stdio.h>
#include <stdbool.h>
//#include "SEGGER_RTT.h"
//#include "hrs3300_i2c.h"
//#include "app_timer.h"
#include "ke_timer.h"
#include "app_task.h"

//////////////////////////////
#include "hrs3300.h"
//#include "hrs3300_alg.h"
#include "hrs3300_reg_init.h"
#include "uart.h"      	// UART initialization
#include "Watch.h"
#include "i2c.h"
#include "DataManage.h"
#include "Watch.h"
//////////////////////////////
#define GSENSER_DATA


// hrs3300 customer config
const uint8_t  hrs3300_bp_timeout_grade = 0;  // max 15
const uint8_t  hrs3300_agc_init_stage = 0x04;  // init AGC state  
const uint8_t  hrs3300_bp_power_grade = 0;
const uint8_t  hrs3300_accurate_first_shot = 0;
const uint8_t  hrs3300_up_factor = 3;
const uint8_t  hrs3300_up_shift = 2;
const uint16_t hrs3300_AMP_LTH = 120;
const uint16_t hrs3300_hr_AMP_LTH = 150;
const uint16_t hrs3300_hr_PVAR_LTH = 10;
// hrs3300 customer config end

//20161117 added by ericy for "low power in no_touch state"
static bool hrs3300_power_up_flg = 0 ;
uint8_t reg_0x7f ;
uint8_t reg_0x80 ;
uint8_t reg_0x81 ;
uint8_t reg_0x82 ;
//20161117 added by ericy for "low power in no_touch state"
volatile uint8_t hrMeasureing;
hrs3300_results_t alg_results;
hrs3300_results_t last_hrs3300_result;
uint8_t chipID;
extern PERSON PersonInfo;
uint8_t HeartRateBuf[20];
uint8_t HR_BufIndex;
uint8_t lastHr;
volatile AxesRaw_t Axis;

void GetAccAxesRaw(AxesRaw_t *Axes)
{
	Axes->AXIS_X = ((int8_t)(Axis.AXIS_X))*64;
	Axes->AXIS_Y = ((int8_t)(Axis.AXIS_Y))*64;
	Axes->AXIS_Z = ((int8_t)(Axis.AXIS_Z))*64;
}

bool Hrs3300_write_reg(uint8_t addr, uint8_t data) 
{
	// I2c custom  return  0(success), 1(fail)

    uint8_t data_buf[2];	
		
	//data_buf[0] = addr;
	//data_buf[1] = data;
	data_buf[0] = data;

	//twi_pin_switch(1);
    //twi_master_transfer(0x88, data_buf, 2, true);	//write	

	i2c_write(HRS_I2C_ADDR,addr,data_buf,1);
	return 0;  	
}

uint8_t Hrs3300_read_reg(uint8_t addr) 
{
    uint8_t data_buf;	
#if 0
	  twi_pin_switch(1);
	  twi_master_transfer(0x88, &addr, 1, false);	//write
	  twi_master_transfer(0x89, &data_buf, 1, true);//read
#endif
	i2c_read(HRS_I2C_ADDR,addr,&data_buf,1);
	return data_buf;  	
}

#ifdef MALLOC_MEMORY
void *hr_malloc(size_t size)
{
	return (void*)malloc(size);
}

void hr_free(void * ptr)
{
	free(ptr);
}
#endif

void nrf_delay_ms(uint16_t ms)
{
	while(ms--);
}

uint16_t Hrs3300_read_hrs(void)
{
	uint8_t  databuf[3];
	uint16_t data;

	databuf[0] = Hrs3300_read_reg(0x09);	// addr09, bit
  databuf[1] = Hrs3300_read_reg(0x0a);	// addr0a, bit
  databuf[2] = Hrs3300_read_reg(0x0f);	// addr0f, bit
	
	data = ((databuf[0]<<8)|((databuf[1]&0x0F)<<4)|(databuf[2]&0x0F));

	return data;
}

uint16_t Hrs3300_read_als(void)
{
	uint8_t  databuf[3];
	uint16_t data;

	databuf[0] = Hrs3300_read_reg(0x08);	// addr09, bit [10:3]
  databuf[1] = Hrs3300_read_reg(0x0d);	// addr0a, bit [17:11]
  databuf[2] = Hrs3300_read_reg(0x0e);	// addr0f, bit [2:0]
	
	data = ((databuf[0]<<3)|((databuf[1]&0x3F)<<11)|(databuf[2]&0x07));
	
	if (data > 32767) data = 32767;  // prevent overflow of other function

	return data;
}



bool Hrs3300_chip_init()
{
	int i =0 ;
	//uint8_t id =0;
	
	for(i = 0; i<5; i++){
		chipID = Hrs3300_read_reg(0x00);
		if (0x21==chipID){
			break;
		}
		else{
			nrf_delay_ms(10);
		}
	}
	
	if (chipID != 0x21){
		UART_PRINTF("hrs3300 id err\r\n");
		return false;
	}
	
	for(i = 0; i < INIT_ARRAY_SIZE;i++)
	{
	    if ( Hrs3300_write_reg( init_register_array[i][0],
                                init_register_array[i][1]) != 0 )
	    {
	       return false;
	    }
  	}	
	
		
		//20161117 added by ericy for "low power in no_touch state"		
		if(hrs3300_power_up_flg == 0){
		  reg_0x7f=Hrs3300_read_reg(0x7f) ;
		  reg_0x80=Hrs3300_read_reg(0x80) ;
		  reg_0x81=Hrs3300_read_reg(0x81) ;
		  reg_0x82=Hrs3300_read_reg(0x82) ;		
			hrs3300_power_up_flg =  1; 
		}
		//20161117 added by ericy for "low power in no_touch state"
	#ifdef RTT_PRINT
	DEBUG_PRINTF(0,">>> hrs3300 init id = %d\r\n", id);
	#endif
		
		

	  return true;
RTN:
	  return false;		
}

void Hrs3300_chip_enable()
{	
  Hrs3300_write_reg( 0x16, 0x78 );
  Hrs3300_write_reg( 0x01, 0xd0 );	
	Hrs3300_write_reg( 0x0c, 0x2e );
	
	return ;	
}

void Hrs3300_chip_disable()
{
	Hrs3300_write_reg( 0x01, 0x08 );
	Hrs3300_write_reg( 0x0c, 0x4e );
	/*Hrs3300_write_reg( 0x01, 0x08 );
	Hrs3300_write_reg( 0x02, 0x80 );
	Hrs3300_write_reg( 0x0c, 0x4e );
	
	Hrs3300_write_reg( 0x16, 0x88 );
	
	Hrs3300_write_reg( 0x0c, 0x22 );
	Hrs3300_write_reg( 0x01, 0xf0 );
	Hrs3300_write_reg( 0x0c, 0x02 );

	Hrs3300_write_reg( 0x0c, 0x22 );
	Hrs3300_write_reg( 0x01, 0xf0 );
	Hrs3300_write_reg( 0x0c, 0x02 );
	
	Hrs3300_write_reg( 0x0c, 0x22 );
	Hrs3300_write_reg( 0x01, 0xf0 );
	Hrs3300_write_reg( 0x0c, 0x02 );
	
	Hrs3300_write_reg( 0x0c, 0x22 );
	Hrs3300_write_reg( 0x01, 0xf0 );
	Hrs3300_write_reg( 0x0c, 0x02 );*/
	
	return ;	
}

void heart_rate_meas_timeout_handler(void)
{
    uint32_t        err_code;
    uint16_t        heart_rate;
	  uint16_t hrm_raw_data;
		uint16_t als_raw_data;
	  uint8_t gsen_data;	
	  
#ifdef BP_CUSTDOWN_ALG_LIB		
	hrs3300_bp_results_t	bp_alg_results ;	
#endif	
	  static uint16_t timer_index =0;
#ifdef GSENSER_DATA
	  AxesRaw_t gsen_buf;
#endif
		
//    UNUSED_PARAMETER(p_context);
		if(hrMeasureing == HR_MEASURE_IDLE){
			return ;
		}
#ifdef GSENSER_DATA
	  GetAccAxesRaw(&gsen_buf);
#endif
	  hrm_raw_data = Hrs3300_read_hrs();
	  als_raw_data = Hrs3300_read_als();  // 20170430
#ifdef GSENSER_DATA
    Hrs3300_alg_send_data(hrm_raw_data, als_raw_data, gsen_buf.AXIS_X, gsen_buf.AXIS_Y, gsen_buf.AXIS_Z, 0); 
#else
	  Hrs3300_alg_send_data(hrm_raw_data, als_raw_data, 0, 0, 0,0);
#endif
		#if 0
	   alg_results = Hrs3300_alg_get_results();	
	   SEGGER_RTT_printf(0,"%d %d %d %d %d %d %d\n", hrm_raw_data, als_raw_data,gsen_buf.AXIS_X,gsen_buf.AXIS_Y,gsen_buf.AXIS_Z,alg_results.hr_result,alg_results.alg_status);
	   #endif

	  timer_index ++;
    if (timer_index >= 25)  {    // get result per second
		timer_index =0;
		alg_results = Hrs3300_alg_get_results();
		if(last_hrs3300_result.alg_status != alg_results.alg_status || last_hrs3300_result.hr_result != alg_results.hr_result)
		{
			if (alg_results.alg_status == MSG_NO_TOUCH)
			{
				spi_send_hr_state(&alg_results);
				//opr_display_hr(0);    // customer can print no touch information here
				//UART_PRINTF("NO TOUCH\r\n");
			}
			else if (alg_results.alg_status == MSG_PPG_LEN_TOO_SHORT)
			{
				spi_send_hr_state(&alg_results);
				//opr_display_wait( ((alg_results.data_cnt/100)%3)+1);  // customer can print waiting information here
				//UART_PRINTF("TOO Short\r\n");
			}
			else
			{
				if(alg_results.hr_result < (230- PersonInfo.Age))
				{
					UART_PRINTF("hr result:%x\r\n",alg_results.hr_result);
					if(hrMeasureing & HR_MEASURE_MANUAL){
						spi_send_hr_state(&alg_results);
						Watch_RealStepsSend();
					}
					HeartRateBuf[HR_BufIndex++] = alg_results.hr_result;
					if(HR_BufIndex >=sizeof(HeartRateBuf)){
						HR_BufIndex = 0;
					}
				}
	#ifdef BP_CUSTDOWN_ALG_LIB					
			bp_alg_results = Hrs3300_alg_get_bp_results();  
			if (bp_alg_results.sbp!= 0){
			   opr_display_bp(bp_alg_results.sbp, bp_alg_results.dbp);
			}
	#endif				
				//opr_display_hr(alg_results.hr_result);  // customer can print real heart rate here
				//UART_PRINTF("%x\r\n",alg_results.hr_result);
			}
			last_hrs3300_result.alg_status = alg_results.alg_status;
			last_hrs3300_result.hr_result = alg_results.hr_result;
		}
	}
}

uint8_t HRS_ChipID(void)
{
	return Hrs3300_read_reg(0x00);
}

bool Hrs3300_Present(void)
{
	chipID = HRS_ChipID();
	return (chipID == HRS_CHIP_ID)?true:false;
}

uint8_t get_last_hr(void)
{
	return last_hrs3300_result.hr_result;
}

void HR_StartMeasurement(uint8_t measureType)
{
	i2c_init(I2C_DEFAULT_SLAVE_ADDRESS,0);
	//nrf_gpio_pin_clear(PIN_HR_POWER);
	if( Hrs3300_Present() == true) // initial hrm chip
	{
		if(hrMeasureing == HR_MEASURE_IDLE){
			Hrs3300_chip_init();
			Hrs3300_chip_enable();
			Hrs3300_alg_open();
			ke_timer_set(APP_HR_TIMER, TASK_APP, 4);
		}
		hrMeasureing |= measureType;
	}
	else{
		i2c_deinit();
	}
}

void HR_StopMeasurement(uint8_t measureType)
{
	if(hrMeasureing)
	{
		hrMeasureing &= ~measureType;	
		if(hrMeasureing == 0)
		{
			ke_timer_clear(APP_HR_TIMER,TASK_APP);//should first stop timer, then disable HRS3300
			Hrs3300_chip_disable();
			i2c_deinit();
			hrMeasureing = HR_MEASURE_IDLE;
			alg_results.alg_status = MSG_ALG_NOT_OPEN;
			last_hrs3300_result.alg_status = MSG_ALG_NOT_OPEN;
			alg_results.hr_result = 0;
			last_hrs3300_result.hr_result = 0;
			hrs3300_power_up_flg = 0;	
		}
	}
}



