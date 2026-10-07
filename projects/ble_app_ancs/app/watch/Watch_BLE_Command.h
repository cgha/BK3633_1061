/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#define ANCS_MASK_CALL          0x01
#define ANCS_MASK_MESSAGE       0x02
#define ANCS_MASK_QQ            0x04
#define ANCS_MASK_WEIXIN        0x08
#define ANCS_MASK_TWITTER       0x10
#define ANCS_MASK_FACEBOOK      0x20
#define ANCS_MASK_WHATSAPP      0x40
#define ANCS_MASK_SKYPE         0x80

#define ANCS_MASK_MAIL          0x0100
#define ANCS_MASK_TIKTOK       	0x0200
#define ANCS_MASK_INSTAGRAM     0x0400
#define ANCS_MASK_LINE	        0x0800
#define ANCS_MASK_KAKAO		    0x1000
#define ANCS_MASK_TELEGRAM      0x2000
#define ANCS_MASK_VIBER	        0x4000
#define ANCS_MASK_LINKEDIN      0x8000
#define ANCS_MASK_OTHER_SOCIAL	0x10000 //Android not support
#define ANCS_MASK_ALL_APP     	0x20000


//0x01,0x02,0x0b,0x0f,0x37ÎªandroidApp³õÊ¼»¯Ë³Ðò
//define the command value
#define CMD_SET_TIME                  0x01
#define CMD_GET_TIME                  0x41
#define CMD_SET_PERSON                0x02
#define CMD_GET_PERSON                0x42
#define CMD_GET_THIS_DAY_RECORD       0x43
#define CMD_DEL_THIS_DAY_RECORD       0x04
#define CMD_SET_DEVICE_ID             0x05
#define CMD_GET_DEVICE_ID             0x45
#define CMD_IS_THIS_DAY_RECORD        0x46
#define CMD_GET_THIS_DAY_RECORD_TOTAL 0x07
#define CMD_GET_THIS_DAY_TARGRT        0x08
#define CMD_START_REALTIME_MODE       0x09
#define CMD_STOP_REALTIME_MODE         0x0A
#define CMD_SET_TARGET_STEPS          0x0B
#define CMD_GET_TARGET_STEPS          0x4B
#define CMD_ACTIVE_DEVICE             0x0D
#define CMD_GET_BAT_LEVEL             0x13
#define CMD_BONDING                  0x20
#define CMD_BONDING_ACK               0x21
#define CMD_SET_ALARM               0x23
#define CMD_GET_ALARM               0x24
#define CMD_SET_SPORT_PERIOD        0x25
#define CMD_GET_SPORT_PERIOD        0x26
#define CMD_GET_MAC               0x22
#define CMD_FACTORY_RESET             0x12
#define CMD_GET_VERSION             0x27
#define CMD_SET_DIST_UNIT             0x0F
#define CMD_GET_DIST_UNIT             0x4F
#define CMD_SET_KARMA_UNIT             0x2B
#define CMD_GET_KARMA_UNIT             0x2F
#define CMD_MCU_SOFT_RESET          0x2E
#define CMD_SET_DISP_MODE          0x30
#define CMD_GET_DISP_MODE          0x31
#define CMD_SET_SLEEP_PERIOD        0x32
#define CMD_GET_SLEEP_PERIOD        0x33
#define CMD_VABRATION_TEST    0x36
#define CMD_SET_TIME_MODE       0x37
#define CMD_GET_TIME_MODE       0x38
#define CMD_SET_BT_NAME           0x3D
#define CMD_GET_BT_NAME           0x3E
#define CMD_SET_PRODUCT_INFO           0x3F
#define CMD_GET_PRODUCT_INFO           0x40
#define CMD_DEBUG                   0x44
#define CMD_UPDATE_FW    0x47
#define CMD_GET_CUR_SPORT_INFO  0x48
#define CMD_ENTER_EXIT_SLEEP        0x49
#define CMD_NOTIFICATION            0x4D
#define UART_CMD_SEND_HEART     0x94
#define CMD_START_ECG            0x99
#define CMD_STOP_ECG            0x98
#define UART_CMD_HEAD     0x5A
#define CMD_TAKE_PHOTO   0x4C
#define CMD_SET_ANCS_MASK   0x60
#define CMD_GET_ANCS_MASK   0x61
#define CMD_GET_PRODUCT_TYPE    0x62
#define CMD_GEAR_ADJUST         0x63
#define CMD_GEAR_ADJUST_FINISH  0x64
#define CMD_GET_SLEEP_STAGE			0x54
#define CMD_APP_TRIG_HR			0x55
#define CMD_GET_MANUAL_HR_RECORD	0x56	
#define CMD_GET_HR_DAY_RECORD		0x50
/* [] END OF FILE */
