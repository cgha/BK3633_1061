
#ifndef __I2C_H__
#define __I2C_H__


#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include <stdint.h>        // standard integer definition
#include <string.h>        // string manipulation
#include <stddef.h>        // standard definition
#include "BK_HCI_Protocol.h"
#include "BK3435_reg.h"





void i2cs_init(void);
void i2cs_deinit(void);
void i2cs_tx_data(uint8_t devAddr, uint8_t addr, uint8_t*buf, uint8_t size);
void i2cs_rx_data(uint8_t devAddr, uint8_t addr, uint8_t*buf, uint8_t size);


#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif      /* __I2C_H__ */


