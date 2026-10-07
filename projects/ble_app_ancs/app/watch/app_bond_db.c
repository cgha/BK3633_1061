/**
 *****************************************************************************************
 *
 * @file app_bond_db.c
 *
 * @brief Bond database code file.
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_BOND_DB
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration
#include <string.h>

#if (BLE_APP_PRESENT)

#include "app_bond_db.h"

#if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)
#include "spi_flash.h"
#elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)
#include "i2c_eeprom.h"
#endif
#include "flash.h"
#include "nvds.h"
#include "app.h"
#include "uart.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

static struct bond_db_header bdb_header ;//__attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY
struct bond_db_data bond_data[APP_BOND_DB_MAX_BONDED_PEERS];
extern struct flash_env_tag flash_env;
/*
 * FUCTION DEFINITIONS
 ****************************************************************************************
 */

#if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)

static void bond_db_spi_flash_init(void)
{
    static int8_t dev_id;

    dev_id = spi_flash_enable(SPI_EN_GPIO_PORT, SPI_EN_GPIO_PIN);
    if (dev_id == SPI_FLASH_AUTO_DETECT_NOT_DETECTED)
    {
        // The device was not identified. The default parameters are used.
        // Alternatively, an error can be asserted here.
        spi_flash_init(SPI_FLASH_DEFAULT_SIZE, SPI_FLASH_DEFAULT_PAGE);
    }
}

static void bond_db_load_flash(void)
{
    bond_db_spi_flash_init();

    spi_flash_read_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db));

    spi_release();
}

static int8_t bond_db_erase_flash_sectors(void)
{
    uint32_t sector_nb;
    uint32_t offset;
    int8_t ret;
    int i;

    // Calculate the starting sector offset
    offset = (APP_BOND_DB_DATA_OFFSET / SPI_SECTOR_SIZE) * SPI_SECTOR_SIZE;

    // Calculate the numbers of sectors to erase
    sector_nb = (sizeof(bdb) / SPI_SECTOR_SIZE);
    if (sizeof(bdb) % SPI_SECTOR_SIZE)
        sector_nb++;

    // Erase flash sectors
    for (i = 0; i < sector_nb; i++)
    {
        ret = spi_flash_block_erase(offset, SECTOR_ERASE);
        offset += SPI_SECTOR_SIZE;
        if (ret != ERR_OK)
            break;
    }
    return ret;
}

static void bond_db_store_flash(void)
{
    int8_t ret;

    bond_db_spi_flash_init();

    ret = bond_db_erase_flash_sectors();
    if (ret == ERR_OK)
    {
        spi_flash_write_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db));
    }

    spi_release();
}

static void bond_db_clear_flash(void)
{
    int8_t ret;

    bond_db_spi_flash_init();

    ret = bond_db_erase_flash_sectors();
    ASSERT_WARNING(ret == ERR_OK);

    spi_release();
}

#elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)

static void bond_db_load_eeprom(void)
{
    uint32_t bytes_read;

    i2c_eeprom_init(I2C_SLAVE_ADDRESS, I2C_SPEED_MODE, I2C_ADDRESS_MODE, I2C_ADDRESS_SIZE);

    i2c_eeprom_read_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db), &bytes_read);
    ASSERT_ERROR(bytes_read == sizeof(struct bond_db));

    i2c_eeprom_release();
}

static void bond_db_store_eeprom(void)
{
    uint32_t bytes_written;

    i2c_eeprom_init(I2C_SLAVE_ADDRESS, I2C_SPEED_MODE, I2C_ADDRESS_MODE, I2C_ADDRESS_SIZE);

    i2c_eeprom_write_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db), &bytes_written);
    ASSERT_ERROR(bytes_written == sizeof(struct bond_db));

    i2c_eeprom_release();
}

#endif

static void bond_db_clear_flash(void)
{
    int8_t ret;

    //bond_db_spi_flash_init();

    //ret = bond_db_erase_flash_sectors();
    //ASSERT_WARNING(ret == ERR_OK);

    //spi_release();
	//flash_erase_sector(0x42000);
	flash_erase(flash_env.space_type , 0x42000, FLASH_ERASE_SECTOR_SIZE, NULL);
}

static void bond_db_load_chip_flash(void)
{
    //bond_db_spi_flash_init();

    //spi_flash_read_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db));

    //spi_release();
	
	//flash_read(flash_env.space_type , 0x42000,sizeof(struct bond_db),  (uint8_t *)&bdb, NULL);
	nvds_tag_len_t length = sizeof(struct bond_db_header);
	nvds_get(NVDS_TAG_BOND_HEADER,&length,(uint8_t*)&bdb_header);
	length = sizeof(struct bond_db_data);
	nvds_get(NVDS_TAG_BOND0,&length,(uint8_t*)&bond_data[0]);
	nvds_get(NVDS_TAG_BOND1,&length,(uint8_t*)&bond_data[1]);
	nvds_get(NVDS_TAG_BOND2,&length,(uint8_t*)&bond_data[2]);
	nvds_get(NVDS_TAG_BOND3,&length,(uint8_t*)&bond_data[3]);
	UART_PRINTF("bond data,%x,%x,%x,%x",bond_data[0],bond_data[1],bond_data[2],bond_data[3]);
}

static void bond_db_store_chip_flash(void)
{
    //int8_t ret;

    //bond_db_spi_flash_init();

    //ret = bond_db_erase_flash_sectors();
    //if (ret == ERR_OK)
    //{
    //    spi_flash_write_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db));
    //}

    //spi_release();
	
	//flash_erase(flash_env.space_type , 0x42000, FLASH_ERASE_SECTOR_SIZE, NULL);
	//flash_write(flash_env.space_type, 0x42000, sizeof(struct bond_db), (uint8_t*)&bdb, NULL);
}

static inline void bond_db_load_ext(void)
{
	bond_db_load_chip_flash();
    #if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)
    bond_db_load_flash();
    #elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)
    bond_db_load_eeprom();
    #endif
}

static inline void bond_db_store_ext(void)
{
    #if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)
    bond_db_store_flash();
    #elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)
    bond_db_store_eeprom();
    #endif
	
	bond_db_store_chip_flash();
}

static inline void bond_db_clear_ext(void)
{
    #if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)
    bond_db_clear_flash();
    #endif
	bond_db_clear_flash();
}

static void bond_db_store_at_idx(struct bond_db_data *data, int idx)
{
	uint8_t bond_tag_index = 0;
    // Update the cache
    //memcpy(&bdb.data[idx], data, sizeof(struct bond_db_data));
	memcpy(&bond_data[idx], data, sizeof(struct bond_db_data));
	
	UART_PRINTF("store bond %x\r\n", idx);
	//bond_db_store_ext();
	
	bond_tag_index = NVDS_TAG_BOND0+idx;
	if(idx < APP_BOND_DB_MAX_BONDED_PEERS){
		if(nvds_put(bond_tag_index,sizeof(struct bond_db_data),(uint8_t*)&bond_data[idx]) != NVDS_OK)
		{
			UART_PRINTF("put tag data fail\r\n");
		}
		else
		{
			UART_PRINTF("put tag data ok\r\n");
		}
		if(nvds_put(NVDS_TAG_BOND_HEADER,sizeof(struct bond_db_header),(uint8_t*)&bdb_header) != NVDS_OK)
		{
			UART_PRINTF("put tag header fail\r\n");
		}
		else
		{
			UART_PRINTF("put tag header ok\r\n");
		}
	}
}

void bond_db_init(void)
{
    // Load bond data from the external memory resource
    bond_db_load_ext();

    // Simple check for garbage in memory (this also catches the 0xFF of cleared memory)
    if ((bdb_header.next_slot > APP_BOND_DB_MAX_BONDED_PEERS) ||
        (bdb_header.start_hdr != BOND_DB_HEADER_START) || (bdb_header.end_hdr != BOND_DB_HEADER_END))
    {
        bond_db_clear();
    }
}

void bond_db_store(struct bond_db_data *data)
{
    int idx_emtpy_slot_found = -1;
    int idx_found = -1;
    int idx = 0;

    // search if it already exists in db
    for(idx = 0; idx < APP_BOND_DB_MAX_BONDED_PEERS; ++idx)
    {
        // skip invalid
        if (bond_data[idx].valid != 0xAA)
        {
            if (-1 == idx_emtpy_slot_found)
            {
                idx_emtpy_slot_found = idx;
            }

            continue;
        }

        if (bond_data[idx].flags && BOND_DB_ENTRY_IRK_PRESENT)
        {
            // match IRK
            if ( 0 == memcmp(&data->irk, &bond_data[idx].irk, sizeof(struct gapc_irk)) )
            {
                idx_found = idx;
                break;
            }
        }
        else
        {
            if ( 0 == memcmp(&data->bdaddr, &bond_data[idx].bdaddr, sizeof(struct gap_bdaddr)) )
            {
                idx_found = idx;
                break;
            }
        }
    }

    if (-1 != idx_emtpy_slot_found)
    {
        idx_found = idx_emtpy_slot_found;
    }
    else
    {
        // no empty slot found - delete the oldest bond data slot
        idx_found = bdb_header.next_slot++;
        bdb_header.next_slot = bdb_header.next_slot % APP_BOND_DB_MAX_BONDED_PEERS;
    }

    // store bond db entry
    bond_db_store_at_idx(data, idx_found);
}

const struct bond_db_data* bond_db_lookup_by_ediv(const rand_nb_t *rand_nb,
                                                  uint16_t ediv)
{
    int idx = 0;
    struct bond_db_data *found_data = 0;

    // search if it already exists in db
    for(idx = 0; idx < APP_BOND_DB_MAX_BONDED_PEERS; ++idx)
    {
        // match rand_nd and ediv
        if ( 0 == memcmp(rand_nb, &bond_data[idx].ltk.randnb, RAND_NB_LEN)
            && ediv == bond_data[idx].ltk.ediv )
        {
            found_data = &bond_data[idx];
            break;
        }
    }

    return found_data;
}

void bond_db_clear(void)
{
    memset((void *)&bdb_header, 0, sizeof(struct bond_db_header) ); // zero bond data
    bdb_header.start_hdr = BOND_DB_HEADER_START;
    bdb_header.end_hdr = BOND_DB_HEADER_END;

    //bond_db_clear_ext();
    //bond_db_store_ext();
	nvds_del(NVDS_TAG_BOND_HEADER);
	nvds_del(NVDS_TAG_BOND0);
	nvds_del(NVDS_TAG_BOND1);
	nvds_del(NVDS_TAG_BOND2);
	nvds_del(NVDS_TAG_BOND3);
	nvds_put(NVDS_TAG_BOND_HEADER,sizeof(struct bond_db_header),(uint8_t *)&bdb_header);
}

#endif  // BLE_APP_PRESENT

/// @} APP_BOND_DB
