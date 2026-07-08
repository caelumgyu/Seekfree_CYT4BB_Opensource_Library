/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * This file has been adapted for the SEEKFREE CYT4BB open-source library.
  * Supports three communication modes: soft I2C, hardware I2C, and hardware SPI.
  *
  ******************************************************************************
  */

#include "vl53l8cx_platform.h"
#include "vl53l8cx.h"
#include "zf_driver_delay.h"

/******************************************************************************/
/*                         Soft I2C (bit-bang) Implementation                  */
/******************************************************************************/
#if (VL53L8CX_COMM_MODE == 0)

#include "zf_driver_soft_iic.h"

extern soft_iic_info_struct vl53l8cx_iic_obj;

#define VL53L8CX_WR_BUFFER_SIZE     ((uint32_t)0x8002U)
static uint8_t vl53l8cx_wr_buffer[VL53L8CX_WR_BUFFER_SIZE];

uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	uint8_t reg_addr[2];
	(void)p_platform;
	reg_addr[0] = (uint8_t)(RegisterAdress >> 8);
	reg_addr[1] = (uint8_t)(RegisterAdress & 0xFFU);
	vl53l8cx_iic_obj.addr = (uint8_t)(p_platform->address >> 1);
	soft_iic_transfer_8bit_array(&vl53l8cx_iic_obj, reg_addr, 2, p_value, 1);
	return 0;
}

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	uint8_t write_data[3];
	(void)p_platform;
	write_data[0] = (uint8_t)(RegisterAdress >> 8);
	write_data[1] = (uint8_t)(RegisterAdress & 0xFFU);
	write_data[2] = value;
	vl53l8cx_iic_obj.addr = (uint8_t)(p_platform->address >> 1);
	soft_iic_write_8bit_array(&vl53l8cx_iic_obj, write_data, 3);
	return 0;
}

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	(void)p_platform;
	if ((size + 2U) > VL53L8CX_WR_BUFFER_SIZE) { return 255; }
	vl53l8cx_wr_buffer[0] = (uint8_t)(RegisterAdress >> 8);
	vl53l8cx_wr_buffer[1] = (uint8_t)(RegisterAdress & 0xFFU);
	if (size > 0U) { memcpy(&vl53l8cx_wr_buffer[2], p_values, size); }
	vl53l8cx_iic_obj.addr = (uint8_t)(p_platform->address >> 1);
	soft_iic_write_8bit_array(&vl53l8cx_iic_obj, vl53l8cx_wr_buffer, size + 2U);
	return 0;
}

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t reg_addr[2];
	(void)p_platform;
	reg_addr[0] = (uint8_t)(RegisterAdress >> 8);
	reg_addr[1] = (uint8_t)(RegisterAdress & 0xFFU);
	vl53l8cx_iic_obj.addr = (uint8_t)(p_platform->address >> 1);
	soft_iic_transfer_8bit_array(&vl53l8cx_iic_obj, reg_addr, 2, p_values, size);
	return 0;
}


/******************************************************************************/
/*                     Hardware I2C (SCB) Implementation                      */
/******************************************************************************/
#elif (VL53L8CX_COMM_MODE == 1)

#include "scb/cy_scb_i2c.h"
#include "gpio/cy_gpio.h"
#include "sysclk/cy_sysclk.h"

#if (VL53L8CX_HARD_IIC_SCB_SEL == 1)
#define VL53L8CX_HARD_IIC_SCB           SCB1
#define VL53L8CX_HARD_IIC_SCL_PORT      GPIO_PRT18
#define VL53L8CX_HARD_IIC_SCL_PIN_NUM   2u
#define VL53L8CX_HARD_IIC_SCL_HSIOM     P18_2_SCB1_I2C_SCL
#define VL53L8CX_HARD_IIC_SDA_PORT      GPIO_PRT18
#define VL53L8CX_HARD_IIC_SDA_PIN_NUM   1u
#define VL53L8CX_HARD_IIC_SDA_HSIOM     P18_1_SCB1_I2C_SDA
#define VL53L8CX_HARD_IIC_PCLK          PCLK_SCB1_CLOCK
#elif (VL53L8CX_HARD_IIC_SCB_SEL == 4)
#define VL53L8CX_HARD_IIC_SCB           SCB4
#define VL53L8CX_HARD_IIC_SCL_PORT      GPIO_PRT6
#define VL53L8CX_HARD_IIC_SCL_PIN_NUM   2u
#define VL53L8CX_HARD_IIC_SCL_HSIOM     P6_2_SCB4_I2C_SCL
#define VL53L8CX_HARD_IIC_SDA_PORT      GPIO_PRT6
#define VL53L8CX_HARD_IIC_SDA_PIN_NUM   1u
#define VL53L8CX_HARD_IIC_SDA_HSIOM     P6_1_SCB4_I2C_SDA
#define VL53L8CX_HARD_IIC_PCLK          PCLK_SCB4_CLOCK
#elif (VL53L8CX_HARD_IIC_SCB_SEL == 5)
#define VL53L8CX_HARD_IIC_SCB           SCB5
#define VL53L8CX_HARD_IIC_SCL_PORT      GPIO_PRT7
#define VL53L8CX_HARD_IIC_SCL_PIN_NUM   2u
#define VL53L8CX_HARD_IIC_SCL_HSIOM     P7_2_SCB5_I2C_SCL
#define VL53L8CX_HARD_IIC_SDA_PORT      GPIO_PRT7
#define VL53L8CX_HARD_IIC_SDA_PIN_NUM   1u
#define VL53L8CX_HARD_IIC_SDA_HSIOM     P7_1_SCB5_I2C_SDA
#define VL53L8CX_HARD_IIC_PCLK          PCLK_SCB5_CLOCK
#else
#error "Unsupported VL53L8CX_HARD_IIC_SCB_SEL value."
#endif

#define VL53L8CX_I2C_TIMEOUT_START_MS   100u
#define VL53L8CX_I2C_TIMEOUT_BYTE_MS    10u
#define VL53L8CX_I2C_TIMEOUT_STOP_MS    100u

static cy_stc_scb_i2c_context_t vl53l8cx_i2c_context;

static uint32_t vl53l8cx_i2c_slave_address(VL53L8CX_Platform *p_platform)
{
	return (uint32_t)(p_platform->address >> 1);
}

static void vl53l8cx_i2c_send_stop(void)
{
	(void)Cy_SCB_I2C_MasterSendStop(VL53L8CX_HARD_IIC_SCB, VL53L8CX_I2C_TIMEOUT_STOP_MS, &vl53l8cx_i2c_context);
}

uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	cy_en_scb_i2c_status_t status;
	uint8_t reg_addr[2];
	uint32_t slave_addr = vl53l8cx_i2c_slave_address(p_platform);
	reg_addr[0] = (uint8_t)(RegisterAdress >> 8);
	reg_addr[1] = (uint8_t)(RegisterAdress & 0xFFU);
	status = Cy_SCB_I2C_MasterSendStart(VL53L8CX_HARD_IIC_SCB, slave_addr, CY_SCB_I2C_WRITE_XFER,  VL53L8CX_I2C_TIMEOUT_START_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, reg_addr[0], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, reg_addr[1], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterSendReStart(VL53L8CX_HARD_IIC_SCB, slave_addr, CY_SCB_I2C_READ_XFER, VL53L8CX_I2C_TIMEOUT_START_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterReadByte(VL53L8CX_HARD_IIC_SCB, CY_SCB_I2C_NAK, p_value, VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterSendStop(VL53L8CX_HARD_IIC_SCB, VL53L8CX_I2C_TIMEOUT_STOP_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { return 255; }
	return 0;
}

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	cy_en_scb_i2c_status_t status;
	uint8_t reg_addr[2];
	uint32_t slave_addr = vl53l8cx_i2c_slave_address(p_platform);
	reg_addr[0] = (uint8_t)(RegisterAdress >> 8);
	reg_addr[1] = (uint8_t)(RegisterAdress & 0xFFU);
	status = Cy_SCB_I2C_MasterSendStart(VL53L8CX_HARD_IIC_SCB, slave_addr, CY_SCB_I2C_WRITE_XFER, VL53L8CX_I2C_TIMEOUT_START_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, reg_addr[0], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, reg_addr[1], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, value, VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterSendStop(VL53L8CX_HARD_IIC_SCB, VL53L8CX_I2C_TIMEOUT_STOP_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { return 255; }
	return 0;
}

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	cy_en_scb_i2c_status_t status;
	uint8_t reg_addr[2];
	uint32_t i;
	uint32_t slave_addr = vl53l8cx_i2c_slave_address(p_platform);
	reg_addr[0] = (uint8_t)(RegisterAdress >> 8);
	reg_addr[1] = (uint8_t)(RegisterAdress & 0xFFU);
	status = Cy_SCB_I2C_MasterSendStart(VL53L8CX_HARD_IIC_SCB, slave_addr, CY_SCB_I2C_WRITE_XFER, VL53L8CX_I2C_TIMEOUT_START_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, reg_addr[0], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, reg_addr[1], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	for (i = 0; i < size; i ++) {
		status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, p_values[i], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
		if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	}
	status = Cy_SCB_I2C_MasterSendStop(VL53L8CX_HARD_IIC_SCB, VL53L8CX_I2C_TIMEOUT_STOP_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { return 255; }
	return 0;
}

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	cy_en_scb_i2c_status_t status;
	uint8_t reg_addr[2];
	uint32_t i;
	uint32_t slave_addr = vl53l8cx_i2c_slave_address(p_platform);
	if (size == 0U) { return 0; }
	reg_addr[0] = (uint8_t)(RegisterAdress >> 8);
	reg_addr[1] = (uint8_t)(RegisterAdress & 0xFFU);
	status = Cy_SCB_I2C_MasterSendStart(VL53L8CX_HARD_IIC_SCB, slave_addr, CY_SCB_I2C_WRITE_XFER, VL53L8CX_I2C_TIMEOUT_START_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, reg_addr[0], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterWriteByte(VL53L8CX_HARD_IIC_SCB, reg_addr[1], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	status = Cy_SCB_I2C_MasterSendReStart(VL53L8CX_HARD_IIC_SCB, slave_addr, CY_SCB_I2C_READ_XFER, VL53L8CX_I2C_TIMEOUT_START_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	for (i = 0; i < size; i ++) {
		uint32_t ack = (i == (size - 1U)) ? CY_SCB_I2C_NAK : CY_SCB_I2C_ACK;
		status = Cy_SCB_I2C_MasterReadByte(VL53L8CX_HARD_IIC_SCB, ack, &p_values[i], VL53L8CX_I2C_TIMEOUT_BYTE_MS, &vl53l8cx_i2c_context);
		if (status != CY_SCB_I2C_SUCCESS) { vl53l8cx_i2c_send_stop(); return 255; }
	}
	status = Cy_SCB_I2C_MasterSendStop(VL53L8CX_HARD_IIC_SCB, VL53L8CX_I2C_TIMEOUT_STOP_MS, &vl53l8cx_i2c_context);
	if (status != CY_SCB_I2C_SUCCESS) { return 255; }
	return 0;
}

void vl53l8cx_iic_hardware_init(void)
{
	cy_stc_gpio_pin_config_t pin_config = {0};
	cy_stc_scb_i2c_config_t  i2c_config = {0};
	pin_config.driveMode = CY_GPIO_DM_OD_DRIVESLOW;
	pin_config.hsiom     = VL53L8CX_HARD_IIC_SCL_HSIOM;
	pin_config.outVal    = 1u;
	Cy_GPIO_Pin_Init(VL53L8CX_HARD_IIC_SCL_PORT, VL53L8CX_HARD_IIC_SCL_PIN_NUM, &pin_config);
	pin_config.hsiom = VL53L8CX_HARD_IIC_SDA_HSIOM;
	Cy_GPIO_Pin_Init(VL53L8CX_HARD_IIC_SDA_PORT, VL53L8CX_HARD_IIC_SDA_PIN_NUM, &pin_config);
	i2c_config.i2cMode = CY_SCB_I2C_MASTER;
	i2c_config.useRxFifo = false;
	i2c_config.useTxFifo = false;
	Cy_SCB_I2C_DeInit(VL53L8CX_HARD_IIC_SCB);
	Cy_SCB_I2C_Init(VL53L8CX_HARD_IIC_SCB, &i2c_config, &vl53l8cx_i2c_context);
	Cy_SCB_I2C_Enable(VL53L8CX_HARD_IIC_SCB);
	Cy_SysClk_PeriphAssignDivider(VL53L8CX_HARD_IIC_PCLK, CY_SYSCLK_DIV_24_5_BIT, 11u);
	Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(VL53L8CX_HARD_IIC_PCLK), CY_SYSCLK_DIV_24_5_BIT, 11u, 9u, 0u);
	Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(VL53L8CX_HARD_IIC_PCLK), CY_SYSCLK_DIV_24_5_BIT, 11u);
	Cy_SCB_I2C_SetDataRate(VL53L8CX_HARD_IIC_SCB, VL53L8CX_HARD_IIC_SPEED_HZ, 8000000u);
}


/******************************************************************************/
/*                       Hardware SPI Implementation                         */
/******************************************************************************/
#elif (VL53L8CX_COMM_MODE == 2)

#include "zf_driver_spi.h"

/* SPI chunk size for large transfers (same as official ST SPI driver) */
#define VL53L8CX_SPI_CHUNK_SIZE      ((uint32_t)4096U)

/* SPI R/W flag: bit 15 = 1 for write, 0 for read */
#define VL53L8CX_SPI_WRITE_MASK(x)   ((uint16_t)((x) | 0x8000U))
#define VL53L8CX_SPI_READ_MASK(x)    ((uint16_t)((x) & 0x7FFFU))

/* Communication buffer for chunked SPI transfers */
static uint8_t vl53l8cx_spi_buf[VL53L8CX_SPI_CHUNK_SIZE + 2];

uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	uint16_t reg_addr = VL53L8CX_SPI_READ_MASK(RegisterAdress);
	vl53l8cx_spi_buf[0] = (uint8_t)(reg_addr >> 8);
	vl53l8cx_spi_buf[1] = (uint8_t)(reg_addr & 0xFFU);
	vl53l8cx_spi_buf[2] = 0x00;
	(void)p_platform;
	spi_transfer_8bit(VL53L8CX_SPI_INDEX, vl53l8cx_spi_buf, vl53l8cx_spi_buf, 3);
	*p_value = vl53l8cx_spi_buf[2];
	return 0;
}

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	uint16_t reg_addr = VL53L8CX_SPI_WRITE_MASK(RegisterAdress);
	uint8_t buf[3];
	(void)p_platform;
	buf[0] = (uint8_t)(reg_addr >> 8);
	buf[1] = (uint8_t)(reg_addr & 0xFFU);
	buf[2] = value;
	spi_write_8bit_array(VL53L8CX_SPI_INDEX, buf, 3);
	return 0;
}

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint32_t position = 0;
	uint32_t chunk_size;
	uint16_t reg_addr;
	(void)p_platform;

	for (position = 0; position < size; position += VL53L8CX_SPI_CHUNK_SIZE)
	{
		if ((position + VL53L8CX_SPI_CHUNK_SIZE) > size)
			chunk_size = size - position;
		else
			chunk_size = VL53L8CX_SPI_CHUNK_SIZE;

		reg_addr = VL53L8CX_SPI_WRITE_MASK((uint16_t)(RegisterAdress + position));

		vl53l8cx_spi_buf[0] = (uint8_t)(reg_addr >> 8);
		vl53l8cx_spi_buf[1] = (uint8_t)(reg_addr & 0xFFU);
		memcpy(&vl53l8cx_spi_buf[2], &p_values[position], chunk_size);

		spi_write_8bit_array(VL53L8CX_SPI_INDEX, vl53l8cx_spi_buf, chunk_size + 2);
	}
	return 0;
}

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint32_t position = 0;
	uint32_t chunk_size;
	uint16_t reg_addr;
	uint32_t i;
	(void)p_platform;

	for (position = 0; position < size; position += VL53L8CX_SPI_CHUNK_SIZE)
	{
		if ((position + VL53L8CX_SPI_CHUNK_SIZE) > size)
			chunk_size = size - position;
		else
			chunk_size = VL53L8CX_SPI_CHUNK_SIZE;

		reg_addr = VL53L8CX_SPI_READ_MASK((uint16_t)(RegisterAdress + position));

		vl53l8cx_spi_buf[0] = (uint8_t)(reg_addr >> 8);
		vl53l8cx_spi_buf[1] = (uint8_t)(reg_addr & 0xFFU);
		memset(&vl53l8cx_spi_buf[2], 0x00, chunk_size);

		/* Full-duplex transfer: write address+dummy, read data (first 2 bytes are garbage) */
		spi_transfer_8bit(VL53L8CX_SPI_INDEX, vl53l8cx_spi_buf, vl53l8cx_spi_buf, chunk_size + 2);

		/* Copy from offset 2 to user buffer */
		for (i = 0; i < chunk_size; i ++)
		{
			p_values[position + i] = vl53l8cx_spi_buf[i + 2];
		}
	}
	return 0;
}

#else
#error "VL53L8CX_COMM_MODE must be 0 (soft I2C), 1 (hardware I2C), or 2 (hardware SPI)."
#endif


/******************************************************************************/
/*                         Common Functions (all modes)                       */
/******************************************************************************/

uint8_t VL53L8CX_Reset_Sensor(
		VL53L8CX_Platform *p_platform)
{
	(void)p_platform;
	return 0;
}

void VL53L8CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 		 size)
{
	uint32_t i, tmp;
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t VL53L8CX_WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs)
{
	(void)p_platform;
	system_delay_ms(TimeMs);
	return 0;
}
