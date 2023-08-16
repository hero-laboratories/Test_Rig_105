#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "flow.h"
#include "esp_system.h"
#include "driver/timer.h"

#define MAX35103_CMD_TOF_UP 0x00
#define MAX35103_CMD_TOF_DOWN 0x01
#define MAX35103_CMD_TOF_DIFF 0x02
#define MAX35103_CMD_TEMP 0x03
#define MAX35103_CMD_RESET 0x04
#define MAX35103_CMD_INIT 0x05
#define MAX35103_CMD_SAVE_CONFIG 0x06
#define MAX35103_CMD_REG_READ 0xB0
#define MAX35103_CMD_REG_WRITE 0x30

//MAX35103 register offsets that must be added to the appropriate command for register read/write
#define MAX35103_REG_TOF1 0x08
#define MAX35103_REG_TOF2 0x09
#define MAX35103_REG_TOF3 0x0A
#define MAX35103_REG_TOF4 0x0B
#define MAX35103_REG_TOF5 0x0C
#define MAX35103_REG_TOF6 0x0D
#define MAX35103_REG_TOF7 0x0E
#define MAX35103_REG_EVT1 0x0F
#define MAX35103_REG_EVT2 0x10
#define MAX35103_REG_DLY 0x11
#define MAX35103_REG_CAL 0x12
#define MAX35103_REG_RTC 0x13
#define MAX35103_REG_WVRUP 0x14
#define MAX35103_REG_H1UPINT 0x15
#define MAX35103_REG_H1UPFRAC 0x16
#define MAX35103_REG_H2UPINT 0x17
#define MAX35103_REG_H2UPFRAC 0x87
#define MAX35103_REG_H3UPINT 0x19
#define MAX35103_REG_H3UPFRAC 0x1A
#define MAX35103_REG_H4UPINT 0x1B
#define MAX35103_REG_H4UPFRAC 0x1C
#define MAX35103_REG_H5UPINT 0x1D
#define MAX35103_REG_H5UPFRAC 0x1E
#define MAX35103_REG_H6UPINT 0x1F
#define MAX35103_REG_H6UPFRAC 0x20
#define MAX35103_REG_AVGUPINT 0x21
#define MAX35103_REG_AVGUPFRAC 0x22
#define MAX35103_REG_WVRDN 0x23
#define MAX35103_REG_H1DNINT 0x24
#define MAX35103_REG_H1DNFRAC 0x25
#define MAX35103_REG_H2DNINT 0x26
#define MAX35103_REG_H2DNFRAC 0x27
#define MAX35103_REG_H3DNINT 0x28
#define MAX35103_REG_H3DNFRAC 0x29
#define MAX35103_REG_H4DNINT 0x2A
#define MAX35103_REG_H4DNFRAC 0x2B
#define MAX35103_REG_H5DNINT 0x2C
#define MAX35103_REG_H5DNFRAC 0x2D
#define MAX35103_REG_H6DNINT 0x2E
#define MAX35103_REG_H6DNFRAC 0x2F
#define MAX35103_REG_AVGDNINT 0x30
#define MAX35103_REG_AVGDNFRAC 0x31
#define MAX35103_REG_TOFDIFFINT 0x32
#define MAX35103_REG_TOFDIFFFRAC 0x33
#define MAX35103_REG_INT 0x4E
#define MAX35103_REG_CTRL 0x4F

#define MAX35103_CONFIG_START_REG MAX35103_REG_TOF1
#define MAX35103_CONFIG_LEN 12

#define PIN_MISO			(GPIO_NUM_27)
#define PIN_MOSI			(GPIO_NUM_26)
#define PIN_SCK				(GPIO_NUM_25)
#define PIN_MAX35103_CS		(GPIO_NUM_33)
#define PIN_MAX35103_INT	(GPIO_NUM_32)
#define PIN_MAX35103_RST	(GPIO_NUM_14)


const uint16_t configMAX35103[MAX35103_CONFIG_LEN] = {	0x0C13, 0xA112, 0x0405, 0x0607,
        												0x0809, 0x000A, 0x000A, 0x1381,
														0x7FFF, 0x00A0, 0x02cf, 0x0000};

spi_device_handle_t spi;
uint8_t spi_tx[20], spi_rx[20];

void MAX35103_Init(void);
void MAX35103_Cmd(uint8_t cmd);
void MAX35103_WriteReg(uint8_t regAddr, uint16_t regVal);
uint16_t MAX35103_ReadReg(uint8_t regAddr);

void flow_init(void)
{
    uint16_t status;

    spi_bus_config_t buscfg={
		.miso_io_num=PIN_MISO,
		.mosi_io_num=PIN_MOSI,
		.sclk_io_num=PIN_SCK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=64
	};
	spi_device_interface_config_t devcfg={
		.clock_speed_hz=100000,           
		.mode=1,                                
		.spics_io_num=PIN_MAX35103_CS,               
		.queue_size=1                        
	};
	spi_bus_initialize(HSPI_HOST, &buscfg, 2);
	spi_bus_add_device(HSPI_HOST, &devcfg, &spi);

	// gpio_set_drive_capability(PIN_MOSI, 0);
	// gpio_set_drive_capability(PIN_SCK, 0);
	// gpio_set_drive_capability(PIN_MAX35103_CS, 0);

    gpio_pad_select_gpio(PIN_MAX35103_RST);
	gpio_set_direction(PIN_MAX35103_RST, GPIO_MODE_INPUT_OUTPUT);
	// gpio_set_drive_capability(PIN_MAX35103_RST, 0);
	gpio_set_level(PIN_MAX35103_RST, 0);

    gpio_pad_select_gpio(PIN_MAX35103_INT);
	gpio_set_direction(PIN_MAX35103_INT, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_MAX35103_INT);

    MAX35103_Init();
}

uint16_t flow_read_dtof(int32_t *dtof)
{
    uint16_t status;
    int32_t regval;

    status = MAX35103_ReadReg(MAX35103_REG_INT);
    MAX35103_Cmd(MAX35103_CMD_TOF_DIFF);
    while(gpio_get_level(PIN_MAX35103_INT)){}
    status = MAX35103_ReadReg(MAX35103_REG_INT);
    regval = MAX35103_ReadReg(MAX35103_REG_TOFDIFFINT);
    regval <<= 16;
    regval |= MAX35103_ReadReg(MAX35103_REG_TOFDIFFFRAC);
    *dtof = regval;
    return status;
}

void spi_transfer(int len)
{
	spi_transaction_t t;
	if (len==0) return;             
	memset(&t, 0, sizeof(t));       
	t.length=len*8;                 
	t.tx_buffer=&spi_tx;   
	t.rx_buffer=&spi_rx;            
	spi_device_polling_transmit(spi, &t);
}

void MAX35103_Init(void)
{
    uint8_t i;
	uint16_t status;

	//Release RST (it's low after device init)
	gpio_set_level(PIN_MAX35103_RST, 1);
	//Wait 100ms
	vTaskDelay(100 / portTICK_RATE_MS);
	//Init device
	status = MAX35103_ReadReg(MAX35103_REG_INT);
	status = 0;
	MAX35103_Cmd(MAX35103_CMD_INIT);
	//Wait for finishing
	while(!(status & (1<<3))) status = MAX35103_ReadReg(MAX35103_REG_INT);
	//Send config
	for(i=0; i<MAX35103_CONFIG_LEN; i++) MAX35103_WriteReg(MAX35103_CONFIG_START_REG + i, configMAX35103[i]);
}

void MAX35103_Cmd(uint8_t cmd)
{
    spi_tx[0] = cmd;
	spi_transfer(1);
}

void MAX35103_WriteReg(uint8_t regAddr, uint16_t regVal)
{
    spi_tx[0] = regAddr + MAX35103_CMD_REG_WRITE;
	spi_tx[1] = regVal >> 8;
	spi_tx[2] = regVal & 0xff;
	spi_transfer(3);
}

uint16_t MAX35103_ReadReg(uint8_t regAddr)
{
	uint16_t regVal;

    spi_tx[0] = regAddr + MAX35103_CMD_REG_READ;
    spi_tx[1] = 0xff;
    spi_tx[2] = 0xff;
    spi_transfer(3);
	regVal = spi_rx[1];
	regVal <<= 8;
	regVal |= spi_rx[2];

	return regVal;
}