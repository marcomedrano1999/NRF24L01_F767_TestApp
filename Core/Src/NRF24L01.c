/*
 * NRF24L01.c
 *
 *  Created on: Apr 13, 2023
 *      Author: Medrano
 */

#include "stm32f7xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi3;
#define NRF24_SPI		&hspi3


void CS_Select(void)
{
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect(void)
{
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

void CE_Enable(void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable(void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

void nrf24_writeReg(uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg | (1<<5);
	buf[1] = Data;

	// Pull the CS to low
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 2, HAL_MAX_DELAY);

	// Pull the CS High to release the device
	CS_UnSelect();
}

// write multiple bytes starting from a particular register
void nrf24_writeRegMulti(uint8_t Reg, uint8_t *data, uint32_t size)
{
	uint8_t buf[2];
	buf[0] = Reg | (1<<5);

	// Pull the CS to low
	CS_Select();

	// Send the address first and then the data
	HAL_SPI_Transmit(NRF24_SPI, buf, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(NRF24_SPI, data, size, HAL_MAX_DELAY);

	// Pull the CS High to release the device
	CS_UnSelect();
}

uint8_t nrf24_ReadReg(uint8_t Reg)
{
	uint8_t data = 0;

	// Pull the CS to low
	CS_Select();

	// Send the register to read
	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, HAL_MAX_DELAY);

	// Received the data
	HAL_SPI_Receive(NRF24_SPI, &data, 1, HAL_MAX_DELAY);

	// Pull the CS High to release the device
	CS_UnSelect();

	return data;
}

void nrf24_ReadReg_Multi(uint8_t Reg, uint8_t *data, uint32_t size)
{
	// Pull the CS to low
	CS_Select();

	// Send the register to read
	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, HAL_MAX_DELAY);

	// Received the data
	HAL_SPI_Receive(NRF24_SPI, data, size, HAL_MAX_DELAY);

	// Pull the CS High to release the device
	CS_UnSelect();
}


void nrf24SendCmd(uint8_t cmd)
{
	// Pull the CS to low
	CS_Select();

	// Send the cmd
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, HAL_MAX_DELAY);

	// Pull the CS High to release the device
	CS_UnSelect();

}


void NRF24_Init(void)
{
	// disable the chip before configuring the device
	CE_Disable();

	// will be configured later
	nrf24_writeReg(NRF24_CONFIG, 0);

	// No auto ACK
	nrf24_writeReg(NRF24_EN_AA, 0);

	// Not enabling any data pipe right now
	nrf24_writeReg(NRF24_EN_RXADDR, 0);

	// 5 bytes for the TX/RX address
	nrf24_writeReg(NRF24_SETUP_AW, 0x03);

	// No retransmission
	nrf24_writeReg(NRF24_SETUP_RETR, 0);

	// Will be setup during Tx or Rx
	nrf24_writeReg(NRF24_RF_CH, 0);

	// Power=0db, data rate = 2Mbps
	nrf24_writeReg(NRF24_RF_SETUP, 0x0E);

	// Enable the chip after configuring the device
	CE_Enable();
}


// set up the Tx mode
void NRF24_TxMode(uint8_t *addr, uint8_t channel)
{
	// disable the chip before configuring the device
	CE_Disable();

	// Select the channel
	nrf24_writeReg(NRF24_RF_CH, channel);

	// Write the tx address
	nrf24_writeRegMulti(NRF24_TX_ADDR, addr,5);

	// Power up the device
	uint8_t config = nrf24_ReadReg(NRF24_CONFIG);
	config |= (1 << 1);
	nrf24_writeReg(NRF24_CONFIG, config);

	// Enable the chip after configuring the device
	CE_Enable();
}


// transmit the data
uint8_t NRF24_Transmit(uint8_t *data, uint32_t len)
{
	uint8_t cmd_to_send=0;

	// select the device
	CS_Select();

	// payload command
	cmd_to_send=NRF24_W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmd_to_send, 1, HAL_MAX_DELAY);

	// send the payload
	HAL_SPI_Transmit(NRF24_SPI, data, len, 1000);

	// Select the device
	CS_Select();

	HAL_Delay(10);

	uint8_t fifostatus = nrf24_ReadReg(NRF24_FIFO_STATUS);

	if((fifostatus & (1 << 4)) && !(fifostatus & (1<<3)))
	{
		cmd_to_send = NRF24_FLUSH_TX;
		nrf24SendCmd(cmd_to_send);

		return 1;
	}
	return 0;
}
