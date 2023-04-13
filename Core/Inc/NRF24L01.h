/*
 * NRF24L01.h
 *
 *  Created on: Apr 13, 2023
 *      Author: Medrano
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_


#include "stm32f7xx.h"

#define NRF24_CE_PORT	GPIOC
#define NRF24_CE_PIN	GPIO_PIN_8

#define NRF24_CS_PORT	GPIOC
#define NRF24_CS_PIN	GPIO_PIN_9



/* Memory map */
#define NRF24_CONFIG			0x00
#define NRF24_EN_AA				0x01
#define NRF24_EN_RXADDR			0x02
#define NRF24_SETUP_AW			0x03
#define NRF24_SETUP_RETR		0x04
#define NRF24_RF_CH				0x05
#define NRF24_RF_SETUP			0x06
#define NRF24_STATUS			0x07
#define NRF24_OBSERVE_TX		0x08
#define NRF24_CD				0x09
#define NRF24_RX_ADDR_P0		0x0A
#define NRF24_RX_ADDR_P1		0x0B
#define NRF24_RX_ADDR_P2		0x0C
#define NRF24_RX_ADDR_P3		0x0D
#define NRF24_RX_ADDR_P4		0x0E
#define NRF24_RX_ADDR_P5		0x0F
#define NRF24_TX_ADDR			0x10
#define NRF24_RX_PW_P0			0x11
#define NRF24_RX_PW_P1			0x12
#define NRF24_RX_PW_P2			0x13
#define NRF24_RX_PW_P3			0x14
#define NRF24_RX_PW_P4			0x15
#define NRF24_RX_PW_P5			0x16
#define NRF24_FIFO_STATUS		0x17
#define NRF24_DYNPD				0x1C
#define NRF24_FEATURE			0x1D


/* Instruction Mnemonics */
#define NRF24_R_REGISTER		0x00
#define NRF24_W_REGISTER		0x20
#define NRF24_REGISTER_MASK		0x1F
#define NRF24_ACTIVATE			0x50
#define NRF24_R_RX_PL_WID		0x60
#define NRF24_R_RX_PAYLOAD		0x61
#define NRF24_W_TX_PAYLOAD		0xA0
#define NRF24_W_ACK_PAYLOAD		0xA8
#define NRF24_FLUSH_TX			0xE1
#define NRF24_FLUSH_RX			0xE2
#define NRF24_REUSE_TX_PL		0xE3
#define NRF24_NOP				0xFF




uint8_t NRF24_Transmit(uint8_t *data);
void NRF24_TxMode(uint8_t *addr, uint8_t channel);
void NRF24_Init(void);
void NRF24_Receive(uint8_t *data);


#endif /* INC_NRF24L01_H_ */
