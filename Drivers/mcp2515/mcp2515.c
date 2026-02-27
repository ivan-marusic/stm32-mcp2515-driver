/*
 * mcp2515.c
 *
 */
#include "mcp2515.h"
#include "stm32f4xx_hal_spi.h"
#include <stdio.h>

/*MCP2515_Reset ensures the controller starts from a known state*/
void MCP2515_Reset(MCP2515_HandleTypeDef *hdev) {
	uint8_t cmd = MCP_RESET;									// 0xC0
	MCP2515_Select(hdev);										// Pull CS low to start SPI communication
	HAL_SPI_Transmit(hdev->hspi, &cmd, 1, hdev->timeout);		// Send RESET command over SPI
	MCP2515_Deselect(hdev);										// Pull CS high to end SPI communication
	HAL_Delay(10);												// Wait for MCP2515 to finish reset
}

uint8_t MCP2515_SendMessage(MCP2515_HandleTypeDef *hdev, CAN_MessageTypeDef *msg) {
	uint8_t buf[14];

	// Load TX buffer 0
	buf[0] = MCP_LOAD_TX | 0x00;								// Command to load TX buffer 0
	buf[1] = (uint8_t)(msg->id >> 3);							// SIDH (Standard ID high bits)
	buf[2] = (uint8_t)(msg->id << 5);							// SIDL (Standard ID low bits)
	buf[3] = 0;													// EID8 (Extended ID, not used)
	buf[4] = 0;													// EID0
	buf[5] = msg->dlc;											// Data Length code

	for (int i = 0; i < msg->dlc; i++) {
		buf[6 + i] = msg->data[i];								// Copy data bytes(data starts from index 6)
	}

	MCP2515_Select(hdev);										// Pull CS low
	HAL_SPI_Transmit(hdev->hspi, buf, 6 + msg->dlc, hdev->timeout);
	MCP2515_Deselect(hdev);										// Pull CS high

	// Send RTS command to transmit
	uint8_t rts = MCP_RTS | 0x01;								// Request to send TX buffer 0
	MCP2515_Select(hdev);										// Pull CS low
	HAL_SPI_Transmit(hdev->hspi, &rts, 1, hdev->timeout);
	MCP2515_Deselect(hdev);										// Pull CS high

	return HAL_OK;
}

uint8_t MCP2515_ReceiveMessage(MCP2515_HandleTypeDef *hdev, CAN_MessageTypeDef *msg) {
	uint8_t opcode;
	uint8_t rxBuffer[13];								// SIDH, SIDL, EID8, EID0, DLC, up to 8 data bytes
	uint8_t cmd[2] = { MCP_READ, MCP_CANINTF };
	uint8_t status;
    uint8_t clearMask;

    // 1. Check CANINTF to see which RX buffer has a message
	MCP2515_Select(hdev);
	HAL_SPI_Transmit(hdev->hspi, cmd, 2, hdev->timeout);
	HAL_SPI_Receive(hdev->hspi, &status, 1, hdev->timeout);
	MCP2515_Deselect(hdev);

	// If RX0IF set → use RXB0
	if (status & CANINTF_RX0IF) {
		opcode = 0x90; 									// READ RX BUFFER for RXB0
		clearMask = CANINTF_RX0IF;
	// Else if RX1IF set → use RXB1
	} else if (status & CANINTF_RX1IF)  {
		opcode = 0x94;									// READ RX BUFFER for RXB1
		clearMask = CANINTF_RX1IF;
	} else {
		return HAL_ERROR;								// No message
	}

	// 2. Read RX buffer
	MCP2515_Select(hdev);
	HAL_SPI_Transmit(hdev->hspi, &opcode, 1, hdev->timeout);
	HAL_SPI_Receive(hdev->hspi, rxBuffer, 13, hdev->timeout);
	MCP2515_Deselect(hdev);

	// 3. Parse Standard ID (11-bit)
	msg->id = (rxBuffer[0] << 3) | (rxBuffer[1] >> 5);

    // 4. Parse DLC and clamp to 8
	msg->dlc = rxBuffer[4] & 0x0F;
	if (msg->dlc > 8) msg->dlc = 8;

	// 5. Copy data bytes into msg->data[]
	for (int i = 0; i < msg->dlc; i++) {
		msg->data[i] = rxBuffer[5 + i];
	}

    // 6. Clear interrupt flag for the buffer we read
	uint8_t clearCmd[4] = {MCP_BITMOD, MCP_CANINTF, clearMask, 0x00};
	MCP2515_Select(hdev);
	HAL_SPI_Transmit(hdev->hspi, clearCmd, 4, hdev->timeout);
	MCP2515_Deselect(hdev);

	return HAL_OK;
}

void MCP2515_SetNormalMode(MCP2515_HandleTypeDef *hdev) {

	// Setting mode
	uint8_t buf[8];
	buf[0] = MCP_WRITE;
	buf[1] = MCP_CANCTRL;
	buf[2] = MODE_NORMAL << 5;									// REQOP bits = 000

	MCP2515_Select(hdev);										// Pull CS low
	HAL_SPI_Transmit(hdev->hspi, buf, 3, hdev->timeout);
	MCP2515_Deselect(hdev);										// Pull CS high

	// Checking mode
	uint8_t cmd[2] = { MCP_READ, MCP_CANSTAT };
	uint8_t status;
	MCP2515_Select(hdev);										// Pull CS low
	HAL_SPI_Transmit(hdev->hspi, cmd, 2, hdev->timeout);		// Transmit READ(0x03) + address(CANSTAT(0x0E))
	HAL_SPI_Receive(hdev->hspi, &status, 1, hdev->timeout);		// Receive register value
	MCP2515_Deselect(hdev);										// Pull CS high

	if ((status & 0xE0) == (MODE_NORMAL << 5)) {				// Masking bits 7:5 (0xE0)
		printf("OPMOD = NORMAL MODE");
	}
}

void MCP2515_Select(MCP2515_HandleTypeDef *hdev) {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);		// The CS pin must be low to enable SPI communication
}

void MCP2515_Deselect(MCP2515_HandleTypeDef *hdev) {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		// The CS pin must be high to disable SPI communication
}

void MCP2515_SetBitrate(MCP2515_HandleTypeDef *hdev) {

	MCP2515_WriteRegister(hdev, MCP_CNF1, MCP_8MHz_500kBPS_CFG1); // BRP = 0, SJW = 1

	MCP2515_WriteRegister(hdev, MCP_CNF2, MCP_8MHz_500kBPS_CFG2); // BTLMODE=1, PhaseSeg1=3, PropSeg=1

	MCP2515_WriteRegister(hdev, MCP_CNF3, MCP_8MHz_500kBPS_CFG3); // PhaseSeg2=3
}

void MCP2515_WriteRegister(MCP2515_HandleTypeDef *hdev, uint8_t address, uint8_t data) {
	uint8_t buf[3];

	buf[0] = MCP_WRITE;											// 0x02
	buf[1] = address;											// Register address
	buf[2] = data;												// Data to write

	MCP2515_Select(hdev);										// Pull CS low
	HAL_SPI_Transmit(hdev->hspi, buf, 3, hdev->timeout);
	MCP2515_Deselect(hdev);										// Pull CS high
}

uint8_t MCP2515_ReadRegister(MCP2515_HandleTypeDef *hdev, uint8_t address) {
	uint8_t rxData;
	uint8_t cmd[2] = { MCP_READ, address};
	MCP2515_Select(hdev);										// Pull CS low
	HAL_SPI_Transmit(hdev->hspi, cmd, 2, hdev->timeout);
	HAL_SPI_Receive(hdev->hspi, &rxData, 1, hdev->timeout);
	MCP2515_Deselect(hdev);										// Pull CS high

	return rxData;
}

void MCP2515_ModifyRegister(MCP2515_HandleTypeDef *hdev, uint8_t address, uint8_t mask, uint8_t data) {
	uint8_t cmd[4] = { MCP_BITMOD, address, mask, data};
	MCP2515_Select(hdev);										// Pull CS low
	HAL_SPI_Transmit(hdev->hspi, cmd, 4, hdev->timeout);		// Send BIT MODIFY command + address + mask + data
	MCP2515_Deselect(hdev);										// Pull CS high
}

