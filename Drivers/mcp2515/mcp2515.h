#ifndef MCP2515_H
#define MCP2515_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stm32f4xx_hal_spi.h>

// MCP2515 SPI Instructions
#define MCP_RESET           		0xC0	// Reset MCP2515: clears registers, sets Configuration mode
#define MCP_READ            		0x03	// Read from register starting at given address
#define MCP_READ_RX         		0x90	// Read RX buffer (CAN message); 0x90 for RXB0, 0x94 for RXB1
#define MCP_WRITE           		0x02	// Write to register starting at given address
#define MCP_LOAD_TX         		0x40	// Load TX buffer with CAN message data
#define MCP_RTS             		0x80	// Request-to-Send: start transmission from TX buffer(s)
#define MCP_READ_STATUS     		0xA0	// Quick status read: TX/RX status bits
#define MCP_RX_STATUS       		0xB0	// Quick RX status: filter match, message type (standard, extended and/or remote)
#define MCP_BITMOD          		0x05	// Modify specific bits in a register (mask + data)

// MCP2515 Registers
#define MCP_CANSTAT         		0x0E	// CAN STATUS REGISTER
#define MCP_CANCTRL         		0x0F	// CAN CONTROL REGISTER
/*These registers can only be modified when the MCP2515 is in
 Configuration mode(automatically selected
 after power-up, a Reset or can be entered from any
 other mode by setting the REQOP[2:0] bits to ‘100’.)*/
#define MCP_CNF1            		0x2A	// CONFIGURATION REGISTER 1
#define MCP_CNF2            		0x29	// CONFIGURATION REGISTER 2
#define MCP_CNF3            		0x28	// CONFIGURATION REGISTER 3
#define MCP_TXB0CTRL        		0x30	// TRANSMIT BUFFER 0 CONTROL REGISTER
#define MCP_TXB0SIDH        		0x31	// TRANSMIT BUFFER 0 STANDARD IDENTIFIER REGISTER HIGH
#define MCP_TXB0SIDL        		0x32	// TRANSMIT BUFFER 0 STANDARD IDENTIFIER REGISTER LOW
#define MCP_TXB0DLC         		0x35	// TRANSMIT BUFFER 0 DATA LENGTH CODE REGISTER
#define MCP_TXB0DATA        		0x36	// TRANSMIT BUFFER 0 DATA BYTE REGISTER
#define MCP_RXB0CTRL        		0x60	// RECEIVE BUFFER 0 CONTROL REGISTER
#define MCP_RXB1CTRL				0x70	// RECEIVE BUFFER 1 CONTROL REGISTER
#define MCP_RXB0SIDH        		0x61	// RECEIVE BUFFER 0 STANDARD IDENTIFIER REGISTER HIGH
#define MCP_RXB1SIDH				0x71	// RECEIVE BUFFER 1 STANDARD IDENTIFIER REGISTER HIGH
#define MCP_RXB0SIDL        		0x62	// RECEIVE BUFFER 0 STANDARD IDENTIFIER REGISTER LOW
#define MCP_RXB1SIDL				0x72	// RECEIVE BUFFER 1 STANDARD IDENTIFIER REGISTER LOW
#define MCP_RXB0DLC         		0x65	// RECEIVE BUFFER 0 DATA LENGTH CODE REGISTER
#define MCP_RXB1DLC					0x75	// RECEIVE BUFFER 1 DATA LENGTH CODE REGISTER
#define MCP_RXB0DATA        		0x66	// RECEIVE BUFFER 0 DATA BYTE REGISTER
#define MCP_RXB1DATA				0x76	// RECEIVE BUFFER 1 DATA BYTE REGISTER
#define MCP_CANINTE        			0x2B	// CAN INTERRUPT ENABLE REGISTER
#define MCP_CANINTF         		0x2C	// CAN INTERRUPT FLAG REGISTER
#define MCP_EFLG            		0x2D	// ERROR FLAG REGISTER

// CANCTRL Register Values
#define MODE_NORMAL         		0x00	// Normal Operation mode; REQOP[2:0]=000
#define MODE_SLEEP          		0x20	// Sleep mode; REQOP[2:0]=001
#define MODE_LOOPBACK       		0x40	// Loop back mode; REQOP[2:0]=010
#define MODE_LISTENONLY    			0x60	// Listen-Only mode; REQOP[2:0]=011
#define MODE_CONFIG        			0x80	// Configuration mode; REQOP[2:0]=100

// CANINTF Flags
#define CANINTF_RX0IF       		0x01	// Interrupt flag: RX buffer 0 has received a message
#define CANINTF_RX1IF				0x02	// Interrupt flag: RX buffer 1 has received a message
#define CANINTF_TX0IF       		0x04	// Interrupt flag: TX buffer 0 message transmission completed

// CAN Speed Configuration (example for 8MHz oscillator)
#define MCP_8MHz_500kBPS_CFG1 		0x00
#define MCP_8MHz_500kBPS_CFG2	 	0x90
#define MCP_8MHz_500kBPS_CFG3 		0x82

typedef struct {
    SPI_HandleTypeDef *hspi;				// pointer to the SPI peripheral
    GPIO_TypeDef *cs_port;					// GPIO port for Chip Select (CS)
    uint16_t cs_pin;						// GPIO pin for CS
    uint32_t timeout;						// Timeout for SPI operations
} MCP2515_HandleTypeDef;

typedef struct {
    uint32_t id;							// CAN message identifier
    uint8_t dlc;							// Data Length Code
    uint8_t data[8];
} CAN_MessageTypeDef;

// Function Prototypes
void MCP2515_Select(MCP2515_HandleTypeDef *hdev);
void MCP2515_Deselect(MCP2515_HandleTypeDef *hdev);
uint8_t MCP2515_ReadRegister(MCP2515_HandleTypeDef *hdev, uint8_t address);
void MCP2515_WriteRegister(MCP2515_HandleTypeDef *hdev, uint8_t address, uint8_t data);
void MCP2515_ModifyRegister(MCP2515_HandleTypeDef *hdev, uint8_t address, uint8_t mask, uint8_t data);
void MCP2515_Reset(MCP2515_HandleTypeDef *hdev);
void MCP2515_SetBitrate(MCP2515_HandleTypeDef *hdev);
void MCP2515_SetNormalMode(MCP2515_HandleTypeDef *hdev);
uint8_t MCP2515_SendMessage(MCP2515_HandleTypeDef *hdev, CAN_MessageTypeDef *msg);
uint8_t MCP2515_ReceiveMessage(MCP2515_HandleTypeDef *hdev, CAN_MessageTypeDef *msg);

#endif // MCP2515_H
