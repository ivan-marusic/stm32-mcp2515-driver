# STM32 MCP2515 CAN Driver (STM32F429 & STM32F411)

This repository contains MCP2515 CAN bus driver written for STM32 microcontrollers using the STM32 HAL.
It includes:
- Custom MCP2515 driver (mcp2515.c/.h)
- Working example projects for STM32F429 and STM32F411
- Hardware setup + photos
- Wiring diagrams
- CAN bus explanation

This project demonstrates communication between two STM32 boards using the MCP2515 (SPI CAN controller) and a twisted‑pair CAN bus.

## Project Overview
The goal of this project is to implement and demonstrate a fully functioning CAN network between:

- STM32F429 + MCP2515
- STM32F411 + MCP2515

Both nodes exchange CAN frames over a standard 2‑wire CAN bus with 120 Ω termination.
This driver was written from scratch:

- SPI communication
- MCP2515 internal register map
- CAN frame structure (standard IDs)
- Interrupt flags, buffers, and bit timing
- Embedded multi‑node networks

## Features
### Custom MCP2515 driver

Reset via SPI (0xC0)
Register read/write
Bit modify (BITMOD)
Load TX buffer
Request‑to‑Send
Receive from RXB0/RXB1
Parse standard 11‑bit CAN IDs
Clear interrupt flags

### CAN configuration

Bit timing for 8 MHz crystal @ 500 kbps
Compatible with TJA1050/MCP2551 transceiver

### Example applications

STM32F429 → sender
STM32F411 → receiver

### Hardware documentation

Wiring diagrams
Real photos
Bus topology

## Repository Structure
stm32-mcp2515-driver/
│
├─ Drivers/
│  └─ mcp2515/
│     ├─ mcp2515.c
│     └─ mcp2515.h
│
├─ examples/
│  ├─ stm32f429_sender/
│  │   └─ main.c
│  └─ stm32f411_receiver/
│      └─ main.c
│
├─ hardware/
│  ├─ stm32f429_mcp2515.jpg
│  ├─ stm32f411_mcp2515.jpg
│  └─ can_bus_wiring.jpg
│
├─ .gitignore
├─ LICENSE
└─ README.md

## Hardware
This project uses two separate CAN nodes:

### Node A – STM32F429 + MCP2515

STM32F429 Discovery board
MCP2515 module with 8 MHz crystal
SPI connections:
MCP2515          STM32F429
SCK       -->     PA5
MISO      -->     PA6
MOSI      -->     PA7
CS        -->     PB12
INT       -->     PB0 (optional)
VCC       -->    5V
GND       -->    GND

### Node B – STM32F411 + MCP2515

STM32F411 Nucleo board
MCP2515 
SPI connections:
MCP2515          STM32F411
SCK       -->     PA5
MISO      -->     PA6
MOSI      -->     PA7
CS        -->     PB12
INT       -->     PB0 (optional)
VCC       -->    5V
GND       -->    GND

## CAN Bus Wiring
Both MCP2515 modules are connected using a twisted pair:
CANH      -->    CANH
CANL      -->    CANL
GND       -->    GND

## Hardware Photos


## Example: CAN Sender (STM32F429)
bash```
CAN_MessageTypeDef msg = {
    .id = 0x123,
    .dlc = 8,
    .data = {1, 2, 3, 4, 5, 6, 7, 8}
};

while (1) {
    MCP2515_SendMessage(&hcan, &msg);
    HAL_Delay(1000);
}```

## Example: CAN Receiver (STM32F411)
bash```
CAN_MessageTypeDef rx;

while (1) {
    if (MCP2515_ReceiveMessage(&hcan, &rx) == HAL_OK) {
        printf("ID: 0x%03lX  DLC: %d  Data:", rx.id, rx.dlc);
        for (int i = 0; i < rx.dlc; i++)
            printf(" %02X", rx.data[i]);
        printf("\r\n");
    }
}```

## Driver Initialization Sequence
Each node must call:
bash```
MCP2515_Reset(&hcan);
MCP2515_SetBitrate(&hcan);     // 500 kbps, 8 MHz
MCP2515_SetNormalMode(&hcan);
```

```
MCP2515_WriteRegister(&hcan, MCP_CANINTE,
                      CANINTF_RX0IF | CANINTF_RX1IF);```

Bitrate
The default bitrate is:

500 kbps @ 8 MHz crystal

Registers used:
Register            Value
CNF1                0x00
CNF2                0x90
CNF3                0x82

