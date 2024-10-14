# Bootloader CAN for STM32 Nucleo-F446RE

This project implements a bootloader for the STM32 Nucleo-F446RE board using CAN communication. It supports two operating modes:

- **Normal Mode**: Standard communication through the CAN bus.
- **Loopback Mode**: CAN communication in loopback mode, allowing internal testing without external devices.

## Features

- Receive commands via CAN to perform operations such as:
  - Flash memory erasure
  - Memory writing
  - Read/Write protection management
- Mode selection between normal and loopback
- Debug messages through the CAN bus

## Repository Structure

- **BOOTLOADER_CAN_NormalMode**: Contains the implementation for the normal communication mode.
- **BOOTLOADER_CAN_LoopBack**: Contains the implementation for the loopback communication mode.

## Getting Started

1. Clone this repository to your local machine.
2. Build and flash the code to your STM32 Nucleo-F446RE using your preferred IDE (e.g., STM32CubeIDE).
3. Use CAN tools or a CAN transceiver to send commands to the bootloader.

### Requirements

- STM32 Nucleo-F446RE board
- CAN transceiver for normal mode
- STM32CubeMX and STM32CubeIDE (or any other STM32 development toolchain)
