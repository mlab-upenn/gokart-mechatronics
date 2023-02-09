# Nucleo-L432KC-IMU 

This project provides the ability to control an LSM9DS1 IMU from an STM32L4 MCU. The project is FreeRTOS-based, specifically using the CMSIS OS v1 wrapper provided by STM32CUBE. 

A console is provided for streaming sensor data over a UART, and for sending commands via same. 

The USART driver for the console is interrupt-driven TX and DMA-driven RX. The I2C driver for communicating with the IMU is interrupt-driven.

This project has been tested using the following hardware:

- STMicroelectronics Nucleo L432KC
- Adafruit LSM9DS1 breakout
- FTDI LC231X TTL Serial
