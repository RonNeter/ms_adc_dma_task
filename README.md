# ADC + DMA Sampling System

## Project Description
This project samples analog signals from 3 sensors using ADC with DMA on an STM32F3 microcontroller and sends the data over UART. It provides a simple, efficient way to monitor multiple analog signals in real-time.

## Features
- Samples 3 analog inputs simultaneously
- Sends formatted data over UART every 100ms
- Allows timestamp reset with button press
- Visual feedback via LED blinking

## Setup
1. Connect analog inputs to PA0, PA1, and PC0
2. Connect to serial terminal (115200 baud)
3. Power on the board

## Usage
- Data format: `Time: X ms, CH1: Y, CH2: Z, CH3: W`
- Press the blue button to reset the timestamp
- LED blinks to indicate system operation

## Notes
This implementation uses direct register configuration for ADC+DMA on the STM32F3 series to ensure reliable operation.
