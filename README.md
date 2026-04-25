# ATtiny25 Fan Control

This repository contains firmware for an **ATtiny25-based replacement design for the MIC502 fan management IC**.

The goal of this project is to reproduce the main MIC502 fan-control behavior using an ATtiny25 microcontroller, including temperature-based PWM control, minimum fan-speed clamp, startup drive, overtemperature fault output, and shutdown/reset behavior.

## Project Overview

The firmware reads a thermistor-conditioned voltage on the VT1 input and converts it into a PWM duty cycle for controlling a brushless DC fan through an external transistor or MOSFET driver.

The design is based on the MIC502 behavior where the control-voltage range from approximately **30% VDD to 70% VDD** corresponds to **0% to 100% PWM duty cycle**.

For this implementation, VT2 is simplified as a digital clamp input. When the VT2 clamp is enabled, the PWM duty cycle is not allowed to fall below 25%. This helps maintain a minimum fan speed and avoids repeated fan start/stop behavior.

## Target Hardware

- MCU: ATtiny25
- IDE: MPLAB X IDE
- Compiler: AVR-GCC / XC8 AVR toolchain
- Programming method: HVSP / High-Voltage Serial Programming
- Clock: Internal 8 MHz oscillator
- Supply voltage: 5 V nominal

## Important Fuse Configuration

This project uses **physical pin 1** of the ATtiny25 as the VT1 analog input.

On the ATtiny25, physical pin 1 is normally:

```text
PB5 / RESET / ADC0
