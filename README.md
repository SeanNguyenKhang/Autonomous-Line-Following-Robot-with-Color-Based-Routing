# Autonomous-Line-Following-Robot-with-Color-Based-Routing
STM32-based autonomous line-following robot with PID control, automatic sensor calibration, and color-based navigation using TCS3200 sensor.



📌 Project Overview

This project implements an autonomous line-following robot using an STM32 microcontroller.
The robot is capable of:

- Following a line using PID control
- Automatically calibrating line sensors
- Detecting a loading station
- Reading package color using a TCS3200 color sensor
- Choosing navigation direction based on detected color
- Reaching a final destination autonomously
The system is designed as a state-machine-based autonomous robot suitable for robotics competitions or embedded systems learning.

⚙️ Hardware Components
- STM32 MCU
- 5-channel TCRT5000 (ADC + DMA)
- Dual DC Motors JGB37-520 12v 333rpm
- Motor Driver TB6612FNG
- Quadrature Encoders
- TCS3200 Color Sensor
- Buck Converter 12VDC - 5V and 3.3V

System Architecture

The robot operates using a Finite State Machine (FSM):

START
   ↓
GO_TO_STATION
   ↓
READ_COLOR
   ↓
ESCAPE_STATION
   ↓
GO_TO_FINISH
   ↓
FINISHED
