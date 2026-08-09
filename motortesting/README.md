# Arduino Bluetooth Controlled Quadcopter 🚁

A custom quadcopter project controlled using an Arduino Nano and an HC-05 Bluetooth module.

## Hardware

- Arduino Nano
- HC-05 Bluetooth module
- DRV8833 motor drivers
- 8520 coreless DC motors
- LiPo battery
- MPU6050 IMU
- Custom quadcopter frame

## Current Features

- Bluetooth RC control
- Variable motor throttle
- Forward and backward throttle control
- Left and right control
- PWM-based motor speed control

## Motor Control

The Arduino Nano receives commands from an Android RC Bluetooth Controller application through the HC-05.

The throttle is implemented progressively:

- Holding Forward increases motor speed every 200 ms.
- Holding Backward decreases motor speed every 200 ms.
- Speed ranges from 0 to 255 PWM.

## Development Status

### Completed
- [x] Basic motor control
- [x] HC-05 Bluetooth communication
- [x] Variable throttle
- [x] DRV8833 motor control

### In Progress
- [ ] MPU6050 integration
- [ ] Pitch/roll stabilization
- [ ] PID controller
- [ ] Stable free flight

## Demo

Video demonstration coming soon.

## Future Improvements

- IMU-based stabilization
- PID control
- Improved flight control
- Wireless telemetry