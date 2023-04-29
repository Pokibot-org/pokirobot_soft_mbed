# Pokirobot V1 Soft MbedOS
Pokibot MbedOS program, intend to work only with the PokiRobot V1, a differential wheeled robot.

This program can be compiled and deploy using MBED CLI 1.

PlatformIO is not supported, as MbedOS package not be updated for some time.

> For more information on How to install Mbed CLI 1 on Windows, please see [Eirbot French Tutorial](https://github.com/eirbot/mbed-os-template-eirbot) 

## Requirements
### Hardware requirements
The following boards are required:
- [Nucleo F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)

### Software requirements
This repository makes use of the following libraries (automatically imported by `mbed deploy` or `mbed import`):

- CATIE : RBDC-Package (Apache 2.0)
  - [6tron_motor](https://github.com/catie-aq/6tron_motor/)
  - [6tron_motor-base](https://github.com/catie-aq/6tron_motor_base/)
  - [6tron_motor-sensor](https://github.com/catie-aq/6tron_motor-sensor/)
  - [6tron_odometry](https://github.com/catie-aq/6tron_odometry/)
  - [6tron_pid](https://github.com/catie-aq/6tron_pid/)
  - [RBDC](https://github.com/catie-aq/RBDC/)

- [MbedOS 6.17](https://github.com/ARMmbed/mbed-os/tree/mbed-os-6.17.0) (Apache 2.0)

## Usage
To clone **and** deploy the project in one command, use `mbed import` :

```shell
mbed import https://github.com/Pokibot-org/pokirobot_soft_mbed pokirobot_soft_mbed
```

Alternatively:

- Clone to "pokirobot_soft_mbed" and enter it:
  ```shell
  git clone https://github.com/Pokibot-org/pokirobot_soft_mbed pokirobot_soft_mbed
  cd mbed-rbdc-pokibot-example
  ```

- Deploy software requirements with:
  ```shell
  mbed deploy
  ```

Compile the project:
```shell
mbed compile
```

Program the board using embedded STLINK
```shell
mbed compile --flash
```

