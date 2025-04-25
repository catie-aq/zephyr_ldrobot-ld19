# LDROBOT LD19 / STL-19P Sensor Driver

LDROBOT Co. LD19 lidar sensor driver for Zephyr OS.

Based on [LD19 Datasheet](https://www.ldrobot.com/images/2023/05/23/LDROBOT_LD19_Datasheet_EN_v2.6_Q1JXIRVq.pdf) and [LD19 Development Manual](https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf). Also compatible with [STL-19P](https://www.ldrobot.com/images/2024/05/06/LDROBOT_STL-19P_Datasheet_EN_v1.0_nZXymYcr.pdf) (LD19 replacement since 2023).

Implemented features:

- [X] Sensor Distance
- [X] CRC data check
- [X] Optional Zephyr [Sensor Trigger](https://docs.zephyrproject.org/latest/hardware/peripherals/sensor/triggers.html)

## Usage

This sensor driver can be used to get sensor data from LD19 LiDAR: point distance and intensity for each angle step (typically 0.8 degrees at 10 Hz).

The PWM pin must be grounded for hardware automated internal speed regulation (10 ± 0.1 Hz).
