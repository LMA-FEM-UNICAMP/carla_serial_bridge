# carla_serial_bridge

A bridge serial-ROS for communicate microAutoware to CARLA through UART.e

---

## Communication protocol

### uC to CARLA

- Data:
  - float Steering Angle;
  - float Steering Velocity;
  - float Speed;
  - float Acceleration;
  - float Jerk;
  - unsigned char Control Mode.

- 30 bytes of data;

- Message structure: `#S%c%c%c%cW%c%c%c%cV%c%c%c%cA%c%c%c%cJ%c%c%c%cM%c$`

- [UART State Machine diagram uC to CARLA](docs/UART%20State%20Machine%20uC%20to%20CARLA.pdf)

### CARLA to uC

- Data:
  - float Longitudinal Speed;
  - float Latitudinal Speed;
  - float Heading Rate;
  - float Steering Status.
- 22 bytes of data
- Message structure: `#A%c%c%c%cB%c%c%c%cC%c%c%c%cD%c%c%c%c$`
- [UART State Machine diagram CARLA to uC](docs/UART%20State%20Machine%20CARLA%20to%20uC.pdf)
