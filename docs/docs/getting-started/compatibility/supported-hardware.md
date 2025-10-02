---
sidebar_position: 2
title: Supported hardware
---

# Supported hardware

The AICA System supports many different types of hardware. The following tables present robotic manipulators and
peripherals for which drivers exist, along with necessary prerequisites and additional software that these might
require.

:::note

This list is work in progress and regularly updated. If your hardware is not listed, please submit a ticket to the  
[AICA Community Feedback board](https://github.com/aica-technology/community) or reach out to the AICA team for
assistance or further details.

As described in [this section](../../concepts/ros-concepts/built-on-ros.md), the AICA System can integrate existing ROS
drivers. This allows you to expand the supported hardware list by using open source or custom-developed ROS packages.

:::

## Robotic manipulators

| Brand        | Robot Controller            | Add-ons | Additional Notes | 
|--------------|-----------------------------|---------|------------------|
| ABB | OmniCore | Robot Web Services, Externally Guided Motion | RobotWare 7.X |
| KUKA | KR C5 | RobotSensorInterface | |
| Mecademic | n/a | n/a | TCP/IP & EtherCAT drivers for Meca500 |
| Stäubli | CS9, CS8C | alter, advCtrlFunctions  | TX2 series |
| Universal Robots | Polyscope > 5.X | n/a | |
| Franka Emika | n/a | Fast Research Interface | beta version available |
| Kassow | n/a | KORD CBun | beta version available |
| Kinova | Gen3 | n/a | beta version available |
| UFACTORY | n/a | n/a | beta version available |

## Peripherals

| Brand        | Supported Devices            | Requirements | Additional Notes | 
|--------------|-----------------------------|------------------------------|------------------|
| SCHUNK | EGU, EGK, EZU mechatronic grippers | Modbus RTU or Ethernet-based (PROFINET, Ethernet/IP, and EtherCAT) interface | |
| ATI | Force / Torque Sensors | Net F/T interface | |
| Bota Systems | Force / Torque Sensors | Serial or EtherCAT interface | |
| [Orbbec](../../examples/guides/orbbec-component.md) | Gemini 33X/435Le/2/2L, Femto Bolt/Mega, Astra 2  | n/a | |
| [Intel RealSense](../../examples/guides/realsense-component.md) |  L515, D4XX | n/a | |

:::tip

You can also use generic cameras and webcams with the Camera Streamer from the `core-vision` components package,
provided your host machine can natively mount them as USB devices.

:::
