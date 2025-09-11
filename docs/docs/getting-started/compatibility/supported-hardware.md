---
sidebar_position: 3
title: Supported hardware
---

# Supported hardware

The AICA System supports many different types of manipulators and sensors. The following tables present these supported
devices, along with necessary prerequisites and additional software that these might require.

:::tip

This list is continuously updated. In cases of unsupported hardware, users may contact the AICA support team for more
information.

:::

## Robotic manipulators

| Brand        | Supported Devices            | Requirements | Additional Notes | 
|--------------|-----------------------------|------------------------------|------------------|
| [ABB](../../examples/guides/abb-guide.md) | IRB 1010, CRB 15000 - 12 | RobotStudio, EGM | - |
| Franka | Emika Panda | **TBA** | **TBA** |
| Kassow | KR810, KR1410 | **TBA** | **TBA** |
| KUKA | KR 10 R1100-2, KR 16, KR 70, KR 210 R2700-2 | **TBA** | **TBA** |
| Mecademic | Meca500 | - | TCP/IP & EtherCAT interfaces available |
| Stäubli | TX2-40, TX2-60 | **TBA** | **TBA** |
| UFACTORY | X-ARM 6 | **TBA** | **TBA** |
| Universal Robots | UR5e, UR10e, UR16e, UR20, UR30 | Polyscope version > 5.X, External control URCap | - |

## Peripherals

| Brand        | Supported Devices            | Requirements | Additional Notes | 
|--------------|-----------------------------|------------------------------|------------------|
| SCHUNK | EGU | **TBA** | **TBA** |

## Sensors

| Brand        | Supported Devices            | Requirements | Additional Notes | 
|--------------|-----------------------------|------------------------------|------------------|
| ATI | **TBA** | **TBA** | **TBA** |
| Bota Systems | MiniONE Pro, SensONE | - | Serial & EtherCAT interfaces available where applicable |
| [Orbbec](../../examples/guides/orbbec-component.md) | Gemini 33X/435Le/2/2L, Femto Bolt/Mega, Astra 2  | - | udev rule application |
| [RealSense](../../examples/guides/realsense-component.md) |  L515, D4XX | - | udev rule application |

:::tip

AICA's Camera Streamer component available in the `components/core-vision` collection can be used to stream images from
any camera that can be natively mounted as a USB device by the host machine. It may also be used with video files. In
cases where the camera is not otherwise supported, a pre-recorded file may be supplied to the component, provided the
application permits it.

:::
