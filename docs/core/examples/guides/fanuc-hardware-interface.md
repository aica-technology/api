---
sidebar_position: 11
title: FANUC manipulators
---

import fanucHI from './assets/fanuc-HI.png'

# FANUC manipulators

FANUC is one of the largest and most established industrial robotic manipulator manufacturers, offering advanced
technology and robust performance for a wide range of automation tasks.

FANUC is one of the first major robot brands that enables accelerating Physical AI implementation through an official
first-party ROS driver in the `ros2_control` framework 
([article from Dec 2025](https://www.fanuc.co.jp/en/product/new_product/2025/202512_robot_physicalai.html)). This allows
advanced programming platforms like the AICA System to control the robots in real-time and deploy complex behaviors
without custom implementations.

:::info

The official driver is still under development and new functionalities are added continuously. Refer to the
[documentation](https://fanuc-corporation.github.io/fanuc_driver_doc/main/index.html) for the latest information for
hardware compatibility. The minimum robot controller software versions are:

- R-30iB Plus, R-30iB Mate Plus: V9.40P/81
- R-30iB Mini Plus: V9.40P/77
- R-50iA series: V10.10P/26

Additionally, one of the following software options are required:
- J519 Stream Motion and R912 Remote Motion
- S636 External Control Package, which includes the previous

:::

To use the FANUC collection, add `collections/fanuc` **v1.0.0** to your configuration in AICA Launcher, currently
supporting the CRX-10iA. Other robot models can be added on request. Simply reach out to the AICA support team for
further information.

## Connecting to a robot

The robot controllers usually provide multiple Ethernet ports. The choice of physical port is up to the user, but it has
to correspond to the port defined in the `$STMO.$PHYS_PORT` system variable. If you don't know how to apply the
necessary changes, contact your FANUC representative or the AICA support team.

Connect the robot with the machine that is running the AICA System and set the IP addresses such that the two devices
are on the same network.

Finally, on the teach pendant, switch from TP to Auto Mode. The driver will automatically set and start the right
program on the robot controller.

## Hardware interface

:::tip

For optimal performance, the rate of the hardware interface rate should be set according to the communication interval
of Stream Motion. For example, if the system variable `$STMO.$COM_INT` is set to 8 [ms], configure 125 Hertz in AICA
Studio. Depending on the robot controller being used, this value might be higher.

:::

Returning to AICA Studio and the hardware interface, it is now possible to define the parameters and connect to the
robot:

<div class="text--center">
  <img src={fanucHI} alt="FANUC hardware interface." style={{ borderRadius: "8px" }} />
</div>

- Robot IP: the IP address of the robot 
- RMI port: the port of the Remote Motion Interface (keep default unless otherwise configured on the robot controller)
- Stream Motion Port: the port of Stream Motion (keep default unless otherwise configured on the robot controller)
- Payload Schedule: the number of the payload to be set on the robot

Click **Start** to start the application and connect to the robot. 