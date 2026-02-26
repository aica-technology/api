---
sidebar_position: 15
title: FANUC manipulators
---

import fanucLocal from './assets/fanuc-local-detail.png'
import fanucRemote from './assets/fanuc-remote.png'
import fanucHI from './assets/fanuc-HI.png'

# FANUC manipulators

**TODO: Intro FANUC text**

FANUC is one of the largest and most established industrial robotic manipulator manufacturers.  

To use the FANUC collection, add `collections/fanuc` **v1.0.0** to your configuration in AICA Launcher, currently
supporting the CRX-10iA. For more robot descriptions, please contact the AICA support team. 

This guide and the provided drivers are designed for FANUC robots, adhering to the specifications provided with 
the FANUC ROS 2 driver. Namely, for full compatibility the following are required:

- Robot controller
    - R-30iB Plus, R-30iB Mate Plus: V9.40P/81 or later
    - R-30iB Mini Plus: V9.40P/77 or later
    - R-50iA series: V10.10P/26 or later

- Additional software options, one of the following:
    - J519 Stream Motion and R912 Remote Motion
    - S636 External Control Package, which includes the previous

For more information, see the [official release instructions](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/environment/system_requirements.html).

:::warning

As stated in the official page, J568 and J570 are no longer required.

:::

## Connecting to a robot

Robot controllers usually provide multiple Ethernet ports. Make sure to connect to the one defined in the 
`$STMO.$PHYS_PORT` system variable. If you do not know how to apply the necessary changes, contact the robot manufacturer
or the AICA support team.

Activate the robot control tablet and, if in Local, set it to Remote mode from the icon on the top right. 

<div class="text--center">
  <img src={fanucLocal} alt="FANUC tablet in local mode." style={{ width: '110px', borderRadius: "8px" }} />
</div>

This is how the tablet screen should look like, with the robot in Remote mode and ready to connect:

<div class="text--center">
  <img src={fanucRemote} alt="FANUC tablet in remote mode." style={{ width: '400px', borderRadius: "8px" }} />
</div>

:::note

You might also need to confirm the payload the first time the controller boots.

:::

## Hardware interface

:::tip

For optimal performance, set the hardware interface rate to 125 Hertz. Depending the controller rate, this might
be set higher. Consult with the original documentation or contact the AICA support team for more details. 

:::

<div class="text--center">
  <img src={fanucHI} alt="FANUC hardware interface." style={{ borderRadius: "8px" }} />
</div>

Returning to AICA Studio and the hardware interface, it is now possible to define the parameters and connect to the
robot:

**TODO**
- Robot IP: the address of the robot 
- RMI port: the port of the FANUC Remote Motion Interface, used to set ? 
- Stream Motion Port: the port to stream motion commands
- Payload Schedule: ?

Click **Start** to start the application and connect to the robot. 