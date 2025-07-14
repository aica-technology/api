---
sidebar_position: 9
title: UR Hardware Interface
---

import eventEdge from './assets/event-edge.png'
import eventSourceHandle from './assets/event-source-handle.png'
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import timerSettings from './assets/timer-settings.png'
import triggerButtons from './assets/trigger-buttons.gif'

# Universal Robots

Universal Robots (UR) offers a variety of collaborative robotic arms widely adopted across industries and research for
their ease of use, simplicity and flexibility. UR robots are easy to use and integrate thanks to a graphical interface
on their teaching pendant and support for tailor-made, customized scripting via URScript.

The ecosystem around UR robots is highly developer-friendly, with open-source drivers, documentation, and integration
support for frameworks such as ROS (Robot Operating System). Their modular architecture and standardized communication
interfaces (e.g., TCP/IP, Modbus, RTDE) make them a popular choice for building custom applications. UR also provides
its own simulation environment (URSim), which allows developers to test and validate robot programs and interfaces
without needing physical hardware.

This guide introduces the hardware interface for UR robots, covering how to integrate and connect to UR manipulators
using AICA Studio. To connect to the simulation environment, check the
[respective documentation page](./ur-sim-guide.md).

TO ADD, TBD:

- use of the dashboard controller with local mode and the program node to hand back control between AICA and UR
- use of the UR impedance controller and hand guiding controller to leverage UR Force mode from AICA Studio
- use of the dashboard server to observe GPIOs, set payload, zero ft sensor etc

# Hardware Interface

TBD: There is already a [page](../../concepts/building-blocks/hardware-interfaces.md) on hardware interfaces. Are these
two conflicting? Should we link to this one?

Within the context of AICA Studio, but also in ROS, hardware interfaces are used as middleware between controllers and
actual hardware. In other words, they are tasked with two-way communication of commands and state feedback between the
robot and the controller that usually lies above.

## Local and Remote Control

The concept of Local and Remote Control on PolyScope can be easiest explained by introducing a _primary_ and _secondary_
device architecture. In Local Control, the controller is the _primary_ and has full authority on loading and starting
programs. In other words, the robot has to be used in person through the teach pendant and any commands sent from an
external source will be rejected. On the other hand, Remote Control allows to control the robot via external sources,
such as sockets, I/Os and the Dashboard Server. In this case, the controller is the _secondary_ and external sources can
load and start programs or directly send URScript commands to the controller.

:::note

Safety features remain active in Remote Control.

:::

Choosing one of the two modes depends on the specific situation at hand. During a development phase, it might be
preferable to create the programs in Local Mode, whereas in a production setting, PLCs would be responsible to load and
start the desired programs while the robot is in Remote Control. With the AICA System, users have the possiblity to get
the best of both modes:

1. Take full control of the robot from an AICA application (requires Remote Control)
2. Run an AICA application as one node of a program (works in both Local and Remote Control)

For the first case, no additional installation steps are required.

TODO example here

The second case requires the External Control URCap to be installed.

TODO link here to urcap

TODO link to example.

## Dashboard controller and exchange of control

TODO: Add dashboard controller example.

## Hand-guiding controller

TODO: Add hand guiding controller example.

Maybe a good point to break into a second page since the impedance controller and force mode might have more things to
discuss.

## Impedance controller
