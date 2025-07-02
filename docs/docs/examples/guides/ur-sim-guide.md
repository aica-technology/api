---
sidebar_position: 5
title: URSim Guide
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

import ursimActivate from './assets/ursim-activate.gif'
import ursimHardwareParams from './assets/ursimHarwareParams.png'

# URSim guide

URSim is a software-in-the-loop simulation tool that allows the behavior of the real Universal Robots controller to be
emulated on a computer. This environment can be utilized to become familiar with the robot interface, as well as to
create and run programs. Please note that compared to the real robot, there are some limitations to the simulator.
Especially the force mode will be limited in use, since there is no physics engine to simulate valid foce values from
the sensor. Other functionality that is not present includes:

- Emergency stop button
- Input IO state
- Self or surrounding object collisions

For more information, please refer to the
[installation page](https://www.universal-robots.com/download/software-ur-series/simulator-linux/offline-simulator-ur-series-e-series-ur-sim-for-linux-5220/).

This guide aims to support users with running URSim and connecting it to AICA Studio. This is a great way to validate
application operation and safe execution before moving to the actual hardware.

## Installation

Executing the following instructions runs URSim in a Docker container. This guide assumes that users have already
installed and configured Docker, as it is also required for AICA Studio. For more information you can check the
[installation and launch instructions](../../getting-started/installation/installation-and-launch.md) on the Getting
started page of our documentation.

Executing the following commands runs URSim in a Docker container:

```bash
git clone https://github.com/aica-technology/simulators.git
cd simulators
./run.sh
```

The last command installs and runs the URSim Docker container, simulating the UR5e robot by default. In case users want
to simulate a different robot model, the `-m` argument can be used. 

```bash
./run.sh -m ur10e
```

Another useful argument is `-v`, which defines the URSim version to be used. It is important to make sure that the URSim
version matches the Polyscope version in the actual hardware to avoid incompatibilities. Please also note that there
might be incompatibilites between 5.1x and 5.2x, with applications built in newer versions not being backwards
compatible.

```bash
./run.sh -v 5.16.1
```

For a detailed description of the available arguments, use:

```bash
./run.sh -h
```

Successfull execution of the command will produce an output such as the following:

```bash
ROBOT_MODEL: ur5e
ROBOT_SERIES: e-series
URSIM_VERSION: 5.22.0
d05e6d832f2837542984c11c473e8fec4005323dc9abdc5913cf95eb4bb284af
Docker URSim is running

To access PolyScope, open the following URL in a web browser.


	http://192.168.56.101:6080/vnc.html

To exit, press CTRL+C
```

:::note

URSim is made for Linux. For other operating systems, a virtual machine is needed. For more information, check the
[official installation instructions](https://www.universal-robots.com/download/software-ur-series/simulator-non-linux/offline-simulator-ur-series-e-series-ur-sim-for-non-linux-5220/).

:::

## Accessing and configuring the simulated robot

Follow the terminal link in a browser to access the simulated robot.

1. In the window that appears, select **Connect**.
2. After the teaching pendant interface loads up, navigate to the settings page by clicking the burger icon in the top
   left corner of the screen.
3. Click on the **System** tab, then select the **Remote Control** tab.
4. Click **Enable** and then **Exit** at the bottom left of the screen.
5. Turn on the robot by pressing the red button located in the bottom left corner of the screen. Click **ON** followed
   by **START** to activate, then click **Exit**.
6. The simulator is now ready to interface with an AICA application.

**TODO: Re-capture the process without switching to remote control**

<div class="text--center">
  <img src={ursimActivate} alt="Activating the simulator" />
</div>

## Local VS Remote Control

The robot can operate in two modes, namely **Local** and **Remote Control**. These mostly differ in their accessibility
and method of interaction. Local Control involves direct operation through the teach pendant and robot's physical
interface, while Remote Control allows for operation from an external device through a network connection. In order to
use the Remote Control, the `external_control` URCap has to be installed. Then, there are two options to operate the
robot remotely:

1. There is a touchpad icon labeled **Local** in the top right corner of the simulator. Click on this icon and then
   select **Remote Control**.
2. Leave this option in **Local Control**, but use the ExternalControl node in the program. This allows the exchange of
   control between remote and local wherever it is desired in the program's execution flow.

TODO: I mention this below, but should we add some more details on headless mode? I remember that depending on the mode
you need to switch between true and false. But probably it's better to mention this in the hardware interface
description below.

:::tip

Setting the robot to Remote Control is not strictly required for the simulator. It is possible to send commands while in
Local Control, where the **Installation** tab can be used to visualize the robot's motions.

:::

## Connecting to AICA Studio

In AICA Studio, open the Hardware section and select the same UR description as the simulated robot. Ensure that the IP
address in the **Robot Ip** field matches the IP address displayed in the terminal. The application can now be run with
the simulated robot being used instead of the real one. Also make sure the **Headless Mode** is set to **true**.

TODO: Some more details on headless mode?

<div class="text--center">
  <img src={ursimHardwareParams} alt="Setting hardware parameters" />
</div>
