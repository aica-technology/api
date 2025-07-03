---
sidebar_position: 7
title: URSim Guide
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

import ursimActivate from './assets/ursim-activate.gif'
import ursimHardwareParams from './assets/ursimHarwareParams.png'

# URSim guide

URSim is a software-in-the-loop simulation tool that allows to emulate the behavior of the real Universal Robots
controller on a computer. This environment can be utilized to become familiar with the robot interface, as well as to
create and run programs. Note that compared to the real robot, there are some limitations to the simulator. Especially
the force mode will be limited in use, since there is no physics engine to simulate valid force values from the sensor.
Other functionality that is not present includes:

- Emergency stop button
- Input IO state
- Self or surrounding object collisions

For more information, please refer to UR's
[download page](https://www.universal-robots.com/download/software-ur-series/simulator-linux/offline-simulator-ur-series-e-series-ur-sim-for-linux-5220/).

This guide aims to support users with running URSim and connecting it to AICA Studio. This is a great way to validate
application operation and safe execution before moving to the actual hardware.

## Installation

Executing the following instructions runs URSim in a Docker container. This guide assumes that users have already
installed and configured Docker, as it is also required for AICA Studio. For more information you can check the section
about Docker from our
[installation and launch instructions](../../getting-started/installation/installation-and-launch.md).

:::note

URSim is made for Linux. For other operating systems, a virtual machine is needed. For more information, check the
[official installation instructions](https://www.universal-robots.com/download/software-ur-series/simulator-non-linux/offline-simulator-ur-series-e-series-ur-sim-for-non-linux-5220/).

:::

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

Another useful argument is `-v`, which defines the URSim version to be used. It is preferable to make sure that the
[URSim version](https://hub.docker.com/r/universalrobots/ursim_e-series/tags) matches the Polyscope version in the
actual hardware to avoid incompatibilities. For example, with the recent addition of OptiMove, UR programs built in 5.21
and upwards might not be backwards compatible.

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

## Accessing and configuring the simulated robot

Follow the terminal link in a browser to access the simulated robot.

1. In the window that appears, select **Connect**.
2. After the teaching pendant interface loads up, navigate to the settings page by clicking the burger icon in the top
   left corner of the screen.
3. Click on the **System** tab, then select the **Remote Control** tab.
4. Click **Enable** and then **Exit** at the bottom left of the screen.
5. Turn on the robot by pressing the red button located in the bottom left corner of the screen. Click **ON** followed
   by **START** to activate, then click **Exit**.

The simulator is now ready to interface with an AICA application.

<!-- The next section gives some background information on
why AICA suggests to enable Remote Control. -->

<div class="text--center">
  <img src={ursimActivate} alt="Activating the simulator" />
</div>

<!-- ## Local and Remote Control

The concept of Local and Remote Control on PolyScope can be easiest explained with the terms _master_ and _slave_. In
Local Control, the controller is the master and has full authority on loading and starting programs. In other words, the
robot has to be used in person through the teach pendant and any commands sent from an external source will be rejected.
On the other hand, Remote Control allows to control the robot via external sources, such as sockets, I/Os and the
Dashboard Server. In this case, the controller is the slave and external sources can load and start programs or directly
send URScript commands to the controller.

:::note

Safety features remain active in Remote Control.

:::

Choosing one of the two modes depends on the specific situation at hand. During a development phase, it might be
preferable to create the programs in Local Mode, whereas in a production setting, PLCs would responsible to load and
start the desired programs while the robot is in Remote Control. With the AICA System, users have the possiblity to get
the best of both modes:

- Take full control of the robot from an AICA application (requires Remote Control)
- Run an AICA application as one node of a program (works in both Local and Remote Control)

For the first case, no additional installation steps are required. TODO example here
The second case requires th External Control URCap to be installed. TODO link here to urcap then link to example. -->
