---
sidebar_position: 5
title: URSim Guide
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

import ursimActivate from './assets/ursim-activate.gif'
import ursimHardwareParams from './assets/ursimHarwareParams.png'


# URSim guide

This guide aims to support users with running URSim and connecting it to AICA Studio. URSim is a useful simulation
environment that provides access to a virtual robot. Users can utilize this environment to get familiar with the 
robot interface, as well as create and run their own programs. Please note that compared to the real robot, 
functionality that is not present includes:

- Emergency stop button
- Input IO state
- Self or surrounding object collisions
- Force mode

For more information, please refer to the [installation page](https://www.universal-robots.com/download/software-ur-series/simulator-linux/offline-simulator-ur-series-e-series-ur-sim-for-linux-5220/). 

<!-- :::note
URSim is made for Linux. For other operating systems, a virtual machine is needed. For more instructions, check the 
[installation page](https://www.universal-robots.com/download/software-ur-series/simulator-non-linux/offline-simulator-ur-series-e-series-ur-sim-for-non-linux-5220/). 
::: -->


## Installation

Executing the following instructions runs URSim in a Docker container.

1. Clone the `simulators` repository provided by AICA. 

```bash
git clone https://github.com/aica-technology/simulators.git
```


TODO: must have Docker installed. Should we add something about that? Probably people reading t
his have already done that. 

2. Navigate to the `ursim` folder inside this repository. 

3. Run the `run.sh` script.

```bash
./run.sh
```

This command installs and runs the URSim Docker container, simulating the UR5e robot by default. In case users 
want to simulate a different robot model, the `-m` flag can be used. For example, to simulate the UR10e, 
run the following command:

```bash
./run.sh -m ur10e
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
2. After the teaching pendant interface loads up, navigate to the settings page by clicking the burger icon 
in the top left corner of the screen.
3. Click on the **System** tab, then select the **Remote Control** tab.
4. Click **Enable** and then **Exit** at the bottom left of the screen.
5. A touchpad icon labeled **Local** should appear in the top right corner of the simulator. Click on this 
icon and then select **Remote Control**.
6. Turn on the robot by pressing the red button located in the bottom left corner of the screen. Click **ON** 
followed by **START** to activate, then click **Exit**.

<div class="text--center">
  <img src={ursimActivate} alt="Activating the simulator" />
</div>

7. The simulator is now ready to interface with an AICA application.

:::tip
Setting the robot to **Remote Control** mode is not strictly required for the simulator.
It is possible to switch to **Local Control**, where the **Installation** tab can be used to visualize 
the robot's motions. But always keep in mind that Remote Control is always required on the real robot. 
:::

## Connecting to AICA Studio

In AICA Studio, open the Hardware section and select the same UR description as the simulated robot. Ensure 
that the IP address in the **Robot Ip** field matches the IP address displayed in the terminal. The 
application can now be run with the simulated robot being used instead of the real one. Also make sure 
the **Headless Mode** is set to **true**.

TODO: Some more details on headless mode? 

<div class="text--center">
  <img src={ursimHardwareParams} alt="Setting hardware parameters" />
</div>