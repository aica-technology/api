---
sidebar_position: 9
title: UR Hardware Interface
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

import urHWIExternalControl from './assets/ur-hwi-external-control.png'
import urHWISequenceGraph from './assets/ur-hwi-sequence-graph.png'
import urHWISequenceRunning from './assets/ur-hwi-sequence-running.gif'

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

TODO example here. Maybe describe the steps that need to be taken (activating remote in the settings, switching to
remote on the top right).

The second case requires the External Control URCap to be installed. This is the easiest way to get started and test the
integration with AICA functionality, as it allows users to keep their workflows intact, and only hand over control to an
AICA application in a controlled manner and whenever that is required. After the application, completes its task,
control may be hand over back to the main node.

TODO link here to urcap

TODO link to example. Screenshot of a proram with the external control node.

## Dashboard controller

TODO: Add dashboard controller example. Should we extend beyond the exchange of control, e.g. set a payload?

Dashboard controller allows interaction with UR's dashboard server to, among else, exchange control, set the payload,
and zero the force-torque sensor. Examples in this section describe how to set up and use this functionality.

### Exchange of control

TODO: Remove the title, since we describe both exchange and payload adjustement, and nothing else will be added probably.

A node running on the teaching pendant can be modified to hand over control to an AICA application that will perform a
task. Once this task is finished, the application will hand over control to the pendant, so that the rest of the program
runs. The following example shows how to achieve that:

1. In the teaching pendant, add the external control URCap and place it where the program should stop and hand over
   control. The command can be found under the URCaps menu on the left.

<div class="text--center">
  <img src={urHWIExternalControl} alt="External Control Node" />
</div>

2. Create a new AICA application, adding a hardware interface and selecting the appropriate UR manipulator.

3. In the settings of the manipulator, set the **Headless mode** to **false**.

4. Click on the **+** icon in the **Controllers** list, and select the **UR Dashboard Controller**.

5. Set up what the AICA application should be doing. For the purposes of this example, the paylod of the robot will be
   adjusted, using the same controller. Click on the **+** icon on the top right and add a new sequence.

6. Set the first step of the sequence to a delay of 2 seconds. Then, click on the **+** icon right next to the sequence
   block to add a second step, which should be an event. Connect it to the **Set payload** service of the Dashboard
   Controller, and click on the gear icon on top of the line to modify the arguments.

7. The service call should be formatted as a dictionary of the required values, the mass and the center of gravity. For
   example, it could look like the following:

```yaml
{ mass: 1.2, 
  cog: [0.15, 0.1, 0.05] 
}
```

8. Add another delay of 2 seconds, and finally another event, connected to the **Hand back control** service of the
   Dashboard Controller.

9. The sequence should start when the pendant hands over control. To achieve that, connect the **Programm running**
   predicate of the controller to the sequence block, setting the event to **Start**. The application graph should look
   like the following:

<div class="text--center">
  <img src={urHWISequenceGraph} alt="Sequence application graph" />
</div>

Click on **Play** to run the AICA application. Repeat the same at the teaching pendant's screen to run the program. The
robot moves through the positions and stops to hand over control to AICA Studio. The payload is adjusted, and control is
handed back to the pendant.

<div class="text--center">
  <img src={urHWISequenceRunning} alt="Exchange of control" />
</div>

## Hand-guiding controller

TODO: Add hand guiding controller example.

Maybe a good point to break into a second page since the impedance controller and force mode might have more things to
discuss.

## Impedance controller
