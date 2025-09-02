---
sidebar_position: 9
title: Universal Robots
---

import urHwiStudio from './assets/ur-hwi-studio.png'
import urHwiSwitchMode from './assets/ur-hwi-switch-mode.gif'
import urHwiHeadlessMode from './assets/ur-hwi-headless-mode.png'
import urHwiRemoteExample from './assets/ur-hwi-remote-example.gif'
import urHWIExternalControl from './assets/ur-hwi-external-control.png'
import urHWISequenceGraph from './assets/ur-hwi-sequence-graph.png'
import urHWISequenceRunning from './assets/ur-hwi-sequence-running.gif'
import urHWIHandGuidingGraph from './assets/ur-hwi-hand-guiding-graph.png'
import urHWIHandGuidingParams from './assets/ur-hwi-hand-guiding-params.png'
import urHWINetworkingSettings from './assets/ur-hwi-networking-settings.png'
import urHWIURProgram from './assets/ur-hwi-ur-program.png'
import urHWIImpedanceController from './assets/ur-hwi-impedance-controller.png'

# Universal Robots

Universal Robots (UR) offers a range of collaborative robotic arms in different sizes and with different payloads that
are widely adopted across industries and research for their accessibility and flexibility. A graphical interface on the
teach pendant allows to easily and intuitively program and integrate UR robots. At the same time, skilled users can get
access to the full capabilities of the manipulators through scripting in URScript language or even by developing
software add-ons, so-called URCaps.

The ecosystem around UR robots is highly developer-friendly, with open-source communication libraries, drivers,
documentation, and integration support for frameworks such as ROS. Additionally, their simulation tool URSim allows
developers to test and validate robot programs and interfaces without needing access to the physical hardware and makes
UR a popular choice building custom applications.

:::note

A guide on installing and running URSim can be found on [this page](./ur-sim-guide.md).

:::

Due to the reasons mentioned above, UR manipulators are often used for prototyping at AICA and have seen extensive
internal development for robot-specific feature integration. The UR hardware collection provided by AICA comes with
special tools and functionalities that are unique to UR robots. This guide intends to explain these concepts and how
they can be leveraged in AICA Studio.

To use the UR collection, add the latest version of `collections/ur-collection` to your configuration in AICA Launcher.
Doing this will add multiple new hardware examples as well as a few controllers to AICA Studio. All UR robots share the
same hardware interface in AICA Studio. The hardware interface has quite a large number of parameters, most of which are
not important for regular use cases.

:::tip

For best results with a UR robot, always set the rate of the hardware interface to 500 Hertz, which corresponds to the
control rate of the real hardware.

:::

<div class="text--center">
  <img src={urHwiStudio} alt="UR Hardware Interface in AICA Studio" />
</div>

<!-- TO ADD, TBD:

- use of the dashboard controller with local mode and the program node to hand back control between AICA and UR
- use of the UR impedance controller and hand guiding controller to leverage UR Force mode from AICA Studio
- use of the dashboard server to observe GPIOs, set payload, zero ft sensor etc -->

<!-- # Hardware Interface

TBD: There is already a [page](../../concepts/building-blocks/hardware-interfaces.md) on hardware interfaces. Are these
two conflicting? Should we link to this one?

Within the context of AICA Studio, but also in ROS, hardware interfaces are used as middleware between controllers and
actual hardware. In other words, they are tasked with two-way communication of commands and state feedback between the
robot and the controller that usually lies above. -->

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

Choosing one of the two modes depends on the specific task at hand. During a development phase, it might be preferable
to create the programs in Local Mode, whereas in a production setting, PLCs would be responsible to load and start the
desired programs while the robot is in Remote Control. With the AICA System, users have the possibility to get the best
of both modes:

1. Take full control of the robot from an AICA application (requires Remote Control)
2. Run an AICA application as one node of a program (works in both Local and Remote Control)

### Full control of the robot from an AICA application

For this first, simpler case, no additional installation steps are required. Apart from setting the correct robot IP,
two requirements have to be met:

- On the robot, Remote Control has to be activated. For that, first activate Remote Control in the system settings as
  explained [here](./ur-sim-guide.md#accessing-and-configuring-the-simulated-robot), then switch from _Local_ to
  _Remote_ mode on the top right corner of the teach pendant. The interface automatically switches to the _Run_ tab and
  disables other tabs, indicating that control has been handed over to external sources.

  <div class="text--center">
    <img src={urHwiSwitchMode} alt="Switch mode from Local to Remote" />
  </div>

- In AICA Studio, make sure that the parameter `Headless Mode` that can be found under the hardware interface parameters
  is set to `True`. This will notify the hardware interface that it will be running headless, i.e. it is in charge of
  providing the full UR program to the robot controller.

  <div class="text--center">
    <img src={urHwiHeadlessMode} alt="Headless Mode" />
  </div>

Finally, implement an application of your choice in AICA Studio. An example with a joint trajectory controller is given
below. Observe how the robot program status goes from _Stopped_ to _Running_ as soon as the hardware interface connects
to the robot.

<div class="text--center">
  <img src={urHwiRemoteExample} alt="Example with Remote Control" />
</div>


<details>
  <summary>Example application</summary>

  ```yaml
  schema: 2-0-4
  dependencies:
    core: v4.4.2
  frames:
    wp_1:
      reference_frame: world
      position:
        x: -0.027943
        y: 0.600701
        z: 0.202217
      orientation:
        w: 0.171776
        x: 0.985056
        y: 0.002386
        z: -0.012313
    wp_2:
      reference_frame: world
      position:
        x: 0.260809
        y: 0.604927
        z: 0.194871
      orientation:
        w: 0.132343
        x: 0.95897
        y: -0.030587
        z: -0.248852
    wp_3:
      reference_frame: world
      position:
        x: 0.147083
        y: 0.552997
        z: 0.328354
      orientation:
        w: 0.012478
        x: 0.999843
        y: 0.000392
        z: -0.012536
  on_start:
    load:
      hardware: hardware
  sequences:
    sequence:
      display_name: Sequence
      steps:
        - delay: 2
        - call_service:
            controller: joint_trajectory_controller
            hardware: hardware
            service: set_trajectory
            payload: "{frames: [wp_1, wp_2, wp_3], durations: [1.0, 1.0, 1.0],
              blending_factors: [1.0]}"
  hardware:
    hardware:
      display_name: Hardware Interface
      urdf: Universal Robots 5e
      rate: 500
      events:
        transitions:
          on_load:
            load:
              - controller: robot_state_broadcaster
                hardware: hardware
              - controller: joint_trajectory_controller
                hardware: hardware
      controllers:
        robot_state_broadcaster:
          plugin: aica_core_controllers/RobotStateBroadcaster
          events:
            transitions:
              on_load:
                switch_controllers:
                  hardware: hardware
                  activate: robot_state_broadcaster
        joint_trajectory_controller:
          plugin: aica_core_controllers/trajectory/JointTrajectoryController
          events:
            predicates:
              has_trajectory_succeeded:
                application: stop
            transitions:
              on_load:
                switch_controllers:
                  hardware: hardware
                  activate: joint_trajectory_controller
  graph:
    positions:
      buttons:
        button:
          x: -460
          y: 600
      hardware:
        hardware:
          x: 620
          y: -20
      sequences:
        sequence:
          x: 40
          y: 560
    buttons:
      button:
        on_click:
          sequence:
            start: sequence
    edges:
      sequence_sequence_event_trigger_2_hardware_hardware_joint_trajectory_controller_set_trajectory:
        path:
          - x: 420
            y: 1060
          - x: 620
            y: 1060
          - x: 620
            y: 900
      sequence_sequence_event_trigger_1_hardware_hardware_joint_trajectory_controller_set_trajectory:
        path:
          - x: 240
            y: 860
      hardware_hardware_joint_trajectory_controller_has_trajectory_succeeded_on_stop_on_stop:
        path:
          - x: 540
            y: 780
          - x: 540
            y: 480
          - x: -20
            y: 480
          - x: -20
            y: 140

  ```
</details>

### Run an AICA application as one node of a program

This second case requires the [External Control URCap](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy/ur_robot_driver/resources) to be installed. This is the easiest way to get started and test the
integration with AICA functionality, as it allows users to keep their workflows intact, and only hand over control to an
AICA application in a controlled manner and whenever that is required. After the application completes its task,
control may be hand over back to the main node. To do that, follow the next steps:

1. The external control URCap needs to be configured to the right remote control address. Navigate to the **Installation** tab
and set the address to the one of the device that will be running the AICA application.

<div class="text--center">
  <img src={urHWINetworkingSettings} alt="External control URCap networking settings" />
</div>

2. The robot should remain in local mode, and the external control node placed within the program in the desired location.

<div class="text--center">
  <img src={urHWIURProgram} alt="UR Program with external control URCap" style={{ width: '40%' }} />
</div>

3. Headless mode in the hardware interface should be set to false, and no controller should be activated when the program starts. 
**(other than the dashboard controller, right?)**

4. The dashboard controller should be used to take and hand back control, check the next section.

5. The AICA application should be started first, the the robot program run. 

Below there is an example of the exact same application, that can be run as a program node. The next section about the dashboard 
controller explains in detail how this works.

<details>
  <summary>Example application, local mode</summary>

  ```yaml
schema: 2-0-4
dependencies:
  core: v4.4.2
frames:
  wp_1:
    reference_frame: world
    position:
      x: -0.027943
      y: 0.600701
      z: 0.202217
    orientation:
      w: 0.171776
      x: 0.985056
      y: 0.002386
      z: -0.012313
  wp_2:
    reference_frame: world
    position:
      x: 0.260809
      y: 0.604927
      z: 0.194871
    orientation:
      w: 0.132343
      x: 0.95897
      y: -0.030587
      z: -0.248852
  wp_3:
    reference_frame: world
    position:
      x: 0.147083
      y: 0.552997
      z: 0.328354
    orientation:
      w: 0.012478
      x: 0.999843
      y: 0.000392
      z: -0.012536
on_start:
  load:
    hardware: hardware
sequences:
  sequence:
    display_name: Sequence
    steps:
      - delay: 2
      - call_service:
          controller: joint_trajectory_controller
          hardware: hardware
          service: set_trajectory
          payload: "{frames: [wp_1, wp_2, wp_3], durations: [1.0, 1.0, 1.0],
            blending_factors: [1.0]}"
hardware:
  hardware:
    display_name: Hardware Interface
    urdf: Universal Robots 5e
    rate: 500
    events:
      transitions:
        on_load:
          load:
            - controller: robot_state_broadcaster
              hardware: hardware
            - controller: joint_trajectory_controller
              hardware: hardware
            - controller: ur_dashboard_controller
              hardware: hardware
    parameters:
      headless_mode: "false"
    controllers:
      robot_state_broadcaster:
        plugin: aica_core_controllers/RobotStateBroadcaster
        events:
          transitions:
            on_load:
              switch_controllers:
                hardware: hardware
                activate: robot_state_broadcaster
      joint_trajectory_controller:
        plugin: aica_core_controllers/trajectory/JointTrajectoryController
        events:
          predicates:
            has_trajectory_succeeded:
              call_service:
                controller: ur_dashboard_controller
                hardware: hardware
                service: hand_back_control
          transitions:
            on_activate:
              sequence:
                start: sequence
      ur_dashboard_controller:
        plugin: aica_ur_controllers/URDashboardController
        events:
          predicates:
            program_running:
              switch_controllers:
                hardware: hardware
                activate: joint_trajectory_controller
            hand_back_control_success:
              application: stop
          transitions:
            on_load:
              switch_controllers:
                hardware: hardware
                activate: ur_dashboard_controller
graph:
  positions:
    hardware:
      hardware:
        x: 780
        y: 0
    sequences:
      sequence:
        x: 80
        y: 380
  edges:
    sequence_sequence_event_trigger_2_hardware_hardware_joint_trajectory_controller_set_trajectory:
      path:
        - x: 420
          y: 1060
        - x: 620
          y: 1060
        - x: 620
          y: 900
    on_start_on_start_hardware_hardware:
      path:
        - x: 440
          y: 40
        - x: 440
          y: 60
    hardware_hardware_joint_trajectory_controller_on_activate_sequence_sequence:
      path:
        - x: 20
          y: 760
        - x: 20
          y: 440
    sequence_sequence_event_trigger_1_hardware_hardware_joint_trajectory_controller_set_trajectory:
      path:
        - x: 280
          y: 920
    hardware_hardware_ur_dashboard_controller_program_running_hardware_hardware_joint_trajectory_controller:
      path:
        - x: 460
          y: 1300
        - x: 460
          y: 640
    hardware_hardware_ur_dashboard_controller_hand_back_control_success_on_stop_on_stop:
      path:
        - x: -20
          y: 1260
        - x: -20
          y: 140
    hardware_hardware_joint_trajectory_controller_has_trajectory_succeeded_hardware_hardware_ur_dashboard_controller_hand_back_control:
      path:
        - x: 680
          y: 840
        - x: 680
          y: 1380
  ```
</details>

## Dashboard controller

Dashboard controller allows interaction with UR's dashboard server to, among else, exchange control, set the payload,
and zero the force-torque sensor. Examples in this section describe how to set up and use this functionality.

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
{ 
  mass: 1.2, 
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

## Hand Guiding controller

It is quite common that users need to manually adjust the position of the manipulator, either for practical -move to a
part approach location and teach it to the robot- or safety reasons. While in Local mode, this can be achieved through
the Freedrive function of the teach pendant, which is, however, in some senses limited. It demands users to press on the
button behind the pendant and then adjust the configuration almost joint by joint, with the feedback being quite light
and unresponsive at times.

For that purpose, AICA offers a hand guiding controller as a part of the UR hardware collection, based on UR's native
force mode, and enriched with additional functionality such as spatial limits. To use it, simply click on the **+** icon
in the **Controllers** list, and select the **UR Hand Guiding Controller**.

<div class="text--center">
  <img src={urHWIHandGuidingGraph} alt="Hand guiding controller graph" />
</div>

Since the controller uses force mode, it needs the name and the reference frame of the force/torque (FT) sensor. In the
settings of the controller set the name of the sensor as **ur_tcp_fts_sensor** and the frame as **ur_tool0**.

:::note

Attempts to guide the robot by pushing on individual links will fail, as the forces must act on the FT sensor on the end
effector.

:::

After pressing **Play**, the manipulator can be hand guided to points in space, driven by the forces applied on its end
effector. In other words, it reads forces in the FT sensor, and "admits" them, trying to set the measured force to zero.
The controller can be further tuned and adjusted by using its parameters:

- Velocity/Force limits: the velocities and forces that can be applied by the controller in force mode (X, Y, Z, RZ, RY,
  RZ).
- Force/Torque threshold: the thresholds above which the hand guiding behavior is activated.
- Compliant axes selection: the axes along which the robot can be hand guided (1-enabled, 0-disabled).
- Hold delay: forces below the thresholds above for this duration will disable hand guiding.
- X/Y/Z limits: spatial boundaries for the end effector motion, vectors of two values for the lower and upper limit.
- Reference orientation: quaternion representing a desired orientation in base frame
- Angular limit: allowed deviation from the reference orientation
- Linear/Angular boundary strength: gains to apply restitution forces/torques, in case linear or angular limits are
  exceeded.

<div class="text--center">
  <img src={urHWIHandGuidingParams} alt="Hand guiding controller parameters" style={{ width: '40%' }} />
</div>

## Impedance controller

AICA's UR impedance controller is tailored to take advantage of UR's force mode to drive the robot in a compliant manner, 
enabling safe, adaptive interaction with the environment.

The controller takes a desired Cartesian state as input. Then, it computes the error between input and current state. Using the defined
stiffness and damping parameters, it applies a desired wrench in space. In cases of obstacle-free space this will result in uninterrupted motion,
while the robot will react compliantly to disturbances and forces applied on the end effector (where the force sensor lies).   

:::note

For the controller to operate, the names of the sensor and the reference frame need to be defined similarly to previous sections. 

:::

<div class="text--center">
  <img src={urHWIImpedanceController} alt="UR Impedance controller parameters" style={{ width: '40%' }} />
</div>