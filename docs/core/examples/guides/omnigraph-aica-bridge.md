---
sidebar_position: 11
title: Using Isaac Sim as a visualizer
---

import selector from './assets/omnigraph-aica-bridge-selector.png'; 
import sceneCreate from './assets/omnigraph-aica-bridge-scene-create.webm'; 
import graph from './assets/omnigraph-aica-bridge-graph.png'; 
import addingGraph from './assets/omnigraph-aica-bridge-add-graph.webm'; 
import runningAICA from './assets/omnigraph-aica-bridge-aica-app.webm';
import integration from './assets/omnigraph-aica-bridge-integration.webm';

# Using Isaac Sim as a visualizer

This guide walks you through the steps required to set up **NVIDIA Isaac Sim** so it can interface with **AICA Studio**
using **OmniGraph** and **ROS 2**. By the end of this tutorial, you’ll have a working simulation environment in Isaac
Sim that communicates with AICA Studio over ROS 2.

**NVIDIA Isaac Sim** is a high-fidelity robotics simulator built on NVIDIA Omniverse. It provides realistic physics,
multi-sensor support, and RTX-based rendering, making it ideal for developing, testing, and validating robot software
before deploying to real hardware.

To connect Isaac Sim with AICA Studio, we will use **ROS 2 Bridge**, an extension that lets Isaac Sim publish and
subscribe to ROS 2 topics and services. Through this bridge, we can exchange robot state, control commands, sensor data,
and more between the simulator and AICA Studio.

**OmniGraph** is a visual, node-based programming system integrated into Isaac Sim. It lets you assemble logic and data
flows, called _Action Graphs_, by connecting pre-built nodes. OmniGraph can include ROS 2 Bridge nodes, enabling
communication between Isaac Sim and ROS 2 without writing code.

This interface can be used in two main ways:

1. **Control a simulated robot in Isaac Sim from an AICA application:** In this mode, an application running in AICA Studio 
  controls a virtual robot hosted in Isaac Sim. The AICA application sends control inputs via ROS 2, and Isaac Sim feeds back 
  the robot state and sensor data (e.g., joint states, poses, perception). From the AICA application’s point of view, the
  simulated robot behaves like real hardware, making this setup well suited for validating and debugging control algorithms 
  before deploying them to a physical robot.

2. **Visualize a robot in Isaac Sim from an AICA application:** In this mode, an application running in AICA Studio controls
  a robot that is not in Isaac Sim (e.g., in a production environment, AICA's mock interface, URSim, ...). The robot’s state 
  (such as joint positions) is streamed to Isaac Sim, which mirrors the robot’s motion in a virtual scene. Isaac Sim 
  is used purely for visualization and does not participate in the control loop. This provides a live digital view of 
  the robot for monitoring, debugging, and demonstration purposes.

In this guide, we will focus on the second use case: **using Isaac Sim as a visualization tool for AICA Studio**. We will
set up a simple simulation environment in Isaac Sim with a robot model and create an OmniGraph that subscribes to joint
commands from an AICA application via ROS 2. 

## Prerequisites

Begin by installing Isaac Sim using the official
[installation instructions](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html). For AICA Core v5,
ensure you have Isaac Sim v5 or later.

Isaac Sim supports multiple installation options, including workstation installation, container-based and
cloud deployment. Choose the method that best fits your workflow. For this guide, we assume you have installed Isaac Sim
locally using the
[workstation installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html)
method.

After installation, launch Isaac Sim with the ROS 2 bridge enabled and set **ROS 2 Jazzy** as the active ROS
distribution. To do this, run the following command:

```bash
cd <path-to-isaac-sim>
./isaac-sim.selector.sh
```

<div class="text--center">
  <img src={selector} alt="NVIDIA Isaac Sim App Selector" />
</div>

This command opens the Isaac Sim launcher. In the launcher:

- Set ROS Bridge Extension to `isaacsim.ros2.bridge`
- Set Use Internal ROS2 Libraries to `jazzy`
- Click Start

## Setting up a simple simulation environment

Once Isaac Sim is running, you can create a simple simulation environment to test the AICA bridge. For this guide, we
will use a basic scene with a ground plane and a AICA's `Generic` robot model.

1. **Create a new scene**: In Isaac Sim, go to `File` > `New` to create a new scene. This will give an environment with
   a default lighting setup.

2. **Add a ground plane**: To add a ground plane to the scene, go to `Create` > `Physics` > `Ground Plane`. This will
   add a flat ground surface to the scene.

3. **Add a robot**: To add AICA's `Generic` robot to the scene, first download the Generic Robot USD model from
   [our repository](https://github.com/aica-technology/isaac-lab/tree/main/usd/robots/aica/generic) into a local
   directory. Then, in Isaac Sim, go to `Content` > `My Computer` in the bottom left part of the screen and navigate to
   the directory where you saved the Generic Robot USD files. Drag and drop the `generic.usd` file into the scene to add
   the robot.

Once down with these steps, your scene should look similar to the one below:

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={sceneCreate} type="video/webm" />
  </video>
</div>
<br/>

## Setting up the OmniGraph AICA Bridge


With the simulation environment set up, the next step is to add an action graph to your scene. This graph will handle
the communication between Isaac Sim and AICA Studio using ROS 2.

In Isaac Sim, go to `Create` > `Graphs` > `Action Graph` to create a new OmniGraph. This will open the OmniGraph editor
in a new tab in the bottom part of the screen.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={addingGraph} type="video/webm" />
  </video>
</div>
<br/>

In the OmniGraph editor, you can create nodes and connect them. The following nodes are required to set up the
communication between Isaac Sim and AICA Studio:

-1- **ROS2 Context**: This node initializes the ROS 2 context and allows and defines the ROS2 domain ID. In order to set
the domain ID, double click on the node to open its properties and set the `Domain ID` field to `30`. This domain ID
must match the one used by AICA Studio to ensure proper communication.

-2- **ROS2 Subscribe Joint State**: This node subscribes to the joint state topic published by AICA Studio. Set the
`Topic Name` field to `/joint_command` to match the topic used by AICA Studio for the Generic robot.

-3- **Articulation Controller**: This node is responsible for controlling the robot's joints based on the received
joint states. Select the Generic robot in the scene as the `Articulation` for this node.

-4- **On Playback Tick**: This node triggers the graph execution on each simulation tick.

Now that you have all the necessary nodes, you can connect them as follows:

- Connect the `Context` signal on `ROS 2 Context` node's output to the `Context` signal on the
  `ROS2 Subscribe Joint State` node's input.
- Connect the `Joint Names` output of the `ROS2 Subscribe Joint State` node to the `Joint Names` input of the
  `Articulation Controller` node.
- Connect the `Joint Positions` output of the `ROS2 Subscribe Joint State` node to the `Position Command` input of the
  `Articulation Controller` node.
- Connect the `Tick` output of the `On Playback Tick` node to the `Exec` input of the `ROS2 Subscribe Joint State` node
  and the `Exec` input of the `Articulation Controller` node.

Your OmniGraph should look similar to the image below:

<div class="text--center">
  <img src={graph} style={{ width: "70%", height: "auto" }} alt="OmniGraph for AICA Bridge Visualization" />
</div>

## Configuring the AICA Application

Using AICA Launcher, launch a configuration that uses the latest AICA Studio version and set the ROS 2 Domain ID to `30`
to match the one set in Isaac Sim. No extra packages are required for this guide.

A simple AICA application that moves a Generic arm using `Joint Trajectory Controller` can be created using the
following:

<details>
  <summary>Simple Joint Trajectory Control Application</summary>

    ```yaml
    schema: 2-0-6
    dependencies:
      core: v5.0.1
    frames:
      wp1:
        reference_frame: world
        position:
          x: 0.492159
          y: -0.020903
          z: 0.487698
        orientation:
          w: 0
          x: -0.707107
          y: 0.707107
          z: 0.000563
      wp2:
        reference_frame: world
        position:
          x: 0.492038
          y: 0.1335
          z: 0.336067
        orientation:
          w: 0.007248
          x: 0.70707
          y: -0.707064
          z: -0.007811
      wp3:
        reference_frame: world
        position:
          x: 0.491953
          y: 0.359307
          z: 0.229181
        orientation:
          w: 0
          x: -0.707107
          y: 0.707107
          z: 0.000563
    on_start:
      load:
        - component: joint_signal_to_joint_state_message
        - hardware: hardware
    components:
      joint_signal_to_joint_state_message:
        component: aica_core_components::ros::JointSignalToJointStateMsg
        display_name: Joint Signal To Joint State Message
        inputs:
          input: /hardware/robot_state_broadcaster/joint_state
        outputs:
          output: /joint_command
    hardware:
      hardware:
        display_name: Hardware Interface
        urdf: Generic six-axis robot arm
        rate: 100
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
            outputs:
              joint_state: /hardware/robot_state_broadcaster/joint_state
            events:
              transitions:
                on_load:
                  switch_controllers:
                    hardware: hardware
                    activate: robot_state_broadcaster
            display_name: Robot State Broadcaster
          joint_trajectory_controller:
            plugin: aica_core_controllers/trajectory/JointTrajectoryController
            events:
              transitions:
                on_load:
                  switch_controllers:
                    hardware: hardware
                    activate: joint_trajectory_controller
              predicates:
                has_trajectory_succeeded:
                  call_service:
                    controller: joint_trajectory_controller
                    hardware: hardware
                    service: set_trajectory
                    payload: '{"times_from_start": [1, 2, 5], "frames": ["wp1", "wp2", "wp3"]}'
    graph:
      positions:
        buttons:
          button:
            x: 340
            y: 800
        components:
          joint_signal_to_joint_state_message:
            x: 460
            y: 140
        hardware:
          hardware:
            x: 1060
            y: 20
      buttons:
        button:
          display_name: Trigger Events Button
          on_click:
            call_service:
              controller: joint_trajectory_controller
              hardware: hardware
              service: set_trajectory
              payload: '{"times_from_start": [1, 2, 5], "frames": ["wp1", "wp2", "wp3"]}'
      edges:
        on_start_on_start_joint_signal_to_joint_state_message_joint_signal_to_joint_state_message:
          path:
            - x: 280
              y: 60
            - x: 280
              y: 200
        on_start_on_start_hardware_hardware:
          path:
            - x: 540
              y: 60
            - x: 540
              y: 80
        hardware_hardware_robot_state_broadcaster_joint_state_joint_signal_to_joint_state_message_input:
          path:
            - x: 440
              y: 480
            - x: 440
              y: 360
        hardware_hardware_joint_trajectory_controller_has_trajectory_succeeded_hardware_hardware_joint_trajectory_controller_set_trajectory:
          path:
            - x: 960
              y: 780
            - x: 960
              y: 860
    ```
</details>


Copy the above YAML content into a **New Application** in AICA Studio and save it. This application will use the Generic
robot model and send joint commands to Isaac Sim via the `/joint_command` topic.

This component is key the bridge between AICA Studio and Isaac Sim as it converts the joint signals from AICA into ROS 2
JointState messages that Isaac Sim can consume:

```yaml
components:
  joint_signal_to_joint_state_message:
    component: aica_core_components::ros::JointSignalToJointStateMsg
    display_name: Joint Signal To Joint State Message
    inputs:
      input: /hardware/robot_state_broadcaster/joint_state
    outputs:
      output: /joint_command
```

Validate the application by pressing Play in AICA Studio. You will see the robot moving between 3 waypoints defined in
the application `wp1`, `wp2`, and `wp3`.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={runningAICA} type="video/webm" />
  </video>
</div>
<br/>

Additionally as can be seen in the AICA Studio, the joint commands are retrieved from the current robot state and that's
exactly why this approach is for visualization purposes only. The state of the robot in Isaac Sim is not fed back to
AICA Studio.

## Interfacing Isaac Sim with AICA Studio

Now that both Isaac Sim and AICA Studio are set up, you can run the simulation to see the communication in action.

1. **Start AICA Application**: First, ensure that your AICA application is running. You can do this by pressing the
   `Start` button in AICA Studio. This will start publishing joint commands to the `/joint_command` topic.

2. **Start Isaac Sim**: If all the steps above have been followed correctly, you just need to press the `Play` button in
   Isaac Sim to start the simulation. The OmniGraph will start executing, and the robot in the scene will begin
   receiving joint commands from AICA Studio.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={integration} type="video/webm" />
  </video>
</div>
<br/>
