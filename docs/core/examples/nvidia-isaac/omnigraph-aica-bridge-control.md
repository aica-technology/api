---
sidebar_position: 2
title: Using Isaac Sim as a simulator
---

import sceneCreate from './assets/omnigraph-aica-bridge-scene-create.webm'; 
import graph from './assets/omnigraph-aica-bridge-graph-control.png'; 
import launcherConfig from'./assets/omnigraph-aica-bridge-launcher.png'; 
import addingGraph from './assets/omnigraph-aica-bridge-add-graph.webm'; 
import application from './assets/omnigraph-aica-bridge-control-application.png'; 
import integration from './assets/omnigraph-aica-bridge-control-integration.webm';

# Using Isaac Sim as a simulator

This guide walks you through the process of setting up **NVIDIA Isaac Sim** as a physics-based robotics simulator that
can be controlled from **AICA Studio** using **OmniGraph** and **ROS 2**. By the end of the tutorial, you will have a
fully functional simulation in Isaac Sim that receives joint commands from AICA Studio, simulates the robot’s motion
with realistic physics, and streams the resulting joint states back in real time, enabling a complete closed-loop
control workflow within a high-fidelity virtual environment.

**NVIDIA Isaac Sim** is a high-fidelity robotics simulator built on NVIDIA Omniverse, providing realistic physics and
RTX-based rendering. The **ROS 2 Bridge** extension allows Isaac Sim to publish and subscribe to ROS 2 topics, enabling
communication with external systems like AICA Studio. **OmniGraph** is a visual, node-based programming system in Isaac
Sim that lets you assemble data flows (called _Action Graphs_) by connecting pre-built nodes, including ROS 2 Bridge
nodes, without writing code. For a more detailed introduction to these concepts, see the companion guide
[Using Isaac Sim as a visualizer](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization).

In that companion guide, we set up a one-way connection where AICA Studio controls a robot (via a mock hardware
interface or real hardware) and Isaac Sim simply mirrors its motion. In that setup, Isaac Sim does not participate in
the control loop, it only visualizes the robot's state.

This guide covers a different direction: **Control a simulated robot in Isaac Sim from an AICA application.**. Here,
Isaac Sim hosts the robot with full physics simulation, and AICA Studio treats it as if it were real hardware. The data
flow is bidirectional:

- **AICA Studio -> Isaac Sim**: AICA Studio sends joint commands (positions, velocities, or efforts) to Isaac Sim via
  ROS 2.
- **Isaac Sim -> AICA Studio**: Isaac Sim simulates the robot's physical response and publishes the resulting joint
  states back to AICA Studio via ROS 2.

From AICA Studio's perspective, the simulated robot in Isaac Sim behaves like real hardware. This makes the setup well
suited for validating and debugging control algorithms in a physics-based environment before deploying them to a
physical robot.

:::tip

If you are looking for a way to simply mirror your robot's state in Isaac Sim for monitoring or demonstration purposes,
refer to [Using Isaac Sim as a visualizer](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization) instead.

:::

## Prerequisites

This guide builds on the same Isaac Sim and ROS 2 setup described in the
[visualizer guide](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization#prerequisites). Make sure you have:

- **Isaac Sim v5 or later** installed (see the
  [official installation instructions](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html))
- Isaac Sim launched with the **ROS 2 bridge enabled** and **ROS 2 Jazzy** selected as the ROS 2 distribution

If you have not done so already, follow the
[Prerequisites](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization#prerequisites) section of the visualizer
guide to install and launch Isaac Sim with the correct settings.

## Setting up a simple simulation environment

The scene setup is the same as in the
[visualizer guide](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization#setting-up-a-simple-simulation-environment).
Create a scene with a ground plane and the `Generic` robot model:

1. **Create a new scene**: In Isaac Sim, go to `File` > `New` to create a new scene with default lighting.

2. **Add a ground plane**: Go to `Create` > `Physics` > `Ground Plane` to add a flat ground surface.

3. **Add a robot**: Download the `Generic` robot USD model from
   [our repository](https://github.com/aica-technology/isaac-lab/tree/main/usd/robots/aica/generic) into a local
   directory. Then, in Isaac Sim, go to `Content` > `My Computer` in the bottom left part of the screen and navigate to
   the directory where you saved the `Generic` robot USD files. Drag and drop the `generic.usd` file into the scene.

Once done with these steps, your scene should look similar to the one below:

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={sceneCreate} type="video/webm" />
  </video>
</div>
<br/>

## Setting up the OmniGraph

With the scene ready, create an action graph that handles **bidirectional** communication between Isaac Sim and AICA
Studio. Unlike the visualizer setup (which only subscribes to joint states), this graph both **publishes** the robot's
current state and **subscribes** to incoming commands.

In Isaac Sim, go to `Create` > `Graphs` > `Action Graph` to create a new OmniGraph.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={addingGraph} type="video/webm" />
  </video>
</div>
<br/>

In the OmniGraph editor, add the following five nodes:

1. **ROS2 Context**: Initializes the ROS 2 context. Double-click the node to open its properties and set the `domain_id`
   to `30`. This must match the domain ID used by AICA Studio.

2. **On Playback Tick**: Triggers the graph execution on each simulation tick.

3. **ROS2 Publish Joint State**: Publishes the simulated robot's current joint state to a ROS 2 topic. Set the
   `topicName` to `/joint_states`. Select the `/world/Generic/root_joint` robot in the scene as the `targetPrim`.

4. **ROS2 Subscribe Joint State**: Subscribes to joint commands sent by AICA Studio. Set the `topicName` to
   `/joint_commands`.

5. **Articulation Controller**: Applies the received joint commands to the robot. Select the `/world/Generic` robot in
   the scene as the `targetPrim`.

Now that you have all the necessary nodes, you can connect them as follows:

- Connect the `Context` output of the `ROS2 Context` node to the `Context` input on both the `ROS2 Publish Joint State`
  and `ROS2 Subscribe Joint State` nodes.
- Connect the `Joint Names` output of the `ROS2 Subscribe Joint State` node to the `Joint Names` input of the
  `Articulation Controller` node.
- Connect the `Position Command` output of the `ROS2 Subscribe Joint State` node to the `Position Command` input of the
  `Articulation Controller` node.
- Connect the `Tick` output of the `On Playback Tick` node to the `Exec In` input of the `ROS2 Publish Joint State`
  node, the `ROS2 Subscribe Joint State` node, and the `Articulation Controller` node.

Your OmniGraph should look similar to the image below:

<div class="text--center">
  <img src={graph} style={{ height: "auto" }} alt="OmniGraph for AICA Bridge Control" />
</div>

:::info

Compared to the
[visualizer guide's OmniGraph](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization#setting-up-the-omnigraph),
this graph has one additional node: `ROS2 Publish Joint State` (to send state back to AICA).

:::

## Configuring the AICA Application

On the AICA side, the key difference from the visualizer setup is the **hardware interface**. Instead of using a mock
interface and a separate component to publish joint states, we use a **topic-based ROS 2 hardware interface** that
communicates directly with Isaac Sim over ROS 2 topics. This interface:

- **Subscribes** to the `/joint_states` topic to read the simulated robot's state from Isaac Sim
- **Publishes** to the `/joint_commands` topic to send commands to Isaac Sim

Use AICA Launcher to create a configuration that uses the latest AICA Studio version. Set the ROS 2 `Domain ID` to `30`
to match the one configured in Isaac Sim, and include the custom community package that provides the
`topic_based_ros2_control/TopicBasedSystem` hardware interface plugin

```
ghcr.io/aica-technology/topic-based-ros2-control:v0.1.0
```

Your launcher configuration should look similar to the image below:

<div class="text--center">
  <img src={launcherConfig} style={{ height: "70%", width: "70%" }} alt="AICA Launcher Configuration" />
</div>

### Creating a New Hardware with a Topic-Based ROS 2 Interface

First, create a new URDF in AICA Studio to communicate with Isaac Sim. This involves duplicating an existing hardware
and swapping out the plugin in the URDF.

1. In AICA Studio, go to the **Hardware** tab.
2. Click on the `Generic six-axis robot arm` to open it and use **Save As** to create a copy with a new name. Name it
   `Generic six-axis robot arm (Topic-Based Interface)`.
3. In the URDF editor, replace the content of the URDF with the following and click **Save**.

<details>
  <summary>Generic six-axis robot arm (Topic-Based Interface) URDF</summary>

    ```xml

  <?xml version="1.0" ?>
  <robot name="generic">
    <link name="world"/>
    <joint name="to_world" type="fixed">
      <parent link="world"/>
      <child link="base_link"/>
      <origin rpy="0 0 1.5708" xyz="0 0 0"/>
    </joint>
    <link name="base_link">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="package://aica_generic_description/meshes/generic/visual/base_link.dae"/>
        </geometry>
      </visual>
    </link>
    <joint name="joint_1" type="revolute">
      <origin rpy="0 0 0" xyz="0 0 0.154"/>
      <parent link="base_link"/>
      <child link="link_1"/>
      <axis xyz="0 0 1"/>
      <limit effort="87.0" lower="-6.28" upper="6.28" velocity="3.14"/>
    </joint>
    <link name="link_1">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="package://aica_generic_description/meshes/generic/visual/link_1.dae"/>
        </geometry>
      </visual>
    </link>
    <joint name="joint_2" type="revolute">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="link_1"/>
      <child link="link_2"/>
      <axis xyz="1 0 0"/>
      <limit effort="87.0" lower="-6.28" upper="6.28" velocity="3.14"/>
    </joint>
    <link name="link_2">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="package://aica_generic_description/meshes/generic/visual/link_2.dae"/>
        </geometry>
      </visual>
    </link>
    <joint name="joint_3" type="revolute">
      <origin rpy="0 0 0" xyz="0 0 0.436"/>
      <parent link="link_2"/>
      <child link="link_3"/>
      <axis xyz="1 0 0"/>
      <limit effort="87.0" lower="-6.28" upper="6.28" velocity="3.14"/>
    </joint>
    <link name="link_3">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="package://aica_generic_description/meshes/generic/visual/link_3.dae"/>
        </geometry>
      </visual>
    </link>
    <joint name="joint_4" type="revolute">
      <origin rpy="0 0 0" xyz="0.113 -0.406 0"/>
      <parent link="link_3"/>
      <child link="link_4"/>
      <axis xyz="1 0 0"/>
      <limit effort="12.0" lower="-6.28" upper="6.28" velocity="3.14"/>
    </joint>
    <link name="link_4">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="package://aica_generic_description/meshes/generic/visual/link_4.dae"/>
        </geometry>
      </visual>
    </link>
    <joint name="joint_5" type="revolute">
      <origin rpy="0 0 0" xyz="0 -0.099 0"/>
      <parent link="link_4"/>
      <child link="link_5"/>
      <axis xyz="0 -1 0"/>
      <limit effort="12.0" lower="-6.28" upper="6.28" velocity="3.14"/>
    </joint>
    <link name="link_5">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="package://aica_generic_description/meshes/generic/visual/link_5.dae"/>
        </geometry>
      </visual>
    </link>
    <joint name="joint_6" type="revolute">
      <origin rpy="0 0 0" xyz="0 0 -0.105"/>
      <parent link="link_5"/>
      <child link="link_6"/>
      <axis xyz="0 0 1"/>
      <limit effort="12.0" lower="-6.28" upper="6.28" velocity="3.14"/>
    </joint>
    <link name="link_6">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="package://aica_generic_description/meshes/generic/visual/link_6.dae"/>
        </geometry>
      </visual>
    </link>
    <joint name="link_6-tool0" type="fixed">
      <parent link="link_6"/>
      <child link="tool0"/>
      <origin rpy="3.1416 0 -1.5708" xyz="0 0 0"/>
    </joint>
    <link name="tool0"/>
    <ros2_control name="TopicBasedSystem" type="system">
      <hardware>
        <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
        <param name="joint_states_topic">/joint_states</param>
        <param name="joint_commands_topic">/joint_commands</param>
      </hardware>
      <joint name="joint_1">
        <command_interface name="position">
          <param name="min">-6.28</param>
          <param name="max">6.28</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-3.14</param>
          <param name="max">3.14</param>
        </command_interface>
        <command_interface name="effort">
          <param name="min">-87</param>
          <param name="max">87</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity" />
        <state_interface name="effort" />
      </joint>
      <joint name="joint_2">
        <command_interface name="position">
          <param name="min">-6.28</param>
          <param name="max">6.28</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-3.14</param>
          <param name="max">3.14</param>
        </command_interface>
        <command_interface name="effort">
          <param name="min">-87</param>
          <param name="max">87</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity" />
        <state_interface name="effort" />
      </joint>
      <joint name="joint_3">
        <command_interface name="position">
          <param name="min">-6.28</param>
          <param name="max">6.28</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-3.14</param>
          <param name="max">3.14</param>
        </command_interface>
        <command_interface name="effort">
          <param name="min">-87</param>
          <param name="max">87</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0</param>
        </state_interface>
        <state_interface name="velocity" />
        <state_interface name="effort" />
      </joint>
      <joint name="joint_4">
        <command_interface name="position">
          <param name="min">-6.28</param>
          <param name="max">6.28</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-3.14</param>
          <param name="max">3.14</param>
        </command_interface>
        <command_interface name="effort">
          <param name="min">-12</param>
          <param name="max">12</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity" />
        <state_interface name="effort" />
      </joint>
      <joint name="joint_5">
        <command_interface name="position">
          <param name="min">-6.28</param>
          <param name="max">6.28</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-3.14</param>
          <param name="max">3.14</param>
        </command_interface>
        <command_interface name="effort">
          <param name="min">-12</param>
          <param name="max">12</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity" />
        <state_interface name="effort" />
      </joint>
      <joint name="joint_6">
        <command_interface name="position">
          <param name="min">-6.28</param>
          <param name="max">6.28</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-3.14</param>
          <param name="max">3.14</param>
        </command_interface>
        <command_interface name="effort">
          <param name="min">-12</param>
          <param name="max">12</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity" />
        <state_interface name="effort" />
      </joint>
    </ros2_control>
  </robot>

    ```

</details>

In this URDF, we define a `ros2_control` hardware interface that uses the `topic_based_ros2_control/TopicBasedSystem`
plugin. This plugin is parameterized with the names of the ROS 2 topics to subscribe to for joint states and publish to
for joint commands as shown in the highlighted lines below:

```xml
  <ros2_control name="TopicBasedSystem" type="system">
    <hardware>
      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
      #highlight-next-line
      <param name="joint_states_topic">/joint_states</param>
      #highlight-next-line
      <param name="joint_commands_topic">/joint_commands</param>
    </hardware>
    ...
  </ros2_control>
```

In fact, this is the only modification required in the Generic six-axis robot arm URDF. By replacing the hardware plugin
`aica_core_interfaces/MockInterface` with `topic_based_ros2_control/TopicBasedSystem` and properly configuring the topic
names, the hardware interface is converted from a mock implementation, used solely for state simulation, into an
interface that communicates with Isaac Sim via ROS 2.

:::warning

The topic names configured in the URDF must match those set in the OmniGraph nodes in Isaac Sim. The hardware interface
subscribes to `/joint_states` (published by the `ROS2 Publish Joint State` node) and publishes to `/joint_commands`
(consumed by the `ROS2 Subscribe Joint State` node).

:::

### Creating the application

Copy the YAML content below into a new application in AICA Studio and save it. This application uses the Joint
Trajectory Controller to move the `Generic` robot between three waypoints.

<details>
  <summary>Joint Trajectory Control application</summary>

    ```yaml
    schema: 2-0-6
    dependencies:
      core: v5.1.0
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
        - hardware: hardware
    hardware:
      hardware:
        display_name: Hardware Interface
        urdf: Generic six-axis robot arm (Topic-Based Interface)
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
                    payload: '{"times_from_start": [5, 7, 8], "frames": [wp1, wp2, wp3]}'
    graph:
      positions:
        buttons:
          button:
            x: 360
            y: 780
        hardware:
          hardware:
            x: 1060
            y: 0
      buttons:
        button:
          display_name: Trigger Events Button
          on_click:
            call_service:
              controller: joint_trajectory_controller
              hardware: hardware
              service: set_trajectory
              payload: '{"times_from_start": [5, 7, 8], "frames": [wp1, wp2, wp3]}'
    ```

</details>

Notice that, unlike the visualizer guide's application, this one does **not** include a `JointSignalToJointStateMsg`
component. There is no need to convert and publish joint states, the topic-based hardware interface handles all
communication with Isaac Sim directly through the configured ROS 2 topics.

Your application should look similar to the image below:

<div class="text--center">
  <img src={application} style={{ height: "auto" }} alt="AICA Application for AICA Bridge Control" />
</div>

## Interfacing Isaac Sim with AICA Studio

With both Isaac Sim and AICA Studio configured, you can run the full simulation loop:

1. **Start the AICA application**: Press the `Start` button in AICA Studio. The application will begin sending joint
   commands to the `/joint_commands` topic and reading joint states from the `/joint_states` topic.

2. **Start Isaac Sim**: Press the `Play` button in Isaac Sim. The OmniGraph will begin executing: it subscribes to
   commands from AICA, applies them to the simulated robot, and publishes the resulting joint states back.

You should see the `Generic` robot in Isaac Sim moving between the three waypoints defined in the application. Unlike
the visualizer setup, the robot's motion is driven by actual physics simulation; AICA Studio is sending commands
directly to the simulated robot, and Isaac Sim is computing the physical response in real time rather than simply
mirroring state from a mock interface.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={integration} type="video/webm" />
  </video>
</div>

<br/>

:::tip

If the robot does not move or behaves unexpectedly, verify the following:

- The ROS 2 Domain ID is the same in both Isaac Sim (ROS2 Context node) and AICA Studio (Launcher configuration).
- The topic names match: `/joint_states` for state and `/joint_commands` for commands.
- The `Generic` robot is selected as the target in both the `ROS2 Publish Joint State` and `Articulation Controller`
  nodes.
- The hardware rate in AICA Studio matches the simulation tick rate in Isaac Sim.
:::
 
<br/>
