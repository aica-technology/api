---
sidebar_position: 2
title: Using Isaac Sim as a simulator
---

# Using Isaac Sim as a simulator

This guide walks you through the steps required to set up **NVIDIA Isaac Sim** as a physics-based simulator that can be
controlled from **AICA Studio** using **OmniGraph** and **ROS 2**. By the end of this tutorial, you'll have a working
simulation in Isaac Sim that receives joint commands from AICA Studio, simulates the robot's response with full physics,
and streams the resulting joint states back.

In the companion guide [Using Isaac Sim as a visualizer](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization), we
set up a one-way connection where AICA Studio controls a robot (via a mock hardware interface or real hardware) and
Isaac Sim simply mirrors its motion. In that setup, Isaac Sim does not participate in the control loop, it only
visualizes the robot's state.

This guide covers the opposite direction: **using Isaac Sim as the simulated hardware**. Here, Isaac Sim hosts the robot
with full physics simulation, and AICA Studio treats it as if it were real hardware. The data flow is bidirectional:

- **AICA Studio - Isaac Sim**: AICA Studio sends joint commands (positions, velocities, or efforts) to Isaac
  Sim via ROS 2.
- **Isaac Sim - AICA Studio**: Isaac Sim simulates the robot's physical response and publishes the resulting joint
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
[Prerequisites](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization#prerequisites) section of the visualizer guide
to install and launch Isaac Sim with the correct settings.

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

## Setting up the OmniGraph

With the scene ready, create an action graph that handles **bidirectional** communication between Isaac Sim and AICA
Studio. Unlike the visualizer setup (which only subscribes to joint states), this graph both **publishes** the robot's
current state and **subscribes** to incoming commands.

In Isaac Sim, go to `Create` > `Graphs` > `Action Graph` to create a new OmniGraph.

In the OmniGraph editor, add the following six nodes:

1. **ROS2 Context**: Initializes the ROS 2 context. Double-click the node to open its properties and set the `Domain ID`
   to `30`. This must match the domain ID used by AICA Studio.

2. **On Playback Tick**: Triggers the graph execution on each simulation tick.

3. **ROS2 Publish Joint State**: Publishes the simulated robot's current joint state to a ROS 2 topic. Set the
   `Topic Name` to `/joint_states`. Select the `Generic` robot in the scene as the `targetPrim`.

4. **ROS2 Subscribe Joint State**: Subscribes to joint commands sent by AICA Studio. Set the `Topic Name` to
   `/joint_command`.

5. **Articulation Controller**: Applies the received joint commands to the robot. Select the `Generic` robot in the
   scene as the target articulation.

Now that you have all the necessary nodes, you can connect them as follows:

- Connect the `Context` output of the `ROS2 Context` node to the `Context` input on both the
  `ROS2 Publish Joint State` and `ROS2 Subscribe Joint State` nodes.
- Connect the `Simulation Time` output of the `Isaac Read Simulation Time` node to the `Timestamp` input of the
  `ROS2 Publish Joint State` node.
- Connect the `Joint Names` output of the `ROS2 Subscribe Joint State` node to the `Joint Names` input of the
  `Articulation Controller` node.
- Connect the `Position Command` output of the `ROS2 Subscribe Joint State` node to the `Position Command` input of
  the `Articulation Controller` node.
- Connect the `Tick` output of the `On Playback Tick` node to the `Exec In` input of the
  `ROS2 Publish Joint State` node, the `ROS2 Subscribe Joint State` node, and the `Articulation Controller` node.

:::info

Compared to the
[visualizer guide's OmniGraph](/core/examples/nvidia-isaac/omnigraph-aica-bridge-visualization#setting-up-the-omnigraph), this
graph has one additional node: `ROS2 Publish Joint State` (to send state back to AICA).

:::

## Configuring the AICA Application

On the AICA side, the key difference from the visualizer setup is the **hardware interface**. Instead of using a mock
interface and a separate component to publish joint states, we use a **topic-based ROS 2 hardware interface** that
communicates directly with Isaac Sim over ROS 2 topics. This interface:

- **Subscribes** to the `/joint_states` topic to read the simulated robot's state from Isaac Sim
- **Publishes** to the `/joint_command` topic to send commands to Isaac Sim

Using AICA Launcher, launch a configuration that uses the latest AICA Studio version and set the ROS 2 Domain ID to `30`
to match the one set in Isaac Sim. Make sure that your configuration includes the following custom community package, which
provides the `topic_based_ros2_control/TopicBasedSystem` hardware interface plugin:

```
ghcr.io/aica-technology/topic-based-ros2-control:v0.1.0
```

### Creating a topic-based hardware interface

First, configure the hardware interface in AICA Studio to communicate with Isaac Sim. This involves duplicating an
existing hardware and swapping out the plugin in the URDF.

1. In AICA Studio, go to the **Hardware** tab.
2. Click on the `Generic six-axis robot arm` to open it and use **Save As** to create a copy with a new name. For
   example, name it `Generic six-axis robot arm (Topic-Based Interface)`.
3. In the URDF editor, locate the `<ros2_control>` section and replace the `<hardware>` tag with the following:

```xml
<hardware>
  <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
  <param name="joint_states_topic">/joint_states</param>
  <param name="joint_commands_topic">/joint_command</param>
</hardware>
```

4. Save your changes.

:::warning

The topic names configured in the URDF must match those set in the OmniGraph nodes in Isaac Sim. The hardware interface
subscribes to `/joint_states` (published by the `ROS2 Publish Joint State` node) and publishes to `/joint_command`
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
component. There is no need to manually convert and publish joint states, the topic-based hardware interface handles
all communication with Isaac Sim directly through the configured ROS 2 topics.

## Interfacing Isaac Sim with AICA Studio

With both Isaac Sim and AICA Studio configured, you can run the full simulation loop:

1. **Start the AICA application**: Press the `Start` button in AICA Studio. The application will begin sending joint
   commands to the `/joint_command` topic and reading joint states from the `/joint_states` topic.

2. **Start Isaac Sim**: Press the `Play` button in Isaac Sim. The OmniGraph will begin executing: it subscribes to
   commands from AICA, applies them to the simulated robot, and publishes the resulting joint states back.

You should see the `Generic` robot in Isaac Sim moving between the three waypoints defined in the application. Unlike
the visualizer setup, the robot's motion is driven by actual physics simulation; AICA Studio is sending commands
directly to the simulated robot, and Isaac Sim is computing the physical response in real time rather than simply
mirroring state from a mock interface.


:::tip

If the robot does not move or behaves unexpectedly, verify the following:

- The ROS 2 Domain ID is set to `30` in both Isaac Sim (ROS2 Context node) and AICA Studio (Launcher configuration).
- The topic names match: `/joint_states` for state and `/joint_command` for commands.
- The `Generic` robot is selected as the target in both the `ROS2 Publish Joint State` and `Articulation Controller`
  nodes.
- The hardware rate in AICA Studio matches the simulation tick rate in Isaac Sim.

:::
