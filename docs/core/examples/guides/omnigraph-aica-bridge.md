---
sidebar_position: 11
title: Interface with Isaac Sim using OmniGraph and ROS 2
---

import selector from './assets/omnigraph-aica-bridge-selector.png'; import scene from
'./assets/omnigraph-aica-bridge-scene.png';

# Interface with Isaac Sim using OmniGraph and ROS 2

This guide will walk you through the steps required to set up **NVIDIA Isaac Sim** to interface with **AICA Core** using
**OmniGraph** and **ROS 2**. By the end of this guide, you will have a basic simulation environment in Isaac Sim that
communicates with AICA Core via ROS 2.

Isaac Sim is a powerful robotics simulation platform built on NVIDIA Omniverse, providing high-fidelity physics
simulation and photorealistic rendering. Isaac Sim can be extended using OmniGraph, a graph-based visual programming
system that allows users to create custom simulation behaviors and interactions without writing code. That Omnigraph
capability is leveraged in this guide to create a bridge between Isaac Sim and AICA Core using ROS 2.

This interface can be used in two distinct ways:

-1- **Control robots in Isaac Sim from AICA Core**: You can send commands from AICA Core to control the robot in Isaac
Sim and feedback the robot's state back from Isaac Sim to AICA Core. From AICA's perspective, the robot in Isaac Sim
behaves like a real robot. This is useful for testing and validating robot control algorithms in a simulated environment
before deploying them to real hardware.

-2- **Visualize Robots in Isaac Sim from AICA Core:** Isaac Sim can be used as a visualization tool for robots
controlled by AICA Core. In this setup, the robot inside Isaac Sim mirrors the real robot’s state and behavior. This
allows you to visualize production line operations or robot movements in a virtual environment, providing a real-time
view of what the robots are doing on the factory floor.

## Prerequisites

Begin by installing Isaac Sim following the official
[installation instructions](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html). AICA Core runs
on **ROS 2 Jazzy**, so make sure you install a version of Isaac Sim that supports this distribution (Isaac Sim **v5.0 or
later**).

Isaac Sim can be installed in several ways depending on your workflow, including workstation installation,
container-based deployment, or cloud deployment. Choose the installation method that best suits your environment. For
this guide, we assume that you have installed Isaac Sim locally using the
**[workstation installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html)**
method.

Once Isaac Sim is installed, launch it with the ROS 2 bridge enabled and specify **ROS 2 Jazzy** as the active ROS
distribution. You can do this by running the following command in your terminal:

```bash
cd <path-to-isaac-sim>
./isaac-sim.selector.sh
```

<div class="text--center">
  <img src={selector} alt="NVIDIA Isaac Sim App Selector" />
</div>

This will open the Isaac Sim launcher as shown in the image. In the launcher, select the `isaacsim.ros2.bridge` option
in `ROS Bridge Extension` to enable the ROS 2 bridge and choose `jazzy` as `Use Internal ROS2 Libraries`, then click
`Start`.

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

Once down with these steps, your scene should look similar to the image below:

<div class="text--center">
  <img src={scene} alt="Scene with Generic robot on a ground plane" />
</div>

## Setting up the OmniGraph AICA Bridge

### Isaac Sim as a visualization tool for AICA Core

With the simulation environment set up, the next step is to add an action graph to your scene. This graph will handle
the communication between Isaac Sim and AICA Core using ROS 2.

In Isaac Sim, go to `Create` > `Graphs` > `Action Graph` to create a new OmniGraph. This will open the OmniGraph editor
in a new tab in the bottom part of the screen.

<div class="text--center">
  <img alt="Video show casing adding an action graph" />
</div>

In the OmniGraph editor, you can create nodes and connect them. The following nodes are required to set up the
communication between Isaac Sim and AICA Core: -1- **ROS 2 Context**: This node initializes the ROS 2 context and allows
and defines the ROS2 domain ID. In order to set the domain ID, double click on the node to open its properties and set
the `Domain ID` field to `30`. This domain ID must match the one used by AICA Core to ensure proper communication.

-2- **ROS 2 Joint Subscriber**: This node subscribes to the joint state topic published by AICA Core. Set the
`Topic Name` field to `/joint_states` to match the topic used by AICA Core for the Generic robot.

-3- **Arcticulation Controller**: This node is responsible for controlling the robot's joints based on the received
joint states.

-4- **On Playback Tick**: This node triggers the graph execution on each simulation tick.

Now that you have all the necessary nodes, you can connect them as follows:

- Connect the `Context` signal on `ROS 2 Context` node's output to the `Context` signal on the `ROS 2 Joint Subscriber`
  node's input.
- Connect the `Joint Names` output of the `ROS 2 Joint Subscriber` node to the `Joint Names` input of the
  `Articulation Controller` node.
- Connect the `Joint Positions` output of the `ROS 2 Joint Subscriber` node to the `Target Positions` input of the
  `Articulation Controller` node.
- Connect the `Tick` output of the `On Playback Tick` node to the `Exec` input of the `ROS 2 Joint Subscriber` node and
  the `Exec` input of the `Articulation Controller` node.

Your OmniGraph should look similar to the image below:

<div class="text--center">
  <img alt="OmniGraph for AICA Bridge Visualization" />
</div>

### Isaac Sim as a robot controller from AICA Core

## Configuring the AICA Application

Using AICA Launcher, launch a configuration that uses the latest AICA Core version and set the ROS 2 Domain ID to `30`
to match the one set in Isaac Sim. No extra packages are required for this guide.

A simple AICA application that moves a Generic arm using `Joint Trajectory Controller` can be created using the
following YAML configuration:

<details>
  <summary>Simple Joint Trajectory Control Application</summary>

    ```yaml
    ```

</details>
