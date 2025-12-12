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
this guide, we assume that you have installed Isaac Sim locally using the **workstation installation** method.

Once Isaac Sim is installed, launch it with the ROS 2 bridge enabled and specify **ROS 2 Jazzy** as the active ROS
distribution. You can do this by running the following command in your terminal:

```bash
cd <path-to-isaac-sim>
./isaac-sim.selector.sh
```

<div class="text--center">
  <img src={selector} alt="NVIDIA Isaac Sim App Selector" />
</div>

This will open the Isaac Sim launcher as shown in the image. In the launcher, select the option in
`ROS Bridge Extension` to enable the ROS 2 bridge and choose `Jazzy` as `Use Internal ROS2 Libraries`, then click
`Start`.

## Setting up a simple simulation environment

Once Isaac Sim is running, you can create a simple simulation environment to test the AICA bridge. For this guide, we
will use a basic scene with a ground plane and a AICA's `Generic` robot model.

1. **Create a new scene**: In Isaac Sim, go to `File` > `New` to create a new scene. This will give an environment with
   a default lighting setup.

2. **Add a ground plane**: To add a ground plane to the scene, go to `Create` > `Physics` > `Ground Plane`. This will
   add a flat ground surface to the scene.

3. **Add a robot**: To add AICA's `Generic` robot to the scene, first download the Generic Robot USD model from
   [our repository](https://github.com/aica-technology/isaac-lab/tree/main/usd/robots/universal_robots/ur5e) into a
   local directory. Then, in Isaac Sim, go to `Content` > `My Computer` in the bottom left part of the screen and
   navigate to the directory where you saved the UR5e USD file. Drag and drop the `generic.usd` file into the
   scene to add the robot.

Once down with these steps, your scene should look similar to the image below:

<div class="text--center">
  <img src={scene} alt="Scene with UR5e robot on a ground plane" />
</div>

## Setting up the OmniGraph AICA Bridge

With the simulation environment set up, the next step is to add an action graph to your scene. This graph will handle
the communication between Isaac Sim and AICA Core using ROS 2.

In Isaac Sim, go to `Create` > `Graphs` > `Action Graph` to create a new OmniGraph. This will open the OmniGraph editor in a
new tab in the bottom part of the screen.

