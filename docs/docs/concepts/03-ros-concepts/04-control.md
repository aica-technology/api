---
sidebar_position: 4
title: Controlling robots
---

# Controlling robots with ros2_control

ROS nodes are easy to conceptualize as pure functional components that process input topics and generate output
topics. However, the end goal in robotics is often not just to perform some computation, but to control real physical
hardware such as a robot arm.

Interfacing with hardware peripherals imposes additional challenges, including:

- networking and communication beyond the ROS RMW publisher / subscriber mechanism
- hardware-specific control interfaces and requirements
- safety and reliability around controller and hardware limits
- potential real-time requirements for deterministic control performance

The `ros2_control` project defines an open standard for writing controllers and hardware interfaces with the aim of
simplifying the integration of new and existing robot hardware within a ROS-based applications through hardware
abstraction. It specifies the following key concepts.

## Controller Manager

The idea behind hardware abstraction is to hide the additional complexity of hardware-specific software behind a
standardized API. The Controller Manager is a process that handles hardware interfaces and controllers and passes
messages between them in a standard format. In brief, the Controller Manager allows controllers in the ROS 2 graph to
read the **state** of a robot and to write a **command** to the robot in a real-time loop.

### State interfaces

State interfaces provide the current state of the robot. This is most commonly defined as the robot joint positions,
and may also include joint velocities, torques or other available sensor data.

### Command interfaces

Command interfaces represent the desired control action that a robot should execute. The nature of a command interface
depends on the robot actuators, but commonly includes joint positions, velocities or torques. For real-time control,
velocity and torque are the preferred command modes. For other actuators such as robot grippers, the command interface
might instead specify a gripper distance or force.

## Hardware interfaces

Communication protocols and message standards vary between robot brands. Hardware interfaces, also known as hardware
components or hardware plugins, are software packages that implement robot-specific communication drivers. They are
responsible for reading data from the robot and translating them into the standard state interface format for the
Controller Manager. They are also responsible for translating command interfaces from the Controller Manager into real
robot control actions.

## Controllers

As might be expected, controllers are responsible for calculating and sending desired control actions to the Controller
Manager via the standardized command interfaces. They also have access to state interfaces from the Controller Manager
to use as feedback in closed-loop control.

Controllers are otherwise similar to ROS 2 lifecycle components; they have parameters, can publish or subscribe to
messages on the ROS 2 network to interact with other processes in the application, and can be dynamically loaded,
activated, deactivated or unloaded.

:::info

Read more about `ros2_control`
in [the official ros2_control documentation](https://control.ros.org/master/index.html).

:::