---
sidebar_position: 1
title: Motion generation
---

import signalRosPose from './assets/signal-ros-pose.png'

# Motion generation

Motion generation plays an important role in robotics because it determines how a robot moves to accomplish its task. At
its core, motion generation is about planning and executing that allow a robot to reach a desired position or follow a
specific path, while taking into account constraints such as obstacles, joint limits, and safety requirements. This
process is essential for enabling robots to perform useful work, whether it's picking up objects, assembling parts, or
navigating through an environment.

One can differentiate between two approaches to motion generation; offline and online.

**Offline motion generation** plans trajectories in advance using models of the robot and environment. This method
enables optimized, collision-free paths and is ideal for predictable, structured settings. Generated trajectories are
rolled out in **open loop**, meaning that there is no feedback mechanism that would inform the system about
disturbances. In other words, it lacks adaptability to real-time changes or sensor feedback. In the AICA System, the
[Joint Trajectory Controller](../guides/jtc-guide.md) often acts as simple offline motion generator, creating and
rolling out a trajectory through a certain number of waypoints in a given amount of time without taking into account the
environment.

**Online motion generation** computes movements in real time as the robot operates, allowing adaptation to dynamic
environments and sensor input. This flexibility is essential for applications where the robot must be responsive and
adaptive, such as working alongside humans, handling objects whose positions are not precisely known in advance, or when
the task is highly driven by sensor feedback. Most often, such systems are **closed loop**, meaning that the current
state of the robot and the environment is used to inform what to do next. The downside of this approach is that it often
demands more computational resources and may not always find the most optimal path.

The choice between offline and online motion generation depends on the requirements of the application. In many cases, a
combination of both approaches is used; an initial trajectory is planned offline, and then online adjustments are made
as needed to handle changes or uncertainties in the environment.
