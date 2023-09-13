---
sidebar_position: 2
---

# Built on ROS 2

The [Robot Operating System (ROS)](https://www.ros.org) is a collection of open-source software libraries and tools
built by roboticists, for roboticists. AICA embraces the ROS philosophies of modularity, distributed processing and
extensible open standards to support increasingly advanced robot applications.

At its core, ROS 2 is a middleware publisher/subscriber mechanism that enables different processes to communicate via
messages. ROS 2 is becoming more accessible with every new distribution, but understanding how to design, configure,
launch and manage ROS processes for any particular robotic application can be a daunting task. Fully leveraging the
capabilities of ROS from scratch often constitutes a degree in computer science or robotics in itself. The true power of
ROS comes from the abstractions provided by community packages and extensions that each address a particular niche and
can be combined to solve complex problems.

The AICA framework is built on ROS 2, but it is not intended to supersede existing ROS 2 distributions, packages or
tools. AICA applications and building blocks are fully compatible with existing ROS packages and community tools and
can be used and deployed alongside them.

For new developers, AICA provides an easy springboard into the wider world of ROS. Advanced users can take the
modularity and dynamic nature of the AICA framework to new frontiers.

At a glance, the AICA framework extends ROS 2 with the following features:

- Preconfigured and containerized ROS 2 workspaces
- Fully integrated `ros2_control` architecture and hardware drivers for real-time control
- Simplified and unified application syntax (no need for multiple nested launch files)
- Advanced message types and object bindings for Cartesian and joint state signals necessary for robot control
- Standardized component interfaces for effortless connectivity and modularity
- Dynamic, reconfigurable, event-driven application engine

Read on to learn more about key ROS 2 concepts in the next sections, or skip ahead to learn more
about [AICA applications](./04-aica-applications.md).