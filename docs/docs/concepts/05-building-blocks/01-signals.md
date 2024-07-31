---
sidebar_position: 1
---

# Signals

In the AICA System, signals are designed to exchange continuous data between components and controllers in a running
application. They are an abstraction of ROS 2 topics that are assumed to exchange data at a regular, periodic frequency.
This makes them well-suited for use with signal processing components and controllers.

ROS 2 topics are messages sent from publishers to subscribers under a specific namespace (the message "topic")
using some predefined message format (the message "type"). As a result, topics enable ROS nodes to communicate in many
different network topologies and message formats.

Often, the message topic and message type is hardcoded within the implementation of a ROS node. This can make it
difficult to rearrange the network topology of ROS nodes without modifying and recompiling the node implementation
itself.

Signals then are ROS 2 publishers and subscribers with dynamically assigned topics and standardized message types.
This makes it easy to reconfigure the signal connections between different components and controllers in the application
graph without modifying or recompiling any source code. By using standard message types, the signal compatibility
between components and controllers is also simplified.

Additionally, ROS 2 messages are data packets, not data objects. Parsing data from a message, manipulating it and
writing it back into a message can involve a fair amount of boilerplate code.

When developing an AICA component, signals are automatically converted to and from the corresponding data object.

## Basic signal types

The following standard message types are provided for signals.

- Boolean (true / false)
- Integer (whole numbers)
- Double (floating point numbers)
- Vector (array of floating point numbers)
- String (plain text)

## State signals

In robot control applications, the _state_ of a robot or other objects is highly important.

In robotics and control, one of the most useful data types is the state of an object, i.e. its spatial properties.
The joint angles of a robot arm, the 3D position and velocity of a flying drone, or the measurement of an accelerometer
or force-torque sensor are all examples of instantaneous state variables.

AICA signals make it easy for components and controllers to exchange Cartesian and joint state variables in an
internally consistent way. For component developers, state signals are automatically converted into smart data classes
that provide useful functions for conversions, transformations and other manipulations.

The following state variables can be exchanged as signals:

- Joint state
    - Positions
    - Velocities
    - Accelerations
    - Torques
- Cartesian state
    - Pose
        - Position
        - Orientation
    - Twist
        - Linear velocity
        - Angular velocity
    - Acceleration
        - Linear acceleration
        - Angular acceleration
    - Wrench
        - Force
        - Torque

:::info

<!-- TODO: copy and link the markdown documentation of the state representation library directly in the programming reference -->

AICA state signals are built on the
open-source [`state_representation`](https://aica-technology.github.io/control-libraries/versions/v7.1.0/md__github_workspace_source_state_representation__r_e_a_d_m_e.html)
library for Cartesian and joint state classes in C++ and Python.

:::

## Custom messages

The standard primitive and state message types are generally enough to cover the majority of messaging needs in
an AICA application. Having a reduced message definition set is important to maximizing the modularity and compatibility
of components. When components are connected by a signal in an application graph, the application interpreter will try
to assert that the signals have a matching type.

However, any ROS 2 message can be implemented as a signal using the `custom` signal type. As long as the custom type
between two connected components has the same name, the application will be valid.

:::tip

The AICA component library includes signal translator components for commonly used ROS messages (namely `std_msgs`
and `geometry_msgs`) for AICA components to communicate with traditional ROS nodes in an external process.

:::
