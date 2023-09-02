---
sidebar_position: 1
---

# Signals

In the AICA framework, signals are designed to exchange continuous data between components and controllers in a running
application. They are an abstraction of ROS 2 topics.

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

<!-- TODO: not sure if this is too much to put here, since it's more relevant to the programming reference
Additionally, ROS 2 messages are data packets, not data objects. Parsing data from a message, manipulating it and
writing it back into a message can involve a fair amount of boiler-plate code.

When developing an AICA component, signals are automatically converted into the corresponding data object.
-->

:::note

The distinction between signals and topics is also made for a different reason; signals are treated as a continuous,
periodic data exchange (though of course, computationally the messages still happen at discrete intervals). This is in
contrast to AICA events, which use topics to trigger discrete logical behavior.

:::

## Signal types

The following standard message types are provided for signals.

- Boolean (true / false)
- Integer (whole numbers)
- Double (floating point numbers)
- Vector (array of floating point numbers)
- String (plain text)

### State messages

In robot control applications, the _state_ of a robot or other objects is highly important.

AICA signals make it easy for components and controllers to exchange the following state message types.

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

### Custom messages

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
