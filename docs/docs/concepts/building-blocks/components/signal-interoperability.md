---
sidebar_position: 1
title: Signal interoperability
---

# Signal interoperability

As described in the [signals page](../signals.md), AICA signals make it easy to exchange Cartesian and joint state
variables in an internally consistent way. In components, state signals are automatically converted into smart data
classes that provide useful functions for conversions, transformations and other manipulations.

Even though there is no official standard, there are a few signal types that are very commonly used in ROS. For ease of
interoperability, the AICA Core includes several components that translate AICA signals to common ROS messages and back.
These components can be especially valuable when porting existing ROS nodes into AICA Studio using the Component SDK or
when communicating with ROS nodes outside the AICA System.

## AICA signals to common ROS messages

AICA state signals carrying Cartesian or joint space information can be converted to commond ROS message types using the
following components:

| Component name                             | Input signal type                             | Output message type                 |
| ------------------------------------------ | --------------------------------------------- | ----------------------------------- |
| Cartesian Signal to Pose Stamped Message   | Cartesian state or pose                       | `geometry_msgs::msg::PoseStamped`   |
| Cartesian Signal to Twist Stamped Message  | Cartesian state or twist                      | `geometry_msgs::msg::TwistStamped`  |
| Cartesian Signal to Wrench Stamped Message | Cartesian state or wrench                     | `geometry_msgs::msg::WrenchStamped` |
| Joint Signal To Joint State Message        | Joint state, positions, velocities or torques | `sensor_msgs::msg::JointState`      |

## Common ROS messages to AICA signals

Common ROS message types carrying Cartesian or joint space information can be converted back into AICA state signals
using the following components:

| Component name                             | Input message type                  | Output signal type                            |
| ------------------------------------------ | ----------------------------------- | --------------------------------------------- |
| Pose Stamped Message to Cartesian Signal   | `geometry_msgs::msg::PoseStamped`   | Cartesian state containing pose information   |
| Twist Stamped Message to Cartesian Signal  | `geometry_msgs::msg::TwistStamped`  | Cartesian state containing twist information  |
| Wrench Stamped Message to Cartesian Signal | `geometry_msgs::msg::WrenchStamped` | Cartesian state containing wrench information |
| Joint State Message to Joint Signal        | `sensor_msgs::msg::JointState`      | Joint state                                   |

## Behavior

All of these components are simple single-input single-output blocks. Each time a new message is received, it is
translated and immediately published. For that reason, the `rate` parameter doesn't affect the behavior of these
components.

:::tip

See [this page](../../../examples/core-components/signal-interoperability.md) for examples using signal translator
components in AICA Studio.

:::