---
sidebar_position: 5
title: Controllers
---

# Controllers

AICA controllers are implemented according to the [ros2_control](../../ros-concepts/controlling-robots) standard.

The main extension that AICA controllers make is the definition of standard interfaces to communicate
with components and the Event Engine in a modular way. Controllers have parameters, inputs and outputs that can be
connected to components in the application, and expose predicates and transitions to drive application events.

Controllers are evaluated at fixed control intervals, defined by the rate of
the [hardware interface](../hardware-interfaces).

AICA Core includes a number of controller implementations for joint-space and task-space control in position,
velocity or force, including force-sensitive impedance and admittance controllers. Additionally, sequenced motions can
be executed through a [joint trajectory controller](./jtc).

AICA Core can also work with third-party controllers following the `ros2_control` standard, but may have
reduced compatibility with built-in component signal types and dynamically reconfigured parameters.