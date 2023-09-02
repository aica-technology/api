---
sidebar_position: 4
---

# Controllers

AICA controllers are implemented according to the [ros2_control](../03-ros-concepts/04-control.md) standard.

The main extension that AICA controllers make is the definition of standard [signals](./01-signals.md) to communicate
with components in a modular way. Controllers have parameters, inputs and outputs that can be connected to components in
the application.

Controllers are evaluated at fixed control intervals, defined by the rate of
the [hardware interface](./05-hardware-interfaces.md).

The AICA framework includes a number of controller implementation for joint-space and task-space control in position,
velocity or force, including force-sensitive impedance and admittance controllers.

The AICA framework can also work with third-party controllers following the `ros2_control` standard, but may have
reduced compatibility with built-in component signal types and dynamically reconfigured parameters.