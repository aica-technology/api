---
sidebar_position: 1
title: General Purpose Input/Output Controller
---

# General Purpose Input/Output (GPIO) Controller

A **General Purpose Input/Output (GPIO) Controller** provides a standardized way to **read from and write to digital
I/O signals** exposed by a robot or device. These signals are typically used to interact with external hardware such as
grippers, sensors, indicators, safety devices, or auxiliary tooling.

Unlike motion controllers, a GPIO controller does not command joints or Cartesian motion. Instead, it operates on
discrete input and output signals that represent **binary or small-range state** at the hardware level.

:::info

In the AICA framework, we assume GPIOs are used to set and broadcast binary information.

:::

In ROS 2-based robotic systems, GPIO interfaces are commonly described directly in the robot model and accessed through
well-defined control interfaces.

---

## Overview

A GPIO controller allows users to:

- **Set output values** (e.g., enable a tool, open or close a gripper, toggle a relay)
- **Read input values** (e.g., detect sensor states, limit switches, or digital feedback)
- **Interact with external devices** in a deterministic and low-latency manner

The controller acts as a bridge between higher-level application logic and the robot's digital I/O capabilities,
ensuring that GPIO access is consistent, discoverable, and integrated into the overall control architecture.

---

## GPIOs in Robot Models

In ROS 2, GPIOs are typically declared in the robot's **URDF** using dedicated GPIO tags. These tags describe:

- The name and direction of each GPIO (input or output)
- Its association with the robot or a specific component
- The interface through which it can be accessed

By relying on the robot description, a GPIO controller can automatically discover available digital signals and expose
them in a uniform way, without requiring hard-coded assumptions about the underlying hardware.

---

## Using it in AICA Core

In a ROS 2 control setup, a GPIO controller typically operates on **GPIO interfaces defined in the URDF** and exposed by
the underlying hardware interface. Applications can then read or set GPIO values through standard ROS 2 communication
mechanisms, without direct access to low-level drivers.

Two GPIO controllers are bundled in AICA Core by default: a GPIO Broadcaster and a GPIO Output.

### GPIO Broadcaster controller

The controller requires 2 parameters to be set, as per the definition of GPIOs in ROS 2-compatible URDFs:

- GPIO group name
- State interface

This information is then adequate for the controller to retrieve the interface's state and publish 2 predicates:

- `is_high`
- `is_low`

that reflect what was retrieved.

### GPIO Output controller

Similarly to the broadcaster, the controller requires the following 2 parameters to be set:

- GPIO group name
- Command interface

An additional, 3rd, parameter is used to determine the value that will be written to the interface, namely:

- Command

and can be dynamically set at runtime to trigger the desired behavior.