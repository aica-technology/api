---
sidebar_position: 1
title: GPIO
---

# General Purpose Input/Output (GPIO) Controllers

**General Purpose Input/Output (GPIO) Controllers** provide a standardized way to interact with **I/O signals**
exposed by a robot or device. These signals are commonly used to control or monitor external hardware such as grippers,
sensors, indicators, safety devices, or auxiliary tooling.

GPIO controllers operate on input and output signals, providing dedicated handling of **digital (binary)** and
**analog (non-binary)** I/O within the context of a controller. This specialization differentiates them from
motion-oriented controllers and keeps GPIO interaction simple, deterministic, and easy to integrate into higher-level
application logic.

GPIO controllers enable applications to:

- **Read GPIO state** (e.g., sensor feedback, limit switches)
- **Command GPIO outputs** (e.g., enable a tool, toggle a relay)
- **Interact with external devices** with low latency and predictable behavior

AICA provides **generic GPIO controllers** that are specialized for a single responsibility:

- **Broadcasting controllers** expose GPIO state to the AICA application
- **Output controllers** command GPIO values on the hardware

## GPIOs in robot models

In ROS 2, GPIOs are declared in the robot’s **URDF** using dedicated GPIO tags. These tags define:

- The GPIO name and direction (input or output)
- Its association with a robot or device
- The command or state interface used to access it

By relying on the robot description, GPIO controllers can automatically discover available I/O signals and expose them
consistently, without hard-coded assumptions about the underlying hardware.

:::info

Some specialized controllers (for example, controllers for specific robot arms) may combine motion control and I/O
handling within a single controller. You can always refer to the **Help** tab in AICA Studio to determine whether a
controller already claims a GPIO for commanding or state feedback, or whether a standalone GPIO controller as described
below is required.

:::

## Using GPIO controllers in AICA Core

In a ROS 2 control setup, and by extension in AICA Core, GPIO controllers operate on **GPIO interfaces defined in the
URDF** and exposed by the underlying hardware interface.

Two GPIO controllers are bundled by default, each aligned with a specific GPIO role:

- **GPIO Broadcaster Controller:** reads and publishes GPIO state through a general-purpose state interface
- **GPIO Output Controller:** commands GPIO values through a general-purpose command interface

Applications interact with these controllers through standard ROS 2 communication mechanisms, without direct access to
low-level drivers.

### GPIO Broadcaster Controller

The GPIO Broadcaster Controller is responsible for **observing GPIO state** and making it available to the rest of the
system.

It requires two parameters, as defined by ROS 2-compatible URDF GPIO declarations:

- GPIO group name
- State interface

Using this information, the controller retrieves the GPIO state and, for binary interfaces, publishes the following
predicates:

- `is_high`
- `is_low`

The raw state value is also exposed through a signal for direct consumption.

### GPIO Output Controller

The GPIO Output Controller is responsible for **commanding GPIO values** on the hardware.

It requires the following parameters:

- GPIO group name
- Command interface

An additional parameter specifies the value written to the interface:

- `command`

This parameter can be dynamically updated at runtime to trigger the desired hardware behavior.
