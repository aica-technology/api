---
sidebar_position: 1
title: GPIO Controllers
---

# General Purpose Input/Output (GPIO) Controllers

**General Purpose Input/Output (GPIO) Controllers** provide a standardized way to interact with **digital I/O signals**
exposed by a robot or device. These signals are commonly used to control or monitor external hardware such as grippers,
sensors, indicators, safety devices, or auxiliary tooling.

GPIO controllers operate on discrete input and output signals, providing dedicated handling of digital I/Os. This
specialization differentiates them from motion-oriented controllers and keeps GPIO interaction simple,
deterministic, and easy to integrate into higher-level application logic.

---

## Overview

GPIO controllers enable applications to:

- **Read GPIO state** (e.g., sensor feedback, limit switches)
- **Command GPIO outputs** (e.g., enable a tool, toggle a relay)
- **Interact with external devices** with low latency and predictable behavior

Within the AICA framework, GPIO functionality can be handled by controllers that also perform motion control. In
addition, AICA provides **generic GPIO-only controllers** for cases where a digital I/O needs to be managed
independently, such as external devices or sensors without associated motion.

These generic controllers are specialized for a single responsibility:

- **Broadcasting controllers** expose states to the rest of the system
- **Output controllers** command values on the hardware

---

## GPIOs in robot models

In ROS 2, GPIOs are declared in the robot's **URDF** using dedicated GPIO tags. These tags define:

- The GPIO name and direction (input or output)
- Its association with a robot or subcomponent
- The command or state interface used to access it

By relying on the robot description, GPIO controllers can automatically discover available digital signals and expose
them consistently, without hard-coded assumptions about the underlying hardware.

:::info

Some specialized controllers (e.g., for Universal Robot arms) may do motion control and expose I/Os in a single
controller. You may always refer to the **Help** tab in AICA Studio to determine whether a controller is already
claiming a GPIO for input or output, or whether you will need a standalone GPIO controller as described in the
following section.

:::

---

## Using GPIO controllers in AICA Core

In a ROS 2 control setup, and by extension in AICA Core, GPIO controllers operate on **GPIO interfaces defined in the
URDF** and exposed by the underlying hardware interface.

Two GPIO controllers are bundled by default, each aligned with a specific GPIO role:

- **GPIO Broadcaster Controller** – reads and publishes an output state
- **GPIO Output Controller** – sets an input value

Applications interact with these controllers through standard ROS 2 communication mechanisms, without direct access to
low-level drivers.

---

## GPIO Broadcaster Controller

The GPIO Broadcaster Controller is responsible for **observing a digital output state** and making it available to the
rest of the system.

It requires two parameters, as defined by ROS 2-compatible URDF GPIO declarations:

- GPIO group name
- State interface

Using this information, the controller retrieves the GPIO state and publishes the following predicates:

- `is_high`
- `is_low`

These predicates reflect the current value of the GPIO and can be consumed by higher-level logic or decision-making
components.

---

## GPIO Output Controller

The GPIO Output Controller is responsible for **commanding a digital input value** on the hardware.

It requires the following parameters:

- GPIO group name
- Command interface

An additional parameter specifies the value written to the interface:

- `command`

This parameter can be dynamically updated at runtime to trigger the desired hardware behavior.
