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

- The GPIO name and direction (state or command)
- Its association with a robot or device
- The command or state interface used to access it

By relying on the robot description, GPIO controllers can automatically discover available I/O signals and expose them
consistently, without hard-coded assumptions about the underlying hardware.

:::info

Some specialized controllers (for example, controllers for specific robot arms) may combine motion control and I/O
handling within a single controller.

:::

For example, a typical URDF that contains a GPIO command interface will look something like the following:

```xml
<gpio name="sample_gpio_group_1">
  <state_interface name="output_register" />
</gpio>
```

and one that contains a GPIO state interface:

```xml
<gpio name="sample_gpio_group_2">
  <command_interface name="input_register" />
</gpio>
```

or you may see both command and state interfaces bundled under the same group, as follows:

```xml
<gpio name="sample_gpio_group">
  <state_interface name="output_register" />
  <command_interface name="input_register" />
</gpio>
```

## Using GPIO controllers in the AICA framework

In a ROS 2 control setup, and by extension in AICA Core, GPIO controllers operate on **GPIO interfaces defined in the
URDF** and exposed by the underlying hardware interface.

Two GPIO controllers are bundled by default, each aligned with a specific GPIO role:

- **GPIO Broadcaster Controller:** reads and publishes GPIO state through a general-purpose state interface
- **GPIO Output Controller:** commands GPIO values through a general-purpose command interface

Applications interact with these controllers through standard ROS 2 communication mechanisms, without direct access to
low-level drivers.

### GPIO Broadcaster Controller

The GPIO Broadcaster Controller is responsible for **observing GPIO state** and making it available to the rest of the
AICA application.

It requires two parameters:

- GPIO group name (for example `sample_gpio_group_1` in the URDF snippet above)
- State interface (for example `output_register` in the URDF snippet above)

Using this information, the controller retrieves the GPIO state and, for binary interfaces, publishes *Is high* and
*Is low* predicates. The raw state value is also exposed through a signal for direct consumption.

### GPIO Output Controller

The GPIO Output Controller is responsible for **commanding GPIO values** on the hardware.

It requires the following parameters:

- GPIO group name (for example `sample_gpio_group_2` in the URDF snippet above)
- Command interface (for example `input_register` in the URDF snippet above)

The parameter *Command* specifies the value written to the interface and can be dynamically updated at runtime to
trigger the desired hardware behavior.

