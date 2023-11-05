---
sidebar_position: 1
---

# Getting started

This guide will cover the basic steps to get started with the AICA framework, from installation to application examples.

:::caution

The AICA framework is being distributed in a limited beta program. Installation, configuration and run-time instructions
are subject to breaking changes in the near future. We will endeavour to keep this documentation up-to-date with the
latest versions. In case of doubt, contact support@aica.tech for help.

:::

<!-- TODO: write more guides for demo applications:
- Using an interactive marker with RViz
- Using a hardware + controller with a mock interface
- Using the generic interface with a simulator
- Using a real robot
-->

## Compatibility

The AICA framework is officially supported on Ubuntu and macOS for both amd64 and arm64 architectures.

The framework itself runs in a Docker container based on Ubuntu 22.04. Thanks to containerization, the majority of the
installation and usage steps in this guide will be the same for macOS or Ubuntu host platforms unless otherwise
indicated.

:::info

There are some limitations with Docker on macOS compared to Docker on Ubuntu, which can affect advanced usage.
For the best developer experience, use an Ubuntu 22.04 host machine.

### Network sharing

While it is possible to grant a container access to the full host network on Ubuntu, sharing a network between a Mac
host and one or more containers requires slightly more work. This can affect connections to peripheral network devices
such as robots or sensors, or to a wider ROS network, or to other containers or processes on the host.

### Display sharing

Graphical user interfaces and application windows opened inside a container will fail to open and render on a Mac host
without explicit display forwarding. Further, support for OpenGL rendering is limited on macOS, which affects certain
3D programs and simulators commonly used in robotics. For example, opening an RViz window in the container on a Mac host
will fail, while it will work natively on an Ubuntu host.

### Real-time capabilities

To fully leverage the real-time capabilities of AICA controllers, the host machine needs a configured real-time kernel.
This is currently possible with the Ubuntu Pro 22.04 real-time kernel or the `PREEMPT_RT` kernel patch on standard
Ubuntu, but not possible on macOS.

:::