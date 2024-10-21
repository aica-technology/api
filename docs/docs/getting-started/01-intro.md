---
sidebar_position: 1
---

# Getting started

This guide will cover the basic steps to get started with the AICA System, from installation to application examples.

## Compatibility

The AICA System is officially supported on Ubuntu and macOS for both amd64 and arm64 architectures.

AICA Core runs in a Docker container based on Ubuntu 24.04. Thanks to containerization, the majority of the
installation and usage steps in this guide will be the same for macOS or Ubuntu host platforms unless otherwise
indicated.

:::info

There are some limitations with Docker on macOS compared to Docker on Ubuntu, which can affect advanced usage.
For the best developer experience, use a modern Ubuntu host machine (20.04 or higher) with Docker v19 or higher. 

### Network sharing

While it is possible to grant a container access to the full host network on Ubuntu, sharing a network between a Mac
host and one or more containers requires slightly more work. This can affect connections to peripheral network devices
such as robots or sensors, or to a wider ROS network, or to other containers or processes on the host.

### Display sharing

Graphical user interfaces and application windows opened inside a container will fail to open and render on a Mac host
without explicit display forwarding. Further, support for OpenGL rendering is limited on macOS, which affects certain
3D programs and simulators commonly used in robotics. For example, opening an RViz window in the container on a Mac host
will fail, while it will work natively on an Ubuntu host.

AICA Studio is continually being extended to include more live data visualization including interactive 2D and 3D data
views directly in the browser. As a result, the reliance on display sharing and the associated operating system
limitations will be reduced.

### Real-time capabilities

To fully leverage the real-time capabilities of AICA controllers, the host machine needs a configured real-time kernel.
This is currently possible with the Ubuntu Pro 24.04 real-time kernel or the `PREEMPT_RT` kernel patch on standard
Ubuntu, but not possible on macOS.

:::