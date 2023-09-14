---
sidebar_position: 2
---

# Installation

The AICA framework is officially supported on macOS and Ubuntu platforms for both amd64 and arm64 architectures.

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

## Pre-requisites

AICA software is distributed with Docker images and executed with Docker containers.

Before proceeding, be sure to [install Docker Engine](https://docs.docker.com/engine/install/) on the host machine.

## Loading the AICA image

:::caution

Image distribution is being revised and will be change in the near future.

:::

```bash
docker load < aica-technology-base-v1.0.0.tar.gz
```

The command `docker image ls | grep aica-technology` should then include `aica-technology/base`.


:::note

The name and version of the zip file and extracted image may be different depending on the provided distribution.

:::