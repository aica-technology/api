---
sidebar_position: 3
---

# Dynamic composition

Traditionally, ROS nodes are compiled as executables so that they can be launched as stand-alone processes.
ROS 2 _components_ are an extension to nodes that are compiled as shared libraries. Rather than being launched
directly, components are dynamically loaded by other processes.

A component manager is a specialized container process that can load and unload components. By managing component
instances on demand, applications become more efficient but also more modular. The application and the network graph
structure become deploy-time and runtime decisions, rather than compile-time decisions.

Another key benefit of composition is the ability to execute multiple components in the same processing thread,
which enables memory sharing and intra-process communication. Instead of a published message needing to be serialized,
passed through a middleware communication layer and deserialized by a subscriber in a different process, intra-process
communication allows publishers and subscribers in the same process can simply exchange messages as pointers to
in-process memory. This configuration greatly reduces the computational burden compared and latency of messaging, which
is significant when components are sending and receiving complex message packets at high frequencies (as is often
the case in sensor-driven robot control).

:::info

Read more about composition
in [the official ROS 2 documentation](http://docs.ros.org/en/iron/Concepts/Intermediate/About-Composition.html).

:::