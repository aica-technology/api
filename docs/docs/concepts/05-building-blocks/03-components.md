---
sidebar_position: 3
---

# Components

Within an AICA System application, components are the building blocks of advanced robot behaviors.

AICA components are wrappers for [ROS 2 nodes](../03-ros-concepts/01-nodes.md)
and [lifecycle nodes](../03-ros-concepts/02-lifecycle.md)
using [dynamic composition](../03-ros-concepts/03-composition.md) with additional abstractions for enhanced modularity
and developer convenience.

Components process data in a periodic step function, are configured using parameters and transfer data as signals to
other components and hardware interfaces. Key component states are published as predicates. Components may also expose
specific services.

## Periodic behavior

A key design concept for AICA components is the idea of periodic behavior.

In ROS, nodes are generally driven through callbacks, triggered either by a subscription, parameter change event,
or service request. Nodes that wish to publish data continuously normally configure an internal timer to trigger a
callback function at regular intervals.

This periodic execution is built in to AICA components by default. Components have an execution rate defined by
a `rate` parameter (the frequency in Hertz for the number of execution steps per second), and automatically publish data
attributes and predicates periodically.

Components can implement specific calculations to occur on step.

:::note

Lifecycle components only execute the periodic step function when they are in the `ACTIVE` lifecycle state.

:::

## Parameters

Components support the standard ROS 2 parameter interface types.

- Boolean (true / false)
- Integer (whole numbers)
- Double (floating point numbers)
- String (plain text)

They can additionally be an array (list) of multiple values of the same type:

- Boolean array
- Integer array
- Double array
- String array

<!-- TODO: not sure if this goes too deep for the concepts section
Internally, components define parameters as real data objects which are bound to the corresponding ROS 2 parameter message type.
This allows components to also define parameters as mathematical vectors, matrices and the spatial state types defined in
the [`state_representation` library](https://aica-technology.github.io/control-libraries/versions/main/md__github_workspace_source_state_representation__r_e_a_d_m_e.html).
-->

## Signals

Components define signals as inputs (subscribers) and outputs (publishers). Refer to [Signals](./01-signals.md) for more
information.

:::note

By default, lifecycle components only publish outputs when they are in the `ACTIVE` lifecycle state.

:::

## Predicates

Components declare and broadcast key internal states as predicate messages, which are used by the Event Engine
to trigger events. Refer to [Events](./02-events.md) for more information.

### Auto lifecycle events

When a lifecycle components is loaded, it starts in the unconfigured state and normally requires an external trigger to
transition into different states.

In the AICA System, lifecycle components can auto-configure and auto-activate themselves
using [transition events](./02-events.md#transitions). Associating the `on_load` state transition with a
`lifecycle: configure` event enables the component to automatically configure itself. Equivalently, the `on_configure`
transition can be used to trigger a `lifecycle: activate` event.

## Services

Components can provide service endpoints to trigger specific behaviors on demand. For compatibility with the application
syntax and the Event Engine, the component service are limited to one of two request types:

- Service request with no payload
- Service request with a string payload

The service response object for both types contains an execution status and message and is automatically handled by the
Event Engine.