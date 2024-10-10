---
sidebar_position: 3
---

# AICA applications

The ROS 2 concepts of managed lifecycle nodes and dynamic composition (and by extension, lifecycle components) enable
more stateful and modular building blocks for complex applications. A lifecycle component can be flexibly instantiated
and destroyed at run-time, and its lifecycle states and transitions can be managed according to key application states
or events.

Similarly, the `ros2_control` architecture allows controllers to be dynamically loaded and activated or deactivated for
a given hardware interface, enabling reactive and versatile robot behaviors at runtime.

The catch is that components and controllers need to be _managed_; it is the job of some supervisor process
or processes to monitor states and trigger the corresponding events to load, activate, deactivate or unload components,
controllers or hardware interfaces.

All the components and controllers in an application also need to be communicating on matching interfaces; in ROS 2
terminology, they must have matching topic names and message types.

Building an advanced robotic application then results in two key types of challenge:

- designing and implementing the task-specific conditions and events that manage component and controller states
- arranging compatible components and controllers in a network graph of communication channels and messages

If application-specific state logic or graph structure is hard-coded into one or more components, those components cease
to be modular or re-usable for other applications. If, instead, the application logic is hard-coded in one main
executable program, usually according to some state machine paradigm, this in turn generally undermines the modern
concepts of abstraction and dynamic reconfiguration that ROS 2 tools provide.

To address this challenge, AICA Core includes a generalized execution process known as the Event Engine which
dynamically manages component and controller states according to a declarative and easily reconfigurable application
description.

## The Event Engine

The Event Engine (EE) is responsible for managing the states of all the elements of an AICA application.

An AICA application is an interconnected graph of **components**, **controllers** and **hardware interfaces**.
Components process data in a periodic step function and transfer data as **signals** to other components and
controllers. Hardware interfaces are drivers that connect controllers to robots.

The EE manages the application state using **events**, which are used to reconfigure parameters, load
or unload components, manage component lifecycle states, trigger specific service actions and more.

## Declarative application descriptions

Applications are defined using a declarative application description syntax. The application description specifies all
components, controllers and hardware interfaces that may be involved in an application, including their initial
parameter and signal configurations. The application description then also associates conditional event triggers with
the corresponding events.

For example, some component A might be loaded when the application starts. Then, components B and C should only be
loaded after component A is finalized. Finally, controller X should be started when component B is active, and
stopped if component C has an error.

In AICA System, applications are formally described in a YAML file. An application description can also be
represented as a graph of building blocks (components, controllers, hardware interfaces) connected by signals and
events.

The next sections describe the key elements of an AICA application in more detail.