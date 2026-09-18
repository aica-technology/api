---
title: Getting Started
---

import SystemOverview from './getting-started/assets/system-overview-embedded.svg';
import ComponentsControllersHardware from './getting-started/assets/components-controllers-hardware-embedded.svg';

:::important
This organization and its content are not affiliated, associated, authorized, endorsed by, or in any way officially
connected with aicas GmbH, Karlsruhe, Germany (www.aicas.com and https://github.com/aicas). Any references to “AICA” are
strictly historic.
:::

# Adaptive & Intelligent Control Applications

We are committed to making robotics more accessible by providing smart software solutions for advanced robot control.

Robotics software has many layers and comprises many different parts. The following terms are used to distinguish
between different parts of our software stack.

<SystemOverview className="themedSVG" style={{width: "100%"}}/>

## System

The **System** is a software ecosystem comprising:

- **Core**: a powerful robotics workspace and runtime engine (Core)
- **Studio**: a graphical user interface for editing, running and monitoring **System applications**
- **Extensions**: collections of components, controllers, or hardware interfaces hosted on our registry that extend the
  capabilities of Core and Studio
- **Launcher**: a desktop app that can install and run Studio with Core and add-on packages in just a few clicks
- **Component SDK**: tooling to extend the base library with custom extensions that will work seamlessly alongside
  native components

## Core

**Core** is a virtual robotics workspace pre-configured with a growing collection of software modules
for motion generation, signal processing, machine learning and control algorithms. It includes hardware interfaces for
real-time external control of popular robot brands, force-torque sensors and cameras.
The workspace is the foundation and the software modules are the building blocks.

<ComponentsControllersHardware className="themedSVG" style={{width: "100%"}}/>

## Applications

A **System application** is a configuration of components, controllers, and hardware interfaces that are parameterized
and connected in a specific way, generally designed to perform a particular task. One of the main functions of Core is
to load and run System applications.

:::tip
We develop bespoke applications to solve specific automation challenges for clients, and offer more general smart
applications built around a particular use-case (for example, mechanical assembly) that can be re-configured in only a
few steps.

Visit [our website](https://ai-can-change.tech) or [contact us](mailto:contact@ai-can-change.tech) to learn more about
our service offerings.
:::

## Studio

**Studio** is the graphical user interface layer to Core that empowers developers to build and extend their own advanced
robotic applications. The interactive System application editor can be used to dynamically edit, run, and monitor robot
behaviors. Monitor robot and sensor state data directly in the browser, and manage application states and events
precisely through predicates, transitions, conditions, sequences, or interactive buttons.

## Launcher

**Launcher** is a desktop application to easily package and run a System image from a desired version of Core and any
number of official extensions from our registry or custom extensions built from the SDK.