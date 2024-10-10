---
sidebar_position: 1
---

import SystemOverview from './assets/system-overview-embedded.svg';
import ComponentsControllersHardware from './assets/components-controllers-hardware-embedded.svg';

# Introduction

AICA is committed to making robotics more accessible by providing smart software solutions for advanced robot control.

Robotics software has many layers and comprises many different parts. The following terms are used to distinguish
between different parts of the AICA software stack.

<SystemOverview className="themedSVG" style={{width: "100%"}}/>

## AICA Core

**AICA Core** is a virtual robotics workspace pre-configured with a growing collection of software modules
for motion generation, signal processing, machine learning and control algorithms. It includes hardware interfaces for
real-time external control of popular robot brands, force-torque sensors and cameras.
The workspace is the foundation and the software modules are the building blocks.

<ComponentsControllersHardware className="themedSVG" style={{width: "100%"}}/>

## AICA applications

An **application** is a particular configuration of components, controllers and hardware interfaces from the AICA
Core, generally designed to perform a particular task.

:::tip
AICA develops bespoke applications to solve specific automation challenges for clients, and offers more general smart
applications built around a particular use-case (for example, mechanical assembly) that can be re-configured in only a
few steps.

Visit [our website](https://aica.tech) or [contact us](mailto:contact@aica.tech) to learn more about our service
offerings.
:::

## AICA Studio

**AICA Studio** is the graphical user interface layer to AICA Core that empowers developers to build and
extend their own advanced robotic applications. The interactive application editor can be used to dynamically edit, run
and monitor robot behaviors. Monitor robot and sensor state data directly in the browser, and manage application states
and events precisely through predicates, transitions, conditions, sequences or interactive buttons.

## AICA System

The **AICA System** is the software ecosystem that extends AICA Core and Studio. While AICA Core comes bundled with
the basics, AICA is continually developing additional collections of components, controllers or hardware interfaces
that suit particular use-cases or types of robots. These first-party add-on packages are available to download from
the AICA registry.

- **AICA Launcher** is a desktop app that can install and run AICA Studio with Core and add-on packages in just a few
  clicks
- **AICA Studio: Cloud** is an online-only version of the Studio experience to access and edit applications from
  anywhere
- The **Component SDK** allows developers to extend the base library with custom functionality that will work seamlessly
  alongside native components
