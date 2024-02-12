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

## AICA framework

The AICA **framework** is a virtual robotics workspace pre-configured with a growing collection of software libraries
for motion generation, signal processing, machine learning and control algorithms. It includes hardware interfaces for
real-time external control of popular robot brands, force-torque sensors and cameras.
The workspace is the foundation and the software libraries are the building blocks.

<ComponentsControllersHardware className="themedSVG" style={{width: "100%"}}/>

## AICA applications

An **application** is a particular configuration of components, controllers and hardware interfaces from the AICA
framework, generally designed to perform a particular task.

:::tip
AICA develops bespoke applications to solve specific automation challenges for clients, and offers more general smart
applications built around a particular use-case (for example, mechanical assembly) that can be re-configured in only a
few steps.

Visit [our website](https://aica.tech) or [contact us](mailto:contact@aica.tech) to learn more about our service
offerings.
:::

## AICA Developer Interface

The **Developer Interface** is the user interface layer to the AICA framework that empowers developers to build and
extend their own advanced robotic applications. AICA developers use this interface internally when building custom
applications, and decided it was worth sharing with the robotics community as a stand-alone product with the following
features:

- Graphical web-based UI to dynamically edit, run and monitor AICA application
- REST API to interface with and manage AICA applications from CLI or automation cells
- Component SDK to extend the core library with custom logic and behaviors
