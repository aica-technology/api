# AICA Applications

An AICA application is interconnected graph of **components** and **hardware interfaces**. Components process data in a
periodic step function and transfer data as signals to other components and hardware interfaces. Hardware interfaces
are drivers that connect controllers to robots. The application state can be dynamically changed through **events**,
which can change parameters, load or unload components, trigger specific service actions and more.

The nodes and edges of an AICA application graph can be visualized diagrammatically. It is also possible to define the
graph structure in a text format using YAML syntax.

## Validating a YAML Application

The [YAML application schema](./schema/application.schema.json) defines the structural rules of an
AICA application and effectively distinguishes between valid and invalid syntax.
