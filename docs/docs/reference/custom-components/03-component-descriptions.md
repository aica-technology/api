---
sidebar_position: 3
title: Describing a component
---

# Describing a component

To fully support custom components in the AICA System, it is crucial to have a consistent means of
describing individual components and the interfaces they provide. A machine-readable component description should
contain all necessary information to procedurally generate documentation, frontend visualisations and backend services
for any new component.

Components have various interfaces that need to be described:

- signals to send and receive data in the form of inputs and outputs
- parameters with specific types and names that define their behaviour
- services to trigger specific behaviors on request
- predicates that produce special "true / false" messages to indicate specific component states

Aside from interfaces, a component description should also include information on the type of component (regular or
lifecycle), the general component behavior or purpose, and other metadata such as name and registration details.

<!-- FIXME: link to the schema on GitHub once it is on main; relative paths will break if the doc is versioned -->
The elements of a component description are written as a standardized JSON file for each component. The expected
structure of the component description is defined by
the [Component Description JSON schema](../../../../schemas/component-descriptions/schema/component.schema.json)

:::info

JSON schemas for public AICA structures and syntax can be found in the `schemas` directory of
the [API repository](https://github.com/aica-technology/api) on AICA GitHub.

:::

## Component metadata

The basic required properties of a component description are the component name, description, registration and
inheritance, as shown below:

```json title="component_descriptions/foo_component_package_foo.json"
{
  "name": "Foo",
  "description": {
    "brief": "An example component",
    "details": "This is an in-depth description of the example component."
  },
  "registration": "foo_component_package::Foo",
  "inherits": "modulo_components::Component"
}
```

The `name` should be short, descriptive and human-readable, suitable for displaying to the user when listing a directory
of components or rendering a component visually on the frontend.

The `description` property should have a brief one-line description and preferably a more detailed description
that explains the functionality of the component in depth.

### Registration

The `registration` property is used to associate the description with a unique component class. Component classes
are implemented by ROS2 packages and registered under a unique class name. By convention, the class name must be
defined in the namespace of the package, delimited by double colons `::`. For example, some component class `Foo`
in the package `foo_component_package` should be registered as `foo_component_package::Foo`, so that the package can be
inferred from the registered name.

```json
{
  "registration": "foo_component_package::Foo"
}
```

### Inheritance

The implementation of custom component classes can inherit and extend the behaviors of base component classes.
When a base component already has suitable descriptions for signals, parameters, predicates and services, they
should not be duplicated for each derived component class description. Instead, the registration of the base class
can be provided, following the same rules as the [`registration` property](#registration).

Any consumer of the component description can dynamically load the base component description using the registration
details in the `inherits` property and expand the fields into the derived description.

Components that derive directly from `modulo_components::Component` or `modulo_components::LifecycleComponent`
should also specify this under the `inherits` property. These base classes are identified and treated
distinctly by consumers of the component description.

#### Virtual components

With the inheritance pattern, a base component class can implement common properties and behaviors that derived
components extend. In some cases, the base class itself might not be intended to be instantiated or used as a component
directly. For example, a base motion generator may define common parameters, signals and logic, but might produce no
output if the signal calculation is left for derived classes to implement. Such base components may be considered
"virtual" or "abstract".

The `virtual` property is a boolean flag which, if true, indicates that a component class is an abstract or virtual
base class not intended for instantiation. It is an optional property and is assumed to be false if omitted.
Consumers of the component description can use this property to handle virtual base classes distinctly; for example,
virtual components might be hidden in drop-down menus or lists of available and instantiable components.

The core classes `modulo_components::Component` and `modulo_components::LifecycleComponent` are examples of virtual
components, as they provide no meaningful behavior if directly instantiated.

## Signals

<!-- FIXME: link to the schema on GitHub once it is on main; relative paths will break if the doc is versioned -->
The expected structure of the signal description is defined by
the [Signal Description JSON schema](../../../../schemas/signals/schema/signal.schema.json).

Component inputs and outputs are described with a signal name and type, matching the name and type in the implementation
when using the respective `add_input` or `add_output` function. Additionally, a human-readable display name and
description should be included.

An example description of a component with one input and one output, both as joint states, is given below:

```json
{
  "inputs": [
    {
      "display_name": "Joint state",
      "description": "The current joint state from the robot",
      "signal_name": "state",
      "signal_type": "joint_state"
    }
  ],
  "outputs": [
    {
      "display_name": "Joint command",
      "description": "The desired joint command to send to the robot",
      "signal_name": "command",
      "signal_type": "joint_state"
    }
  ]
}
```

## Parameters

<!-- FIXME: link to the schemas on GitHub once they are on main; relative paths will break if the doc is versioned -->
The expected structure of the parameter description is defined by
the [Parameter Description JSON schema](../../../../schemas/parameters/schema/parameter.schema.json).

Parameters contain a value of a certain type, described by the `parameter_type` property. Valid parameter types are
defined in the [Parameter Type JSON schema](../../../../schemas/parameters/schema/parameter_type.schema.json).

If the `parameter_type` property is `state`, then the `parameter_state_type` property must also be defined as a member
of the enumeration
in [Encoded State Type JSON schema](../../../../schemas/parameters/schema/encoded_state_type.schema.json).

Parameters generally have a default value which renders them optional in some cases. When a parameter has no valid
default state and must be set by the user for the component to function, the `default_value` property in the component
description should be set to `null`. This is distinct from a valid default empty parameter state, which can be
expressed with the empty string `""`.

If a parameter is dynamically reconfigurable, as described by the `dynamic` property, it implies that the parameter can
be changed after the component is already configured to influence the run-time behaviour.
This requires the component to either poll the parameter value while running or implement a parameter change callback.

The `internal` property can be used to hide a parameter from public interfaces that use the component description by
indicating that the parameter is intended for internal use only. The intended outcome is for the parameter to be hidden
from auto-generated component visualizations or documentation at a high level.
If the field is omitted from a description, the parameter is assumed to be public.

An example parameter description is shown below.

```json
{
  "parameters": [
    {
      "display_name": "Gain",
      "description": "The gain value by which the input should be scaled",
      "parameter_name": "gain",
      "parameter_type": "double",
      "default_value": "1.0"
    }
  ]
}
```

## Predicates

Predicates are crucial to drive the event logic of the Dynamic State Engine, but they are very simple to declare
and describe. Each predicate of a component indicates a particular run-time state or configuration that is either
"true" or "false". In the implementation, a predicate is a publisher with a boolean type. The component is responsible
for determining the value of a predicate and publishing it under a particular topic name.

In the component description, predicates have a display name and description. The `predicate_name` property is used
to inform the state engine of the hidden topic name to listen to for that predicate.

An example predicate description is shown below.

```json
{
  "predicates": [
    {
      "display_name": "Is in bounds",
      "description": "If true, the target frame is within the collision object",
      "predicate_name": "is_in_bounds"
    }
  ]
}
```

## Services

Services are endpoints provided by a component that can trigger certain behaviours. In the backend implementation,
they are created as ROS2 services with a specific `service_name` using one of two service message types. The first
type is simply an empty trigger service that has no payload, which is the default case. The second type is a trigger
service with a string payload, which can be used to pass parameters to the service call.

The `payload_format` property is used to describe the expected format and usage of the string trigger payload;
it is thereby inferred that the service type is not an empty trigger when this property is present.

Some example service descriptions are shown below.

```json
{
  "services": [
    {
      "display_name": "Record frame",
      "description": "Record a frame from TF and write to file in yaml format",
      "service_name": "record_frame",
      "payload_format": "A YAML-formatted dict with the following name-value fields: 'frame' for the TF to lookup and 'reference_frame' (default 'world')"
    },
    {
      "display_name": "Reset",
      "description": "Reset the component by deleting all recorded frames and removing the output file",
      "service_name": "reset"
    }
  ]
}
```