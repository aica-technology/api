# AICA Applications

An AICA application is interconnected graph of **components** and **hardware interfaces**. Components process data in a
periodic step function and transfer data as signals to other components and hardware interfaces. Hardware interfaces
are drivers that connect controllers to robots. The application state can be dynamically changed through **events**,
which can change parameters, load or unload components, trigger specific service actions and more.

The nodes and edges of an AICA application graph can be visualized diagrammatically. It is also possible to define the
graph structure in a text format using YAML syntax.

---

# The YAML Application

The following sections define the YAML application syntax used to describe an AICA application.

- [Overview](#overview)
- [On Start](#on-start)
- [Hardware Interfaces](#hardware-interfaces)
    - [URDF](#urdf)
    - [Rate](#rate)
    - [Controllers](#controllers)
- [Conditions](#conditions)
  - [Simple Conditions](#simple-conditions)
  - [Conditional Operators](#conditional-operators)
  - [Nested Conditions](#nested-conditions)
- [Components](#components)
    - [Component](#component)
    - [Mapping](#mapping)
    - [Log Level](#log-level)
    - [Parameters](#parameters)
    - [Events](#events)
- [Validating a YAML Application](#validating-a-yaml-application)

## Overview

An application description contains the following top-level elements.

```yaml
on_start:
  ...

hardware_interfaces:
  ...

conditions:
  ...

components:
  ...
```


## On Start

The `on_start` keyword is reserved as a special event trigger when the application is launched.
List the [events](#events) to trigger on startup (for example, to load components).

```yaml
on_start:
  load:
    - component_a
    - component_b
```

## Hardware Interfaces

Hardware interfaces describe the connected robots and their corresponding controllers.

```yaml
hardware_interfaces:
  robot_a:
    urdf: ...
    rate: ...
    controllers:
      ...
  robot_b:
    ...
```

### URDF

The **urdf** field takes the path to a specially formatted robot description file which defines the joint configurations
and the hardware interface driver needed to communicate with the robot.

```yaml
robot_a:
  urdf: /home/ros2/my_robot.urdf
```

### Rate

The **rate** defines the robot control frequency in Hz.

### Controllers

Controllers are the interface between components in the application and hardware in the real world. They convert desired
reference signals into real joint commands according to some internal control law, and convert joint states from the
robot back to signals.

Controllers are listed under a top-level field called **controllers**. Controller names must be unique within the given
hardware interface, and should generally be in `lower_camel_case`.

Under each controller, the **plugin** field refers to a registered controller plugin name.

The **parameters** field then refers to configurable parameters for the given controller.

For example:

```yaml
robot:
  controllers:
    broadcaster:
      plugin: joint_state_broadcaster/JointStateBroadcaster
    twist_controller:
      plugin: compliant_twist_controller/CompliantTwistController
      parameters:
        linear_principle_damping: 10.0
        linear_orthogonal_damping: 10.0
        angular_stiffness: 1.0
        angular_damping: 1.0
```

## Conditions

Conditions are event triggers based on logical combinations of predicates.

Conditions are listed under a top-level field called `conditions`. Condition names must be unique, and should
generally be in `lower_camel_case`.

Conditional events are triggered only on the rising edge of the condition, preventing the repeated execution of an
event if the condition stays true.

Define events to be triggered by a condition by listing them under the condition name. See the [events](#events) section
for available event syntax.


```yaml
conditions:
  condition_1:
    component: ...
    predicate: ...
    events:
      ...
  
  condition_2:
    <conditional_operator>: ...  # not, all, any, one_of
    events:
      ...

```

### Simple Conditions

A simple condition evaluates just a single component predicate and triggers the listed events when it is true.

```yaml
condition_1:
  component: my_component
  predicate: some_component_predicate
  events:
    ...
```

### Conditional Operators

To combine multiple predicates together into a single true / false condition, the following operators can be used.

The operators can refer to one or more component predicates with the syntax
`{ component: component_a, predicate: some_predicate }`

#### Not

The **not** operator takes a single item and negates its value. It is true when the item is false, and false when the
item is true.

```yaml
condition_1:
  not: { component: component_a, predicate: some_predicate }
  events:
    ...
```

#### All

The **all** operator takes a list of items and is true only when every listed item is true.

```yaml
condition_1:
  all:
    - { component: component_a, predicate: some_predicate }
    - { component: component_b, predicate: some_predicate }
  events:
    ...
```

#### Any

The **any** operator takes a list of items and is true when at least one of the listed items is true.

```yaml
condition_1:
  any:
    - { component: component_a, predicate: some_predicate }
    - { component: component_b, predicate: some_predicate }
  events:
    ...
```

#### One Of

The **one_of** operator takes a list of items and is true only when exactly one of the listed items is true.

```yaml
condition_1:
  one_of:
    - { component: component_a, predicate: some_predicate }
    - { component: component_b, predicate: some_predicate }
  events:
    ...
```

### Nested Conditions

The conditional operators can be applied recursively for more complex conditions. The following example could be
collapsed into the equivalent logical pseudocode: `NOT(a AND b AND (c OR d OR (e XOR f)))`

```yaml
conditions:
  nested_condition:
    not:
      all:
        - { component: component_1, predicate: a }
        - { component: component_2, predicate: b }
        - any:
            - { component: component_3, predicate: c }
            - { component: component_4, predicate: d }
            - one_of:
                - { component: component_5, predicate: e }
                - { component: component_6, predicate: f }
```

## Components

Components are listed under a top-level field called `components`. Component names must be unique, and should
generally be in `lower_camel_case`.

```yaml
components:
  component_a:
    component: ...  # required
    log_level: ...  # optional
    mapping: ...    # optional
    parameters: ... # optional
    events: ...     # optional

  component_b:
    ...
```

Each component is defined with a number of fields, as shown below. The fields are defined in the next section.

### Component

The `component` field defines the actual component implementation to use for the component.
It takes a fully qualified class name as registered by the `RCLCPP_COMPONENTS_REGISTER_NODE` macro.

The registered class name of a component should include the package name within the namespace. For example, the
registration `foo_components::Foo` refers to a component `Foo` in package `foo_components`.

```yaml
my_component:
  component: foo_components::Foo
```

### Mapping

The `mapping` field optionally defines overrides for the component name and namespace. Normally, the component node
is instantiated with the same name as the top level component name and put on the base namespace.

By specifying a mapping `name` or `namespace` or both, the instantiated node name is updated accordingly.

```yaml
# Without the mapping directive, the node name becomes /component_a
component_a:
  ...

# With the mapping directive, the node name becomes /my_component_namespace/my_new_component_name
component_b:
  mapping:
    name: my_new_component_name
    namespace: my_component_namespace
```

### Log Level

The `log_level` optionally sets the log severity level for this component.
Supported levels are: [unset, debug, info, warn, error, fatal]

```yaml
my_component:
  log_level: debug
```

### Parameters

The `parameters` field allows initial parameters values to be set using a `name: value` syntax.
Currently, only string and double parameters are supported. These values are only applied when the component
is loaded and are not dynamically reconfigurable.

```yaml
my_component:
  parameters:
    my_string_parameter: "my_string_value"
    my_double_parameter: 2.0
```

#### Component period

The `period` parameter is a special reserved parameter that defines the step period of a component in seconds, which is
the inverse of the execution period.

For example, if an image processing component should run some computation at 20 frames per second, then the
period parameter should be set to 0.05 seconds.

```yaml
my_component:
  parameters:
    period: 0.05
```

### Events

Events drive the emergent behaviour of an application. They are triggered by predicates which the components broadcast
under specific names (depending on the component). For example, a lifecycle component may have the `is_configured` and
`is_active` predicates.

The events are triggered only on the rising edge of the predicate signal, preventing the repeated execution of an
event if the predicate stays true.

Define events to be triggered by a predicate by listing them under the predicate name, as shown below.

```yaml
my_component:
  events:
    is_active:
      <triggered event_a>: ...
      <triggered event_b>: ...
    some_other_predicate_name:
      <triggered event_c>: ...
      <triggered event_bd>: ...
```

The supported events are:

- **set**: Set a parameter
- **service**: Call a service
- **lifecycle**: Change a lifecycle state
- **load**: Load a component
- **unload**: Unload a component
- **transition**: Transition one component to another by unloading the first component and loading the second
- **load_hardware**: Load and initialize a hardware interface
- **unload_hardware**: Unload and destroy a hardware interface
- **load_controller**: Load a controller for a hardware interface
- **unload_controller**: Unload a controller for a hardware interface
- **switch_controllers**: Activate or deactivate controllers for a given hardware interface

#### Simple event syntax

```yaml
event_name:
  # Set a parameter using a name-value pair.
  set:
    parameter: "name_of_the_parameter"
    value: "value_of_the_parameter"

  # Call a specific service on this component with no request payload
  service: "name_of_the_service"

  # Request a lifecycle state change by name (using "configure", "activate", "deactivate", "cleanup")
  lifecycle: "configure"

  load: "required_component_name"
  unload: "optional_target_component_name"
  transition: "required_component_name"

  load_hardware: "name_of_the_interface"
  unload_hardware: "name_of_the_interface"

  load_controller:
    interface: "name_of_the_interface"
    controller: "name_of_the_controller"

  unload_controller:
    interface: "name_of_the_interface"
    controller: "name_of_the_controller"

  switch_controllers:
    interface: "name_of_the_interface"
    start: "controller_one"
    # stop: "controller_two"
```

#### Complex event syntax

```yaml
  # Set multiple parameters through "-" list separation
  set:
    - parameter: "name_of_the_first_parameter"
      value: "value_of_the_first_parameter"
    - parameter: "name_of_the_second_parameter"
      value: "value_of_the_second_parameter"
      component: "optional_target_component_name"

  # Call a specific service on another component and supply a specific request payload.
  service:
    name: "name_of_the_service"
    component: "optional_target_component_name"
    payload: "optional string payload for service"

  # Request a lifecycle state change using a transition and an optional target component
  lifecycle:
    transition: "configure"
    component: "optional_target_component_name"

  # Use in-line dictionaries for more compact syntax
  load_controller: { interface: "name_of_the_interface", controller: "name_of_the_controller" }

  # Use lists to start and stop multiple controllers
  switch_controllers:
    interface: "name_of_the_interface"
    start: [ "controller_one", "controller_two" ]
    stop: [ "controller_three", "controller_four" ] 
```

#### Doing more with services

The service payload can also be written as any standard YAML object. The application parser will automatically encode
the object into a string format when making the service call. In this case, the component service is responsible
for parsing the string back into a YAML object, dict or structure as necessary.

```yaml
  service:
    name: "name_of_the_service"
    component: "optional_target_component_name"
    payload:
      foo: "some content"
      bar: [ x, y, z ]
      baz:
        a: 1
        b: 2
```

#### Special event predicates

**on_load**

The `on_load` predicate is provided by the state engine and set to true after the component
has been loaded. Any events associated with the `on_load` predicate are handled once
on instantiation of the node.

```yaml
component:
  events:
    on_load:
      <some triggered event>: ...
```

**on_unload**

The `on_unload` predicate is similar to the `on_load` predicate and is provided by the state engine.
Any events associated with the `on_unload` predicate are handled once upon destruction of the component interface.

```yaml
component:
  events:
    on_unload:
      <some triggered event>: ...
```


## Validating a YAML Application

The [YAML application schema](./schema/schema/application.schema.json) defines the structural rules of an
AICA application and effectively distinguishes between valid and invalid syntax.

Many modern IDEs and code editors can be configured to support custom schemas and provide in-line validation and
completion of the YAML content.