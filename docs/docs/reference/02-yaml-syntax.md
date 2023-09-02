---
sidebar_position: 1
---

# YAML application syntax

The following sections define the YAML syntax used to describe an AICA application.

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
    display_name: ... # optional
    controllers:
      ...
  robot_b:
    ...
```

### URDF

The **urdf** field refers to a specially formatted robot description file which defines the joint configurations and the
hardware interface driver needed to communicate with the robot.

A hardware interface can be linked to URDF file in one of the following ways:

- By name of the custom URDF uploaded to the AICA database
- By name of an example URDF included in the AICA image (available examples depend on license and distribution versions)
- By the path of a URDF file mounted in the container filesystem
- By URDF string content inserted directly in the YAML (not recommended for large files)

```yaml
# referring to a custom robot description uploaded to the user database
robot_a:
  urdf: My custom robot

# referring to a built-in robot description from the included examples
robot_b:
  urdf: Universal Robots 5e (default configuration)

# using the path to a URDF file mounted in the container filesystem
robot_c:
  urdf: /home/ros2/my_robot.urdf

# defining the URDF content in-line
robot_d:
  urdf: |
    <robot name="example">
        <ros2_control name="ExampleRobotHardwareInterface" type="system">
            <hardware>
                <plugin>robot_interface/GenericInterface</plugin>
            </hardware>
            ...
        </ros2_control>
        ...
    </robot>
```

:::info

Use the Hardware tab in the Developer Interface to manage available URDFs.

Alternatively, use the API endpoints at `/v1/data/hardware` and `/v1/examples/hardware` to manage custom hardware and
view the available built-in example URDFs, respectively.

:::

### Rate

The **rate** defines the robot control frequency in Hz.

### Display name

This optional field can be used to give the hardware interface a more human-readable name (one that doesn't have
to conform to the lower_snake_case naming convention of the YAML syntax). It is only used when rendering the
hardware interface as a node in the AICA interactive graph editor. If omitted, the name is taken directly from the
YAML field (from the previous example, it would default to `robot_a`).

### Controllers

Controllers are the interface between components in the application and hardware in the real world. They convert desired
reference signals into real joint commands according to some internal control law, and convert joint states from the
robot back to signals.

Controllers are listed under a top-level field called **controllers**. Controller names must be unique within the given
hardware interface, and should generally be in `lower_camel_case`.

Under each controller, the **plugin** field refers to a registered controller plugin name.

The **parameters** field then refers to configurable parameters for the given controller.

The **inputs** and **outputs** fields define the ROS2 topics to which each signal of the controller should be connected.
See also [Component Inputs and Outputs](#inputs-and-outputs).

Optionally, the **position** field can be used to specify an X, Y location for rendering the hardware interface
as a node in the AICA interactive graph editor. See also [Component Position](#position).

For example:

```yaml
robot:
  controllers:
    broadcaster:
      plugin: joint_state_broadcaster/JointStateBroadcaster
    twist_controller:
      plugin: "compliant_twist_controller/CompliantTwistController"
      parameters:
        linear_principle_damping: 10.0
        linear_orthogonal_damping: 10.0
        angular_stiffness: 1.0
        angular_damping: { a: 1.0, b: true }
      inputs:
        command: /motion_generator/command_output
      outputs:
        state: /recorder/state_input
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
    component: ...    # required
    display_name: ... # optional
    position: ...     # optional
    log_level: ...    # optional
    mapping: ...      # optional
    parameters: ...   # optional
    inputs: ...       # optional
    outputs: ...      # optional
    events: ...       # optional

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

### Display name

This optional field is identical to the [hardware interface display name](#display-name) and is used to assign a
nicer, human-readable display name to the component when rendered as a node in the AICA interactive graph editor.

### Position

The `position` field is used to define the desired location of the component when rendered as a node in the AICA
interactive graph editor. It has two subfields defining the X and Y location, respectively.

This field only affects visualization of the application graph and has no other run-time effect.
If a position is not specified, the node will be rendered at a procedurally chosen location.

```yaml
my_component:
  position:
    x: 100
    y: 200
```

### Log Level

The `log_level` optionally sets the log severity level for this component.
Supported levels are: [unset, debug, info, warn, error, fatal]

```yaml
my_component:
  log_level: debug
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

### Inputs and Outputs

The `inputs` and `outputs` fields are used to connect component signals together to enable communication, signal
processing and control loops. Each signal is specified using a `name: value` syntax, where the name is the name
of the signal according to the component description, and the value is and the name of the signal topic.
If a component output is assigned to the same topic name as another component input, they are connected, as
illustrated in the example below.

```yaml
my_component:
  inputs:
    robot_state: "/state"
    applied_force: "/force"
  outputs:
    robot_command: "/command"

my_other_component:
  outputs:
    force_torque_sensor: "/force"
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

<!-- TODO: define the syntax for each event in more detail -->

Events drive the emergent behaviour of an application. Define events to be triggered by a predicate by listing them
under the predicate name, as shown below.

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

Read more about [events in the Concepts guide](../concepts/05-building-blocks/02-events.md).

The following events are defined.

#### Load or unload a component

Components can be loaded or unloaded by component name.

```yaml
load: <component_name>
unload: <component_name>
```

It is possible to load or unload multiple components simultaneously by specifying a list of names.

```yaml
load:
  - component_a
  - component_b 
```

#### Transition from one component to another

Component A can invoke a transition to component B as a shorthand for "unload component A, load component B".

```yaml
transition: <component_name>
```

#### Trigger a lifecycle transition


```yaml
# 
  lifecycle: "configure"
```

Request a lifecycle transition on the component that is triggering the event, using one of the available transitions
(`configure`, `activate`, `deactivate`, `cleanup`, or `shutdown`).
```yaml
lifecycle: activate
```

Request a lifecycle transition on a different component.
```yaml
lifecycle:
  transition: activate
  component: <component_name>
```

Use a list to trigger multiple transitions from a single predicate.
```yaml
lifecycle:
  - transition: activate
    component: <component_name>
  - transition: deactivate
    component: <component_name>
```

#### Set a parameter

Set a parameter on the component that is triggering the event.
```yaml
set:
  parameter: <parameter_name>
  value: <parameter_value>
```

Set a parameter on a different component.
```yaml
set:
  parameter: <parameter_name>
  value: <parameter_value>
  component: <component_name>
```

Set a parameter on the controller of a particular hardware interface.
```yaml
set:
  parameter: <parameter_name>
  value: <parameter_value>
  controller: <controller_name>
  interface: <hardware_interface_name>
```

#### Call a service

Call a service with no payload on the component that is triggering the event.
```yaml
service: <service_name>
```

Call a service on a different component.
```yaml
service:
  name: <service_name>
  component: <component_name>
```

Call a service with a string payload.
```yaml
service:
  name: <service_name>
  component: <component_name>
  payload: "..."
```

The service payload can also be written as any standard YAML object. The application parser will automatically encode
the object into a string format when making the service call. In this case, the component service is responsible
for parsing the string back into a YAML object, dict or structure as necessary.

```yaml
service:
  name: <service_name>
  component: <component_name>
  payload:
    foo: "some content"
    bar: [ x, y, z ]
    baz:
      a: 1
      b: 2
```

#### Load or unload a hardware interface

Load and initialize a hardware interface.
```yaml
load_hardware: <hardware_interface_name>
```

Unload and destroy a hardware interface.
```yaml
unload_hardware: <hardware_interface_name>
```

:::caution

All hardware interfaces in the application are automatically loaded and initialized when the application starts.

This behavior may change in the near future.

:::

#### Load or unload a controller

```yaml
load_controller:
  interface: <hardware_interface_name>
  controller: <controller_name>

unload_controller:
  interface: <hardware_interface_name>
  controller: <controller_name>
```

Use a list to load or unload multiple controllers from a single predicate.
```yaml
load_controller:
  - interface: <hardware_interface_name>
    controller: controller_a
  - interface: <hardware_interface_name>
    controller: controller_b
```

#### Start or stop a controller

Use the `switch_controllers` event to list the controllers to be started or stopped for a specific hardware interface.

```yaml
switch_controllers:
  interface: <hardware_interface_name>
  start: [ <controller_name>, <controller_name> ]
  stop: [ "controller_three", "controller_four" ] 
```

:::note

A controller must be loaded before it can be started, and must be stopped before it can be unloaded.

:::

### Special event predicates

#### on_load

The `on_load` predicate is provided by the state engine and set to true after the component
has been loaded. Any events associated with the `on_load` predicate are handled once
on instantiation of the node.

```yaml
component:
  events:
    on_load:
      <some triggered event>: ...
```

#### on_unload

The `on_unload` predicate is similar to the `on_load` predicate and is provided by the state engine.
Any events associated with the `on_unload` predicate are handled once upon destruction of the component interface.

```yaml
component:
  events:
    on_unload:
      <some triggered event>: ...
```

## Validating a YAML Application

<!-- FIXME: link to the schema on GitHub once it is on main; relative paths will break if the doc is versioned -->
The [YAML application schema](../../../schemas/applications/schema/application.schema.json) defines the structural rules
of an AICA application and effectively distinguishes between valid and invalid syntax.

Many modern IDEs and code editors can be configured to support custom schemas and provide in-line validation and
completion of the YAML content.
