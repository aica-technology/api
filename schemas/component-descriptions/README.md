# JSON Schema for Component Descriptions

As components become more numerous and diverse, it is crucial to have a consistent means of describing individual
components and the interfaces they provide. A machine-readable component description should contain all necessary
information to procedurally generate documentation, frontend visualisations and backend services for any new component.

To facilitate this, a JSON schema defines the expected structure and contents of a component description.

The component description is intended to be fully comprehensive and cover the needs of both front and backend users.
For this reason, some low-level concepts from the backend framework are abstracted.

Components fundamentally use signals to send and receive data in the form of inputs and outputs. Components
have parameters with specific types and names that define their behaviour. Components can also provide services
and produce predicates, which are special "true / false" signals to indicate specific component states.
Finally, components can derive from two core classes, namely `modulo_components::Component` and
`modulo_components::LifecycleComponent`. The latter class provides additional lifecycle services, states, predicates
and behaviors.

The sections in this document cover these concepts in more detail.
The last section of this document describes some utility scripts which can be used to interact with the JSON schema,
and finally describes how to format and save the JSON component description file.

## Components

The top-level [component.schema.json](./schema/component.schema.json) defines the expected structure of a component
description. The required core properties are the component name, description, registration and inheritance, as shown
below:

```json
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
are implemented by ROS2 packages and registered under a unique class name. By convention, the class name should be
defined in the namespace of the package. For example, some component class `Foo` in the package `foo_component_package`
should be registered as `foo_component_package::Foo`, so that the package can be inferred from registered name.
Under this convention, it is sufficient to write only the class name in the registration description, as in

```json
{
  "registration": "foo_component_package::Foo"
}
```

In some cases, the component may be registered under a class name which does not include the source package. For such
components, it is necessary to specify the package and class as separate fields in the registration description:

```json
{
  "registration": {
    "package": "foo_component_package",
    "class": "my_foo_namespace"
  }
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
should also specify this under the `inherits` property. These core base classes should be identified and treated
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

The signals of a component should be described according to [signal.schema.json](./schema/signal.schema.json).

A "signal" is the generic term for data on a ROS2 topic. We use "input" to refer to subscriptions, and "output" to refer
to publications. This keeps the language concise and generic for a user who may not be familiar with ROS2 terminology. 

In the backend implementation, components must register their subscriptions and publications under named topics.
For lifecycle components, this happens in `on_configure`, and otherwise happens on construction.

In some cases, topic names can be "fixed" and immutable properties of the class. This is more often true for outputs
than inputs, but can occur in both cases.

However, for a component to be able to subscribe to a topic that another component is publishing at run-time,
the topic name must be configurable from the application either at the point of instantiation or configuration.
The way this is currently achieved is through string parameters that are created when a signal is added. The
name of the topic parameter is derived from the signal name in the format `<signal_name>_topic`.
The component gets the topic name from the topic parameter when registering the corresponding subscriber or publisher.

Similarly, a component must also declare the signal type when registering the subscriptions and publications during
configuration. For dynamically typed signals, parameters are once again employed in the implementation with parameter
name `<signal_name>_type`.

Dynamic typing should be used sparingly to avoid unnecessary complexity. In general, some signal types are implicitly
convertible; a `cartesian_state` output can be matched with a `cartesian_pose` input without additional work. In other
cases, such as for switching between joint-space and Cartesian-space signals, consider using two specialized component
classes deriving from a common base class.

In the frontend application graph, component parameters are displayed as editable fields on the component node.
But, because signals can be represented visually as edges (connections) between input and output nodes,
they should not also appear as editable "parameter" fields; this would bloat the apparent number of parameters of a
component with many signals and cause confusion for the user if there are multiple ways to configure signals.

For this reason, the implementation details of topic names as parameters are abstracted, and those parameters are
treated as "hidden" from the user. In the component description, the parameters `<signal_name>_type` and
`<signal_name>_topic` should not be listed.
In many cases, the literal topic name becomes irrelevant and could be automatically generated, as the
graph will visually show which components are connected and communicating on the corresponding signals. In practice,
the name of the topic between connected components should use the default topic name of the output as described
in the following section.

### Configurable signal topic names

The topic name of a signal is sometimes fixed and sometimes configurable. If the name is
configurable, then it must have an associated hidden parameter that sets the topic name before configuration. 

The following truth table shows the behaviour when connecting the output of one component with an input of another: 

| Output topic is configurable | Input topic is configurable | Result                                              |
|------------------------------|-----------------------------|-----------------------------------------------------|
| False                        | False                       | Compatible only if fixed topic names match          |
| True                         | False                       | Set output topic name from fixed input topic name   |
| False                        | True                        | Set input topic name from fixed output topic name   |
| True                         | True                        | Set input topic name from default output topic name |

The backend application interface is responsible for retrieving and setting the corresponding topic name parameter
from the application graph and component description.

### Configurable signal types

Similar to signal topic names, the signal type may also be configurable through hidden parameters, and has similar
behavior when resolving a new signal connection.

| Output type is configurable | Input type is configurable | Result                                  |
|-----------------------------|----------------------------|-----------------------------------------|
| False                       | False                      | Compatible only if fixed types match    |
| True                        | False                      | Set output type from fixed input type   |
| False                       | True                       | Set input type from fixed output type   |
| True                        | True                       | Set input type from default output type |

This has the extra caveat that the configurable signal type may only support a subset of all available types.
For example, a configurable component input might support JointState or CartesianState signals, but nothing else.
In this case, the `signal_types` property in the signal description should be used along with parameter validation
by the component itself.

### Signal Collection

Just as the topic name or type of a single may be configurable at run-time, so too can the number of input
signals on a component be configurable.

A signal collection is a dynamic grouping of input signals with compatible types. For example, a WeightedSum component
supports a variable number of inputs that are defined at the application level. It may combine the outputs from
two components A and B in one application, but combine five different signals in another.

In the backend application, the dynamically configurable component inputs (subscriptions) of one collection
are generated from a single `string_array` parameter with the name `<input_collection_name>_topics`.
The array of topic names in the parameter is used to register the corresponding subscriptions.

Input collections are treated as a separate property from normal inputs because the structure of the default topic name
and topic parameter use arrays instead of simple strings. In addition, the frontend will also treat
simple input signals and input signal collection differently in terms of visualization and interaction.

When a new signal is added to a signal collection in the frontend, the backend appends the new signal topic name
to the associated hidden array parameter.  

## Parameters

Component classes declare a fixed set of parameters by name and type. The value of parameters are consumed by the
component when it is instantiated, or, in the case of lifecycle components, when it is configured.

Parameters contain a value of a certain type, defined by the [parameter_type.schema.json](./schema/parameter_type.schema.json).
If the `parameter_type` property is `state`, then the `parameter_state_type` property must also be defined as a member
of the enumeration in [encoded_state_type.schema.json](./schema/encoded_state_type.schema.json).

Parameters generally have a default value which renders them optional in some cases. When a parameter has no valid
default state and must be set by the user for the component to function, the `default_value` property in the component
description should be set to `null`. This is distinct from a valid default empty parameter state, which can be
expressed with the empty string `""`.

If a parameter is dynamically reconfigurable, it implies that the parameter can be changed after the component
is already configured to influence the run-time behaviour.
This requires the component to either poll the parameter value while running or implement a parameter change callback.

## Predicates

Predicates are crucial to drive the event logic of the dynamic state engine, but they are very simple to declare
and describe. Each predicate of a component indicates a particular run-time state or configuration that is either
"true" or "false". In the implementation, a predicate is a publisher with a boolean type. The component is responsible
for determining the value of a predicate and publishing it under a particular topic name.

In the component description, predicates have a display name and description. The `predicate_name` property is used
to inform the state engine of the hidden topic name to listen to for that predicate.

## Services

Services are endpoints provided by a component that can trigger certain behaviours. In the backend implementation,
they are created as ROS2 services with a specific `service_name` using one of two service message types. The first
type is simply an empty trigger service that has no payload, which is the default case. The second type is a trigger
service with a string payload, which can be used to pass parameters to the service call.

The `payload_format` property is used to describe the expected format and usage of the string trigger payload;
it is thereby inferred that the service type is not an empty trigger when this property is present.

## Scripts

Two simple utility scripts are provided to view the schema and to validate JSON component descriptions.

To view the schema specifications in a nicer format, run the [`./serve-html.sh`](./serve-html.sh) script in this
directory. This will invoke a dockerized process in which a Python module `json-schema-for-humans` generate a static
HTML representation of the schema which is served on a localhost port for viewing in the browser.

To validate a component description, run the [`./validate.sh`](./validate.sh) script with a path to a JSON file.
The path can be relative or absolute. For example,
`./validate.sh examples/foo.json` or `./validate.sh /path/to/foo.json`.

Any invalidities in the component description, such as missing required fields, will be reported.

## Saving the component description

The component description JSON file should be saved in a `component_descriptions` directory of the package as a
`lower_snake_case` version of the registered class name.

For example, a component class in package `foo_package` registered as `foo_package::Foo` should be saved as follows:
```
foo_package/
    component_descriptions/
        foo_package_foo.json
    include/
        foo_package/
            Foo.hpp
    src/
        Foo.cpp
    ...
```

The component package `CMakeLists.txt` should include the following installation directive, which will install all
descriptions to `${WORKSPACE}/install/<package_name>/component_descriptions`.

```cmake
install(DIRECTORY ./component_descriptions
        DESTINATION .)
```

## CLion configuration for automatic validation

CLion supports JSON schemas to directly validate syntax and formatting in the IDE.
https://www.jetbrains.com/help/clion/json.html

To set up validation for the component description schema, follow the steps given in
[Configure a custom JSON schema](https://www.jetbrains.com/help/clion/json.html#6768673a).

For step 3, give the Schema a useful name (such as "Component Description") and choose JSON Schema version 7.
In the "Schema file or URL field", enter the following URL:
```
https://raw.githubusercontent.com/aica-technology/component-sdk/main/schema/schema/component.schema.json
```

For step 4, either manually specify files and directories to be validated, or use the file path pattern
`component_descriptions/*.json` to detect all component description in compatible packages.

CLion will then provide autocompletion for fields and types, and warn when required fields are missing.
This is very useful when generating new component descriptions.
However, CLion 2022 only supports up to JSON Schema version 7 and not the newest draft 2020-12,
which provides some additional complex validation logic. For this reason, certain conditional validation
will not be provided in the IDE. It is therefore recommended to use the [validation script](#scripts) for
a final check of the description validity.
