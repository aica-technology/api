---
sidebar_position: 2
title: Writing a component
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Writing a component

The following sections show example code snippets to illustrate how custom component classes can be implemented in
Python or C++.

## Inheritance and registration

Custom component classes should derive from either the `Component` or `LifecycleComponent` class of the
`modulo_components` library. The following examples will use the `Component` class, but the usage for
`LifecycleComponent` is equivalent.

Refer to the [modulo documentation](https://aica-technology.github.io/modulo/versions/main) for a comprehensive API
reference on components.

<!-- TODO: link to Concepts page on Components vs LifecycleComponents -->

:::info
The `LifecycleComponent` class extends the common component interface with additional functions that can be overridden
for customized behavior. These functions are `on_step_callback()` and the collection of state transition callbacks.

<details>
  <summary>Lifecycle state transition callbacks</summary>
  <ul>
    <li><code>on_configure_callback()</code></li>
    <li><code>on_activate_callback()</code></li>
    <li><code>on_deactivate_callback()</code></li>
    <li><code>on_cleanup_callback()</code></li>
    <li><code>on_shutdown_callback()</code></li>
    <li><code>on_error_callback()</code></li>
  </ul>
</details>
:::

Registration is the term for defining and exporting a unique class name for the component so that it can be dynamically
loaded by the AICA state engine.

<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

```python title="py_component.py"
from modulo_components.component import Component

class PyComponent(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        # use the constructor to declare parameters, predicates, services, signals and periodic callback functions
```

To register the component, define an entry point under `python_components` in `setup.cfg` that refers to the class
implementation.

```toml title="setup.cfg"
[options.entry_points]
python_components =
    custom_component_package::PyComponent = custom_component_package.py_component:PyComponent
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp title="include/custom_component_package/CppComponent.hpp"
#pragma once

#include <modulo_components/Component.hpp>

namespace custom_component_package {
class CppComponent : public modulo_components::Component {
public:
  explicit CppComponent(const rclcpp::NodeOptions& options);
};
}  // namespace custom_component_package
```

```cpp title="src/CppComponent.cpp"
#include "custom_component_package/CppComponent.hpp"

namespace custom_component_package {
CppComponent::CppComponent(const rclcpp::NodeOptions& options) : modulo_components::Component(options) {
  // use the constructor to declare parameters, predicates, services, signals and periodic callback functions
}

} // namespace custom_component_package

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(custom_component_package::CppComponent)
```

To register the component, use the `RCLCPP_COMPONENTS_REGISTER_NODE` macro in the source file.
Then add, register and export the component library in `CMakeLists.txt`:

```cmake title="CMakeLists.txt"
ament_auto_add_library(custom_component SHARED ${PROJECT_SOURCE_DIR}/src/CppComponent.cpp)
rclcpp_components_register_nodes(custom_component "custom_component_package::CppComponent")

install(TARGETS custom_component
        LIBRARY DESTINATION lib)
```

</TabItem>
</Tabs>

:::caution

When registering a component, the class name must be defined in the namespace of the package, delimited by double
colons `::`. For example, the component classes `CppComponent` and `PyComponent` in the
package `custom_component_package` should be registered as `custom_component_package::CppComponent`
and `custom_component_package::PyComponent` respectively, so that the package can be inferred from the registered name.

:::

## Adding parameters

A component parameter can be added either as a class attribute or declared in-line.

Component implementations use parameter objects and types as defined in the `state_representation` library.
These parameter objects are automatically mapped and bound to the corresponding ROS 2 parameter on the parameter
interface.

<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

```python title="custom_component.py"
import state_representation as sr
```

```python title="custom_component.py"
    def __init__(self, node_name: str, *args, **kwargs):
        # ...
        
        # define a parameter as a class attribute, then add it to the component using the attribute name
        self._parameter_a = sr.Parameter("A", sr.ParameterType.INT)
        self.add_parameter("_parameter_a", "Description of parameter A")
        
        # or, define and add a parameter in-line
        self.add_parameter(sr.Parameter("B", sr.ParameterType.DOUBLE), "Description of parameter B")
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp title="include/custom_component_package/CppComponent.hpp"
class CppComponent : public modulo_components::Component {
public:
  explicit CppComponent(const rclcpp::NodeOptions& options);

protected:
  std::shared_ptr<state_representation::Parameter<int>> parameter_a_;
};
```

```cpp title="src/CppComponent.cpp"
CppComponent::CppComponent(const rclcpp::NodeOptions& options):
    modulo_components::Component(options),
    parameter_a_(std::make_shared<state_representation::Parameter<int>>("A")) {
  // ...
  
  // define a parameter as a class property, then add it to the component
  this->add_parameter(this->parameter_a_, "Description of parameter A");
  
  // or, define and add a parameter in-line
  this->add_parameter(std::make_shared<state_representation::Parameter<double>>("B"), "Description of parameter B");
}
```

</TabItem>
</Tabs>

### Accessing parameter values

To access the parameter value in other parts of the component implementation, use the `get_value()` method either on
the class attribute or on the parameter object returned by `get_parameter()`

<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

```python
self._parameter_a.get_value()

self.get_parameter("B").get_value()
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp
this->parameter_a_->get_value();

this->get_parameter("B")->get_value();
```

</TabItem>
</Tabs>

### Default value

If no default value is declared, the parameter object will be empty until set.

<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

```python
x = sr.Parameter("X", sr.ParameterType.INT)  # no default value!
x.is_empty()  # evaluates to True
x.get_value()  # raises an EmptyStateError

y = sr.Parameter("Y", 10, sr.ParameterType.INT)  # default value: 10
y.is_empty()  # evaluates to False
y.get_value()  # evaluates to 10
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp
auto x = std::make_shared<state_representation::Parameter<int>>("X");  // no default value!
x->is_empty();  // evaluates to true
x->get_value();  // raises an EmptyStateException

auto y = std::make_shared<state_representation::Parameter<int>>("Y", 10);  // default value: 10
y->is_empty();  // evaluates to false
y->get_value();  // evaluates to 10
```

</TabItem>
</Tabs>

## Validating parameters

Whenever a component parameter is changed, the special `on_validate_parameter_callback()` function is triggered.
Override this function to perform parameter validation logic.

If the function returns false, the incoming parameter change is rejected and the internal component parameter is not
changed. If the function returns true, the incoming parameter change is accepted and applied to the component parameter.

The validation function can freely mutate the incoming parameter value before returning true, for example to constrain
a numerical value between some upper and lower bounds.

<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

```python title="custom_component.py"
    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() == "A":
            if parameter.is_empty():
                self.get_logger().warn("Parameter A cannot be empty")
                return False
        elif parameter.get_name() == "B":
            if parameter and parameter.get_value() < 0.0:
                self.get_logger().warn(f"Parameter B cannot be negative ({parameter.get_value()}). "
                                        "Setting value to 0.0 instead")
                parameter.set_value(0.0)
        return True
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp title="include/custom_component_package/CppComponent.hpp"
class CppComponent : public modulo_components::Component {
// ...

protected:
  bool
  on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;
};
```

```cpp title="src/CppComponent.cpp"
bool CppComponent::on_validate_parameter_callback(
    const std::shared_ptr<ParameterInterface>& parameter
) {
  if (parameter->get_name() == "A") {
    if (parameter->is_empty()) {
      RCLCPP_WARN(this->get_logger(), "Parameter A cannot be empty");
      return false;
    }
  } else if (parameter->get_name() == "B") {
    if (parameter && parameter->get_value() < 0.0) {
      RCLCPP_WARN(this->get_logger(), "Parameter B cannot be negative (%f). Setting value to 0.0 instead",
                  parameter->get_value());
      parameter->set_value(0.0);
    }
  }
  return true;
}
```

</TabItem>
</Tabs>


:::tip

Use the logging interface provided by `modulo_components` wherever appropriate. This will forward log messages from
custom components to ROS logs and the user interface.

:::

## Adding signals

Components make it possible to bind data objects to publishers or subscribers with class attributes. For inputs, this
means that the associated input data object is automatically updated every time a new message is received. For outputs,
this means that internal changes to the output data object will automatically be updated in the publisher.

The binding logic is supported for a number of common message data types. To interface with non-standard messages
(for example, to communicate with ROS 2 nodes using custom message types outside the AICA System), it is possible to
define raw publishers and subscribers following standard ROS 2 conventions.

### Signal name

Component signals are declared and managed with unique signal names. The signal name is used to determine the default
topic. For a signal added under the name `foo`, the default topic will be `~/foo`. Adding a signal also automatically
creates and associates a parameter with that signal with the parameter name `[signal_name]_topic`, which is used to
override the default topic name at runtime. For a signal name `foo`, the parameter associated to the signal topic will
be called `foo_topic`.

For these reasons, signal names must adhere to the following rules:

- Signal names must be unique for all inputs and outputs of the component
- Signal names must be written in `lower_snake_case` (using only lowercase letters, numbers and underscores)
- Signal names cannot start with a number or underscore.

### Message types

The supported message types are defined in the `std_msgs` library and include:

- Boolean
- integer
- floating point numbers (doubles)
- floating point array
- string

For robotic applications, AICA components make it particularly easy to receive, manipulate and send joint and Cartesian
states using the `state_representation` and `clproto` message encoding libraries.

State objects can be bound to inputs or outputs and are sent in a special `EncodedState` message type.

:::note

In C++, the message type is automatically inferred from the data type when binding inputs or outputs.

Because Python is dynamically typed, the message type of the class attribute must be specified when binding inputs or
outputs. The specific message types are available from the following imports.

```python
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray, String

from modulo_core.encoded_state import EncodedState
```

:::

### Inputs

To add an input, bind a class attribute of a supported type to a subscriber using the `add_input` function.

The following example adds two inputs: one as an integer and one as joint positions.

<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

```python title="custom_component.py"
from std_msgs.msg import Int32

from modulo_core.encoded_state import EncodedState
from state_representation import JointPositions
```

```python title="custom_component.py"
    def __init__(self, node_name: str, *args, **kwargs):
        # ...
        
        # define input data objects as a class attribute
        self._input_number = 0
        self._input_positions = JointPositions()
        
        # bind the attribute to an output using the attribute name and message type
        self.add_input("number", "_input_number", Int32)
        self.add_input("positions", "_input_positions", EncodedState)
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp title="include/custom_component_package/CppComponent.hpp"
class CppComponent : public modulo_components::Component {
public:
  explicit CppComponent(const rclcpp::NodeOptions& options);

private:
  std::shared_ptr<int> input_number_;
  std::shared_ptr<state_representation::JointPositions> input_positions_;
};
```

```cpp title="src/CppComponent.cpp"
CppComponent::CppComponent(const rclcpp::NodeOptions& options):
    modulo_components::Component(options),
    input_number_(std::make_shared<int>(0)),
    input_positions_(std::make_shared<state_representation::CartesianPose>()) {
  // ...
  
  // the message type is inferred from the data type
  this->add_input("number", this->input_number_);
  this->add_input("pose", this->input_positions_);
}
```

</TabItem>
</Tabs>

The value of the class attribute will automatically be updated whenever a new message is received.

#### User callbacks

Sometimes it is useful to react to an incoming message by triggering custom callback behavior. For example,
a component might want to keep track of the number of input messages it received.

<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

```python title="custom_component.py"
    def __init__(self, node_name: str, *args, **kwargs):
        # ...
        
        self._input_number = 0
        self._number_of_samples = 0

        # bind a custom callback function to be triggered by the input
        self.add_input("number", "_input_number", Int32, user_callback=self._number_callback)
    
    def _number_callback():
        self._number_of_samples += 1
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp title="include/custom_component_package/CppComponent.hpp"
class CppComponent : public modulo_components::Component {
public:
  explicit CppComponent(const rclcpp::NodeOptions& options);

private:
  void number_callback();

  std::shared_ptr<int> input_number_;
  int number_of_samples_;
};
```

```cpp title="src/CppComponent.cpp"
CppComponent::CppComponent(const rclcpp::NodeOptions& options):
    modulo_components::Component(options),
    input_number_(std::make_shared<int>(0)),
    number_of_samples_(0) {
  // ...
  
  // bind a custom callback function to be triggered by the input
  this->add_input("number", this->input_number_, [this]() {
      this->number_callback();
  });
}

void CppComponent::number_callback() {
  this->number_of_samples_++;
}
```

</TabItem>
</Tabs>

:::tip

The bound class attribute is updated _before_ the user callback is triggered. Accessing the class attribute within the
user callback function will always yield the latest value.

:::

### Outputs

To add an output, bind a class attribute of a supported type to a publisher using the `add_output` function.

The following example adds two outputs: one as a floating point number and one as a Cartesian pose.

<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

:::note

When using the `EncodedState` type for an output, the specific underlying state type must also be defined using the
`MessageType` enumeration in the `clproto` library.

:::

```python title="custom_component.py"
from std_msgs.msg import Float64

from modulo_core.encoded_state import EncodedState

from clproto import MessageType
from state_representation import CartesianPose
```

```python title="custom_component.py"
    def __init__(self, node_name: str, *args, **kwargs):
        # ...
        
        # define output data objects as a class attribute
        self._output_number = 3.14
        self._output_pose = CartesianPose()
        
        # bind the attribute to an output using the attribute name and message type
        self.add_output("number", "_output_number", Float64)
        
        # for encoded states, further define the expected state type when binding the output
        self.add_output("pose", "_output_pose", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp title="include/custom_component_package/CppComponent.hpp"
class CppComponent : public modulo_components::Component {
public:
  explicit CppComponent(const rclcpp::NodeOptions& options);

private:
  std::shared_ptr<double> output_number_;
  std::shared_ptr<state_representation::CartesianPose> output_pose_;
};
```

```cpp title="src/CppComponent.cpp"
CppComponent::CppComponent(const rclcpp::NodeOptions& options):
    modulo_components::Component(options),
    output_number_(std::make_shared<double>(3.14),
    output_pose_(std::make_shared<state_representation::CartesianPose>()) {
  // ...
  
  // the message type is inferred from the data type
  this->add_output("number", this->output_number_);
  this->add_output("pose", this->output_pose_);
}
```

</TabItem>
</Tabs>

The value of the class attribute can then be freely changed in other parts of the component implementation to update the
published value. For example, it may be changed in a periodic step function, or as a result of parameter validation, a
service call, an input subscription callback or a lifecycle transition.

:::note

Empty states will not be published.

Lifecycle components publish outputs only when the component is in the `ACTIVE` lifecycle state.

:::

<!-- TODO
## Adding services
-->

<!-- TODO
## Adding predicates
-->

## Adding periodic behavior

Every component has a built-in `rate` parameter that defines the execution rate of periodic behaviors in Hertz. For
example, a component with a rate of 10 Hertz will execute periodic behaviors ten times per second. The evaluation
of periodic behaviors is referred to as the component "step".

:::note

Lifecycle components evaluate periodic callback functions only when the component is in the `ACTIVE` lifecycle state.

:::

Custom periodic behavior can be implemented by overriding the `on_step_callback` function. For example, a random number
generator component might run a pseudo-random function every step so that the published output is always updated.

:::tip

Custom periodic callback functions are always evaluated _before_ outputs are published. If a periodic callback function
makes any changes to the value of a data object that is bound to and output, the latest value will be published in the
same component step.

:::


<Tabs groupId="programming-language">
<TabItem value="python" label="Python">

```python title="custom_component.py"
from random import random

from std_msgs.msg import Float64
```

```python title="custom_component.py"
    def __init__(self, node_name: str, *args, **kwargs):
        # ...
        
        self._random_number = random()
        self.add_output("random_number", "_random_number", Float64)
    
    def on_step_callback():
        self._random_number = random()
```

</TabItem>
<TabItem value="c++" label="C++">

```cpp title="include/custom_component_package/CppComponent.hpp"
class CppComponent : public modulo_components::Component {
public:
  explicit CppComponent(const rclcpp::NodeOptions& options);

private:
  void on_step_callback() override;

  std::shared_ptr<double> random_number_;
};
```

```cpp title="src/CppComponent.cpp"
// ...

#include <random>

static double generate_random_number() {
    return static_cast<double>(rand()) / RAND_MAX;
}

CppComponent::CppComponent(const rclcpp::NodeOptions& options):
    modulo_components::Component(options),
    random_number_(std::make_shared<double>(this->generate_random_number())) {
  // ...
  
  this->add_output("random_number", this->random_number_);
}

void CppComponent::on_step_callback() {
  *this->random_number_ = this->generate_random_number();
}
```

</TabItem>
</Tabs>