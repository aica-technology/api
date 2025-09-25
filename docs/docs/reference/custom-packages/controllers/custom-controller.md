---
sidebar_position: 2
title: Writing a controller
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Writing a controller

The following sections show example code snippets to illustrate how custom controller classes can be implemented in
Python or C++.

## Inheritance and registration

Custom controller classes should derive from either the `modulo_controllers::RobotControllerInterface` such that access
to various modulo wrappers and helpers is granted. Alternatively, classes can directly inherit from ROS 2 control's
`controller_interface::ControllerInterface` if, for example, transitioning from a pure ROS 2 controller to an AICA-style
controller. However, it is highly recommended to use our open-source `modulo` base class to fully benefit from what AICA
has to offer.

Refer to the [modulo documentation](https://aica-technology.github.io/modulo/versions/main) for a comprehensive API
reference on controllers.

:::info
The `RobotControllerInterface` class extends the common `ControllerInterface` with additional functions that can be
overridden for customized behavior. These functions are `evaluate()` and the collection of state transition callbacks.

<details>
  <summary>Lifecycle state transition callbacks</summary>
  <ul>
    <li><code>add_interfaces()</code></li>
    <li><code>on_configure()</code></li>
    <li><code>on_activate()</code></li>
    <li><code>on_deactivate()</code></li>
    <li><code>evaluate()</code></li>
  </ul>
</details>
:::

Registration is the term for defining and exporting a unique class name for the controller so that it can be dynamically
loaded by ROS 2 control's `Controller Manager` (see more [here](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)).


```cpp title="include/custom_controller_package/MyController.hpp"
#pragma once

#include <modulo_controllers/RobotControllerInterface.hpp>

namespace custom_controller_package {

class MyController : public modulo_controllers::RobotControllerInterface {
public:
  MyController();

  CallbackReturn add_interfaces() override;
  CallbackReturn on_configure() override;
  CallbackReturn on_activate() override;
  CallbackReturn on_deactivate() override;

private:
  controller_interface::return_type evaluate(const rclcpp::Time& time, const std::chrono::nanoseconds& period) override;

  bool
  on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;
};
}  // namespace custom_controller_package
```

```cpp title="src/MyController.cpp"
#include "custom_controller_package/my_controller.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

using namespace state_representation;

namespace custom_controller_package {

MyController::MyController()
    : modulo_controllers::RobotControllerInterface(true, hardware_interface::HW_IF_POSITION) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MyController::add_interfaces() {
  auto ret = modulo_controllers::RobotControllerInterface::add_interfaces();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // declaration of parameters, interfaces, inputs, outputs, etc
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MyController::on_configure() {
  auto ret = modulo_controllers::RobotControllerInterface::on_configure();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // configuration steps before activation
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MyController::on_activate() {
  auto ret = modulo_controllers::RobotControllerInterface::on_activate();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // activation steps before running
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MyController::on_deactivate() {
  auto ret = modulo_controllers::RobotControllerInterface::on_deactivate();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // deactivation steps
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type MyController::evaluate(const rclcpp::Time&, const std::chrono::nanoseconds&) {
  // implementation of a control logic
  return controller_interface::return_type::OK;
}

bool MyController::on_validate_parameter_callback(const std::shared_ptr<ParameterInterface>& parameter) {
  // validation of parameters (if needed)
  return modulo_controllers::RobotControllerInterface::on_validate_parameter_callback(parameter);
}

}// namespace custom_controller_package

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(custom_controller_package::MyController, controller_interface::ControllerInterface)%
```

To register the controller, use the `PLUGINLIB_EXPORT_CLASS` macro in the source file. Then, register and describe the
controller in `controller_plugins.xml`:

```xml
<library path="custom_controller_package">
    <class name="custom_controller_package/MyController" type="custom_controller_package::MyController" base_class_type="controller_interface::ControllerInterface">
        <description>
            A description for your controller.
        </description>
    </class>
</library>
```

:::caution

When registering a controller, the class name must be defined in the namespace of the package, delimited by double
colons `::`. For example, the controller classes `MyController` in the package `custom_controller_package` should be
registered as `custom_controller_package::MyController`, so that the package can be inferred from the registered name.

:::

:::info

Notice that here we specify:

```xml
base_class_type="controller_interface::ControllerInterface"
```

such that ROS knows the root of the inheritance tree is compatible with what is expected. That is not affected by
whether you are inheriting from `modulo_controllers::RobotControllerInterface`.

:::


Finally, export the controller library in `CMakeLists.txt`:

```cmake title="CMakeLists.txt"
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)
```

## Adding parameters

A controller parameter can be added either as a class attribute or declared in-line.

Controller implementations use parameter objects and types as defined in the `state_representation` library.
These parameter objects are automatically mapped and bound to the corresponding ROS 2 parameter on the parameter
interface.


```cpp title="include/custom_controller_package/MyController.hpp"
class MyController : public modulo_controllers::Controller {
public:
  explicit MyController(const rclcpp::NodeOptions& options);

protected:
  std::shared_ptr<state_representation::Parameter<int>> parameter_a_;
};
```

```cpp title="src/MyController.cpp"

MyController::MyController()
    : modulo_controllers::RobotControllerInterface(true, hardware_interface::HW_IF_POSITION),
    parameter_a_(std::make_shared<state_representation::Parameter<int>>("A")) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MyController::add_interfaces() {
  auto ret = modulo_controllers::RobotControllerInterface::add_interfaces();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  //...

  // define a parameter as a class property, then add it to the controller
  this->add_parameter(this->parameter_a_, "Description of parameter A");

  // or, define and add a parameter in-line
  this->add_parameter(std::make_shared<state_representation::Parameter<double>>("B"), "Description of parameter B");

  //...

  return CallbackReturn::SUCCESS;
}
```

### Accessing parameter values

To access the parameter value in other parts of the controller implementation, use the `get_value()` method either on
the class attribute or on the parameter object returned by `get_parameter()`

```cpp
auto x = std::make_shared<state_representation::Parameter<int>>("X");  // no default value!
x->is_empty();  // evaluates to true
x->get_value();  // raises an EmptyStateException

auto y = std::make_shared<state_representation::Parameter<int>>("Y", 10);  // default value: 10
y->is_empty();  // evaluates to false
y->get_value();  // evaluates to 10
```

## Validating parameters

Whenever a controller parameter is changed, the special `on_validate_parameter_callback()` function is triggered.
Override this function to perform parameter validation logic.

If the function returns false, the incoming parameter change is rejected and the internal controller parameter is not
changed. If the function returns true, the incoming parameter change is accepted and applied to the controller parameter.

The validation function can freely mutate the incoming parameter value before returning true, for example to constrain
a numerical value between some upper and lower bounds.

```cpp title="include/custom_controller_package/MyController.hpp"
class MyController : public modulo_controllers::RobotControllerInterface {
// ...

protected:
  bool
  on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;
};
```

```cpp title="src/MyController.cpp"

bool MyController::on_validate_parameter_callback(const std::shared_ptr<ParameterInterface>& parameter) {
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
  return modulo_controllers::RobotControllerInterface::on_validate_parameter_callback(parameter);
}
```

:::tip

Use the logging interface provided by `modulo_controllers` wherever appropriate. This will forward log messages from
custom controllers to ROS logs and the user interface.

:::

## Adding signals

Controllers make it possible to bind data objects to publishers or subscribers with class attributes. For inputs, this
means that the associated input data object is automatically updated every time a new message is received. For outputs,
this means that internal changes to the output data object will automatically be updated in the publisher.

The binding logic is supported for a number of common message data types. To interface with non-standard messages
(for example, to communicate with ROS 2 nodes using custom message types outside the AICA System), it is possible to
define raw publishers and subscribers following standard ROS 2 conventions.

### Signal name

Controller signals are declared and managed with unique signal names. The signal name is used to determine the default
topic. For a signal added under the name `foo`, the default topic will be `~/foo`. Adding a signal also automatically
creates and associates a parameter with that signal with the parameter name `[signal_name]_topic`, which is used to
override the default topic name at runtime. For a signal name `foo`, the parameter associated to the signal topic will
be called `foo_topic`.

For these reasons, signal names must adhere to the following rules:

- Signal names must be unique for all inputs and outputs of the controller
- Signal names must be written in `lower_snake_case` (using only lowercase letters, numbers and underscores)
- Signal names cannot start with a number or underscore.

<!-- TODO: following needs to be updated -->
### Message types

The supported message types are defined in the `std_msgs` library and include:

- Boolean
- integer
- floating point numbers (doubles)
- floating point array
- string

For robotic applications, AICA controllers make it particularly easy to receive, manipulate and send joint and Cartesian
states using the `state_representation` and `clproto` message encoding libraries.

State objects can be bound to inputs or outputs and are sent in a special `EncodedState` message type.

<!-- todo: everything below this point needs to be restructured to better fit the way we do control loops -->

### Inputs

To add an input, bind a class attribute of a supported type to a subscriber using the `add_input` function.

The following example adds two inputs: one as an integer and one as joint positions.

```cpp title="include/custom_controller_package/MyController.hpp"
class MyController : public modulo_controllers::RobotControllerInterface {
// ...

private:
  std::shared_ptr<int> input_number_;
  std::shared_ptr<state_representation::JointPositions> input_positions_;
};
```

```cpp title="src/MyController.cpp"
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MyController::add_interfaces() {
  auto ret = modulo_controllers::RobotControllerInterface::add_interfaces();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // ...

  // the message type is inferred from the data type
  this->add_input("number", this->input_number_);
  this->add_input("pose", this->input_positions_);
}
```

The value of the class attribute will automatically be updated whenever a new message is received.

#### User callbacks

Sometimes it is useful to react to an incoming message by triggering custom callback behavior. For example,
a controller might want to keep track of the number of input messages it received.

```cpp title="include/custom_controller_package/MyController.hpp"
class MyController : public modulo_controllers::RobotControllerInterface {
public:
  MyController();

private:
  void number_callback();

  std::shared_ptr<int> input_number_;
  int number_of_samples_;
};
```

<!-- TODO: not sure our controller interface has this, check -->
```cpp title="src/MyController.cpp"
MyContorller::MyController()
    : modulo_controllers::RobotControllerInterface(true, hardware_interface::HW_IF_POSITION),
    input_number_(std::make_shared<int>(0)),
    input_positions_(std::make_shared<state_representation::CartesianPose>()) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MyController::add_interfaces() {
  auto ret = modulo_controllers::RobotControllerInterface::add_interfaces();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // ...

  // bind a custom callback function to be triggered by the input
  this->add_input("number", this->input_number_, [this]() {
      this->number_callback();
  });
}

void MyController::number_callback() {
  this->number_of_samples_++;
}
```

:::tip

The bound class attribute is updated _before_ the user callback is triggered. Accessing the class attribute within the
user callback function will always yield the latest value.

:::

### Outputs

To add an output, bind a class attribute of a supported type to a publisher using the `add_output` function.

The following example adds two outputs: one as a floating point number and one as a Cartesian pose.

```cpp title="include/custom_controller_package/MyController.hpp"
class MyController : public modulo_controllers::RobotControllerInterface {
public:
  MyController();

private:
  std::shared_ptr<double> output_number_;
  std::shared_ptr<state_representation::CartesianPose> output_pose_;
};
```

```cpp title="src/MyController.cpp"
MyController::MyController()
    : modulo_controllers::RobotControllerInterface(true, hardware_interface::HW_IF_POSITION),
    output_number_(std::make_shared<double>(3.14),
    output_pose_(std::make_shared<state_representation::CartesianPose>()) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MyController::add_interfaces() {
  auto ret = modulo_controllers::RobotControllerInterface::add_interfaces();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // ...

  // the message type is inferred from the data type
  this->add_output("number", this->output_number_);
  this->add_output("pose", this->output_pose_);
}
```

The value of the class attribute can then be freely changed in other parts of the controller implementation to update the
published value. For example, it may be changed in a periodic step function, or as a result of parameter validation, a
service call, an input subscription callback or a lifecycle transition.

<!-- TODO
## Adding services
-->

<!-- TODO
## Adding predicates
-->

## Adding periodic behavior

Every controller adheres to the rate set for its hardware interface. For example, a hardware interface with a rate of 10
Hertz will have controllers that update their state ten times per second. The evaluation of periodic behaviors is
referred to as the controller "evaluation".

```cpp title="include/custom_controller_package/MyController.hpp"
class MyController : public modulo_controllers::RobotControllerInterface {
public:
  MyController();

private:
  controller_interface::return_type evaluate(const rclcpp::Time& time, const std::chrono::nanoseconds& period) override;

  std::shared_ptr<double> random_number_;
};
```

```cpp title="src/MyController.cpp"
// ...

#include <random>

static double generate_random_number() {
    return static_cast<double>(rand()) / RAND_MAX;
}

MyController::MyController(const rclcpp::NodeOptions& options):
    modulo_controllers::Controller(options),
    random_number_(std::make_shared<double>(this->generate_random_number())) {
  // ...

  this->add_output("random_number", this->random_number_);
}

controller_interface::return_type MyController::evaluate(const rclcpp::Time&, const std::chrono::nanoseconds&) {
  *this->random_number_ = this->generate_random_number();
  return controller_interface::return_type::OK;
}
```