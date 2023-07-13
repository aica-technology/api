# JSON Schema for Controller Descriptions

AICA controllers are ros2 control plugins

Controller descriptions are similar to component descriptions in terms of inputs, outputs and parameters.
Refer to the relevant documentation in the component descriptions folder for a starting point.

### Plugin

The `plugin` property is similar to the `registration` property of a component, and defines the unique name under
which the controller plugin class is registered.

### Control type

The `control_type` property defines which joint command interface the controller will claim. If the control type is
omitted from a given controller description, the controller is assumed to claim no command interfaces.