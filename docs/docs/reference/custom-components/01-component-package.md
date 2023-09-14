---
sidebar_position: 1
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Creating a component package

Custom components can be defined in ROS 2 packages and used alongside the standard library of AICA components in the
AICA application framework. Components can be implemented in C++ or Python. A component package can contain multiple
components in either language.

## Package infrastructure

A minimal directory structure for an example package named `custom_component_package` with C++ and Python component
implementations is shown below.

```
custom_component_package
│
├── component_descriptions
│   ├── custom_component_package_cpp_component.json
│   └── custom_component_package_py_component.json
│
├── custom_component_package
│   └── py_component.py
│
├── include
│   └── custom_component_package
│       └── CppComponent.hpp
│
├── src
│   └── CppComponent.cpp
│
├── CMakeLists.txt
├── package.xml
└── setup.cfg
```

The following sections describe the package contents in more detail.

### package.xml

The package manifest file defines the package metadata (name, version, description, maintainer and license) and package
dependencies. An example including minimal dependencies is given below.

```xml title="package.xml"
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
    <version>0.0.1</version>
    <name>custom_component_package</name>
    <description>An example package for custom components</description>
    <maintainer email="john@example.com">John Doe</maintainer>
    <license>MIT</license>

    <buildtool_depend>ament_cmake_auto</buildtool_depend>
    <buildtool_depend>ament_cmake_python</buildtool_depend>

    <depend>modulo_components</depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

### CMakeLists.txt

The package build instructions are defined in a `CMakeLists.txt` file.

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.15)
project(custom_component_package)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

# highlight-next-line
### Register and install C++ Components here ###

# install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# export JSON component descriptions in the component_descriptions directory
install(DIRECTORY ./component_descriptions
        DESTINATION .)

ament_auto_package()
```

:::note

C++ components need to be added as library targets, registered with the `rclcpp_components_register_nodes` macro
and finally exported with the `install` directive. See the C++ example in the next section.

<!-- TODO: link to the next section -->

:::

### setup.cfg

If the package contains any Python components, they need to be registered in a separate `setup.cfg` file. Refer to
the Python example in the next section.

<!-- TODO: link to the next section -->

If a package contains no Python components, the `setup.cfg` file is not needed.

### Source directories

Relative to the package root, C++ components should have header files defined in an `include/<package_name>` directory
and source files defined in a `src` directory.

Python components should be defined in a subdirectory matching the package name.

:::caution

Any changes to the standard directory structure must be reflected in the `CMakeLists.txt` file accordingly.

:::

### Component descriptions

To fully support custom components in the AICA application framework, each new component should be fully described
by a JSON file according to the component description schema.

<!-- TODO: link to the description schema page -->
