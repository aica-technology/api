---
sidebar_position: 1
title: Creating a controller package
---

# Creating a controller package

:::info

The creation of a controller package can be greatly simplified by using
our [Package Template](https://github.com/aica-technology/package-template). It comes with an interactive wizard that
will guide you through the first initialization of the package. The wizard supports initializing a package with both C++
and Python controllers, as well as, controllers.

A lot of the information in this page is also available in the README of the template repository.

:::

Custom controllers can be defined in ROS 2 packages and used alongside the standard library of AICA controllers in AICA Core. Controllers can be implemented **only** in C++. A controller package can contain multiple controllers at a time,
as long as they are properly added in `CMakeLists.txt` and `controller_plugins.xml`, both of which we will cover later
in this page.

## Package infrastructure

A minimal directory structure for an example package named `custom_controller_package` with C++ and Python controller
implementations is shown below.


```
custom_controller_package
|
├── controller_descriptions
│   └── custom_controller_package_my_controller.json
|
├── include
│   └── custom_controller_package
│       └── my_controller.hpp
|
├── src
|   └── my_controller.cpp
|
├── package.xml
├── controller_plugins.xml
└── CMakeLists.txt
```

The following sections describe the package contents in more detail.

### package.xml

The package manifest file defines the package metadata (name, version, description, maintainer and license) and package
dependencies. An example including minimal dependencies is given below.

```xml title="package.xml"
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_controller_package</name>
  <version>0.0.1</version>
  <description>An example package for custom controllers</description>
  <maintainer email="john@example.com">TODO</maintainer>
  <license>GPLv3</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>

  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>

  <depend>modulo_controllers</depend>

  <test_depend>ament_cmake_gmock</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

The package build instructions are defined in a `CMakeLists.txt` file.

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.15)
project(custom_controller_package)

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(control_libraries 9.2.0 CONFIG REQUIRED COMPONENTS state_representation robot_model controllers)

include(InstallAicaDescriptions)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/my_controller.cpp)

target_link_libraries(${PROJECT_NAME} ${control_libraries_LIBRARIES})

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install_aica_descriptions(./controller_descriptions ${CMAKE_INSTALL_PREFIX}/controller_descriptions)

ament_auto_package()
```

:::note

C++ controllers need to be added as library targets and exported with the `pluginlib_export_plugin_description_file` directive. See the example in the [next section](./custom-controller.md).

:::

### Source directories

Relative to the package root, controllers should have header files defined in an `include/<package_name>` directory and
source files defined in a `src` directory.

:::caution

Any changes to the standard directory structure must be reflected in the `CMakeLists.txt` file accordingly. Also, notice
that namespaces are important and part of `controller_plugins.xml`, hence, any changes in namespacing or the package
name need to be reflected there too.

:::

### Controller descriptions

To fully support custom controllers in AICA Core, each new controller should be fully described by a JSON file according
to the controller description schema.

<!-- TODO: link to the description schema page -->
