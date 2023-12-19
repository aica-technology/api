---
sidebar_position: 5
title: Package builder configuration reference
---

# `aica-package.toml` configuration reference

The `aica-package.toml` file is used to configure the build process for a component package.

## `TOML` syntax

The configuration uses the simple [TOML](https://toml.io/en/) file format which is easy to read and write. Here are a
few examples:

```toml
# add a category
[category]
# then you can add keys and values
key-string = "value string"
key-bool = true
key-int = 42
key-float = 3.14

# you can nest categories
[category.sub-category]
key = "value"
```

Note that each `string` value needs to be quoted, but keys and values of other types do not.

## `aica-package.toml` file structure

This example contains every possible key and value that can be used in an `aica-package.toml` file and explains each of
them.

### `#syntax`

Each `aica-package.toml` should start with `#syntax=ghcr.io/aica-technology/package-builder`. This is used to specify
the Docker syntax and version used to build the package.

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v0.0.9
```

You can find the available
versions [here](https://github.com/orgs/aica-technology/packages/container/package/package-builder).

### `[build]`

Required. The only `[build.type]` currently supported is `ros`.

```toml title="aica-package.toml"
[build]
type = "ros"
```

#### `[build.cmake_args]`

Optional. In this category, you can specify arguments which will be passed to CMake through `-DK1=V1 -DK2=V2 ...`

```toml title="aica-package.toml"
[build.cmake_args]
# here are a few examples
PACKAGE_NAME = "my_package"
USE_FEATURE_X = "ON"
```

#### `[build.environment]`

Required. This category describes the environment (ROS version, libraries, etc.) that will be used to build the
components.

`[build.environment.ssh]` is optional (defaults to `false`). If set to `true`, any call to `CMake`, `pip` or custom
stage will have access to the SSH credentials given to Docker with `--ssh default`. This is useful if you need to clone
a private repository in CMakeLists.txt or install a private Python package from git.

```toml title="aica-package.toml"
[build.environment]
ssh = false
```

`[build.environment.aica]` is required. `[build.environment.aica.image]` is the tag of the
AICA `ghcr.io/aica-technology/ros2-control` image that will be used to build the components. Those images are tagged
after the ROS2 distributions and are
available [here](https://github.com/aica-technology/docker-images/pkgs/container/ros2-control).

```toml title="aica-package.toml"
[build.environment.aica]
image = "iron"
```

##### `[build.environment.aica.libraries]`

Optional. `[build.environment.aica.libraries]` is used to specify the AICA libraries that will be installed in the
image.

:::note
Those libraries will not be available at runtime, only while building. If you need them for building _and_ running, add
them to the `[build.packages.XYZ.dependencies]` section below instead.
:::

:::note
Those libraries are built for specific versions of ROS2, so make sure that the version you are using is compatible with
your `[build.environment.aica.image]`.
:::

Components usually require the `control-libraries` library. You can find the available
versions [here](https://github.com/aica-technology/control-libraries/pkgs/container/control-libraries). If you are
building a hardware interface, you might want to use the `network-interfaces` library; you can find the available
versions [here](https://github.com/aica-technology/network-interfaces/pkgs/container/network-interfaces).

:::note
Those libraries are built in a specific way to be compatible with the AICA packaging system. Custom libraries are not
available yet.
:::

```toml title="aica-package.toml"
[build.environment.aica.libraries]
"@aica/foss/control-libraries" = "v7.2.0"
"@aica/foss/network-interfaces" = "v2.0.1"
```

##### `[build.environment.aica.ros]`

Optional. `[build.environment.aica.ros]` is used to specify the ROS2 packages that will be installed in the image.

:::note
Those packages will not be available at runtime, only while building.
:::

:::note
Those packages are built for specific versions of ROS2, so make sure that the version you are using is compatible with
your `[build.environment.aica.image]`.
:::

Components usually require the `modulo` package. You can find the available
versions [here](https://github.com/aica-technology/modulo/pkgs/container/modulo).

You can also use any other ROS2 package that has been built with `aica-package.toml`. For example, if you had a
component built with a tag of `ghcr.io/myorg/mypackage:v1.0.0`, you could use it like this:

```toml title="aica-package.toml"
[build.environment.aica.ros]
"@aica/foss/modulo" = "v3.2.0"
"@myorg/mypackage" = "docker-image://ghcr.io/myorg/mypackage:v1.0.0"
```

#### `[build.packages]`

Required. This category describes the different components that will be built.

The name of the component doesn't need to match the one set in `package.xml`; it is only used during the build.
`[build.packages.XYZ.source]` is required and defines the path to the component source code relative to the Docker
context (usually the root of the repository).

```toml title="aica-package.toml"
[build.packages.my_component]
source = "my_component"
```

You can add as many components as you want, e.g.

```toml title="aica-package.toml"
[build.packages.another]
source = "another"

[build.packages.evenmore]
source = "different-folder/event-more"
```

##### `[build.packages.XYZ.dependencies.apt]`

Optional. This category allows you to specify the APT packages that will be installed in the image. This is useful if
you need to install some system dependencies.

Available packages are based on the Ubuntu version used by the specified version of ROS2. You can find the list of
packages [here](https://packages.ubuntu.com/) and the Ubuntu version used by the ROS2
distribution [here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).

:::note
`*` is the only supported version for now. It will install the latest version of the package.
:::

:::note
Packages specified in `<*depend>` in `package.xml` will be installed through `rosdep` automatically.
:::

```toml title="aica-package.toml"
[build.packages.my_component.dependencies.apt]
libopencv-dev = "*"
libyaml-cpp-dev = "*"
```

##### `[build.packages.XYZ.dependencies.pip]`

Optional. This category allows you to specify the Python packages that will be installed in the image.

:::info
By default, `aica-package.toml` will check if your `[build.packages.XYZ.source]` folder contains a `requirements.txt`
file and install the packages specified inside.

You can override the name of the file by using the `[build.packages.XYZ.dependencies.pip.file]` key.
:::

:::warning
The `[build.packages.XYZ.dependencies.pip.file]` key and the `[build.packages.XYZ.dependencies.pip.packages]` list are
mutually exclusive; you can only use one of them.
:::

:::info
`[build.packages.XYZ.dependencies.pip.packages]` roughly follows the format
of `pip`: https://pip.pypa.io/en/stable/reference/requirements-file-format/
:::

```toml title="aica-package.toml"
[build.packages.my_component.dependencies.pip]
# e.g. if your requirements file is called `aica-requirements.txt`
file = "aica-requirements.txt"

# OR
[build.packages.my_component.dependencies.pip.packages]
numpy = "2.0.0" # equivalent to `numpy==2.0.0`
opencv-python = "\*" # equivalent to `opencv-python`
scipy = ">=1.0.0" # equivalent to `scipy>=1.0.0`
```

#### `[build.stages]`

Optional. This category allows you to define custom stages that can be used with `docker build --target`. These are
useful if you want to run some extra commands during or at the end of the build process or if you need a custom output.

Here is an example that will call some Python code during the build process. The name of the stage, `list`, will be used
with `docker build --target list` to run this stage.

`[build.stages.XYZ.from]` is required; it is the stage of the build process on top of which this stage will run. It can
be any of:

- `base`: only the environment has been applied
- `code`: based on `base`, the code has been copied)
- `development`: based on `code`, runnable image setup for debugging with dependencies installed
- `build`: based on `code`, the component has been built and the dependencies installed
- `test`: based on `build`, the component has been tested
- `production`: scratch image containing just to built component

`[build.stages.XYZ.run]` is required. It is the command that will be run in the stage. The command will be run as a
non-root user with the `colcon` workspace setup and the current working directory will be the `colcon` workspace. The
following environment variables should be available:

- `WORKSPACE`: the path to the `colcon` workspace, which is also the current working directory
- `USER`: the name of the non-root user
- `HOME`: the path to the home directory of the non-root user
- `ROS_DISTRO`: the ROS2 distribution used to build the component
- `ROS2_WORKSPACE`: the path to the workspace where the standard ROS2 packages are installed as well as
  the `[build.environment.aica.ros]` packages
- any other environment variables set by `colcon`'s `${WORKSPACE}/install/setup.bash`

```toml title="aica-package.toml"
[build.stages.list]
from = "development"
run = """
python3 ${WORKSPACE}/src/my_component/script.py
"""
```

#### Advanced usage

##### `docker-image://`

As documented above, you can use `docker-image://` to specify your own `[build.environment.aica.ros]` packages. However,
you can also use it anywhere you are giving the tag to a Docker image. This is useful if you want to use a custom base
image for example.

Some examples:

```toml title="aica-package.toml"
[build.environment.aica]
image = "docker-image://ghcr.io/myorg/myimage:v1.0.0"

# and/or

[build.environment.aica.libraries]
"@aica/foss/control-libraries" = "docker-image://ghcr.io/myorg/myimage:v1.0.0"
```

##### `build-context://`

Similarly to `docker-image://`, you can use `build-context://` to specify a context given to `docker build`
with `--build-context` (see [here](https://docs.docker.com/engine/reference/commandline/buildx_build/#build-context)).
This can also be used in `[build.packages.XYZ.source]` to build a component from a folder outside your root context or
from another Docker image.

Some examples:

```toml title="aica-package.toml"
[build.environment.aica.libraries]
"@aica/foss/control-libraries" = "build-context://cl"

# and/or

[build.packages.component]
source = "build-context://my_source"
```

### Examples

#### Basic setup

Run with:

```bash
docker build -f aica-package.toml -t my_component .
```

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v0.0.9

[build]
type = "ros"

[build.environment.aica]
image = "iron"

[build.environment.aica.libraries]
"@aica/foss/control-libraries" = "v7.2.0"

[build.environment.aica.ros]
"@aica/foss/modulo" = "v3.2.0"

[build.packages.component]
source = "./custom_component_package"
```

#### With dependencies

Run with:

```bash
docker build -f aica-package.toml -t my_component .
```

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v0.0.9

[build]
type = "ros"

[build.environment.aica]
image = "iron"

[build.environment.aica.libraries]
"@aica/foss/control-libraries" = "v7.2.0"

[build.environment.aica.ros]
"@aica/foss/modulo" = "v3.2.0"

[build.packages.component]
source = "./custom_component_package"

# `requirements.txt` will be used by default

[build.packages.component.dependencies.apt]
libopencv-dev = "*"
```

#### Advanced usage

Run with:

```bash
docker build -f aica-package.toml \
  --build-context my_source=../my_folder \
  --build-context base=ghcr.io/myorg/myimage:v1.0.0 \
  --build-context cl=ghcr.io/myorg/myimage:v2.0.0 \
  -t my_component \
  .
```

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v0.0.9

[build]
type = "ros"

[build.environment.aica]
image = "docker-image://base"

[build.environment.aica.libraries]
"@aica/foss/control-libraries" = "build-context://cl"

[build.environment.aica.ros]
"@aica/foss/modulo" = "v3.2.0"

[build.packages.component]
source = "build-context://my_source"
```

#### Custom stage

```bash
docker build -f aica-package.toml -t my_component --target list .
```

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v0.0.9

[build]
type = "ros"

[build.environment.aica]
image = "iron"

[build.packages.component]

[build.stages.list]
from = "development"
run = """
python3 ${WORKSPACE}/src/my_component/script.py
"""
```

## `docker build` usage

We use a custom Docker syntax, which allows us to define the configuration through `aica-package.toml` instead of
a `Dockerfile`. However, we are still using `docker` directly to build. This means we have access to all the
options `docker build` provides but also that you don't need to install or learn anything new to build your component.

Some useful options:

- `-t` or `--tag`: tag the image with a name and an optional tag, e.g. `-t my_component` or `-t my_component:latest`
- `--platform`: build for a specific platform, e.g. `--platform linux/amd64` or `--platform linux/arm64`
- `--build-arg`: this can be used to override any key in `aica-package.toml`,
  e.g. `--build-arg config.build.environment.aica.image=jazzy`
- `--ssh`: this can be used to pass SSH credentials to Docker, e.g. `--ssh default`. You will also need to
  set `[build.environment.ssh]` to `true` in `aica-package.toml`
- `--build-context`: this can be used to pass a context to Docker, e.g. `--build-context my_source=../my_folder`. You
  can then use `build-context://my_source` in `aica-package.toml` to build a component from a folder outside your root
  context or from another Docker image
- `--target`: this can be used to run a custom stage, e.g. `--target list`. You can then use `[build.stages.list]`
  in `aica-package.toml` to define the stage
- `--no-cache`: this can be used to force Docker to rebuild the image from scratch
- `--progress`: this can be used to change the progress output, e.g. `--progress plain` or `--progress auto`
