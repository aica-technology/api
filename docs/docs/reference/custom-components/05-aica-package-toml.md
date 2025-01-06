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
#syntax=ghcr.io/aica-technology/package-builder:v1
```

You can find the available
versions [here](https://github.com/orgs/aica-technology/packages/container/package/package-builder).

The upcoming sections describe the syntax for the `package-builder` version `1.0.0` and later. If you are migrating from an older version, please refer to the corresponding guide:

- [Migrating from `0.0.X`](/docs/reference/custom-components/aica-package-migrations#migrating-from-0-0-x)

### `[build]`

#### `[build.type]`

Required. The only `[build.type]` currently supported is `ros`.

```toml title="aica-package.toml"
[build]
type = "ros"
```

#### `[build.ssh]`

Optional. Default is `false`. If set to `true`, any call to `CMake`, `pip` or custom stage will have access to the SSH credentials given to Docker with `--ssh default`. This is useful if you need to clone a private repository in CMakeLists.txt or install a private Python package from git.

```toml title="aica-package.toml"
[build]
ssh = false
```

#### `[build.image]`

Required. `[build.image]` is the tag of the AICA `ghcr.io/aica-technology/ros2-ws` image that will be used to build the components. Those images are tagged after the versions of the ROS 2 distributions and are available [here](https://github.com/aica-technology/docker-images/pkgs/container/ros2-ws).

```toml title="aica-package.toml"
[build]
image = "v2.0.0-jazzy"
```

#### `[build.cmake-args]`

Optional. In this category, you can specify arguments which will be passed to CMake through `-DK1=V1 -DK2=V2 ...`

```toml title="aica-package.toml"
[build.cmake-args]
# here are a few examples
PACKAGE_NAME = "my_package"
USE_FEATURE_X = "ON"
```

#### `[build.apt-repos]`

Optional. This category allows you to add extra APT repositories to the image. This is useful if you need to install some packages that are not available in the default repositories, which is common for third-party packages.

Multiple syntaxes are supported:

##### Using a `deb` file

```toml title="aica-package.toml"
[build.apt-repos]
cuda = { deb-uri = 'https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/{{ if eq .Arch "amd64" }}x86_64{{else}}sbsa{{end}}/cuda-keyring_1.0-1_all.deb' }
```

Note that `{{ if eq .Arch "amd64" }}x86_64{{else}}sbsa{{end}}` is a template that will be replaced by `x86_64` if the architecture is `amd64` and `sbsa` otherwise. See [here](https://pkg.go.dev/text/template) for more information on the templating syntax.

##### Using a repository (with an optional keyring)

```toml title="aica-package.toml"
[build.apt-repos.librealsense]
type = "deb" # optional, defaults to "deb", can also be "deb-src"
uri = "https://librealsense.intel.com/Debian/apt-repo"
distribution = "jammy" # optional, will default to the current distribution
components = ["main"]
keyring = "https://librealsense.intel.com/Debian/librealsense.pgp" # optional
```

Note that in this case we use a subcategory to specify the property of this repository, but it can also be expressed using the `{}` syntax like the previous example:

```toml title="aica-package.toml"
[build.apt-repos]
librealsense = { type = "deb", uri = "https://librealsense.intel.com/Debian/apt-repo", distribution = "jammy", components = ["main"], keyring = "https://librealsense.intel.com/Debian/librealsense.pgp" }
```

#### `[build.dependencies]`

Optional. `[build.dependencies]` is used to specify the AICA libraries and ROS 2 packages that will be installed in the
image.

:::note
Those libraries and packages will not be available at runtime, only while building. If you need them for building _and_ running, add them to the `[build.packages.XYZ.dependencies]` section below instead.
Libraries that are only required at runtime, and not at build time, can be declared with `[build.run-dependencies]` instead.
:::

:::note
Those libraries and packages are built for specific versions of ROS 2, so make sure that the version you are using is compatible with your `[build.image]`.
:::

Components usually require the `control-libraries` library. You can find the available
versions [here](https://github.com/aica-technology/control-libraries/pkgs/container/control-libraries). 

Components usually also require the `modulo` package. You can find the available
versions [here](https://github.com/aica-technology/modulo/pkgs/container/modulo).

:::note
Libraries are built in a specific way to be compatible with the AICA packaging system. Custom libraries are not
available yet.
:::

:::note
Starting with version `1.0.0` of the `package-builder`, all libraries and packages need to have special metadata associated in their image. This is done automatically when building with newer versions of `package-builder`. This means you won't be able to use older versions of certain libraries and packages with newer versions of `package-builder`.
:::

```toml title="aica-package.toml"
[build.dependencies]
"@aica/foss/control-libraries" = "v9.0.1"
"@aica/foss/modulo" = "v5.1.0"
"@myorg/mypackage" = "docker-image://ghcr.io/myorg/mypackage:v1.0.0"
```

##### Adding version constraints

Because `package-builder` will use your dependencies in the metadata of the component, the version of the dependencies will be expected when the component is used. You can tweak _which_ exact version is expected by using version constraints.

```toml title="aica-package.toml"
[build.dependencies]
"@aica/foss/control-libraries" = {version = "v9.0.1", constraints = ">= 9.0.1"}
```

:::note
By default, the version constraint is derived from the given version. It will be `~> X.Y` where `X` and `Y` are the major and minor versions of the given version. This means that any version with the same major version and a minor version greater than or equal to the given minor version will be accepted.
:::

:::note
You can find more information about version constraints and their syntax [here](#version-constraints).
:::

##### Adding build-time dependencies

Sometimes you might want to add a library but not require it in the metadata of your component. We call those dependencies build-only dependencies. This might be useful when using a static library that is built into your component and thus is not needed at runtime.

```toml title="aica-package.toml"
[build.dependencies]
"@myorg/mylib" = {version = "v7.5.0", build-only = true}
```

:::note
You can also add version constraints to build-only dependencies.
:::

#### `[build.run-dependencies]`

Optional. This field is similar to `[build.dependencies]` but its content is not downloaded and added to the image during the build. Instead, the information contained there is used in the metadata of your component. When someone uses your component, they will be notified that they need to use a specific version of the added dependencies matching the one specified in this section.

```toml title="aica-package.toml"
[build.run-dependencies]
"@myorg/mypackage" = ">= 1.0.0"
```

:::note
The value expected for each entry is not a version but a version constraint. You can find more information about this concept and its syntax in the corresponding section [here](#version-constraints).
:::

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

By default, available packages are based on the Ubuntu version used by the specified version of ROS2. You can find the list of
packages [here](https://packages.ubuntu.com/) and the Ubuntu version used by the ROS2
distribution [here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html). See [this section](#buildapt-repos) to learn how to add extra APT repositories.

:::tip
You can either provide a version to be installed or `*` to install the latest available version.
:::

:::note
Packages specified in `<*depend>` in `package.xml` will be installed through `rosdep` automatically.
:::

```toml title="aica-package.toml"
[build.packages.my_component.dependencies.apt]
libopencv-dev = "*"
libyaml-cpp-dev = "2.55.1"
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
  the `[build.dependencies]` packages
- any other environment variables set by `colcon`'s `${WORKSPACE}/install/setup.bash`

```toml title="aica-package.toml"
[build.stages.list]
from = "development"
run = """
python3 ${WORKSPACE}/src/my_component/script.py
"""
```

### `[metadata]`

Required.

#### `[metadata.version]`

Required. Must be a [semver-compliant](https://semver.org/) version string.

```toml title="aica-package.toml"
[metadata]
version = "1.0.0"
```

#### `[metadata.description]`

Optional. A short description of the component.

```toml title="aica-package.toml"
[metadata]
description = "My awesome component"
```

#### `[metadata.collection]`

Required when using multiple `[build.packages]`, otherwise ignored. This must contain the names of the collection being built (the set of multiple components).

`[metadata.collection.name]` is required. It is the name of the collection.

`[metadata.collection.ros-name]` is optional. This must be specified if `[metadata.collection.name]` is not a valid ROS package name.

```toml title="aica-package.toml"
[metadata.collection]
name = "my-collection"
ros-name = "my_collection" # required because `my-collection` is not a valid ROS package name
```

#### Advanced usage

##### `docker-image://`

As documented above, you can use `docker-image://` to specify your own `[build.dependencies]` packages. However,
you can also use it anywhere you are giving the tag to a Docker image. This is useful if you want to use a custom base
image for example.

Some examples:

```toml title="aica-package.toml"
[build]
image = "docker-image://ghcr.io/myorg/myimage:v1.0.0"

# and/or

[build.dependencies]
"@aica/foss/control-libraries" = "docker-image://ghcr.io/myorg/myimage:v1.0.0"
```

##### `build-context://`

Similarly to `docker-image://`, you can use `build-context://` to specify a context given to `docker build`
with `--build-context` (see [here](https://docs.docker.com/engine/reference/commandline/buildx_build/#build-context)).
This can also be used in `[build.packages.XYZ.source]` to build a component from a folder outside your root context or
from another Docker image.

Some examples:

```toml title="aica-package.toml"
[build.dependencies]
"@aica/foss/control-libraries" = "build-context://cl"

# and/or

[build.packages.component]
source = "build-context://my_source"
```

##### Version constraints

Version constraints follow the syntax of the [Terraform version constraints](https://www.terraform.io/docs/language/expressions/version-constraints.html) which is similar to syntax used by NPM, yarn or pip. Here is a quick summary:

- Versions constraints are composed of one or more conditions separated by commas, e.g. `>= 1.0.0, < 2.0.0`.
- Each version specified must be a valid [semver](https://semver.org/) version, e.g. `1.0.0`.
- The following operators are supported:
  - `=` or no operator: allow only the exact version, cannot be combined with other conditions.
  - `!=`: exclude a specific version.
  - `>`, `>=`, `<`, `<=`: comparison against a specific version, allowing any version matching the operator. `>` allows newer version and `<` allows older version.
  - `~>`: allows only the right most number of the version to increase. This is useful to allow only patch or minor versions to increase, e.g. `~> 1.0` allows `1.1`, `1.2`, etc. but not `2.0` and `~> 1.0.0` allows `1.0.1`, `1.0.2`, etc. but not `1.1.0`.

### Examples

#### Basic setup

Run with:

```bash
docker build -f aica-package.toml -t my_component .
```

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v1

[build]
type = "ros"
image = "v2.0.0-jazzy"

[build.dependencies]
"@aica/foss/control-libraries" = "v9.0.1"
"@aica/foss/modulo" = "v5.1.0"

[build.packages.component]
source = "./custom_component_package"
```

#### With dependencies

Run with:

```bash
docker build -f aica-package.toml -t my_component .
```

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v1

[build]
type = "ros"
image = "v2.0.0-jazzy"

[build.dependencies]
"@aica/foss/control-libraries" = "v9.0.1"
"@aica/foss/modulo" = "v5.1.0"

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
#syntax=ghcr.io/aica-technology/package-builder:v1

[build]
type = "ros"
image = "docker-image://base"

[build.dependencies]
"@aica/foss/control-libraries" = "build-context://cl"
"@aica/foss/modulo" = "v5.1.0"

[build.packages.component]
source = "build-context://my_source"
```

#### Custom stage

```bash
docker build -f aica-package.toml -t my_component --target list .
```

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v1

[build]
type = "ros"
image = "v2.0.0-jazzy"

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
  e.g. `--build-arg config.build.image=jazzy`
- `--ssh`: this can be used to pass SSH credentials to Docker, e.g. `--ssh default`. You will also need to
  set `[build.ssh]` to `true` in `aica-package.toml`
- `--build-context`: this can be used to pass a context to Docker, e.g. `--build-context my_source=../my_folder`. You
  can then use `build-context://my_source` in `aica-package.toml` to build a component from a folder outside your root
  context or from another Docker image
- `--target`: this can be used to run a custom stage, e.g. `--target list`. You can then use `[build.stages.list]`
  in `aica-package.toml` to define the stage
- `--no-cache`: this can be used to force Docker to rebuild the image from scratch
- `--progress`: this can be used to change the progress output, e.g. `--progress plain` or `--progress auto`
- `--label`: this can be used to add metadata to the image, e.g. `--label my_label=my_value`
