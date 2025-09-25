---
sidebar_position: 4
title: Building a controller package
---

# Building a controller package

To use custom controllers in AICA applications, the controller package must be built and then included in the AICA image
workspace. The easiest way to do this is to use a `aica-package.toml` file.

## Creating a `aica-package.toml` file

A basic `aica-package.toml` file for a controller package stored in the `custom_controller_package` directory would look
roughly like this:

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/package-builder:v1

[metadata]
version = "0.0.1"

[build]
type = "ros"
image = "v2.0.0-jazzy"

[build.dependencies]
"@aica/foss/control-libraries" = "v9.0.1"
"@aica/foss/modulo" = "v5.1.0"

[build.packages.controller]
source = "./custom_controller_package"
```

`aica-package.toml` takes care of installing any dependencies (Python or system libraries) that you require for
your controller. It is also able to build multiple controllers together, so you can include all your controllers in a
single package.

See [this page](./aica-package-toml.md) for a full reference of the `aica-package.toml` file and its capabilities.

## Building

You can then build your controller using the following command:

```bash
docker build -f aica-package.toml -t custom-controller .
```

## Launching your custom controller

In AICA Launcher, include the docker image path from the build step as a custom package in the system configuration.

Refer to
the [installation and launch](../../getting-started/installation/installation-and-launch.md#configuring-the-aica-system-image)
section for more details.
