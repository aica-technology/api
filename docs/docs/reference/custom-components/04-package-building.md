---
sidebar_position: 4
title: Building a component package
---

# Including a component package

To use custom components in AICA applications, the component package must be built and then included in the AICA image
workspace. The easiest way to do this is to use a `aica-package.toml` file.

## Creating a `aica-package.toml` file

A basic `aica-package.toml` file for a component package stored in the `custom_component_package` directory would look
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

[build.packages.component]
source = "./custom_component_package"
```

`aica-package.toml` takes care of installing any dependencies (Python or system libraries) that you require for
your component. It is also able to build multiple components together, so you can include all your components in a
single package.

See [this page](./05-aica-package-toml.md) for a full reference of the `aica-package.toml` file and its capabilities.

## Building

You can then build your component using the following command:

```bash
docker build -f aica-package.toml -t custom-component .
```

## Launching your custom component

In AICA Launcher, include the docker image path from the build step as a custom package in the system configuration.

Refer to
the [installation and launch](../../getting-started/03-installation-and-launch.md#configuring-the-aica-system-image)
section for more details.
