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

```toml
#syntax=ghcr.io/aica-technology/package-builder:v0.0.13

[build]
type = "ros"

[build.environment.aica]
image = "iron"

[build.environment.aica.libraries]
"@aica/foss/control-libraries" = "v7.3.0"

[build.environment.aica.ros]
"@aica/foss/modulo" = "v4.0.0"

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

:::tip

Refer to [this documentation page](../../getting-started/installation#configuring-a-runtime-image-with-add-on-packages)
to learn how to run an application with your new component.

:::
