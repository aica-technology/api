---
sidebar_position: 3
title: Installation and Launch
---

## Pre-requisites

AICA System software is distributed with Docker images and executed with Docker containers.

Before proceeding, [install Docker Engine](https://docs.docker.com/engine/install/) on the host machine.

:::info

For Ubuntu users, make sure to follow
the [post installation steps](https://docs.docker.com/engine/install/linux-postinstall/) to create the `docker` group
and add your user.

:::

## AICA Launcher

AICA Launcher is the easiest way to get up and running with the AICA System. It manages licensing, package distribution
and the Docker toolchain so that no command-line access is necessary. It also embeds AICA Studio: Edge directly inside
the launcher.

### Installation

Find the latest release on the AICA GitHub organization under https://github.com/aica-technology/api/releases, and
search for AICA Launcher (for
example, [AICA Launcher v1.0.0](https://github.com/aica-technology/api/tree/launcher/v1.0.0)).

Download the prebuilt launcher app based on your operating system and architecture. For example,
`aica-launcher-linux-amd64.zip` for Linux machines with an AMD processor or `aica-launcher-darwin-arm64.zip` for macOS
on Apple Silicon.

Unzip the file and extract the AICA Launcher executable to your machine, then launch it by double-clicking or
right-clicking to start the program.

:::tip

On macOS, you may be unable to open the AICA Launcher app because of default security settings.

```
“AICA Launcher” can’t be opened because Apple cannot check it for malicious software.
```

To bypass this, right-click on the app and choose Open, and then click Open on the dialog that appears.

:::

### Entering an AICA System License

When the AICA Launcher is started for the first time (or after explicitly logging out), you will be prompted to enter
your AICA System License. As an example, the online license key to enter might look similar to
`5614D1-3E7A6C-932DEB-8C4189-F6B0F2-V3`.

![aica-launcher-login](./assets/aica-launcher-login.png)

Once the license is copied into the text field, it will automatically check the validity of the license and log in if
successful. If the license is not recognized, check your internet connection
or [contact AICA support](mailto:support@aica.tech).

### Configuring the AICA System image

AICA Launcher provides direct access to AICA Core and Studio, and additionally simplifies installing add-on packages
from the registry or from custom SDK contributions.

A "configuration" refers to a specific version of AICA Core and the extra packages and versions that should be included.
Defining configurations helps to ensure repeatable builds and deployments, even if AICA Core or any of the referenced
packages are subsequently updated or changed.

The simplest configuration just specifies the AICA Core Version, without any packages. Official packages can be added
from a dropdown, with a version subselection for each package. [Custom packages](../reference/01-intro.md) can be built
using the SDK and then added to the configuration using a Docker image URI.

Advanced configuration options allow setting additional Docker parameters, including real-time permissions and volume
mounting for persistent storage or file-system interactions.

![aica-launcher-package](./assets/aica-launcher-package.png)

### Launching a configuration

Press the Launch button for any given configuration to start up the AICA System and enter AICA Studio. For the
first time launching a configuration, the AICA Core software image and any additional packages first need to be pulled
from the registry and bundled into the specific runtime image. Depending on network speeds, this may take a while. After
the image has been built, launching the configuration a second time will be much faster as the results are cached.

![aica-launcher-studio](./assets/aica-launcher-studio.png)

## Manual installation and launch

For advanced users or users that deal with headless machines, there exists the option to [manually perform the login,
build and launch steps](../reference/02-manual-installation-launch.md) that the AICA Launcher handles automatically.