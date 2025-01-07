---
sidebar_position: 1
title: Manual Installation and Launch
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

The following sections explain how to install and launch AICA Core and any additional packages manually from the command
line without the use of AICA Launcher. The pre-requisites are still a valid license and a host with Docker installed.
For the rest of this guide, it will be assumed that a valid license has been saved to a file called `aica-license.toml`
on the host machine.

## Logging in to the AICA package registry

To authenticate docker to login and pull images from the registry, run the following command (no replacement of
`USERNAME` required):

```shell
cat aica-license.toml | docker login registry.licensing.aica.tech -u USERNAME --password-stdin
```

## Configuring AICA packages with a manifest file

A runtime application image is configured using a simple **manifest file** defining the version of AICA Core to use and
optionally defining additional add-on packages. The manifest file contains a custom docker syntax header pointing to
AICA's app-builder tool, and the `docker build` command is used to bundle all listed packages into a final runtime
image.

### Configuring a minimal runtime image with a version of AICA Core

The manifest file must contain a syntax header and a list of packages. The minimal version of the manifest includes
only AICA Core as the `core` package. The version can be changed according to the available releases.

:::info

In the past, you might have seen applications using the `aica-package.toml` filename. While you can use any filename as
we do not enforce any, we recommend using `aica-application.toml` to avoid confusion with the `aica-package.toml` file
which is used for building packages using `package-builder`.

:::

```toml title="aica-application.toml"
#syntax=ghcr.io/aica-technology/app-builder:v2

[core]
image = "v4.2.0"
```

### Configuring a runtime image with add-on packages

A manifest can include additional components and hardware collections as add-on packages. For any available package
listed in the AICA registry, specify the package and version with the `@aica/` prefix. The following example manifest
file includes two add-on packages: version 2.0.0 of the `components/rl_policy_components` component package and version
4.1.0 of the `collections/ur-collection` hardware collection package.

:::note

Starting with version `2.0.0` of the `app-builder`, all packages need to have special metadata associated in their
image. This is done automatically when building with newer versions of `app-builder`. This means you won't be able to
use older versions of certain libraries and packages with newer versions of `app-builder`.

:::

```toml title="aica-application.toml"
#syntax=ghcr.io/aica-technology/app-builder:v2

[core]
"image" = "v4.2.0"

[packages]
# add components
"@aica/components/rl-policy-components" = "v2.0.0"

# add hardware collections
"@aica/collections/ur-collection" = "v4.1.0"
```

### Including custom packages

The AICA framework allows developers to build their own
[custom components](../reference/custom-components/01-component-package.md). These packages can be included under a
custom name using the `docker-image://` prefix to specify the docker image name or path. For example, a custom component
package that was locally built using `docker build [...] --tag my-custom-component-package` could be included as
`docker-image://my-custom-component-package`. Community and third-party packages may also be available on other docker
registries such as DockerHub or GitHub Container Registry and can be included with the associated docker path.

```toml title="aica-application.toml"
#syntax=ghcr.io/aica-technology/app-builder:v2

[core]
"image" = "v4.2.0"

[packages]
# add a custom package from a local docker image path
"my-local-package" = "docker-image://my-custom-component-package"

# add a package from any docker path such as GitHub Container Registry
"my-ghcr-package" = "docker-image://ghcr.io/user/package:tag"
```

## Building an AICA runtime application image

:::note

[Log in to the package registry](#logging-in-to-the-aica-package-registry) before building the image to authorize docker
to access AICA packages.

:::

Once the desired packages have been configured in a manifest file, a `docker build` command can be used to build the
runtime application image. In this example, a manifest file saved as `aica-application.toml` is used to build an image
with the name `aica-runtime`.

```shell
docker build -f aica-application.toml -t aica-runtime .
```

The command `docker image ls | grep aica-runtime` should then list the `aica-runtime` image.


## Starting the application container

You can start the AICA application container with the following command.

:::note

Change `/path/to/license` in the command below to the location of the `aica-license.toml` file from above. For example,
use `~/.aica-license.toml` to keep the license file hidden in the home folder.

:::

<!-- TODO: edit this if we ever go away from --net=host -->

<Tabs groupId="os">
<TabItem value="linux" label="Linux">

```bash
docker run -it --rm \
  --privileged \
  --net=host \
  -v /path/to/license:/license:ro \
  aica-runtime
```

</TabItem>
<TabItem value="mac" label="macOS">

```bash
docker run -it --rm \
  --privileged \
  -p 8080:8080 -p 18000-18100:18000-18100/udp \
  -v /path/to/license:/license:ro \
  aica-runtime
```

:::note

If port 8080 is already used on the host, use `-p HOST_PORT:8080` to avoid conflicts. Do not remap ports `18000-18100`.

:::

</TabItem>
</Tabs>

When the container starts up, it will generate some initial output in the terminal window that should look something
like the example below:

```console
[2024-11-18 13:43:12 +0000] [87] [INFO] Starting gunicorn 21.2.0
[2024-11-18 13:43:12 +0000] [87] [INFO] Listening at: http://0.0.0.0:5000 (87)
[2024-11-18 13:43:12 +0000] [87] [INFO] Using worker: eventlet
[2024-11-18 13:43:12 +0000] [100] [INFO] Booting worker with pid: 100
[INFO] [rosapi_node-1]: process started with pid [151]
[INFO] [1731937392.265880595] [EventEngine.ServiceHandler]: Initializing event engine services
[INFO] [1731937392.270243234] [event_engine]: No initial application provided. Use the event engine service interface to set, initialize and start an application.
[2024-11-18 13:43:13 +0000] [100] [INFO] Starting sync of cloud applications
[INFO] [1731937393.151675196] [EventEngineInterface]: Successfully connected to Event Engine services
[2024-11-18 13:43:13 +0000] [100] [INFO] Synced cloud applications: 0 added, 0 updated, 0 deleted, 1 total
```

:::info

If there are any errors, check that the license file is valid and has been mounted correctly. For example, the following
error would be shown from a correctly mounted but invalid license file:

```console
[ERROR] [1731937393.377201011] [licensing]: Error: license is invalid (ERR - license is invalid), please check that it is correct
```

Contact AICA support if the container does not start correctly despite a valid license file.

There can also be harmless warnings that appear if Cloud Storage is not set up or if the license verification takes
longer that a few seconds:

```console
[2024-11-18 13:38:16 +0000] [135] [INFO] Starting sync of cloud applications
[2024-11-18 13:38:16 +0000] [135] [WARNING] Sync failed
[2024-11-18 13:08:42 +0000] [151] [INFO] Waiting for licensing status... 5
[WARN] [1731935323.407252919] [EventEngine.ServiceHandler]: (404): Could not determine any license status
```

:::

## Stopping the application container

To shut down the AICA application container at any time, press CTRL+C in the original terminal window. Alternatively,
to stop the application container from a different terminal window, look up the container name
with `docker container ps` and then run `docker container stop <container_name>`.

## Access the AICA Studio

Visit [localhost:8080](http://localhost:8080) in the browser while the container is running to view AICA Studio.

## Access the REST API

Visit [localhost:8080/api](http://localhost:8080/api) to see the Swagger UI and documentation for the REST API.

## Connect a terminal session to the container

It is sometimes useful to connect to the application container while it is running to inspect files or run commands.
This can be accomplished with the `docker container exec` command using the container name.

:::note

Change `CONTAINER_NAME` in the command below to the name of the running container.

:::

```bash
docker container exec -it -u ros2 CONTAINER_NAME /bin/bash
```

The flags `-it -u ros2` tell docker to attach an interactive terminal as the `ros2` user. The command `/bin/bash`
starts a shell process.

:::tip

You can find the names of all running containers with the `docker ps` command.

Docker containers can also be given an explicit name when started with `docker run --name CONTAINER_NAME [...]`.

:::

Once attached, run any commands in the context of the container. For example, `ros2 topic list` will show the current
ROS 2 topics.

Detach from the container with CTRL+D or with the `exit` command.

### Display sharing

:::note

Native display sharing for GUI applications is only supported on Linux hosts.

:::

To run GUI applications like RViz in the docker container, it is necessary to share additional environment variables
for the display and X11 server.

:::caution

Running `xhost +` disables access control to the X11 server on the host, allowing the container to access the display.
It also allows any other process to access the server, which carries security implications. Use with caution.

Access control can be re-enabled with `xhost -`.

:::

```bash
xhost +
docker container exec -it -u ros2 -e DISPLAY="$DISPLAY" -e XAUTHORITY="$XAUTH" CONTAINER_NAME /bin/bash
```

You should then be able to run `rviz2` inside the container and see the window appear.

## Persistent user data

AICA applications and URDF hardware can be uploaded to a user database through the API or AICA Studio. Because the
docker container is isolated from the host filesystem, the local database will be lost when the container exists. To
persist local data between sessions, create a dedicated directory somewhere on the host. For example, use 
`mkdir ~/.aica-data` to keep the data folder hidden in the home folder. Then execute the normal run command with an
additional volume mount for the user data.

:::note

Change `/path/to/data` in the command below to a desired location for the data (e.g., `~/.aica-data` or elsewhere)

:::

<Tabs groupId="os">
<TabItem value="linux" label="Linux">

```bash
docker run -it --rm \
  --privileged \
  --net=host \
  -v /path/to/license:/license:ro \
  #highlight-next-line
  -v /path/to/data:/data:rw \
  aica-runtime
```

</TabItem>
<TabItem value="mac" label="macOS">

```bash
docker run -it --rm \
  --privileged \
  -p 8080:8080 -p 18000-18100:18000-18100/udp \
  -v /path/to/license:/license:ro \
  #highlight-next-line
  -v /path/to/data:/data:rw \
  aica-runtime
```

</TabItem>
</Tabs>