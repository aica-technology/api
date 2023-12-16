---
sidebar_position: 4
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Running the image

This guide assumes that the AICA runtime application image was tagged as `aica-runtime`. If the generated image has a
different name, change the name of in the following instructions accordingly.

## Starting the application container

You can start the AICA application container with the following command.

:::note

Change `/path/to/license` in the command below to the location of the personal AICA developer license assigned to you.
For example, use `~/.aica-license.toml` to keep the license file hidden in the home folder.

:::


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
  -p 8080:8080 -p 5000:5000 \
  -v /path/to/license:/license:ro \
  aica-runtime
```

</TabItem>
</Tabs>

When the container starts up, it will generate some initial output in the terminal window that should look something
like the example below:

```console
[2023-12-11 14:48:38 +0000] [65] [INFO] Starting gunicorn 21.2.0
[2023-12-11 14:48:38 +0000] [65] [INFO] Listening at: http://0.0.0.0:5000 (65)
[2023-12-11 14:48:38 +0000] [65] [INFO] Using worker: gthread
[2023-12-11 14:48:38 +0000] [69] [INFO] Booting worker with pid: 69
[INFO] [1702306118.970110638] [StateEngine.ServiceHandler]: Initializing state engine services
[INFO] [1702306118.977229190] [state_engine]: No initial application provided. Use the state engine service interface to set, initialize and start an application.
[INFO] [1702306119.566053151] [StateEngineInterface]: Successfully connected to Dynamic State Engine services
```

If there are any errors, check that the license  (for example, `Error: license is invalid!` or `)

```console
[ERROR] [1702306228.377201011] [licensing]: Error: license is invalid (ERR - license is invalid), please check that it is correct
```

## Stopping the application container

To shut down the AICA application container at any time, press CTRL+C in the original terminal window. Alternatively,
to stop the application container from a different terminal window, look up the container name
with `docker container ps` and then run `docker container stop <container_name>`.

## Access the Developer Interface

Visit [localhost:8080](http://localhost:8080) in the browser while the container is running to view the Developer
Interface.

## Access the REST API

Visit [localhost:5000](http://localhost:5000) to see the Swagger UI and documentation for the REST API.

## Persistent user data

AICA applications and URDF hardware can be uploaded to a user database through the API or Developer Interface UI.
Because the docker container is isolated from the host filesystem, the local database will be lost when the container
exists. To persist local data between sessions, create a dedicated directory somewhere on the host. For example,
use `mkdir ~/.aica-data` to keep the data folder hidden in the home folder. Then execute the normal run command with an
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
  -p 8080:8080 -p 5000:5000 \
  -v /path/to/license:/license:ro \
  #highlight-next-line
  -v /path/to/data:/data:rw \
  aica-runtime
```

</TabItem>
</Tabs>
