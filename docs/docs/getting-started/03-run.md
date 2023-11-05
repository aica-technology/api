---
sidebar_position: 3
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Running the image

This guide assumes that the AICA runtime application image was tagged as `aica-runtime`. If the generated image has a
different name, change the name of in the following instructions accordingly.

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
  -p 3000:3000 -p 5000:5000 \
  -v /path/to/license:/license:ro \
  aica-runtime
```

</TabItem>
</Tabs>

## Access the Developer Interface

Visit [localhost:3000](http://localhost:3000) in the browser to view the Developer Interface.

## Access the REST API

Visit [localhost:5000](http://localhost:5000) to see the Swagger homepage and documentation for the REST API.

## Persistent user data

AICA applications and URDF hardware can be uploaded to a user database through the API or Developer Interface UI.
Because the docker container is isolated from the host filesystem, the local database will be lost when the container
exists. To persist local data between session, create a dedicated directory somewhere on the host. For example,
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
  -p 3000:3000 -p 5000:5000 \
  -v /path/to/license:/license:ro \
  #highlight-next-line
  -v /path/to/data:/data:rw \
  aica-runtime
```

</TabItem>
</Tabs>
