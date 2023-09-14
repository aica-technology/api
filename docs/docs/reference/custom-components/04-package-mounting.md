---
sidebar_position: 4
title: Mounting a component package
---

# Including a component package

:::caution

We are working on better packaging tools to include custom component packages in AICA applications.

As a result, the information in this section is subject to change in the near future.

:::

To use custom components in AICA applications, the component package must be copied and installed in the AICA image
workspace. The easiest way to do this is to create a new docker image extending the base AICA image.

This guide assumes that the AICA software image was distributed as `aica-technology/base`. If the base image has a
different name, change the first line of the example below accordingly.

```dockerfile title="Dockerfile"
FROM aica-technology/base

WORKDIR ${WORKSPACE}
SHELL ["/bin/bash", "-c"]

COPY --chown=${USER}:${USER} ./src ${WORKSPACE}/src

RUN sudo chown -R ${USER}:${USER} ${WORKSPACE} && source ${WORKSPACE}/install/setup.bash && colcon build
```

Build the image with the following command (replacing the output image name as desired).

```bash
docker build -t aica-technology/base:custom-component .
```

Then, in place of launching the `aica-technology/base` image, launch the customized image (in this
example, `aica-technology/base:custom-component`).

:::tip

Refer to [this documentation page](../../getting-started/03-run.md) to learn how to run the image.

:::