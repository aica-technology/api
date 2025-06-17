---
sidebar_position: 5
title: Orbbec camera component
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

import orbbecMountedVolume from './assets/orbbec-mounted-udev-volume.png'
import orbbecNewApp from './assets/orbbec-new-app.gif'
import orbbecRvizColor from './assets/orbbec-play-rviz.gif'
import orbbecCameraParameters from './assets/orbbec-camera-parameters.png'
import orbbecThresholdParameters from './assets/orbbec-threshold-params.png'


# Orbbec camera component

AICA's `orbbec` package includes support for a range of Orbbec camera models, including:

- Gemini 33X
- Gemini 435Le
- Gemini 2/2L
- Femto Bolt/Mega
- Astra 2

This guide describes all necessary steps to install, configure and run the component,
using AICA Studio.  

## Installation

:::note

For now, the Orbbec camera component has only been tested on Linux machines. Other platforms might be officially supported in the future.

:::

To ensure proper access to the camera device, special udev rules have to be applied to the Linux system
the camera is connected to. Run the following commands in a terminal: 

```shell
wget -O 99-obsensor-libusb.rules https://raw.githubusercontent.com/aica-technology/OrbbecSDK_ROS2/refs/tags/aica/v2.4.3/orbbec_camera/scripts/99-obsensor-libusb.rules
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

:::tip
These commands will download and apply the udev rules. To ensure that the rules have been applied
correctly, the [Orbbec Viewer](https://github.com/orbbec/OrbbecSDK/releases) can be used to check 
the camera feed.
:::


Additionally, in order to ensure the camera can be accessed from within the container, the `/run/udev/` volume
has to be mounted. In AICA Launcher, expand the **Advanced Settings** at the bottom, select
**Add a volume mount** and define the volume as shown in the following image:

<div class="text--center">
  <img src={orbbecMountedVolume} alt="Mounted udev volume" />
</div>

Select **Launch AICA Studio** to proceed. 

## Creating and running the example

Start by creating a new application. 

1. Remove the hardware interface that is included in new applications by default.
2. Press the (+) button on the top right, and locate the **Orbbec Camera** component. Click to add
to the graph. 
3. Next, connect the start block to the component.

<div class="text--center">
  <img src={orbbecNewApp} alt="Creating a new Orbbec Camera component" />
</div>

4. Press **Play** to start the application. 
5. To see the live camera feed, click on the gear icon on the bottom right and select
**Launch RViz**.
6. In RViz, select _Add > By topic > /orbbec_camera/color_image > Image_. This adds a panel that shows the
live color image. The depth image can also be found under _/orbbec_camera/depth_image > Image_.

<div class="text--center">
  <img src={orbbecRvizColor} alt="Starting and checking camera live stream" />
</div>

## Parametrizing the Orbbec camera component

Click on the small gear icon on the `Orbbec Camera` block to view and edit the available parameters.

<div class="text--center">
  <img src={orbbecCameraParameters} alt="Basic Orbbec camera parameters" />
</div>

Hovering over the exclamation marks next to the names shows a detailed decription of the parameter. 
Let's explain some of these here:

- `Color/Depth Width/Height/FPS`: these refer to the resolution and frame rate of the color and depth
images. Keep in mind that only specific pairs of integer values apply here. For more information 
check the camera's documentation. 

:::tip
The Orbbec Viewer can also come in handy here, as it shows the available profiles for the connected camera.
:::

- `Enable Alignment`: flag that activates the spatial alignment of the depth image to the
corresponding color image. Generates a depth image with the same size as the color, with depth
expressed in the color camera coordinate system. 
- `Enable Threshold Filter`: flag that activates a threshold filter in the depth image. If activated, corresponding
filter parameters can be directly defined in the YAML editor to configure the filter behavior. To see what parameters
can be defined, read the description by hovering over the exclamation mark.
    <div class="text--center">
      <img src={orbbecThresholdParameters} alt="Threshold filter tooltip parameters" />
    </div>
    If the additional parameters are not provided, default values will be used. For instance, activating and configuring
    the threshold filter would look like this in the YAML editor: 

    ```yaml
    components:
      orbbec_camera:
        component: orbbec_camera::OBCameraNodeDriver
        display_name: Orbbec Camera
        parameters:
          enable_threshold_filter: true
          threshold_filter_max: 2000
          threshold_filter_min: 500
    ```

Click on **Generate Graph** to make the changes take effect. Start the application, launch RViz and add
a panel for the depth image. Only objects between 0.5 and 2 meters should appear. Keep in mind that most
cameras have a minimum Z depth; if the maximum threshold is set below this value, the image will appear
blank, but noisy and arbitrarily small/large depth values may still be published. For more information about that, check the camera's documentation. 

If needed, follow the same process to activate and configure other available filters.  