---
sidebar_position: 8
title: Realsense camera component
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

import orbbecMountedVolume from './assets/orbbec-mounted-udev-volume.png'
import realsenseNewApp from './assets/realsense-new-app.gif'
import realsenseRvizColor from './assets/realsense-play-rviz.gif'
import realsenseCameraParameters from './assets/realsense-camera-parameters.png'
import realsenseTemporalParameters from './assets/realsense-temporal-params.png'
import realsensePackage from './assets/realsense-collection.gif'

# RealSense camera component

AICA's `realsense` package includes support for a range of Realsense camera models. This guide describes all necessary
steps to install, configure and run the component using AICA Studio.

TODO: Are we adding a list of supported camera models here as well?

## Installation

:::note

For now, the RealSense camera component has only been tested on Linux machines. Other platforms might be officially
supported in the future.

:::

To ensure proper access to the camera device, special udev rules have to be applied to the Linux system the camera is
connected to. The following commands clone the `librealsense` repo, apply the rules and then remove the directory:

TODO: Is this version specific? Are we going to discuss different Ubuntu versions?

```shell
git clone --branch v2.54.1 --depth 1 https://github.com/IntelRealSense/librealsense
cd realsense
git checkout development
sudo ./scripts/setup_udev_rules.sh
# should we provide the commands to remove the directory to reduce clutter and also make
# clear no further installation or libraries are needed?
cd ..
rm -r librealsense
```

:::tip

The color, depth and infrared image streams should work without having to install the udev rules, but there might be
issues when the IMU measurement streams are activated. For this reason, it is suggested to install the rules anyway.

:::

Start the AICA Launcher and add the `intel-realsense` package to your configuration.

<div class="text--center">
  <img src={realsensePackage} alt="Adding the RealSense package" />
</div>

Select **Launch AICA Studio** to proceed.

## Creating and running the example

Start by creating a new application.

1. Remove the hardware interface that is included in new applications by default.
2. Press the (+) button on the top right, and locate the **RealSense Camera** component. Click to add to the graph.
3. Next, connect the component to the start block.

<div class="text--center">
  <img src={realsenseNewApp} alt="Creating a new RealSense Camera component" />
</div>

4. Press **Play** to start the application.
5. To see the live camera feed, click on the gear icon on the bottom right and select **Launch RViz**.
6. In RViz, select _Add > By topic > /realsense_camera/color_image_raw > Image_. This adds a panel that shows the live
   color image. The depth image can also be found under _/realsense_camera/depth_image_rect_raw > Image_.

<div class="text--center">
  <img src={realsenseRvizColor} alt="Starting and checking camera live stream" />
</div>

TBD: Are we keeping the tip? Since it is not even needed for Orbbec, I think we can remove it.

:::tip

In case the component does not produce a video stream, try mounting the `/run/udev/` volume. In AICA Launcher, expand
the **Advanced Settings** at the bottom, select **Add a volume mount** and define the volume as shown below. If you
still encounter problems getting the video stream, contact the AICA support team.

<div class="text--center">
  <img src={orbbecMountedVolume} alt="Mounted udev volume" />
</div>

:::

## Parametrizing the RealSense Camera component

Click on the small gear icon on the `RealSense Camera` block to view and edit the available parameters.

<div class="text--center">
  <img src={realsenseCameraParameters} alt="Basic RealSense camera parameters" />
</div>

Hovering over the exclamation marks next to the names shows a detailed description of the corresponding parameter. Let's
explain some of these here:

- `Color/Depth profile`: these refer to the resolution and frame rate of the color and depth images. Keep in mind that
  only specific pairs of values apply here. For more information check the camera's documentation.
- `Enable alignment`: flag that activates the alignment of the depth image to the corresponding color image. Enables the
  **Aligned depth image** output, containing a depth image with the same size as the color.
- `Enable temporal filter`: flag that activates a temporal filter which improves depth data persistency by manipulating
  per-pixel values based on previous frames. If activated, corresponding filter parameters can be directly defined in
  the YAML editor to configure the filter behavior. To see what parameters can be defined, read the description by
  hovering over the exclamation mark.

  <div class="text--center">
    <img src={realsenseTemporalParameters} alt="Threshold filter tooltip parameters" /> 
  </div>

  If the additional parameters are not provided, default values will be used. For instance, activating and configuring
  the temporal filter would look like this in the YAML editor:

      ```yaml
      components:
        realsense_camera:
          component: realsense2_camera::RealSenseNodeFactory
          display_name: RealSense Camera
          parameters:
            device_type: L515
            temporal_filter.enable: true
            temporal_filter.holes_fill: 1
            temporal_filter.filter_smooth_alpha: 0.9
            temporal_filter.filter_smooth_delta: 10
      ```

If needed, follow the same process to activate and configure other available filters. For more information and detailed
description of the filters, check the
[librealsense repository](https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md).
