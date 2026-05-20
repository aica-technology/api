---
sidebar_position: 14
title: Fiducial Markers
---

import stagDetectorExample from './assets/stag-detector-example.png'
import stagMarkerDetection from './assets/stag-marker-detection.webm'
import stagMarkerNumOne from './assets/stagDetector-predicates_1.png'
import stagMarkerNumZero from './assets/stagDetector-predicates_0.png'

# Fiducial Markers

Different types of fiducial markers are used in robotics to provide precise 3D pose estimation and identification for cameras, enabling or improving robotic calibration and object manipulation.

AICA's `core-vision` package gives you the choice between using two commonly used markers, the STag and ArUco.

:::tip
Performing the [intrinsic calibration](./camera-calibration.md) of the camera improves the precision for fiducial marker detection and tracking.
:::

This guide provides an example of STag marker detection. Using the ArUco marker follows a very similar process.

## Preparing fiducial markers

A fiducial marker is an object placed in the field of view of an image for use as a point of reference or a measure. STag and ArUco markers are two of the common types of fiducial marker systems used for real-time 6D pose estimation. This section explains how to obtain, download and print these markers.

### Obtaining markers

- **ArUco marker**: ArUco markers can be generate online (e.g., from [here](https://chev.me/arucogen/)), which permits choosing the dictionary, marker ID, and marker size. It can be then exported as PDF or SVG for printing.

- **STag marker**: STag marker set can be either downloaded from [public Google Drive](https://drive.google.com/drive/folders/0ByNTNYCAhWbIV1RqdU9vRnd2Vnc?resourcekey=0-9ipvecbezW8EWUva5GBQTQ) or obtained from the [ROS2 STag project repository](https://github.com/usrl-uofsc/stag_ros/tree/ros2-devel) or the generator/reference files linked by the project. In practice, you’ll want to obtain the marker PDF/SVG or generate the markers from the project’s reference generator, then print them at true size.

### Printing markers

After choosing the marker family, selecting the library/dictionary, and the marker ID, download it as PDF or SVG. Use the actual size of the marker (100% scale) for printing, so the black border and marker geometry are not resized. Also it is recommended to print with high contrast and avoid compression artifacts.

If possible, print on a rigid and flat sheet of paper to reduce warping, since fiducial detection is sensitive to distortion. As another solution, you can fix the printed marker on a rigid surface, such as a piece of wood or cardboard.

After printing, measure the marker’s outer dimensions and compare them with the intended size from the generator. This matters because calibration will be wrong if the marker size in the software does not match the physical print.

## Using the STag detector

Launch AICA Studio with a configuration that contains the `core-vision` package and create a new application.

1. Remove the hardware interface that is included in new applications by default.
2. From the `Scene` menu, use the `Add Component` tab and look for the **Camera Streamer** and **STag Detector** components, either by searching
   or by manually going under the `Core Vision Components` menu. Add both of them to the graph.
3. Next, connect both components to the start block. Moreover, connect the outputs of the Camera Streamer to the relevant inputs of the STag Detector.
4. Enable **auto-configure** and **auto-activate** on both components.
5. By selecting any of the components, you can find all the available component parameters in the right panel under Settings.
6. If an intrinsic camera calibration is performed prior to this, add the file path of the camera configuration file as a parameter to the **Camera Streamer** component.

By this point, you should have something like the following:

<div class="text--center">
  <img src={stagDetectorExample} alt="CameraStreamer configuration alongside STagDetector component" style={{ borderRadius: "8px" }}/>
</div>

:::info
The Camera Streamer parameters are explained in the [CameraStreamer component guide](./camera-streamer.md).
:::

## STag Detector parameters

- **Rate**: The rate parameter doesn't affect the behavior of the component as the detection process
  occurs on reception of a new image.
- **Bundle file**: The filepath to a predefined marker bundle configuration. This additional feature is described in a separate guide (coming soon).
- **Marker selection**: The name(s) of the marker(s) that we want to recognize. If any of these markers enters the camera frame, the `is_any_selected_marker_detected` predicate is set to **True**. Also if a decision needs to be made based on the existence of a specific STag marker in the camera frame, its name should be indicated in this parameter. The markers name should always be prepended with the value of `Prefix`.
- **Marker size**: Determines the side length of a square that specifies the marker in meters.
- **Library**: This is the ID number of the HD library utilized by STag markers. The allowed numbers are `[11, 13, 15, 17, 19, 21, 23]`.
- **Error correction**: This parameter sets how permissive the detector is. Meaning it controls how many bit errors the STag detector is allowed to tolerate when decoding a marker ID. Lower values mean stricter matching, but detection would be less robust to blur, noise, bad lighting, or partial image degradation. Higher values mean more tolerant matching. The detector can still recognize a marker even if some bits are read incorrectly, so detection is more robust, but the risk of wrong matches increases. The lower and upper limits for this parameter are 0 and 11 respectively (inclusive).
- **Prefix**: This is the prefix used for marker names.

## STag Detector predicates

- **Is any marker detected**: This predicate will be set to **True** if any marker is detected in the camera frame, even though its name is not indicated in the 'Marker Selection' parameter.
  As you can see in the screenshot below, the marker name specified in the `Marker selection` parameter is stag_1, but the marker recognized in the camera frame is stag_0, yet `Is any marker detected` predicate is set to **True**.

<div class="text--center">
  <img src={stagMarkerNumOne} alt="Is any marker detected at all" style={{ borderRadius: "8px" }}/>
</div>

- **Is any selected marker detected**: If one or more marker names are indicated in the `Marker selection` parameter, and if any of them appears in the camera frame, this predicate with be set to **True**. If the names of none of the markers present in the camera frame is indicated as a parameter, this predicate will remain **False**.
  In the screenshot below the name of the marker appearing in the camera frame matches the name indicated in the `Marker selection` parameter.

<div class="text--center">
  <img src={stagMarkerNumZero} alt="Is any of the selected markers detected" style={{ borderRadius: "8px" }}/>
</div>

- **Is a marker bundle detected**: If a registered group of markers is detected by the camera, this parameter will be set to **True**. Otherwise it will remain **False**.

## Running the application

After setting up the proper parameters for Camera Streamer and STag Detector:

1. Press **Start** to start the application.
2. To see the live camera feed, select **Launch RViz** from the Launcher settings
3. In RViz, select _Add > By topic > /stag_detector/annotated_image > Image_. This adds a panel that shows the live image. The marker should be detected in the camera.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={stagMarkerDetection} type="video/webm" />
    STag marker detection video.
  </video>
</div>

The following YAML snippet contains the full application of the STag Detector above:

<details>
  <summary>Example application, STag Detector</summary>

```yaml
schema: 2-0-6
dependencies:
  core: v5.1.0
on_start:
  load:
    - component: stag_detector
    - component: camera_streamer
components:
  stag_detector:
    component: core_vision_components::pose_detection::STagDetector
    display_name: STag Detector
    events:
      transitions:
        on_configure:
          lifecycle:
            component: stag_detector
            transition: activate
        on_load:
          lifecycle:
            component: stag_detector
            transition: configure
    parameters:
      marker_selection:
        value:
          - stag_0
        type: string_array
    inputs:
      image: /camera_streamer/image
      camera_info: /camera_streamer/camera_info
  camera_streamer:
    component: core_vision_components::image_streaming::CameraStreamer
    display_name: Camera Streamer
    events:
      transitions:
        on_configure:
          lifecycle:
            component: camera_streamer
            transition: activate
        on_load:
          lifecycle:
            component: camera_streamer
            transition: configure
    parameters:
      camera_info_path:
        value: /data/ost.yaml
        type: string
      undistorted_image_cropping:
        value: false
        type: bool
    outputs:
      image: /camera_streamer/image
      camera_info: /camera_streamer/camera_info
graph:
  positions:
    on_start:
      x: -60
      y: 0
    stop:
      x: -60
      y: 100
    components:
      stag_detector:
        x: 820
        y: 0
      camera_streamer:
        x: 200
        y: 60
  edges:
    on_start_on_start_camera_streamer_camera_streamer:
      path:
        - x: 100
          y: 60
        - x: 100
          y: 120
    camera_streamer_image_stag_detector_image:
      path:
        - x: 700
          y: 280
        - x: 700
          y: 260
    camera_streamer_camera_info_stag_detector_camera_info:
      path:
        - x: 660
          y: 360
        - x: 660
          y: 300
```

</details>

:::info
The process for using ArUco markers follows a similar process.
:::
