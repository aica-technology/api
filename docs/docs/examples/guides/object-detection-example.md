---
sidebar_position: 8
title: Using YOLO to track objects
unlisted: true
---

import exampleApp from './assets/object-detection-example-app.gif'
import cameraCalibration from './assets/camera-calibration.gif'
import yoloExecutor from './assets/object-detection-yolo-executor.jpg'
import yoloExecutorParameters from './assets/object-detection-yolo-executor-parameters.png'
import boundingBoxTracker from './assets/object-detection-robot-control.jpg'

# Using YOLO to track objects

YOLO (You Only Look Once) is a real-time object detection algorithm that predicts bounding boxes and class probabilities
directly from an image in a single pass through a neural network. Unlike older methods that scan an image multiple times,
YOLO processes the entire image at once, making it extremely fast and well-suited for many applications, including
robotics.

## A YOLO example using the AICA framework <!-- TODO: I suggest we make this a standalone (short) example since we need it in other components (e.g., marker detectors) -->

This page details how to run a `YoloExecutor` component (i.e., a component that can use various YOLO models for
inference), and demonstrates how it could be used as part of an AICA application. We show how to create a custom
component which makes use of a bounding box to adapt an arm's motion such that it maintains the subject centered. Here,
we will use the `CameraStreamer` component from `components/core-vision` to access a `sensor_msgs::msg::Image` signal,
but any signal of the same type can be used instead. The YOLO executor component that is covered in following sections
can be found under `collections/advanced-perception` with a valid AICA license.

<div class="text--center">
  <img src={exampleApp} alt="Moving the robot towards an object in RViz" />
</div>

## Setup

### [optional] Camera calibration

Depending on how you are generating the `sensor_msgs::msg::Image` signal and/or the camera you are using, you may want
to consider running a calibration to ensure accurate predications in terms of bounding box pixel coordinates. To
facilitate the process, AICA provides a docker image you can use to run one of ROS' main packages for this task.

First, clone our docker image repository:

```shell
https://github.com/aica-technology/docker-images.git
```

and open a terminal at the root of `docker-images`. Then:

```shell
cd camera_calibration
```

Within that folder you will find a `build-run.sh` script that, as the name suggests, will build a Docker image and run a
container with the camera calibration software as an entrypoint.

Before running the script, make sure to generate a **checkerboard** pattern (e.g., from
[here](https://calib.io/pages/camera-calibration-pattern-generator)). The calibrator will use this pattern to determine
how the picture is distorted and ultimately generate the necessary matrices that can be used to undistort an image
coming from your camera. Take note of the checkerboard width, height, and box size. Notice that the calibrator is
detecting the internal corners of the outermost boxes, so a 8x11 checkerboard will have a 7x10 area with which the
calibrator will work. Print the checkerboard and attach it to a flat surface throughout the calibration process.
Then, run the `build-run.sh` script specifying the necessary parameters similarly to:

```shell
./build-run.sh --calibration-height 7 --calibration-width 11 --calibration-square 0.015
```

Notice that the calibration square side size is in meters. If you are using AICA's `CameraStreamer` component to produce
an image stream, the above command should already work. If you are using your own node to stream images, you will more
than likely need to specify which topic the calibrator needs to subscribe to by adding the
`--calibration-topic YOUR_ROS_TOPIC` argument.

After successfully executing the script, a pop-up window displaying your image
stream should appear. If the window is not displaying the image stream, make sure that
your image streaming component is running and, for non-`CameraStreamer` nodes, that the correct topic name is set.

Move the checkerboard in various positions and orientations until the `CALIBRATE` button is no longer grayed out (and
most of the bars are green, indicating good sample size). Once it becomes available, press on it to start computing the
camera matrices. After it becomes available, click on the `SAVE` button to save a recording of the process. You
will notice a `calibration` directory has been created on your host machine under `docker-image/camera_calibration` that
contains a compressed file. The file itself contains the images that were sampled along with a yaml file containing the
camera calibration information. Finally, move this file into the `data` folder of your AICA configuration such that it
becomes available within AICA containers. When using `CameraStreamer`, you only need to specify the calibration's path.
For custom components, make sure to read the camera parameters and apply the necessary undistortion technique(s).

<div class="text--center">
  <img src={cameraCalibration} alt="Camera calibration process" />
</div>

### Obtaining YOLO inference models

The `YoloExecutor` component works with `.onnx` model files. However, many of the available YOLO models are widely
available in Pytorch (`.pt`) formats instead. To convert between formats, you can use AICA's utilities to do so within
a Docker container and maintain your host system unpolluted.


First, clone our docker image repository (if you followed the calibration section, you should already have it!):

```shell
https://github.com/aica-technology/docker-images.git
```

and open a terminal at the root of `docker-images`. Then:

```shell
cd yolo_model_converter
./build-run.sh
```

By default, this will download and convert `yolo12n` for you. If you wish to specify one of the other models that
Ultralytics is offering, simply specify it as follows:

```shell
./build-run.sh --model yoloZZZZ
```

:::note

If you have `ultralytics` installed in your Python environment or you are willing to install it, then download any of
the models available [here](https://github.com/sunsmarterjie/yolov12). For the purposes of this example, you can opt
for a smaller model such as the `yolo12n`.

To convert a `.pt` file to `.onnx`, run the following Python code with `ultralytics` installed:

```python
from ultralytics import YOLO
model = YOLO("yolo12n.pt")
model.export(format="onnx")  # creates 'yolo12n.onnx'
```

This approach is also necessary if you are using custom models instead of the ones distributed by Ultralytics.
:::

## Class file

For this example, also download the standard `coco.yaml` class
file [here](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml) and move it to your
`data` folder, where you also stored your YOLO model.

## AICA Launcher configuration

In AICA Launcher, create a configuration with the following core version and packages:

- AICA Core v4.4.2
- `collections/advanced_perception v1.0.0` for the `YoloExecutor` component <!-- TBD -->
- `components/core-vision v1.0.0` for the `CameraStreamer` component  <!-- TODO: bump the version here -->

<!-- TODO: add toolkit images here -->

## Using the YOLO executor

Let us build a YOLO application from scratch.

- Create a new application
- Remove the default Hardware Interface node for now
- Add the Camera Streamer component from the core vision package
    - Set the `Source` parameter to a video device or file accordingly
    - Enable **auto-configure** and **auto-activate**
- Add the YOLO Executor component
    - Set the `Model path` parameter to the `.onnx` file, e.g., `/data/yolo12n.onnx`
    - Set the `Classes path` parameter to the yaml label file, e.g., `/data/coco.yaml`
    - Set the `Rate` parameter to match your camera's FPS (actual publishing rate may vary depending on your system's
computational power, especially if running on a CPU)
    - Enable **auto-configure** and **auto-activate**
- Connect the output of the start node to each component to load them when the application is started
- Connect the `Image` output of the Camera Streamer to the `Image` input of the `YoloExecutor`

Additional parameters can be used to tune the performance of YoloExecutor (both in the computational and prediction
sense). The following picture shows the available parameters:

<div class="text--center">
  <img src={yoloExecutorParameters} alt="Overview of a YoloExecutor parameters" />
</div>

More specifically, you can adapt:
- `Model path`: filepath to your YOLO model
- `Classes path`: filepath to your classes file, making the mapping between predicted object IDs and object names
- `Object class`: will narrow the `Detections` output to the selected classes alone (one or more classes included in the
class file)
- `Confidence threshold`: the minimum score a predicted bounding box must have to be considered a valid detection
- `IOU Threshold`: used during Non-Maximum Suppression (NMS) to decide whether two bounding boxes represent the same
object. For example, if `IOU threshold` is set to 0.5, any box that overlaps more than 50% with a higher-scoring box
will be discarded.
- `Device`: to determine which device should be used to run the inference, but is subject to the way you bundle your
AICA configuration. That is, if you use a CPU toolkit image but set `Device` to GPU, then the component will ultimately
gracefully fall back to using the CPU
- `Number of CPU threads`: to get the most out of your system's resources. Notice that this parameter has no effect when
a GPU is used

To complement the parameters and enable event-driven logic when using the component, two predicates exist, namely:
- `Is any selected object detected`: True if one of the objects in the `Object class` list is detected
- `Is any object detected`: True if there is any known object (i.e., as per the class file provided) in the image stream
(including but not limited to `Object class`)

Your application should now look similar to the following picture:

<div class="text--center">
  <img src={yoloExecutor} alt="Overview of a YoloExecutor application" />
</div>

### Running the application

Open the application we built in the previous step, if you are not already there. Then:
- open **RViz**: from the bottom-right gear icon **→** "Launch RViz"
- in **RViz**: press Add **→** By topic **→** `/yolo_executor/annotated_image/Image` to view the YOLO model's annotated
output. It should show the camera images with bounding boxes drawn around key objects. The bounding boxes are
published on the `yolo_executor/detections` topic as `std_msgs/msg/String`, which is in fact a JSON string with
object-related information (e.g., bounding box coordinates, class name, class id, ...).

:::note

Only users with a Linux host can visualize the image stream with RViz. On macOS, AICA Launcher will not show the RViz
option.

:::

## Tracking an object with YOLO

The bounding boxes generated by YOLO can be used to move a robot towards an object. Let us take an example were we will
emulate a camera mounted on a robot arm and we want to command the robot such that it tries to maintain a
selected class in the middle of the image frame. For simplicity, we specify a single object class for the `YoloExecutor`
to detect, and assume that only one object of the type can appear in the image at any time.

### Creating a custom twist generator component

We first need to create a custom component that given a bounding box of an item will generate a twist indicating where
the frame should move to keep the object centered. More information about custom components can be found
[here](https://docs.aica.tech/docs/category/custom-components/). The following component has been designed with regular
off-the-self webcams and the standard YOLO models in mind, meaning:

1. The object orientation is not taken into account when generating a twist. That is, no angular velocity is generated.
2. No depth information is available and/or considered. The component operates in pixel space and is invariant to an
object's movement along the depth axis. That is, only 2D motion will be observed, and the depth axis will have zero
velocity.

#### Set up the repository

- Create a git repository from the [package-template](https://github.com/aica-technology/package-template)
- Clone the repository, enter the directory, and run:

  ```bash
  ./initialize_package.sh
  ```
  Name it `object_detection_utils` and include a Python Lifecycle component

- Rename `py_lifecycle_component.py` to `bounding_box_tracker.py` in `source/component_utils/object_detection_utils/`
- Rename `py_lifecycle_component.json` to `object_detection_utils_bounding_box_tracker.json` in
`source/component_utils/component_descriptions/`
- Register the component in `source/component_utils/setup.cfg` like this:

```cfg
[options.entry_points]
python_components =
    object_detection_utils::BoundingBoxTracker = object_detection_utils.bounding_box_tracker:BoundingBoxTracker
```

#### Component code

As discussed, the component expects a JSON string as input (e.g. from `YoloExecutor`) containing a bounding box. From
that bounding box, it generates a `CartesianTwist` which can be used to control a robot. Below you will find the
implementation of the component, which can be copied directly into your `bounding_box_tracker.py`.

<details>
<summary>bounding_box_tracker.py</summary>

```python
import json
import numpy as np

from std_msgs.msg import String
from lifecycle_msgs.msg import State as LifecycleState

from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core import EncodedState
import state_representation as sr


class BoundingBoxTracker(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self._decay_rate: sr.Parameter = sr.Parameter("decay_rate", 1.0, sr.ParameterType.DOUBLE)
        self._twist = sr.CartesianTwist.Zero("object", "camera_frame")
        self._latest_twist = sr.CartesianTwist()
        self._camera_frame = sr.Parameter("camera_frame", "camera_frame", sr.ParameterType.STRING)
        self._reference_frame = sr.Parameter("reference_frame", sr.ParameterType.STRING)

        self.add_parameter(
            sr.Parameter("image_size", [640, 480], sr.ParameterType.DOUBLE_ARRAY),
            "Image resolution [width, height] in pixels",
        )
        self.add_parameter("_camera_frame", "Reference frame for the output twist")
        self.add_parameter(
            "_reference_frame", "Optional reference frame with respect to which the output twist should be recomputed"
        )
        self.add_parameter(sr.Parameter("gains", [0.0001, 0.0001], sr.ParameterType.DOUBLE_ARRAY), "Control gains (Kp)")
        self.add_parameter(
            sr.Parameter("deadband", [15.0, 15.0], sr.ParameterType.DOUBLE_ARRAY),
            "Deadband for the error measurements [width, height], within which no twist is generated",
        )
        self.add_parameter("_decay_rate", "Exponential decay rate")

        self.add_input("detections", self._on_receive_detections, String)
        self.add_output("twist", "_twist", EncodedState)

        self.add_tf_listener()

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        match name:
            case "image_size" | "deadband" | "gains":
                if len(parameter.get_value()) != 2:
                    self.get_logger().error(f"{name} must be a list of two floats")
                    return False
                if any(g < 0 for g in parameter.get_value()):
                    self.get_logger().error(f"{name} must be non-negative")
                    return False
            case "reference_frame" | "camera_frame":
                if not parameter.is_empty() and not parameter.get_value():
                    self.get_logger().error(f"{name} parameter cannot be empty")
                    return False
            case "decay_rate":
                if parameter.get_value() < 0:
                    self.get_logger().error("Decay rate must be non-negative")
                    return False
        return True

    def on_activate_callback(self) -> bool:
        if self._reference_frame.is_empty():
            self._reference_frame.set_value(self._camera_frame.get_value())
        self._twist = sr.CartesianTwist.Zero("object", self._reference_frame.get_value())
        self._latest_twist = sr.CartesianTwist.Zero("object", self._camera_frame.get_value())
        return True

    def on_step_callback(self) -> None:
        decay_factor = np.exp(-self._decay_rate.get_value() * self._latest_twist.get_age())
        twist = sr.CartesianTwist.Zero("object", self._camera_frame.get_value())
        twist.set_linear_velocity(np.array(self._latest_twist.get_linear_velocity()) * decay_factor)

        if not self._camera_frame.get_value() == self._reference_frame.get_value():
            try:
                tf = self.lookup_transform(self._camera_frame.get_value(), self._reference_frame.get_value())
            except Exception as e:
                self.get_logger().error(f"Failed to lookup transform: {e}")
                return
            self._twist = tf * twist
        else:
            self._twist = twist

    def _on_receive_detections(self, msg: String):
        if self.get_lifecycle_state().state_id != LifecycleState.PRIMARY_STATE_ACTIVE:
            self.get_logger().warning("Component is not active. Ignoring incoming detections.")
            return

        image_size = self.get_parameter("image_size").get_value()  # type: ignore
        gains = self.get_parameter("gains").get_value()  # type: ignore
        try:
            detections = json.loads(msg.data)["detections"]
            positions = {}
            for i, d in enumerate(detections):
                center = np.asarray([d["box"]["x"] + d["box"]["width"] / 2.0, d["box"]["y"] + d["box"]["height"] / 2.0])
                positions[f"{d['class_name']}_{i}"] = center

            if len(positions):
                obj = positions[
                    list(positions.keys())[0]
                ]  # ! naively assumes we are tracking only the first object detected
                width_error = obj[0] - image_size[0] / 2
                height_error = obj[1] - image_size[1] / 2
                vx = 0.0
                vy = 0.0
                if abs(width_error) > self.get_parameter("deadband").get_value()[1]:  # type: ignore
                    vx = width_error * gains[0]
                if abs(height_error) > self.get_parameter("deadband").get_value()[0]:  # type: ignore
                    vy = height_error * gains[1]

                self._latest_twist.set_linear_velocity(
                    vx, vy, 0.0
                )  # !z is explicitly zero for the purposes of this component
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
        except KeyError as e:
            self.get_logger().error(f"Missing key in JSON: {e}")
```

</details>

And its corresponding description file:

<details>
<summary>object_detection_utils_bounding_box_tracker.json</summary>

```json
{
  "$schema": "https://docs.aica.tech/schemas/1-1-1/component.schema.json",
  "name": "Bounding box tracker",
  "description": {
    "brief": "Reads bounding boxes and outputs an interactive marker",
    "details": "This component receives object detection outputs in JSON format and computes the corresponding twist command for the detected object to remain at the middle of the frame. The Z axis is not considered (i.e., only X-Y plane commands will be issued)."
  },
  "registration": "object_detection_utils::BoundingBoxTracker",
  "inherits": "modulo_components::LifecycleComponent",
  "inputs": [
    {
      "display_name": "Detections",
      "description": "Detection JSON string containing bounding box information",
      "signal_name": "detections",
      "signal_type": "string"
    }
  ],
  "outputs": [
    {
      "display_name": "Twist",
      "description": "The twist command",
      "signal_name": "twist",
      "signal_type": "cartesian_twist"
    }
  ],
  "parameters": [
    {
      "display_name": "Image size",
      "description": "Image resolution [width, height] in pixels",
      "parameter_name": "image_size",
      "parameter_type": "double_array",
      "default_value": "[640, 480]"
    },
    {
      "display_name": "Camera frame",
      "description": "Reference frame for the output twist",
      "parameter_name": "camera_frame",
      "parameter_type": "string",
      "default_value": "camera_frame"
    },
    {
      "display_name": "Reference frame",
      "description": "Optional reference frame with respect to which the output twist should be recomputed",
      "parameter_name": "reference_frame",
      "parameter_type": "string",
      "default_value": null,
      "optional": true
    },
    {
      "display_name": "Control gains",
      "description": "Control gains (Kp) for the twist command",
      "parameter_name": "gains",
      "parameter_type": "double_array",
      "default_value": "[0.0001, 0.0001]"
    },
    {
      "display_name": "Deadband",
      "description": "Deadband for the error measurements, within which no twist is generated",
      "parameter_name": "deadband",
      "parameter_type": "double_array",
      "default_value": "[15.0, 15.0]"
    },
    {
      "display_name": "Decay rate",
      "description": "Exponential decay rate for the twist (per second)",
      "parameter_name": "decay_rate",
      "parameter_type": "double",
      "default_value": "1.0"
    }
  ]
}
```

</details>

Enter the component folder in terminal and run

```bash
docker build -f aica-package.toml -t object-detection-utils .
```

Next, edit the AICA Launcher configuration and enter `object-detection-utils` under `Custom Packages`. After launching,
you should see the `Object detection utils` package listed in the `Add Component` menu, as well as a `BoundingBoxTracker`
component under that menu.

### Application setup

- Add the `BoundingBoxTracker` component to the previously configured application
    - Turn on **auto-configure** and **auto-activate**
    - Set the `Rate` to roughly match your camera's FPS, e.g., to 30
    - Set the `Control gains` to values that are sensible for your robot and your desired responsiveness
    - Set `Deadband` (i.e., a banded region of acceptable error within which no twist is generated) to your liking
    - Set the `Decay rate` (i.e., the rate at which a twist will decay if no object is detected) per your application's
  needs
- Connect the `Detections` output of `YoloExecutor` to the `Detections` input of `BoundingBoxTracker`

#### Commanding the robot with the generated twist

Add a Hardware Interface node to the application and select the `Generic six-axis robot arm` in the `URDF` selection.
Press on the **+** button to add a new controller and select the `IK Velocity Controller`. Make sure to enable the
**auto-configure** and **auto-activate** options.

Back at your `BoundingBoxTracker` component:

- Open the parameter menu and set `Camera frame` to `tool0`. That means that we assume the camera is mounted on `tool0`,
the robot's end effector. You can replace this frame with a recorded one or an existing frame in your robot model's tree.
- From the same menu, set `Reference frame` to `world`. By default, the twist is generated at the same reference frame
as `Camera frame`, but you can optionally set the output reference frame explicitly. This may be useful when a
controller, such as the `IK Velocity Controller`, expects a command in a specific reference frame (e.g., `world` in this
case)
- Finally, connect the `Twist` output of this component to the `Command` input of the `IK Velocity Controller`.

:::tip

If you are using one of the other robot models that AICA offers, make sure to change the `Camera frame` parameter to
your robot's end-effector frame, or to record a frame in world coordinates from the `3D Viz` menu.

:::

You are now all set to run this application. For reference and a quick visual validation, the final graph should look
like the following picture:

<div class="text--center">
  <img src={boundingBoxTracker} alt="Bounding box tracker application overview" />
</div>

If you copied the code from this example, the `YoloExecutor` will be set to track a pair of **scissors** across the frame.
Pick up a pair, play the application, and see how the robot adapts to your movements. Remember, in a real-world scenario
the camera would be attached to the robot and motion would stop as soon as the object was centered. Here, however, the
camera is fixed and motionless, so you have to position the object at the middle of your camera frame to prevent the
robot from moving in the 2D plane.

Once you have tested this application, go ahead and pick another object that is included in the
[coco](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml) dataset and try again.
Some objects are easier to recognize than others, so you may have to adapt the `YoloExecutor`'s parameters or even opt
for a larger YOLO model.

:::tip
If you are planning to use this demo with a physical robot, consider using a `Velocity Impedance Controller` to
introduce some compliance/safety into your application. This would merely require replacing your `IK Velocity Controller`
and parametrizing the `Velocity Impedance Controller` to your liking.
:::

### Application code

<details>
<summary>YAML application</summary>

```yaml
schema: 2-0-4
dependencies:
  core: v4.2.0
on_start:
  load:
    - hardware: hardware
    - component: camera_streamer
components:
  yolo_executor:
    component: advanced_perception::object_detection::YoloExecutor
    display_name: YOLO Executor
    events:
      transitions:
        on_load:
          lifecycle:
            component: yolo_executor
            transition: configure
        on_configure:
          lifecycle:
            component: yolo_executor
            transition: activate
        on_activate:
          load:
            component: bounding_box_tracker
    parameters:
      rate: !!float 30.0
      model_file: /data/yolo12n.onnx
      classes_file: /data/coco.yaml
      object_class:
        - scissors
      num_threads: 4
    inputs:
      image: /camera_streamer/image
    outputs:
      detections: /yolo_executor/detections
  camera_streamer:
    component: core_vision_components::image_streaming::CameraStreamer
    display_name: Camera Streamer
    events:
      transitions:
        on_load:
          lifecycle:
            component: camera_streamer
            transition: configure
        on_configure:
          lifecycle:
            component: camera_streamer
            transition: activate
        on_activate:
          load:
            component: yolo_executor
    outputs:
      image: /camera_streamer/image
  bounding_box_tracker:
    component: object_detection_utils::BoundingBoxTracker
    display_name: Bounding box tracker
    events:
      transitions:
        on_load:
          lifecycle:
            component: bounding_box_tracker
            transition: configure
        on_configure:
          lifecycle:
            component: bounding_box_tracker
            transition: activate
    parameters:
      camera_frame: tool0
      reference_frame: world
    inputs:
      detections: /yolo_executor/detections
    outputs:
      twist: /yolo_to_marker/twist
hardware:
  hardware:
    display_name: Hardware Interface
    urdf: Generic six-axis robot arm
    rate: 100
    events:
      transitions:
        on_load:
          load:
            - controller: robot_state_broadcaster
              hardware: hardware
            - controller: ik_velocity_controller
              hardware: hardware
    controllers:
      robot_state_broadcaster:
        plugin: aica_core_controllers/RobotStateBroadcaster
        events:
          transitions:
            on_load:
              switch_controllers:
                hardware: hardware
                activate: robot_state_broadcaster
      ik_velocity_controller:
        plugin: aica_core_controllers/velocity/IKVelocityController
        inputs:
          command: /yolo_to_marker/twist
        events:
          transitions:
            on_load:
              switch_controllers:
                hardware: hardware
                activate: ik_velocity_controller
graph:
  positions:
    on_start:
      x: -20
      y: -360
    stop:
      x: -20
      y: -260
    components:
      yolo_executor:
        x: 720
        y: -180
      camera_streamer:
        x: 200
        y: -300
      bounding_box_tracker:
        x: 1400
        y: 200
    hardware:
      hardware:
        x: 1920
        y: -380
  edges:
    yolo_to_marker_marker_pose_signal_point_attractor_attractor:
      path:
        - x: 1360
          y: 520
        - x: 1360
          y: 680
    yolo_executor_detections_yolo_to_marker_json_input:
      path:
        - x: 1160
          y: 120
        - x: 1160
          y: 220
        - x: 860
          y: 220
        - x: 860
          y: 520
    yolo_to_marker_twist_hardware_hardware_ik_velocity_controller_command:
      path:
        - x: 1820
          y: 380
        - x: 1820
          y: 420
    yolo_executor_detections_yolo_to_marker_detections:
      path:
        - x: 1200
          y: 120
        - x: 1200
          y: 380
    on_start_on_start_camera_streamer_camera_streamer:
      path:
        - x: 140
          y: -320
        - x: 140
          y: -240
    yolo_executor_on_activate_bounding_box_tracker_bounding_box_tracker:
      path:
        - x: 1300
          y: 0
        - x: 1300
          y: 260
    camera_streamer_image_yolo_executor_image:
      path:
        - x: 640
          y: 0
        - x: 640
          y: 120
    yolo_executor_detections_bounding_box_tracker_detections:
      path:
        - x: 1220
          y: 120
        - x: 1220
          y: 420
```

</details>