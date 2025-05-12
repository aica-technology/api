---
sidebar_position: 4
---

# Using YOLO to Track Objects

This example provides a use case for `Object Detection Components`. We show how to create a custom component which converts bounding boxes into a 3D pose. This requires AICA Core v4.3.2, `collections/object-detection`. The example works with both a realsense camera using `collections/intel-realsense-collection` or a video played through a custom component which is provided.

## Testing the YOLO Executor
### Setup
Launch AICA Studio with:
AICA Core v4.3.2
- `collections/object-detection`
- `collections/intel-realsense-collection`
- Under **Advanced Settings**, add a volume containing the folder where you will store your YOLO model files and link it to `/files`.

### Setting Up the Application

- Remove the **Hardware Interface**.
- Add the following components:
  - `Object Detection Components/YOLO Executor`
  - `Realsense2 Camera` or if you do not have a camera, you can [create a video player component](#create-a-video-player-component) instead.

- Connect both components’ **load nodes** to load on program start.
- Connect the **RGB Image** output of the Realsense2 Camera or video player to the **RGB Image** input of the YOLO Executor.

**Configuring YOLO Executor**

- Set to **auto-configure** and **auto-activate**.
- Set the following:
  - **Model Path**: YOLO model in `.onnx` format e.g., `/files/yolo12n.onnx`
  - **Classes Path**: Path to the class labels e.g., `/files/coco.yaml`
  - **Rate**: This is hardware dependent but 3 should work on most machines.

*Convert YOLO Model to ONNX*

The YOLO model expects an ONNX file, you can create one from a pretrained [`.pt`](https://github.com/sunsmarterjie/yolov12) model, (we use the lightweight version — YOLO12n). Then use Python (with `ultralytics` installed) to convert the `.pt` model to `.onnx`:
```python
from ultralytics import YOLO

# Load the model
model = YOLO("yolov12n.pt")

# Export the model to ONNX format
model.export(format="onnx")  # creates 'yolov12n.onnx'
```

These YOLO models are pre-trained on coco - you can download the Classes file for coco [here](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml). If you fine tune YOLO to your use case you may need to create a custom yaml file.

### Running the Application

- Start the application in AICA Studio.
- Open **RViz**: bottom-right gear icon → "Launch RViz"
- Open **RViz** → **Add** → **By topic** → `/yolo_executor/annotated_image/Image`  
  to view the YOLO model's annotated output. It should show the camera images with bounding boxes drawn around key objects in it.

> Only users with a Linux host can visualize the robot with RViz. On macOS, AICA Launcher will not show the RViz option.

## Tracking an Object with YOLO
We use YOLO to generate bounding boxes which in turn are used to position a 3D marker and drive the robot. We first need to create a custom component which can estimate a pose from a position in the camera frame. More information about custom components can be found [here](https://docs.aica.tech/docs/category/custom-components/). We make 3 simplifying assumptions:
  1. Camera is fixed and angled downward
  2. Known camera position pointing down: `(x=0, y=0.6, z=0.6)`
  2. Objects are on a flat surface at `z=0`

### Creating a Custom Marker Component
1. **Set up the repository:**
   - Create a git repository from the [component-template](https://github.com/aica-technology/component-template).
   - Clone it, enter the directory, and run:
     ```bash
     ./initialize_package.sh
     ```
   - Name it `component_utils`, do not include any components as we will create them from scratch.

    **Create files and classes:**

    - Create `yolotomarker.py` in `source/component_utils/component_utils/`.
    - Create `yolotomarker.json` in `source/component_utils/component_descriptions/`.
    - Register the component in `source/component_utils/setup.cfg` with:
      ```cfg
      component_utils::YoloToMarker = component_utils.yolotomarker:YoloToMarker
      ```

#### Component Code
Below is the core implementation, the component reads JSON input from YOLO Executor, projects bounding boxes into 3D using FoV, camera height, outputs a `CartesianState` for robot control.

Copy the following into `yolotomarker.py`:
```python
import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import String
from modulo_core import EncodedState
import clproto
import json
import numpy as np
from copy import deepcopy

class YoloToMarker(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        # Inputs
        self.json_input = ""
        self.add_input("json_input", "json_input", String)

        # Outputs
        self._state_command = sr.CartesianState()
        self.add_output("state_command", "_state_command", EncodedState, clproto.MessageType.CARTESIAN_STATE_MESSAGE)

        # Parameter placeholders
        self.to_find = 'cup'
        self.fov_x = 69
        self.fov_y = 42
        self.start_pos_x = 0
        self.start_pos_y = 0.6
        self.start_pos_z = 0
        self.camera_height = 0.6
        self.img_shape = np.asarray([840, 480])

        # Parameters
        self.add_parameter(sr.Parameter("to_find", sr.ParameterType.STRING), "thing to find and track")
        self.add_parameter(sr.Parameter("to_find", sr.ParameterType.STRING), "thing to find and track")
        self.add_parameter(sr.Parameter("fov_x", 69.0, sr.ParameterType.DOUBLE), "Camera FoV in X")
        self.add_parameter(sr.Parameter("fov_y", 42.0, sr.ParameterType.DOUBLE), "Camera FoV in Y")
        self.add_parameter(sr.Parameter("start_pos_x", 0.0, sr.ParameterType.DOUBLE), "Centre position of camera (x)")
        self.add_parameter(sr.Parameter("start_pos_y", 0.6, sr.ParameterType.DOUBLE), "Centre position of camera (y)")
        self.add_parameter(sr.Parameter("camera_height", 1.0, sr.ParameterType.DOUBLE), "Centre position of camera (z)")
        self.add_parameter(sr.Parameter("goal_z", 0.0, sr.ParameterType.DOUBLE), "Target position (Z)")

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        return True

    def on_configure_callback(self) -> bool:
        # Load parameter values
        self.fov_x = self.get_parameter_value("fov_x")
        self.fov_y = self.get_parameter_value("fov_y")
        self.start_pos_x = self.get_parameter_value("start_pos_x")
        self.start_pos_y = self.get_parameter_value("start_pos_y")
        self.camera_height = self.get_parameter_value("camera_height")
        self.goal_z = self.get_parameter_value("goal_z")

        self.camera_pos = np.asarray([self.start_pos_x, self.start_pos_y, self.camera_height])

        # Compute ratio of pixels/meters 
        fov = np.radians(np.asarray([self.fov_x, self.fov_y]))
        self.pix_xy = self.camera_height * np.tan(fov/2)
        return True

    def on_activate_callback(self) -> bool:
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def __centre_pt_to_position(self, xy):
        # Convert image center coordinates to relative camera-space position
        ratio = (xy - self.img_shape/2)/(self.img_shape/2)
        ratio[1] = -ratio[1]
        return ratio * self.pix_xy

    
    def on_step_callback(self):
        try:
            data = json.loads(self.json_input)

            # Convert bounding boxes to positions
            results = {
                key: {
                    'centre': np.asarray([
                        (coords[0] + coords[2]) / 2,
                        (coords[1] + coords[3]) / 2
                    ]),
                    'position': self.__centre_pt_to_position(
                        np.asarray([
                            (coords[0] + coords[2]) / 2,
                            (coords[1] + coords[3]) / 2
                        ])
                    )
                }
                for key, coords in data.items()
            }

            # Get target object
            to_find = self.get_parameter_value("to_find")
            position = results[to_find + '_1']['position']

            # Create full 3D position
            position = (np.append(position, self.goal_z)).reshape((3,1))
            position[:2, 0] += self.camera_pos[:2]

            # Build state command
            orientation = np.zeros((4, 1))
            linear_velocity = np.zeros((3, 1))
            angular_velocity = np.zeros((3, 1))

            command = deepcopy(self._state_command)
            command.set_position(position)
            command.set_orientation(orientation)
            command.set_linear_velocity(linear_velocity)
            command.set_angular_velocity(angular_velocity)

            # Post state command
            self._state_command = command

            self.get_logger().info(f'{results}')
            self.get_logger().info(f'{command}')

        except Exception as e:
            self.get_logger().error(f'Error in on_step_callback: {e}')
```

The component description is defined in `yolotomarker.json`:
```json
{
  "$schema": "https://docs.aica.tech/schemas/1-1-1/component.schema.json",
  "name": "YOLO to marker",
  "description": {
    "brief": "Reads bounding boxes and outputs an interactive marker",
    "details": "This component computes an approximate 3D position from 2D bounding boxes assuming a fixed camera position and flat object surface."
  },
  "registration": "component_utils::YoloToMarker",
  "inherits": "modulo_components::LifecycleComponent",
  "inputs": [
    {
      "display_name": "Json Input",
      "description": "",
      "signal_name": "json_input",
      "signal_type": "string"
    }
  ],
  "outputs": [
    {
      "display_name": "State command",
      "description": "The state command",
      "signal_name": "state_command",
      "signal_type": "cartesian_state"
    }
  ],
  "parameters": [
    {
      "display_name": "Thing to track",
      "description": "The object to track",
      "parameter_name": "to_find",
      "parameter_type": "string",
      "default_value": null
    },
    {
      "display_name": "FoV X",
      "description": "Camera FoV in X",
      "parameter_name": "fov_x",
      "parameter_type": "double",
      "default_value": 69.0,
      "dynamic": true
    },
    {
      "display_name": "FoV Y",
      "description": "Camera FoV in Y",
      "parameter_name": "fov_y",
      "parameter_type": "double",
      "default_value": 42.0,
      "dynamic": true
    },
    {
      "display_name": "Start Pos X",
      "description": "Centre position of camera (x)",
      "parameter_name": "start_pos_x",
      "parameter_type": "double",
      "default_value": 0.0,
      "dynamic": true
    },
    {
      "display_name": "Start Pos Y",
      "description": "Centre position of camera (y)",
      "parameter_name": "start_pos_y",
      "parameter_type": "double",
      "default_value": 0.6,
      "dynamic": true
    },
    {
      "display_name": "Camera Height",
      "description": "Centre position of camera (z)",
      "parameter_name": "camera_height",
      "parameter_type": "double",
      "default_value": 1.0,
      "dynamic": true
    },
    {
      "display_name": "Goal Z",
      "description": "Target position (Z)",
      "parameter_name": "goal_z",
      "parameter_type": "double",
      "default_value": 0.0,
      "dynamic": true
    }    
  ]
}
```
Build and load the component in terminal, enter the component folder and run  
`docker build -f aica-package.toml -t objectdetection .` 

Next, configure AICA Studio and add `objectdetection` under **Custom Packages**. You should see `Component Utils` under *Add Component* and be able to add **YOLO to marker**. We can test that our component is working by connecting JSON input to the Bounding Boxes output of **YOLO Executor**. When we run the application we should see our component logging every iteration. 


**Improvements**
The component could be further improved by:
- Add error checks for bounding box and parameters.
- Make image size (`img_shape`) a configurable parameter
- Adding logic for when nothing is found in the image.

## Final Notes

For more accurate 3D tracking, explore AICA’s `foundation_pose_estimator` with `GroundedSAM` or `GroundedSAM2`. These offer more advanced geometry estimation.

## Create a Video Player Component
To emulate a camera by playing a video frame by frame, [create a custom component](#tracking-an-object-with-yolo).

**Create files and classes:**

- Create `videoplayer.py` in `source/component_utils/component_utils/`.
- Create `videoplayer.json` in `source/component_utils/component_descriptions/`.
- Register the component in `source/component_utils/setup.cfg` with:
  ```cfg
  component_utils::VideoPlayer = component_utils.videoplayer:VideoPlayer
  ```

#### Component Code
Update videoplayer.py with the following:

```python
import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import cv2

class VideoPlayer(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Create Placeholders
        self.video_feed = None
        self.video_path = ''
        self._cap = None 
        self._cv_bridge = CvBridge()

        # Define parameters
        self.add_parameter(sr.Parameter('video_path', sr.ParameterType.STRING), "Path to the video file.")     

        # Define outputs
        self.add_output("video_feed", "video_feed", SensorImage)

    def on_activate_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        # Load video, create open cv VideoCapture instance.
        video_path = self.get_parameter_value("video_path")

        self._cap = cv2.VideoCapture(video_path)

        if not self._cap.isOpened():
            self.get_logger().error(f"Failed to open video: {video_path}")
            return False

        self.get_logger().info(f"Successfully opened video: {video_path}")
        return True

    def on_deactivate_callback(self) -> bool:
        if self._cap:
            self._cap.release()
        return True

    def on_step_callback(self):
        if self._cap is None:
            self.get_logger().warn("VideoCapture not initialized")
            return

        # read frame
        ret, frame = self._cap.read()

        # restart video at end.
        if not ret:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # rewind
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().error("Failed to read video frame after rewind")
                return
        
        # publish frame using cv_bridge
        self.video_feed = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
```

Note that we are using
```python
from cv_bridge import CvBridge
import cv2
```

We can add these as requirements in `source/component_utils/requirements.txt
```txt
cv_bridge
opencv-python==4.11.0.86
```

The component description is defined by `videoplayer.json` which should be replaced with the following:

```json
{
  "$schema": "https://docs.aica.tech/schemas/1-1-1/component.schema.json",
  "name": "Video Player Component",
  "description": {
    "brief": "Outputs a video as if it is a camera.",
    "details": "TODO"
  },
  "registration": "component_utils::VideoPlayer",
  "inherits": "modulo_components::LifecycleComponent",
  "parameters": [
    {
      "display_name": "Video Path",
      "description": "Path to the video file.",
      "parameter_name": "video_path",
      "parameter_type": "string",
      "default_value": ""
    }
  ],
  "outputs": [
    {
      "display_name": "Video Feed",
      "description": "The image with bounding segmentation information drawn on it.",
      "signal_name": "video_feed",
      "signal_type": "other",
      "custom_signal_type": "sensor_msgs::msg::Image"
    }
  ]
}
```
This defines parameters and inputs.


Build and Load the component in terminal, enter the component folder and run  
`docker build -f aica-package.toml -t objectdetection .`  
Next, configure AICA Studio and add `objectdetection` under **Custom Packages**.

Store your video in the same location as the model files, or add a separate volume.

`Component Utils` should be found under *Add Component*, add **Video Player Component** and set **auto-configure** and **auto-activate**. In the component settings, set the `Video Path` parameter, e.g. `/files/video.MOV`.