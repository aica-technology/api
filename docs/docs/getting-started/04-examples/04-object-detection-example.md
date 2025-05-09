---
sidebar_position: 4
---

# Using YOLO to Track Objects

This example creates a custom component used with AICA Core v4.3.2, `collections/object-detection`, and `collections/intel-realsense-collection`. If you do not have a RealSense camera, an alternative custom component that plays a video is provided.

## Using the YOLO Executor
### Setup
Launch AICA Studio with:
- AICA Core v4.3.2
- `collections/object-detection`
- `collections/intel-realsense-collection`
- Under **Advanced Settings**, add a volume. Browse to the folder where you will store your YOLO model files and link it to `/files`.

### Adding Components

- Remove the **Hardware Interface**.
- Add the following components:
  - `Object Detection Components/YOLO Executor`
  - `Realsense2 Camera`

> If you do not have a camera, you can [create a video player component](#create-a-video-player-component) instead.

- Connect both components’ **load nodes** to load on program start.
- Connect the **RGB Image** output of the `Realsense2 Camera` to the **RGB Image** input of the `YOLO Executor`.

### Configuring YOLO Executor

- Set to **auto-configure** and **auto-activate**.
- Set the following:
  - **Model Path**: YOLO model in `.onnx` format e.g., `/files/yolo12n.onnx`
  - **Classes Path**: Path to the class labels e.g., `/files/coco.yaml`
  - **Rate**: This is hardware dependent but 3 should work on most machines.

To create an onnx YOLO model you can [download](https://github.com/sunsmarterjie/yolov12) a `.pt` model, (we use the lightweight version — YOLO12n). Then use Python (with `ultralytics` installed) to convert the `.pt` model to `.onnx`:
```python
from ultralytics import YOLO

# Load the model
model = YOLO("yolov12n.pt")

# Export the model to ONNX format
model.export(format="onnx")  # creates 'yolov12n.onnx'
```

These YOLO models are pre-trained on coco - you can download the Classes file for coco [here](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml). 

### Running the Application

- Launch the application in AICA Studio.
Open RViz using the gear menu icon in the bottom right of AICA Launcher and choosing the "Launch RViz" option.
- Open **RViz** → **Add** → **By topic** → `/yolo_executor/annotated_image/Image`  
  to view the YOLO model's annotated output.

> Only users with a Linux host can visualize the robot with RViz. On macOS, AICA Launcher will not show the RViz option.

## Tracking an Object with YOLO
We can use the bounding boxes from the YOLO Executor to track an object with a robot. 
To do this we:
- Perform object detection with the YOLO Executor component, based on an camera or video feed. 
- Create a marker in 3D space based on the position of YOLO's bounding boxes.
- Drive the robot towards the marker.

AICA Studio provides most of the tools to do this, but we will need to create a simple component to create a marker from bounding boxes.
We outline a component which takes the json output from yolo as well as camera parameters and estimates the object's position. For simplicity we assume that the camera position is fixed, pointing down at a  known position, e.g., `(x=0, y=0.6, z = 0.6)`, and that the objects are at `z=0`. For more advanced pose estimation we can use `foundation_pose_estimator component` with the `GroundedSAM` or `GroundedSAM2` components.

Start creating a custom component:
1. **Set up the repository:**
   - Use the [component-template](https://github.com/aica-technology/component-template).
   - Clone it, enter the directory, and run:
     ```bash
     ./initialize_package.sh
     ```
   - Name it `component_utils` and include only a **Python Lifecycle Component**.

    **Create files and classes:**
    - In `source/component_utils/component_utils/`:
      - Create `videoplayer.py`

    - In `source/component_utils/component_descriptions/`:
      - Create `videoplayer.json`

    - In `source/component_utils/setup.cfg`:
      - Register the component with:
        ```cfg
        component_utils::VideoPlayer = component_utils.videoplayer:VideoPlayer
        ```

## Create a Video Player Component
To emulate a camera by playing a video frame by frame, [create a custom component](#tracking-an-object-with-yolo).



1. **Create files and classes:**

   - In `source/component_utils/component_utils/`:
     - Create `videoplayer.py`

   - In `source/component_utils/component_descriptions/`:
     - Create `videoplayer.json`

   - In `source/component_utils/setup.cfg`:
     - Register the component with:
       ```cfg
       component_utils::VideoPlayer = component_utils.videoplayer:VideoPlayer
       ```
2. **Create Python component.**
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
3. **Update json.**

Replace the contents of `videoplayer.json` with the following:

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

4. **Build and Load the Component**  
In terminal, enter the component folder and run  
`docker build -f aica-package.toml -t objectdetection .`  
Next, configure AICA Studio and add `objectdetection` under **Custom Packages**.

You should see `Component Utils` under *Add Component* and be able to add the **Video Player Component**.  
In the component settings, set the `Video Path` parameter, e.g. `/files/video.MOV`.

You should store your video in the same location as the model files.