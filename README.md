# Localization Test GUI

Web control panel for running AUV localization tests from a browser.

The intended deployment is:

- Ubuntu robot PC runs ROS 2, MAVROS, DVL, EKF, this GUI server, and bag recording.
- MacBook opens the web page over Ethernet.
- MacBook ROS `joy_node` can publish `/joy` over DDS, while this GUI monitors `/joy` health from the robot PC.

## One-click Run

On the Ubuntu robot PC:

```bash
./kmu26_auv_web_gui/scripts/start_kmu26_auv_web_gui.sh
```

To add a desktop/app launcher on Ubuntu:

```bash
./kmu26_auv_web_gui/scripts/install_desktop_launcher.sh
```

After that, open **KMU26 AUV Web GUI** from the app menu or desktop icon.

## ROS Run

```bash
ros2 run kmu26_auv_web_gui server --host 0.0.0.0 --port 8080
```

Then open:

```text
http://<ubuntu-robot-ip>:8080
```

## Python Dependencies

This package uses FastAPI and Uvicorn for the web server.

```bash
python3 -m pip install fastapi uvicorn
```

## Vision Control Tab

The **Vision Control** tab integrates the `auv_buoy_vision_control` package without
changing the existing localization controls. It can start and stop these local ROS 2
launch files:

```text
auv_buoy_vision_control/laptop_yolo_detection.launch.py
auv_buoy_vision_control/auv_bbox_controller.launch.py
```

The tab monitors the completed YOLO inference frame from
`/vision/yolo/annotated/compressed`, `/vision/buoy_bbox`,
`/mission/state`, `/mission/control_enable`, and `/mission/rc_command`. Bounding
boxes are rendered by the detector on the exact frame used for inference, so the web
server does not need OpenCV. The detector model path and mission control parameters
are applied when their process is started.

The vision console shows all 18 values calculated by the mission controller on its
dedicated `/mission/rc_command` monitor topic. This mirrors the controller's output to
`/mavros/rc/override` without mixing in joystick or other publisher messages. The
configured vertical, yaw, and forward channels are labeled and highlighted without
hiding the remaining channels. Normal commands are displayed as PWM microseconds;
MAVROS special commands are displayed as `RELEASE` for raw `0` and `NO COMMAND` for
raw `65535`.

Build and source a workspace containing both packages before starting the GUI. Keep
autonomous control disabled until the camera, depth sign, PWM direction, and MAVROS
mode have been checked on the real vehicle.
