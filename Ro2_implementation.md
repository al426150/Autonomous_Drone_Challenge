# Direct ROS 2 Implementation (Aerostack2): Rescue People

This document explains the direct **ROS 2 (`rclpy`)** solution for the **Rescue People** autonomous drone mission. Instead of using the `HAL` abstraction layer, this version communicates directly with the **Aerostack2** simulation interfaces under the `/drone` namespace, following the official ROS 2 guidelines from the [JdeRobot Robotics Academy Rescue People documentation](https://jderobot.github.io/RoboticsAcademy/exercises/Drones/rescue_people).

## 1. How the Code Works (Simple Overview)

To keep the implementation simple and reliable, everything runs inside a single ROS 2 node (`RescuePeopleDirectNode`) executed by a $30\text{ Hz}$ timer (`control_loop`), as that was the limit that Unibotics indicated me in the CPU version. As specified in the official documentation, importing `WebGUI` initializes `rclpy` internally and bridges the debug image topics to the browser interface.

Instead of calling high-level `HAL` wrappers, the node uses standard ROS 2 topics, services, and `TF2` transforms:

| Task | ROS 2 / Aerostack2 Interface | Type | How It Is Used |
| :--- | :--- | :--- | :--- |
| **Read Drone Position** | `/tf` (`earth` $\rightarrow$ `drone/base_link`) and `/drone/self_localization/pose` | `tf2_ros` / `PoseStamped` | Reads the drone's local ENU coordinates $(X_c, Y_c, Z_c)$ and heading $\psi$ at $30\text{ Hz}$. |
| **Arm & Takeoff / Land** | `/drone/set_arming_state`, `/drone/set_offboard_mode`, `/drone/platform/state_machine_event` | `SetBool` / `SetPlatformStateMachineEvent` | Arms the motors, enables offboard mode, and triggers `ARM`, `TAKE_OFF`, `TOOK_OFF`, `LAND`, and `LANDED`. |
| **Switch Control Mode** | `/drone/controller/set_control_mode` | `SetControlMode` | Switches Aerostack2 between `POSITION` mode (takeoff & landing) and `SPEED` mode (transit & spiral). |
| **Move by Position** | `/drone/motion_reference/pose` | `geometry_msgs/msg/PoseStamped` | Holds a fixed target point during takeoff, final boat alignment, and vertical touchdown. |
| **Move by Velocity** | `/drone/motion_reference/twist` | `geometry_msgs/msg/TwistStamped` | Flies smoothly in ENU coordinates during transit and the outward spiral sweep. |
| **Platform Status** | `/drone/platform/info` | `as2_msgs/msg/PlatformInfo` | Checks whether the drone is armed, in offboard mode, landed, taking off, or flying. |
| **Ventral Camera** | `/drone/ventral_cam/image_raw` | `sensor_msgs/msg/Image` | Converts camera frames to OpenCV images via `CvBridge` to detect survivors. |
| **WebGUI Panels** | `/webgui/image_debug_right` & `/webgui/image_debug_left` | `sensor_msgs/msg/Image` | Streams the annotated color feed (right) and processed grayscale feed (left) to the browser. |

## 2. Step-by-Step Mission Flow

At every tick of the $30\text{ Hz}$ control loop, the drone updates its position from `TF2`, processes the latest camera frame, and advances through an **8-state machine**:

1. **`WAIT_CONNECTION`:** Waits until `/drone/platform/info` and `TF2` localization are active, records the boat deck starting position $(X_{\text{start}}, Y_{\text{start}}, Z_{\text{start}})$, and calculates the survivors' zone center $(X_{\text{center}}, Y_{\text{center}})$.
2. **`TAKEOFF`:** Arms the drone, activates offboard mode, and sends `ARM` $\rightarrow$ `TAKE_OFF` $\rightarrow$ `TOOK_OFF` events while climbing to $4.0\text{ m}$ above the boat deck.
3. **`GO_TO_ZONE`:** Switches to velocity control (`SPEED` in `LOCAL_ENU_FRAME`) and flies toward the search center at $5.5\text{ m/s}$, braking smoothly on arrival.
4. **`SPIRAL`:** Sweeps the water along an outward **Archimedean Spiral** ($2.8\text{ m}$ loop spacing, starting at $\theta_0 = 180^\circ$) at a steady $2.8\text{ m/s}$. Each frame is checked with OpenCV's Haar Cascade across $8$ angles ($0^\circ, 90^\circ, 180^\circ, 270^\circ, 45^\circ, 135^\circ, 225^\circ, 315^\circ$).
5. **`RETURN`:** As soon as all $6$ survivors are located (or radius $> 20.0\text{ m}$), flies back toward $(X_{\text{start}}, Y_{\text{start}})$ at $2.8\text{ m/s}$, slowing down early so it does not overshoot the boat.
6. **`ALIGN_BOAT`:** Within $3.0\text{ m}$ of the boat, switches back to `POSITION` control and holds $(X_{\text{start}}, Y_{\text{start}})$ until centered within $0.65\text{ m}$ for $\approx 0.67\text{ s}$.
7. **`LANDING`:** Sends the `LAND` event while commanding a vertical descent onto the boat deck ($Z_{\text{start}} - 0.10\text{ m}$). Once on deck, sends `LANDED` and disarms the motors.
8. **`FINISHED`:** Prints the final rescue report with the GPS and local ENU coordinates of all $6$ survivors.

## 3. Coordinate Conversions & Camera Projection

### GPS to Local ENU Coordinates

Using local flat-earth scale factors at the safety boat's latitude ($M_{\text{lat}} = 111\,132.9\text{ m/deg}$ and $M_{\text{lon}} = 111\,319.5 \cdot \cos(\text{lat}_{\text{boat}})\text{ m/deg}$), the survivors' GPS target is converted into local meters relative to the boat:

$$X_{\text{center}} = X_{\text{start}} + (\text{lon}_{\text{surv}} - \text{lon}_{\text{boat}}) \cdot M_{\text{lon}}, \qquad Y_{\text{center}} = Y_{\text{start}} + (\text{lat}_{\text{surv}} - \text{lat}_{\text{boat}}) \cdot M_{\text{lat}}$$

### Orientation & Survivor Ground Projection

The drone's yaw angle $\psi$ is extracted from its `TF2` orientation quaternion $(q_x, q_y, q_z, q_w)$:

$$\psi = \text{atan2}\big(2(q_w q_z + q_x q_y),\; 1 - 2(q_y^2 + q_z^2)\big)$$

When a face is detected inside the central $84\%$ of the ventral image, its unrotated pixel offset $(\Delta u, \Delta v)$ from the image center is projected onto the water plane using the calibrated field-of-view scale $s_{\text{FOV}} = 1.65$:

$$s_{\text{px}} = \frac{\max(Z_c, 1.0) \cdot s_{\text{FOV}}}{W_{\text{img}}}$$

$$X_{\text{face}} = X_c - \Delta v \cdot s_{\text{px}}, \qquad Y_{\text{face}} = Y_c - \Delta u \cdot s_{\text{px}}$$

Detections separated by at least `DUPLICATE_RADIUS = 3.2 m` from already recorded victims are saved and converted back to GPS coordinates.

## 4. Performance & Simulation Results

* **Survivors Detected:** $6 / 6$ unique detections (zero duplicates).
* **Execution Time:** $\approx 9\text{ minutes}$ wall-clock time at **$40\%$ Gazebo Real-Time Factor (RTF)**, which corresponds to **$\approx 3.6\text{ minutes}$ of simulated flight time**.
* **Landing Accuracy:** Smooth handoff from velocity transit to position hold over the safety boat, touching down reliably on the deck.
