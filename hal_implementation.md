# HAL-Based Implementation: Rescue People

This document details the autonomous navigation, coordinate transformation, and computer vision pipeline implemented using JdeRobot's **Hardware Abstraction Layer (`HAL`)** and **`WebGUI`** APIs to solve the *Rescue People* challenge.

---

## 1. System Architecture & State Machine

To ensure predictable, modular behavior and clear terminal telemetry during execution, the mission logic is structured as a deterministic **Finite State Machine (FSM)** running inside the `Frequency.tick()` control loop:

1. **`TAKEOFF`:** Commands the quadrotor to ascend to the target search altitude ($z = 4.0\text{ m}$) using `HAL.takeoff(ALTITUDE)` and records the initial boat deck coordinates $(X_{\text{start}}, Y_{\text{start}}, Z_{\text{start}})$.
2. **`GO_TO_ZONE`:** Navigates in a straight line at constant altitude and fixed heading ($\text{yaw} = 0.0\text{ rad}$) toward the estimated midpoint of the survivors using `HAL.set_cmd_pos(x, y, z, az)`.
3. **`SPIRAL`:** Executes an outward **Archimedean Spiral** around the survivors' midpoint while continuously analyzing frames from the ventral camera (`HAL.get_ventral_image()`).
4. **`RETURN`:** Triggered once all expected victims are found or the maximum search radius is reached, commanding the drone back to $(X_{\text{start}}, Y_{\text{start}})$.
5. **`LANDING`:** Initiates descent onto the safety boat deck via `HAL.land()` and monitors vertical altitude until touchdown.
6. **`FINISHED`:** Prints the final deduplicated rescue report containing both GPS and local metric coordinates of all located survivors.

---

## 2. Coordinate Frame Transformation (GPS $\leftrightarrow$ Local ENU)

The `HAL.set_cmd_pos(x, y, z, az)` API operates in a local metric **ENU (East-North-Up)** Cartesian frame relative to the simulation world origin, where $+X$ points **East**, $+Y$ points **North**, and $+Z$ points **Up**.

Given the reference GPS coordinates in Degrees, Minutes, and Seconds (DMS):
* **Safety Boat (Takeoff Origin):** $40^\circ 16' 48.2''\text{ N},\; 3^\circ 49' 03.5''\text{ W}$
* **Estimated Survivors Midpoint:** $40^\circ 16' 47.23''\text{ N},\; 3^\circ 49' 01.78''\text{ W}$

First, DMS values are converted to signed decimal degrees ($\text{lat}, \text{lon}$). Because the operational area spans less than $100\text{ m}$, a local tangent-plane metric projection provides sub-centimeter accuracy relative to full ellipsoidal UTM:

$$M_{\text{lat}} = 111\,132.9\text{ m/deg}, \qquad M_{\text{lon}} = 111\,319.5 \cdot \cos(\text{lat}_{\text{boat}})\text{ m/deg}$$

The relative displacement vector from the safety boat to the search center is computed as:

$$\Delta X_{\text{ENU}} = (\text{lon}_{\text{survivors}} - \text{lon}_{\text{boat}}) \cdot M_{\text{lon}} \approx +40.63\text{ m (East)}$$

$$\Delta Y_{\text{ENU}} = (\text{lat}_{\text{survivors}} - \text{lat}_{\text{boat}}) \cdot M_{\text{lat}} \approx -29.93\text{ m (South)}$$

Whenever a survivor is detected at local position $(X_c, Y_c)$, the inverse transformation recovers their global GPS coordinates for the final rescue report:

$$\text{lat}_{\text{victim}} = \text{lat}_{\text{boat}} + \frac{Y_c - Y_{\text{start}}}{M_{\text{lat}}}, \qquad \text{lon}_{\text{victim}} = \text{lon}_{\text{boat}} + \frac{X_c - X_{\text{start}}}{M_{\text{lon}}}$$

---

## 3. Area Coverage: Archimedean Spiral

Once the drone reaches the estimated midpoint $(X_{\text{center}}, Y_{\text{center}})$, it systematically sweeps the surrounding water using an **Archimedean Spiral**, whose polar radius $r$ grows linearly with the swept angle $\theta$:

$$r(\theta) = a + b\theta = b' \cdot \frac{\theta}{2\pi}$$

where:
* $a = 0\text{ m}$ (starting from the center of the search area),
* $b' = 3.5\text{ m}$ is the radial spacing between consecutive loops, chosen so that adjacent passes overlap within the ventral camera's ground footprint at $4.0\text{ m}$ altitude,
* $\theta$ starts at a small initial angle ($\theta_0 = 20^\circ$) and increments by $\Delta\theta = 20^\circ$ each time the drone comes within `TOLERANCE` ($1.0\text{ m}$) of the active waypoint.

At each step, polar coordinates $(r, \theta)$ are projected back into local Cartesian waypoints:

$$\Delta X = r \cos(\theta), \qquad \Delta Y = r \sin(\theta)$$

$$X_{\text{new}} = X_{\text{center}} + \Delta X, \qquad Y_{\text{new}} = Y_{\text{center}} + \Delta Y$$

To prevent unbounded exploration over open water, the spiral terminates immediately when either all expected targets are registered (`EXPECTED_SURVIVORS = 6`) or the radial distance exceeds the safety boundary (`MAX_RADIUS = 25.0 m`).

---

## 4. Computer Vision & Face Recognition Techniques

Real-time victim detection is performed on the ventral camera feed (`HAL.get_ventral_image()`) using OpenCV's **Viola-Jones Haar Cascade** classifier (`haarcascade_frontalface_default.xml`):

1. **Multi-Orientation Rotation Search ($8$ Angles):**
   Standard frontal Haar Cascades fail when a face is tilted more than $\approx 25^\circ$. Because floating survivors lie in arbitrary orientations in the water while the drone flies in *Heads-Free* mode ($\text{yaw} = 0.0\text{ rad}$), every grayscale frame is evaluated across $8$ discrete orientations:
   $$\alpha \in \{0^\circ, 90^\circ, 180^\circ, 270^\circ, 45^\circ, 135^\circ, 225^\circ, 315^\circ\}$$
   As soon as a face is detected at angle $\alpha$, bounding boxes are drawn on the rotated frame and the search short-circuits to save CPU cycles.

2. **Wave & Sun-Glint False-Positive Suppression:**
   Applying global histogram equalization (`cv2.equalizeHist`) was tested during development but discarded because it amplified contrast on ocean wave crests, causing false-positive detections. Operating directly on raw grayscale intensity combined with `minNeighbors=5` and `minSize=(22, 22)` effectively rejects water reflections while preserving true facial detections.

3. **Spatial Deduplication (`DUPLICATE_RADIUS`):**
   Because the drone captures multiple consecutive frames while flying over the same person, every candidate detection at $(X_c, Y_c)$ is checked against all previously stored victim coordinates $(X_i, Y_i)$ using the Euclidean distance:
   $$d_i = \sqrt{(X_c - X_i)^2 + (Y_c - Y_i)^2} \ge 2.5\text{ m}$$
   A detection is registered as a new survivor only if its distance to every previously logged victim is at least $2.5\text{ m}$.

4. **Dual-Panel WebGUI Telemetry:**
   * **Right Panel (`WebGUI.showImage`):** Displays the live RGB feed annotated with current FSM state, survivor counter, and green bounding boxes around detected faces.
   * **Left Panel (`WebGUI.showLeftImage`):** Displays the processed grayscale inspection frame used by the cascade classifier.

---

## 5. Tuned Flight & Perception Parameters

| Parameter | Value | Purpose |
| :--- | :--- | :--- |
| `ALTITUDE` | $4.0\text{ m}$ | Balances ventral camera ground coverage against facial pixel resolution. |
| `SPIRAL_SPACING` ($b'$) | $3.5\text{ m}$ | Radial distance between spiral turns to guarantee full visual overlap. |
| `THETA_STEP` ($\Delta\theta$) | $20^\circ$ ($0.349\text{ rad}$) | Small angular step producing smooth, near-circular spiral trajectories. |
| `TOLERANCE` | $1.0\text{ m}$ | Waypoint acceptance radius to maintain steady motion without abrupt stops. |
| `MAX_RADIUS` | $25.0\text{ m}$ | Maximum spiral search radius before aborting and returning to the boat. |
| `EXPECTED_SURVIVORS` | $6$ | Target survivor count triggering early return once all victims are found. |
| `DUPLICATE_RADIUS` | $2.5\text{ m}$ | Minimum separation distance to avoid registering the same person twice. |
| `scaleFactor` | $1.1$ | Image pyramid scaling step in `detectMultiScale`. |
| `minNeighbors` | $5$ | Minimum overlapping candidate rectangles required to confirm a face. |
| `minSize` | $(22, 22)\text{ px}$ | Minimum bounding-box size in pixels to filter distant wave noise. |

---

## 6. Experimental Results & Landing Detection Fix

### Detection Performance
In this first HAL-based implementation, the drone consistently navigates directly to the survivors' zone, executes the Archimedean spiral, and **reliably locates at least 5 out of the 6 survivors** (and up to 6/6 depending on viewing angle and lighting) before autonomously returning to the safety boat.

### Boat Deck Touchdown Fix
During initial testing, the drone completed the return maneuver and landed smoothly on the safety boat, but the final coordinate report in the `FINISHED` state did not print. This occurred because the safety boat's deck sits above sea level ($Z_{\text{start}} > 0.35\text{ m}$), so checking a fixed threshold `cz <= 0.35` never evaluated to `True`. Recording the initial deck elevation `start_z = float(init_pos[2])` at startup and checking `cz <= start_z + 0.25` resolves this issue and ensures the final rescue summary prints immediately upon touchdown.

---
