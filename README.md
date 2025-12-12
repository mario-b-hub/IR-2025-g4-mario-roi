# Autonomous Maze Navigator: Reactive LiDAR Navigation System

## Project Overview
This repository contains a ROS 2 Python node designed to navigate unknown maze environments autonomously. Unlike mapping-based approaches (SLAM), this system utilizes a **reactive behavior architecture**. It processes raw LiDAR data in real-time to make deterministic decisions based on immediate environmental geometry.

The robot is capable of solving:
1.  **Simple loops** (Square donuts).
2.  **Complex turns** (Left and Right corners).
3.  **Dead-ends and T-Junctions** (Cross/Corridor intersections).

---

## 1. System Architecture: The "Sensing" Logic

To navigate reliability, the robot cannot rely on single laser rays, which are prone to noise or "infinite" readings when angles are steep. Instead, we implemented a **Sector Averaging System**.

### Virtual Sensors
Instead of looking at 360 points, the algorithm abstracts the `LaserScan` data into three critical sectors. We define a "window" of degrees around specific angles and compute the arithmetic mean of valid readings (filtering out `inf` and `nan` values).

* **Front Sector (0 rad):** Detects immediate collisions.
* **Left Sector (+45 deg):** Measures lateral clearance.
* **Right Sector (-45 deg):** Measures lateral clearance.

![A top-down view of the robot in Gazebo. Overlay three red laser beams projecting from the robot's LiDAR.](/LIDAR_LASERS.png)

---

## 2. Navigation Logic: Finite State Machine

The core intelligence is governed by a **Finite State Machine (FSM)** with two primary modes of operation. This ensures the robot commits to an action (driving or turning) and doesn't oscillate unpredictably between them.

### State 0: Corridor Centering (The "Cruise" Mode)
When the path ahead is clear (Distance > `STOP_DIST`), the robot moves forward. However, it doesn't just move blindly; it actively centers itself.

We utilize a **Proportional Controller (P-Control)** logic to maintain the centerline. The angular velocity ($\omega$) is calculated based on the error between the left and right distances:

$$Error = Distance_{left} - Distance_{right}$$
$$\omega_{correction} = K_p \times Error$$

* If the robot is too close to the **left**, the error becomes negative, pushing the robot to the **right**.
* If the robot is too close to the **right**, the error becomes positive, pushing the robot to the **left**.

![Capture the robot moving through a straight corridor, but slightly off-center (closer to the left wall). Add an arrow indicating the robot's forward motion, and a curved arrow indicating the rotational correction being applied to push it back towards the center.](AUTOMATIC_POSITON_CENTERING_DECISION.png)

### State 1: The Decision Threshold (Cornering)
When the Front Sector detects a wall closer than the threshold (`STOP_DIST`), the robot triggers a state transition. This is not a random turn; it is a **calculated decision** based on available geometry.

1.  **Stop & Analyze:** The robot halts linear motion.
2.  **Compare Sectors:** The system compares `self.left_dist` vs `self.right_dist`.
3.  **Latch Decision:**
    * If $Left > Right$: Set direction multiplier to **-1** (Turn Left).
    * If $Right > Left$: Set direction multiplier to **+1** (Turn Right).

This dynamic decision-making allows the robot to handle T-junctions and Cross-junctions by mathematically choosing the "path of least resistance" (the largest opening).

![The robot facing a L-junction (a wall directly in front, open space to the left). Text overlays: Show "Left Dist: 1.2m" and "Right Dist: 0.5m" to visually justify why the robot is about to turn Left.](TURN_ORIENTATION_DECISION.png)

---

## 3. Hysteresis and Stability

A common failure point in reactive robots is "jittering" at the threshold of a state change. To prevent this, we implemented **Hysteresis** in the distance thresholds.

* **Entering Turn State:** Happens when obstacle is at **1.0 meters** (`STOP_DIST`).
* **Exiting Turn State:** Only happens when the path is clear up to **2.5 meters** (`RESUME_DIST`).

This gap ensures the robot completes the turn fully and is facing a clear long corridor before it attempts to switch back to the "Cruise" logic. This prevents the robot from trying to drive forward while still facing the corner of a wall.

![A render showing the robot performing a 90-degree turn. Mark the state of the robot (wich is STOPED) with a red circle on the flor. A overlay text: Mesuring distance: 1.0m, justifying why the robot has stop.](STOP_STATE_DECISON.png)

---

## Technical Specifications & Tuning

| Parameter | Value | Description |
| :--- | :--- | :--- |
| `STOP_DIST` | 1.0 m | Safety buffer to initiate turns. |
| `RESUME_DIST` | 2.5 m | Clearance required to resume forward motion. |
| `KP` | 0.45 | Proportional gain for corridor centering. |
| `Control Loop` | 0.2s | Frequency of the decision cycle (5 Hz). |

## How to Run
*(Include here your specific commands to launch the simulation and the node)*
