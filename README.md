# 🧠 ROS2 Project – Autonomous TurtleBot with RGB Detection and Motion Planning

This project features a **ROS2-based autonomous TurtleBot** operating within a Gazebo simulation. The robot uses a combination of **computer vision (OpenCV)** and **motion planning** to explore its environment, detect colored blocks (red, green, blue), and autonomously navigate toward the blue one — stopping approximately 1 meter away from it.

---

## 🎯 Project Objective

> Simulate an autonomous robotic system that:
- Uses a live camera feed to detect red, green, and blue blocks.
- Highlights the detected colors using bounding boxes and labels.
- Navigates toward the **blue block** and stops within **~1 meter** of it.
- Uses simple **randomized exploration** and **reactive obstacle avoidance** to move around the environment when no blue block is in sight.

---

## 🤖 How the Robot Operates

### 🔍 1. **Color Detection (OpenCV)**
- The robot subscribes to the `/camera/image_raw` topic and processes each frame.
- It uses **HSV thresholding** to detect red, green, and blue blocks.
- For each detected color:
  - Draws a **bounding box**, labels the color + estimated distance.
  - Calculates the **center of the object** using `cv2.moments()`.

### 🚶 2. **Exploration Behavior**
- When no blue box is detected, the robot:
  - Moves forward slowly.
  - Randomly rotates left or right using `angular.z = ±random()`.
- This allows it to cover the environment gradually without a predefined map.

### 📦 3. **Blue Block Centering**
- If a **blue block is detected** but **off-center**, the robot:
  - Rotates left or right to align the object to the center of the frame.
  - Only starts moving toward it when it is properly centered.

### 📏 4. **Distance-Based Stopping**
- The robot estimates distance based on the **area of the blue block's contour**.
- Once the robot is **within a 1-meter ± tolerance window**, it stops moving.

### 🛑 5. **Obstacle Avoidance**
- Uses LiDAR from the `/scan` topic.
- If an object is detected within `0.4 meters`, the robot:
  - Stops forward motion.
  - Performs a rotation or reverse + turn to avoid collision.

---

# Run the Python node
ros2 run ros2_project_sc22rhsm first_step.py
