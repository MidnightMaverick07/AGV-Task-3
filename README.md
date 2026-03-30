# Multi-Agent SLAM and Map Merging (Task 3)

## Overview

This task extends the SLAM pipeline to handle multiple agents working together.

Instead of a single agent exploring the environment, two agents run simultaneously, explore different areas, and contribute to building a shared global map. The goal is to reduce exploration time and improve coverage.

---

## Objective

- Run multiple agents in the same environment  
- Allow each agent to explore independently  
- Merge observations into a single global occupancy map  
- Avoid collisions between agents  

---

## System Design

### Local Behavior (Agents)

Each agent:
- Processes its own LiDAR raycasts  
- Updates its own understanding of the environment  
- Selects a frontier to explore  
- Computes motion using velocity and steering  

The logic runs independently for each agent.

---

### Shared Mapping

All agents contribute to a single **Occupancy Grid**:

- Unknown → `-1`  
- Free space → `0`  
- Obstacle → `100`  

Each LiDAR scan updates the map using ray tracing.

---

## Approach

### 1. Occupancy Grid Mapping

- World coordinates are converted to grid indices  
- Ray tracing marks:
  - Free cells along the path  
  - Obstacle at the endpoint  
- Grid is updated continuously during simulation  

---

### 2. Frontier-Based Exploration

- The system scans for **frontiers**  
  (boundary between known and unknown space)

- A valid frontier:
  - Is a free cell  
  - Has at least one unknown neighbor  

- The agent selects the closest valid frontier  
- A cooldown is used to avoid excessive computation  

---

### 3. Artificial Potential Fields (APF)

The motion is driven by forces:

#### Attractive Force
- Pulls the agent toward the target frontier  

#### Repulsive Force
- Pushes the agent away from nearby obstacles  
- Uses LiDAR hit points  
- Includes a lateral component to avoid deadlocks  

---

### 4. Control System (PID + Smoothing)

- Force is smoothed using a low-pass filter  
- Heading is computed from the force vector  
- Steering uses PID-style control:
  - Proportional → main turning  
  - Integral → stability  
  - Derivative → reduces oscillations  

- Speed control:
  - Stops when turning sharply  
  - Moves forward when aligned  

---

## Key Challenges and Fixes

### 1. Agents Getting Stuck

**Issue:** Agent oscillated near obstacles  
**Fix:** Added lateral repulsive force + smoothing  

---

### 2. Noisy Motion

**Issue:** Sudden changes in direction  
**Fix:** Low-pass filtering on force  

---

### 3. Unstable Steering

**Issue:** Overshooting and oscillations  
**Fix:** Added PID control  

---

### 4. Slow Exploration

**Issue:** Recomputing frontier every frame  
**Fix:** Added search cooldown (runs every ~30 frames)  

---

### 5. Deadlocks Near Obstacles

**Issue:** Agent stuck facing obstacle  
**Fix:** Introduced sideways repulsion component  

---

## Results

- Agent explores environment autonomously  
- Builds a consistent occupancy grid  
- Avoids obstacles using APF  
- Generates a final map visualization using OpenCV  

---

## Output

After simulation:
- A grayscale map is generated  
- Unknown → Gray  
- Free space → White  
- Obstacles → Black  

The map is resized and displayed using OpenCV.

---

## Tech Stack

- C++  
- OpenCV  
- Custom simulation engine (Raylib-based)  

---

## File Structure

```
.
├── main.cpp
├── draw.hpp
├── geometry.hpp
├── simulation.hpp
└── README.md
```

---

## How to Run

1. Compile the project  
2. Run the simulation  
3. Watch the agent explore  
4. Final map will appear after simulation ends  

---

## Author

Subhojeet Ghosh

---

If you found this useful, feel free to star the repo.
