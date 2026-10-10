understanding : 
- drone takes off, flies down a hallway while recentering, lands (all by itself);
- everything is calculated in real-time
- we dont interact with the drone (terminal) after the drone launches so essentially it is autonomous. 

state machine : 
- three states (takeoff, flying, landing)
- lands when frontwall is detected. 
- Raw LiDAR rays can be noisy. Instead of trusting a single laser beam, the code looks at a small slice of angles (like a 5-degree sector) to the left, right, and front, and takes the median value. This keeps the drone smooth and prevents jittery movements
- while cruising continuously checks if right wall distance matchces left wall distance;
- lateral velocity adjustment (this is something which is fuzzy for me
)

my addition to the base project 
- tried to implement diagnosis of problem and future possibility of making this system agentic (auto adjustment)
- cant send raw logs to llm (ofc). so calculating mean, std, iqr, min, max over a n-second timeline. send this to llm over a comm channel and get a diagnosis and then return commands to drone to self-correct or change course (depending on use case);
- we can use langgraph or langsmith to make even complex workflows surrounding agentic hardware connections. 






```markdown
# Setup & Execution Guide

Follow these steps to set up your environment, build the package, and run the autonomous corridor-centering drone simulation.

---

## 1. Setup Check Instructions

Before building or running anything, verify that your development environment is properly configured:

1. **Verify OS and ROS 2 Distribution:**
   Ensure you are running Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic installed:
   ```bash
   ros2 --version
   gz sim --version

```

2. **Verify Workspace Structure:**
Confirm your workspace contains the `corridor_drone_pkg` inside your `src/` directory:
```bash
cd ~/corridor_ws
ls src/corridor_drone_pkg

```


*You should see folders like `launch/`, `worlds/`, and `corridor_drone_pkg/`.*
3. **Source ROS 2 Environment:**
Load the core ROS 2 environment variables into your terminal:
```bash
source /opt/ros/jazzy/setup.bash

```



---

## 2. Build Instructions

1. Navigate to your workspace root:
```bash
cd ~/corridor_ws

```


2. Build the package using `colcon` with symlink installation enabled (this allows Python file changes to take effect immediately without rebuilding):
```bash
colcon build --packages-select corridor_drone_pkg --symlink-install

```


3. Source your local workspace overlay:
```bash
source install/setup.bash

```



---

## 3. Run Simulation Instructions

1. Launch the entire simulation stack (this starts Gazebo Harmonic with the 30-meter corridor, boots up the parameter bridge, and starts the controller with a 5-second stabilization delay):
```bash
ros2 launch corridor_drone_pkg corridor_sim.launch.py

```


2. **What to Expect During Execution:**
* **Startup Delay (0–5s):** Gazebo boots up, stabilizes physics, and spawns the quadcopter safely on the floor.
* **Takeoff (5s–8s):** The drone automatically triggers its vertical ascension command.
* **Cruise & Centering:** The drone moves forward ($0.5\text{ m/s}$) while dynamically adjusting its lateral velocity using sector-filtered LiDAR ranges to stay dead-center between the walls.
* **Landing & Stop:** Upon detecting the closing front wall, it executes a smooth vertical descent and safely terminates.
* **Telemetry:** The terminal prints real-time logs alongside 30-second statistical JSON summaries (`mean`, `std`, `min`, `max`, `iqr`) ready for LLM agent diagnostics.


3. **Verify Linter & Tests (Optional):**
To make sure your codebase passes all PEP 8 style checks and evaluation requirements:
```bash
colcon test --packages-select corridor_drone_pkg --event-handlers console_direct+

```



```

```