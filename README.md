# **Smart Cleaning Robot**

## **Overview**

This is a ROS-based autonomous cleaning robot that integrates indoor mapping, voice control, and innovative cleaning functionalities. This project features expandable modules, making it a valuable tool for research, education, and real-world applications.

**Develope and Test Environment: ROS Noetic, Python3, Ubuntu 20.04.6**

### **Key Features**

- **GUI Control Panel**: A user-friendly interface to manage all modules without command-line interaction.
- **Voice Control**: Real-time voice recognition for hands-free operation.
- **Mapping**: Efficient exploration and map saving using the Explore_Lite package.
- **Cleaning**: Two cleaning modules for full-coverage path planning, including a fully self-designed solution.

### **Develop Progress**

- Proposal => V1 => V2 => __Final Version of the Semester (Now)__
- Simulation Demo: https://youtu.be/rqXiXsVubhQ?si=gXh5D5b1vyPDEKCA

### Final Demo Setup: A Modular Network-Enabled Cleaning System

For the final demonstration, we will replicate a realistic cleaning scenario in a controlled environment. The setup ensures seamless communication and modular scalability, simulating a robust system for future multi-robot collaboration.

**Hardware Configuration:**

* TurtleBot3 (Cleaning Robot)
  * Role: A single cleaning robot performing mapping, exploration, and cleaning tasks.
  * Connectivity: Communicates with the central system via the router's local network.
* No Internet Access Local Network Router (Network Center)
  * Role: Acts as the network center to establish a local LAN for communication.
  * Features: Provides stable IP addresses for devices, enabling consistent communication across the network.
* Computer with Linux OS (Computing Center and Control Terminal):
  * Role: Acts as the central computing tower and the portable control panel
    * Running the GUI control panel, represents portable control panel
    * Managing mapping, exploration, and cleaning processes
    * Collecting and visualizing data in real-time.

**Benefits of the Setup:**

* Flexibility: Easy to transport and test in various locations with no dependency on external networks.
* Security: Local network ensures data is contained within the system, reducing security risks.
* Scalability: Paves the way for testing more complex, multi-robot systems in the same architecture.

## **Getting Started**

### **Program Entry**

To start the program:

1. **For Simulation**:

  Remember to set localhost settings in ~/.bashrc

```bash
   roslaunch control_panel panel_sim.launch
```

  No additional commands are required for simulation environment, this launch file handles all controls.

2. **For Real Robot**:

  Remember to set real IP settings in ~/.bashrc and update turtlebot3's settings.

  For Real Environment, you need to run roscore on PC firstly:

```bash
roscore
```

  Before bringup, if the environment has no Internet access, or no RTC module on turtlebot3, you need sync time.

  For Real Environment, you need to ssh to turtlebot3 and bring up it secondly:

```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

  After above instructions, you can now run the panel to start the program:

```bash
   roslaunch control_panel panel_real.launch
```

  After running, you must do following to avoid data loss or hardware damage:

```bash
sudo shutdown -h now
```

## **System Architecture**

### **Abstraction Modules**

<img src="attached_files/V2/presentation_clean_robot/Slide4.png" alt="GUI Screenshot" style="max-width: 100%; height: auto;">

### **Directory Structure**

<img src="attached_files/V2/presentation_clean_robot/Slide5.png" alt="GUI Screenshot" style="max-width: 100%; height: auto;">

Note: The Cleaning Module I (CCPP) is not in branch `master`, please see its implementation in branch `backup`.

---

## **Modules and Responsibilities**

### **1. Panel Module**

- **Developer**: Pang Liu
- **Description**:
  - Self-designed GUI for controlling all program modules.
  - Provides buttons and voice control integration for seamless operation.
- **Key Features**:
  - Start SLAM, exploration, and cleaning processes.
  - Save and load maps.
  - Route analysis and visualization in RViz.
  - Robot movement control.
  - Developer-friendly logs for debugging.
- Real running illustration:
  - <img src="attached_files/V2/project_resources/control_panel_gui.png" alt="GUI Screenshot" width="400">

### **2. Voice Control Module**

- **Developer**: Pang Liu
- **Description**:
  - Real-time voice recognition using the **Vosk model**.
  - Publishes recognized commands to the `voice_commands` topic.
  - Enables voice-activated control of exploration and cleaning.
- <img src="attached_files/V2/presentation_clean_robot/Slide10.png" alt="GUI Screenshot" style="max-width: 100%; height: auto;">

### **3. Mapping Module**

- **Developer**: Zhenxu Chen
- **Description**:

  - Based on the **Explore_Lite** package, customized for fast exploration and map saving.
- **Workflow**:

  1. **Start SLAM**: Launches `turtlebot3_slam.launch` for SLAM and RViz.

  - <img src="attached_files/V2/project_resources/slam_1_start_slam_gmapping.png" alt="GUI Screenshot" width="300">

  2. **Start Exploration**: Begins autonomous exploration using `explore.launch`.

  - <img src="attached_files/V2/project_resources/slam_2_start_exploration.png" alt="GUI Screenshot" width="300">
  - <img src="attached_files/V2/project_resources/slam_3_explore2.png" alt="GUI Screenshot" width="300">
  - <img src="attached_files/V2/project_resources/slam_3_explore3.png" alt="GUI Screenshot" width="300">

  3. **Save Map**: Saves the map as `.pgm` and `.yaml` files in the `/maps` directory.

  - <img src="attached_files/V2/project_resources/slam_4_save_map.png" alt="GUI Screenshot" width="300">

  4. **Finish Mapping**: Stops SLAM and exploration nodes.

  - <img src="attached_files/V2/project_resources/slam_5_saved_map_check.png" alt="GUI Screenshot" width="300">

### **4-1. Cleaning Module I**

- **Developer**: Zhenxu Chen
- **Description**:
  - Based on the **CCPP package** for full-coverage path planning and cleaning.
  - Utilizes `move_base` for navigation.
- Note: `<Cleaning Module I>` is developed at branch `backup`
- CCPP Package: https://wiki.ros.org/full_coverage_path_planner
  - The CCPP package will use saved map to plan a full coverage route and allow the robot following the route.
  - Video: https://drive.google.com/file/d/1F1Hh0JKD9KMvRVsC_EX5ZwptzUVWLEi8/view?usp=drive_link
  - <img src="attached_files/V2/project_resources/ccpp.png" alt="GUI Screenshot" width="300">

### **4-2. Cleaning Module II**

- **Developer**: Pang Liu
- **Description**:
  - Fully self-designed cleaning functionality split into two submodules:
    - **Route Analysis Submodule**:
      - Reads saved maps and analyzes routes using a three-value map (-1 for obstacles, 0 for uncleaned areas, 1 for cleaned areas).
      - Plans paths using sampling intervals and a greedy algorithm to find valid connections.
    - **Route Follow Submodule**:
      - Executes the planned path, marking cleaned areas in real-time (still under debugging).

#### **Sub Module I: Route Analysis**

- **Detailed introduction of `route_plan.py` (core script):**

  1. Get the latest map (map data of `OccupancyGrid` message type) through `/map` topic.

  - <img src="attached_files/V2/project_resources/slam_5_saved_map_check.png" alt="GUI Screenshot" width="300">

  2. Convert `OccupancyGrid` data to a grid map represented by a NumPy array.
  3. Perform obstacle expansion on the map (taking into account the safety distance of the robot).

  - <img src="attached_files/V2/project_resources/route_analysis_1.png" alt="GUI Screenshot" width="300">

  4. **Generate a three-value map**: `-1`, `0`, and `1` are used to represent obstacles, unvisited areas, and visited areas respectively.
  5. Generate path points in the map through a fixed sampling interval. Each path point includes world coordinates and grid coordinates.
  6. Use **greedy algorithm** to find valid connections between path points and check whether there are obstacles between two points.

  - <img src="attached_files/V2/project_resources/route_analysis_2.png" alt="GUI Screenshot" width="300">
- **After the connection is completed:**

  - Use `matplotlib` to draw the path points and connected line segments and save them as an image.
  - **The logic of finding valid connections:**
    - Each path point can only be connected to the path points adjacent to it.
    - **Definition of connection:** up, down, left, and right.
    - Isolated path points are not considered in the connection.
- Use RViz and route_show (button [Show Route]) to see the points and route:

  - <img src="attached_files/V2/project_resources/route_analysis_3.png" alt="GUI Screenshot" width="400">
  - <img src="attached_files/V2/project_resources/route_analysis_4.png" alt="GUI Screenshot" width="400">

#### **Sub Module II: Route Follow**

- **Main Logic**

  - (1) Follow the route based on route_plan analyzed
  - (2) When reach a red point, that point will turn to green
  - (3) If the robot found the red point is not reachable, might be a wall, might be a moving obstacle, then the point will turn to black.
- The full logic of `route_follow.py`:

  - <img src="attached_files/V2/project_resources/route_follow_flow.png" alt="GUI Screenshot" style="max-width: 100%; height: auto;">
- **black point demo:**

  - <img src="attached_files/V2/project_resources/route_follow_black_point.png" alt="GUI Screenshot" width="300">
- **red point turn to green point demo:**

  - <img src="attached_files/V2/project_resources/route_follow_1.png" alt="GUI Screenshot" width="600">

---

## **How to Use**

### **Control Panel Buttons**

- **Start SLAM**: Launches SLAM and RViz.
- **Start/Stop Exploration**: Begins or halts autonomous exploration.
- **Save Map**: Saves the current map to the `/maps` directory.
- **Analyze Route**: Uses `route_plan.py` to plan paths based on the saved map.
- **Show Route**: Visualizes the planned route in RViz.
- **Start Cleaning**: Executes the cleaning routine (based on the selected cleaning module).
- **Robot Control**: Allows manual control of the robot via `/cmd_vel`.
- **Quit Program**: Shuts down the system.

---

## **Key Technologies**

1. **Mapping Module**:
   - **SLAM**: Uses GMapping for real-time map creation and localization.
   - **Explore_Lite**: Implements frontier-based exploration.
2. **Cleaning Module II**:
   - **Route Analysis**:
     - Reads maps as `OccupancyGrid` messages.
     - Processes maps using NumPy for obstacle inflation and path planning.
     - Generates waypoints with greedy algorithms.
   - **Route Follow**:
     - Executes planned routes, dynamically updating cleaned areas in RViz.
3. **Voice Control Module**:
   - Powered by the Vosk speech recognition model for offline voice command processing.

---

## **Future Plans**

- Refine and debug the Route Follow submodule.
- Open-source the project to foster collaboration on smart cleaning robot innovations.
- Create a tutorial for building autonomous cleaning robots step-by-step.
- Expand the frontier exploration module with self-designed algorithms.

---

## **Contributors**

- **Pang Liu**: Panel Module, Voice Control, Cleaning Module II.
- **Zhenxu Chen**: Mapping Module, Cleaning Module I.
