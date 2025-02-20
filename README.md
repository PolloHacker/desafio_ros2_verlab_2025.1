# desafio_ros2_verlab_2025.1
## How to Run the Project

### Prerequisites
Ensure you have the following installed:
- ROS 2 (Jazzy)
- Gazebo
- colcon

### Setup Instructions

1. **Clone the repository:**
    ```bash
    git clone https://github.com/PolloHacker/desafio_ros2_verlab_2025.1.git
    cd desafio_ros2_verlab_2025.1
    ```

3. **Install dependencies:**
    ```bash
    rosdep install --from-paths src --rosdistro jazzy -y
    ```

4. **Build the workspace:**
    ```bash
    colcon build
    ```

5. **Source the workspace:**
    ```bash
    source install/setup.bash
    ```

### Running the Project

1. **Launch the simulation:**
    ```bash
    ros2 launch scout_gazebo_sim scout_mini_test_world.launch.py
    ```

2. **Control the robot:**  
    - To control the scout mini in the simulation, you'll need to start the `teleop_twist_keyboard` package from your terminal:

        ```bash
        ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=scout_mini/cmd_vel
        ```
    - After that, start the wall avoidance system:

        ```bash
        ros2 run scout_avoid_walls avoid_walls
        ```

        It will make sure the robot doesn't crash into an obstacle by reading key commands from keyboard and 