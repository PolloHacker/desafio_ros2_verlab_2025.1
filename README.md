# desafio_ros2_verlab_2025.1
## How to Run the Project

### Prerequisites
Ensure you have the following installed:
- ROS 2 (Jazzy)
- Gazebo
- colcon

### Setup Instructions

1. **Clone the repository:**
    ```sh
    git clone https://github.com/PolloHacker/desafio_ros2_verlab_2025.1.git
    cd desafio_ros2_verlab_2025.1
    ```

3. **Install dependencies:**
    ```sh
    rosdep install --from-paths src --rosdistro jazzy -y
    ```

4. **Build the workspace:**
    ```sh
    colcon build
    ```

5. **Source the workspace:**
    ```sh
    source install/setup.bash
    ```

### Running the Project

1. **Launch the simulation:**
    ```sh
    ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py
    ```

2. **Control the robot:**
    ```sh
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=scout_mini/cmd_vel
    ```
