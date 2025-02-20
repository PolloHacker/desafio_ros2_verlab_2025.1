# desafio_ros2_verlab_2025.1
## How to Run the Project

### Prerequisites
Ensure you have the following installed:
- ROS 2 (Foxy, Galactic, or Rolling)
- colcon
- vcs (for importing repositories)

### Setup Instructions

1. **Clone the repository:**
    ```sh
    git clone https://github.com/yourusername/desafio_ros2_verlab_2025.1.git
    cd desafio_ros2_verlab_2025.1
    ```

2. **Import dependencies:**
    ```sh
    vcs import < src/ros2.repos
    ```

3. **Install dependencies:**
    ```sh
    rosdep install --from-paths src --ignore-src -r -y
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

### Additional Information

- **Uninstall:**
    To uninstall the project, you can run the following command:
    ```sh
    colcon build --cmake-target uninstall
    ```

- **Clean Build:**
    To clean the build, use:
    ```sh
    colcon build --cmake-target clean
    ```

For more detailed information, refer to the documentation within each package.