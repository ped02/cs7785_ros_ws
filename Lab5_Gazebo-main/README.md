# Files for 7785 Lab 5

***For Lab 5, you are required to use the Navigation Stack in the Gazebo environment. This should help you with your code development for this lab as well as the final lab. The files in this repository are used to create the simulated environment you will map and navigate through. If you wreck your Turtlebot3 files or the Gazebo files associated with them, repull the `turtlebot3` directory if you pulled the github repository directly or reinstall the debian package to fix the error.***

*** :warning: This should all be done on your computer as it is only meant for the simulator! :warning: ***

While you do not need to have a simulated camera for this lab, if you wish to modify the Turtlebot3 simulation files to include a camera (which can be useful for the final lab) please follow the direction in this Github repository: [https://github.gatech.edu/swilson64/turtlebot3_sim_update](https://github.gatech.edu/swilson64/turtlebot3_sim_update)


## Adding the necessary files to the Turtlebot3 Gazebo Simulation Repository

1. Add the maze files to the Turtlebot3 Gazebo files. Copy the `7785_maze` directory in this git repository into the directory,
        <p style="text-align: center;"> `<YOUR TURTLEBOT3 ROS WS>/src/turtlebot3_simulations/turtlebot3_gazebo/models/` </p>

> :warning: Note that this directory path may change depending on which ROS2 workspace you used to clone the the Turtlebot packages. :warning:

2. Move or copy the `7785_maze.launch.py` file in this git repository into the directory,
        <p style="text-align: center;"> `<YOUR TURTLEBOT3 ROS WS>/src/turtlebot3_simulations/turtlebot3_gazebo/launch/` </p>

> :warning: Note that this directory path may change depending on which ROS2 workspace you used to clone the the Turtlebot packages. :warning:

3. Move or copy the `7785_maze.world` file in this git repository into the directory,
        <p style="text-align: center;"> `<YOUR TURTLEBOT3 ROS WS>/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/` </p>

> :warning: Note that this directory path may change depending on which ROS2 workspace you used to clone the the Turtlebot packages. :warning:

4. Return back to the `turtlebot3_ws` directory and run `colcon build`.
5. Make sure to run `source install/setup.bash` for changes to reflect.
6. To use the Gazebo environment, run:
`ros2 launch turtlebot3_gazebo 7785_maze.launch.py `
