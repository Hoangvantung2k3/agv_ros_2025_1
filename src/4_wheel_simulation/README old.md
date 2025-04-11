
# AGV_IVSR: RPI Software

### Run:
1. roslaunch carmodel_description spawn.launch
spawn the car in gazebo (and rviz)

2. roslaunch offboard offboard.launch
running offboard, the car will move to the base defined in launch file

### Note:
1. if you are running in a simualation, let the sim_mode and arm_mode be true.
2. before running the 2nd step in Run, you can run:
- rosrun offboard odom_to_csv.py 
That command is to record the data of odometry to a csv file, remember to modify the path in the odom_to_csv.py file to be suitable for your environment.
Then you can use "rosrun plot_juggler plot_juggler" to observe the data
- rosrun offboard odom_to_path.py to publish the odom path in real time while the car is moving.

### Note:
1. Edit configuration in gazebo.xacro to control 4 wheels instead of 2 wheels like before.
2. Edit configuration in rollers.xacro, wheel.xacro, bot_description.xacro to modify the simulation car similar to the real car.


# IVSR AGV

A list of necessary packages for building an AGV

## Setup

In your current working directory

```bash
  mkdir -p ~/agv_ws/src
  cd agv_ws/src
  git clone https://github.com/giangthewalkingman/AGV_IVSR.git
  cd AGV_IVSR
  git checkout -b software_rpi
  cd ../..
  catkin build
```
    
## Running Ewok Planning in simulation

You should have at least 4 terminals.
- All of terminals:

```bash
  cd ~/agv_ws
  source ./devel/setup.bash
```
- Terminal 1: chạy file spawn launch
```bash
  roslaunch carmodel_description planner.launch
```
*This cmd spawns the car, T265 camera for local position navigation.*
- Terminal 2: chuyển d455 sang point cloud 1 
```bash
  rosrun offboard pc1gen_and_visualize.py
```
*This cmd launches the d455 depth camera.*
- Terminal 3: chọn chế độ, có PID và planning
```bash
  roslaunch offboard offboard.launch #offboard Mode 
  3 #choose this mode for planning
```

*This cmd sets the car in offboard mode, ready for doing mission. Mode 3 is planning.*
- Terminal 4: chạy planning
```bash
  roslaunch ewok_optimization optimization_point.launch
```
*This cmd runs ewok planning.*

## Images

![AGV Body](https://github.com/giangthewalkingman/AGV_IVSR/tree/software_rpi/docs/images/body-agv.png)

