
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
- Terminal 1:
```bash
  roslaunch carmodel_description planner.launch
```
*This cmd spawns the car, T265 camera for local position navigation.*
- Terminal 2:
```bash
  rosrun offboard pc1gen_and_visualize.py
```
*This cmd launches the d455 depth camera.*
- Terminal 3:
```bash
  roslaunch offboard offboard.launch #offboard Mode
  3 #choose this mode for planning
```
*This cmd sets the car in offboard mode, ready for doing mission. Mode 3 is planning.*
- Terminal 4:
```bash
  roslaunch ewok_optimization optimization_point.launch
```
*This cmd runs ewok planning.*

## Images

![AGV Body](https://github.com/giangthewalkingman/AGV_IVSR/tree/software_rpi/docs/images/body-agv.png)

