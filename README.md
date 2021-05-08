# ducksim

## How to deploy cloud infrastructure
TODO

## Install ducksim locally
Before you can simulate a local robot or run rviz, you must first install ROS and then
build the ducksim package from source. The package was tested with ROS Melodic but
should also work with ROS Noetic.

### Build ducksim from source
```
# Create a workspace and clone the code
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
git clone https://github.com/jeholliday/ducksim.git src/ducksim
# Build and install the code
catkin_make
catkin_make install
```

### Run a simulated robot
A simulated robot can be run locally and interact with the other applications in
the cloud.
```
# Set ROS_IP to address of tailscale interface
export ROS_IP=`ip a show dev tailscale0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1`
export ROS_MASTER_URI=http://<master_ip>:11311
rosrun ducksim duck_node.py
```

### Run Rviz visuzlization
Rviz is a tool for visualizing all kinds of data. The provided configuration file
can configure Rviz to visualize the current state of the robot simulation.
```
export ROS_IP=`ip a show dev tailscale0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1`
export ROS_MASTER_URI=http://<master_ip>:11311
rviz -d ~/catkin_ws/install/share/ducksim/rviz/turtlesim_demo.rviz
```