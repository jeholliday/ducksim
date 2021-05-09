# ducksim

## How to deploy cloud and local infrastructure
1. See [infastructure/docker/README.md](infastructure/docker/README.md) for how to
   install Tailscale and build the docker images.
2. Start Kubernetes Pods
```
kubectl apply -f infastructure/kubernetes/couchdb-job.yaml
kubectl apply -f infastructure/kubernetes/roscore-job.yaml
kubectl apply -f infastructure/kubernetes/simulation-job.yaml
kubectl apply -f infastructure/kubernetes/static-transform-job.yaml
kubectl apply -f infastructure/kubernetes/herder-job.yaml
kubectl apply -f infastructure/kubernetes/couchdb-logger-job.yaml
```
3. Start Local robots following directions below
4. Access CouchDB at http://<master_ip>:30006/_utils

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
source ~/catkin_ws/install/setup.bash
# Set ROS_IP to address of tailscale interface
export ROS_IP=`ip a show dev tailscale0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1`
export ROS_MASTER_URI=http://<master_ip>:11311
rosrun ducksim duck_node.py
```

### Run Rviz visuzlization
Rviz is a tool for visualizing all kinds of data. The provided configuration file
can configure Rviz to visualize the current state of the robot simulation.
```
source ~/catkin_ws/install/setup.bash
export ROS_IP=`ip a show dev tailscale0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1`
export ROS_MASTER_URI=http://<master_ip>:11311
rviz -d ~/catkin_ws/install/share/ducksim/rviz/turtlesim_demo.rviz
```