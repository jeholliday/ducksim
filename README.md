# ducksim

# Step 1: Install Ducksim locally

Before you can simulate a local robot or run Rviz, you must first install ROS and then
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

# Step 2: Deploy and Configure Cloud Infrastructure

## Install Tailscale on Cloud VMs
You will need to change the advertised routes to match the pod CIDRs of your cluster.
```
# Install Tailscale
# https://tailscale.com/download/linux

# Start Tailscale
echo 'net.ipv4.ip_forward = 1' | sudo tee -a /etc/sysctl.conf
echo 'net.ipv6.conf.all.forwarding = 1' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p /etc/sysctl.conf
sudo tailscale up --advertise-routes=10.2.0.0/16

# Allow the route in the Tailscale admin dashboard
```

## Start a Docker registry on one cloud VM
```
# Start a insecure docker registry
docker run -d -p 5000:5000 --restart=always --name registry registry:2
```

## Build Docker Images
Change the IP address to that of your docker registry
```
sudo docker build --build-arg DUCKSIM_VER=$(date +%Y%m%d-%H%M%S) -t ros .
sudo docker tag ros:latest <docker_registry>:5000/ros
sudo docker push <docker_registry>:5000/ros
```

## Start Kubernetes Pods
In each of the provided jobs, you will need to replace the IP address with where ROS master can be found.
```
kubectl apply -f infastructure/kubernetes/couchdb-job.yaml
kubectl apply -f infastructure/kubernetes/roscore-job.yaml
kubectl apply -f infastructure/kubernetes/simulation-job.yaml
kubectl apply -f infastructure/kubernetes/static-transform-job.yaml
kubectl apply -f infastructure/kubernetes/herder-job.yaml
kubectl apply -f infastructure/kubernetes/couchdb-logger-job.yaml
```

# Step 3: Start a Local Robot

## Configure Local Robot
Notice the `--accept-routes` flag, which is necessary on Linux to accept the routes
to the pod network published by the Cloud VMs.
```
# Install Tailscale
# https://tailscale.com/download/linux

# Start Tailscale
sudo tailscale up --accept-routes
```

## Run a simulated robot
A simulated robot can be run locally and interact with the other applications in
the cloud. 
```
source ~/catkin_ws/install/setup.bash
# Set ROS_IP to address of tailscale interface
export ROS_IP=`ip a show dev tailscale0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1`
export ROS_MASTER_URI=http://<master_ip>:11311
rosrun ducksim duck_node.py
```

## Run Rviz visuzlization
Rviz is a tool for visualizing all kinds of data. The provided configuration file
can configure Rviz to visualize the current state of the robot simulation.
```
source ~/catkin_ws/install/setup.bash
export ROS_IP=`ip a show dev tailscale0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1`
export ROS_MASTER_URI=http://<master_ip>:11311
rviz -d ~/catkin_ws/install/share/ducksim/rviz/turtlesim_demo.rviz
```

## View Logs
The logs from each node can be found by looking in the CouchDB admin panel at:
`http://<master_ip>:30006/_utils`

# Step 4: Teardown Cloud Infrastructure

## Stop Kubernetes Pods
```
kubectl delete -f infastructure/kubernetes/couchdb-logger-job.yaml
kubectl delete -f infastructure/kubernetes/herder-job.yaml
kubectl delete -f infastructure/kubernetes/static-transform-job.yaml
kubectl delete -f infastructure/kubernetes/simulation-job.yaml
kubectl delete -f infastructure/kubernetes/roscore-job.yaml
kubectl delete -f infastructure/kubernetes/couchdb-job.yaml
```