## Configure Every Cloud VM
Change the advertised routes to match the pod CIDRs of your cluster.
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

## Configure Local Robot
```
# Install Tailscale
# https://tailscale.com/download/linux

# Start Tailscale
sudo tailscale up --accept-routes
```

## Start a Docker registry on one cloud VM
```
# Start a insecure docker registry
docker run -d -p 5000:5000 --restart=always --name registry registry:2
```

## Build Docker Images
Change the IP address to that of your main Cloud VM
```
sudo docker build --build-arg DUCKSIM_VER=$(date +%Y%m%d-%H%M%S) -t ros .
sudo docker tag ros:latest 45.33.119.237:5000/ros
sudo docker push 45.33.119.237:5000/ros
```