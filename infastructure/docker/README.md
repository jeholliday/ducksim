sudo docker build -f dockerfile -t ros .
sudo docker tag ros:latest 129.114.26.80:5000/ros
sudo docker push 129.114.26.80:5000/ros