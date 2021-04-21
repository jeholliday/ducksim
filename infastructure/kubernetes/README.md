# Create cluster
sudo  kubeadm init --pod-network-cidr=10.244.0.0/16 --apiserver-cert-extra-sans=129.114.26.80

# Copy /etc/kubernetes/admin.conf to $HOME/.kube

# Untaint master
kubectl taint nodes team8-vm2 node-role.kubernetes.io/master:NoSchedule-


kubectl apply -f https://raw.githubusercontent.com/coreos/flannel/master/Documentation/kube-flannel.yml
