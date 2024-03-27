#!/bin/bash
set -e

# Install deps
sudo apt-get update -y -qq
sudo apt-get install -y -qq \
	ca-certificates \
  curl \
  gnupg \
  lsb-release

# Install GPG key and docker sources
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
sudo apt-get update -y -qq
sudo apt-get install -q -yy \
  docker-ce \
  docker-ce-cli \
  containerd.io \
  docker-compose-plugin

# Create and add user to docker group
groupadd -f docker
sudo usermod -aG docker ${USER}
echo "You now need to log out and log back in so that your group membership is re-evaluated"
