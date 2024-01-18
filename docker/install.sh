#!/bin/bash

if command -v docker >/dev/null 2>&1; then
    echo "Skipping Docker install..."
else
    echo "Installing Docker..."
    sudo apt-get update
    sudo apt-get install -qq ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    # Add the repository to Apt sources:
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update


    sudo apt-get install -qq docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
 fi

 sudo docker build --build-arg CACHE_BUST=$(date +%s) -t iti0201 .

sudo rm -rf /usr/bin/robot_test /usr/bin/stop_robot
sudo ln -s $PWD/robot_test /usr/bin
sudo ln -s $PWD/stop_robot /usr/bin
