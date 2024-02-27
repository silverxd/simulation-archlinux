#!/bin/bash

# TODO: Replace apt-get with pacman, install yay if needed, update dockerfile to use yay or other AUR helpers, test, write better documentation
# Install yay if no AUR helpers are present:
#
# sudo pacman -S --needed git base-devel
# git clone https://aur.archlinux.org/yay.git
# cd yay
# makepkg -si
# cd ..
# rm -rf yay
#
# Great example of how the script should be reformatted:
# https://koodivaramu.eesti.ee/alvatal/id-kaart/-/blob/main/manjaro-id-kaart.sh
# (also includes example usage of yay and pacman commands)


if [[ -f "/usr/bin/docker" ]]
then
    echo "Skipping Docker install..."
else
    echo "Installing Docker..."
    sudo apt-get update
    sudo apt-get install -qq ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo rm -rf /etc/apt/keyrings/docker.gpg
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

SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
  SOURCE=$(readlink "$SOURCE")
  [[ $SOURCE != /* ]] && SOURCE=$DIR/$SOURCE # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
cd $DIR
sudo docker build --build-arg CACHE_BUST=$(date +%s) -t iti0201 .
sudo docker images -q --filter=dangling=true | xargs -I {} sudo docker rmi {}

sudo rm -rf /usr/bin/robot_test /usr/bin/stop_robot /usr/bin/key_install
sudo ln -s $PWD/robot_test /usr/bin
sudo ln -s $PWD/stop_robot /usr/bin
sudo ln -s $PWD/key_install /usr/bin
