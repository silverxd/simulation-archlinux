#!/bin/bash
ssh-keygen -f $HOME/iti0201_key -t rsa -b 4096
echo "Copy the following to TalTech GitLab..."
cat $HOME/iti0201_key.pub
