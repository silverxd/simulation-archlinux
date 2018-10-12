#!/bin/bash
echo $#
count=$#
for ((i=0; i<$count; i++)){
    if [ $1 -eq 1 ]; then
        DIR='robot'
    else
        DIR='pibot'
    fi
    sshpass -p 'password' ssh pi@192.168.0.9$1 "cd $DIR && git pull origin master"
    echo "Updated $1"
    shift
}
echo "Thank you, come again!"
