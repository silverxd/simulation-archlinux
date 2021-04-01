#!/bin/bash

# just print this out
pkill -9 gazebo & pkill -9 gzserver & pkill -9 gzclient
# exit gracefully by returning a status 
exit 0