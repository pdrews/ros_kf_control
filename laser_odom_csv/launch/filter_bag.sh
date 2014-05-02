#! /bin/bash

rosbag filter $1 $2 'topic == "/tf" or
                     topic == "cmd_drive" or
                     topic == "/cmd_vel" or
                     topic == "/gps/odom" or
                     topic == "/gps/vel" or
                     topic == "/imu/mag" or
                     topic == "/imu/rpy" or
                     topic == "/imu/data" or
                     topic == "/compass/compass"'
