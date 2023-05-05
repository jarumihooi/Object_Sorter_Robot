# COSI119-final-objectSorter
objectSorter

By Isaac Goldings, David Pollack, and Jeremy Huey
Contact at isaacgoldings@brandeis.edu, davidpollack@brandeis.edu, jhuey@brandeis.edu

## To run ====
(check env requirements below)
$ roslaunch object_sorter prod_v5.launch

## About ====
Our project is a robot that can sort objects by color. It will demonstrate this by picking the objects up with a claw and moving them into designated 
areas of an arena based on their color.

The project integrates qrcode/fiducial recognition, color recognition, and error for motion control. 

We think this project should be evaluated primarily based on whether or not it can successfully preform the task of sorting object without error, and secondly based on how efficiently and smoothly it preforms this task.

![image](https://github.com/campusrover/labnotebook/blob/master/images/robot_done_sorting.png)

## Installation notes ====
Environment required: linux with ros.
Robot required: A "Platform" robot with a claw, enough motor to move objects. 
Other materials: Printed large fiducials from aruco https://clover.coex.tech/en/aruco.html
(change the fiducial numbers for "red", "green" and "mixed" in "fiducial_recognition.py"). Paste the fiducials on some sort or elevated surface, about 0.5m above the ground.
3 unique fiducials, some number of distinctly colored cans, eg 3 red 3 green. 

Git clone this repository. 
The package name is stored in the package.xml. You can goto it via: $ roscd object_sorter
First time install packages ==== 
  sudo apt-get update
  sudo apt-get install ros-noetic-aruco-detect
  sudo apt-get install ros-noetic-fiducials
  sudo apt-get install ros-noetic-fiducial-msgs
  sudo apt-get install xterm
  check if the image topic is as expected by using $ rostopic list.
    for platform2, expecting: /raspicam_node/image/compressed
