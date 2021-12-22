# Home_service_robot
Udacity Robotics nano-degree final project

The files for this project are included above but not in the tree order demanded in the project's criteria. This is because the program is too large to upload, and so I have included a link (https://drive.google.com/drive/u/0/folders/1AxiUyirk8hWNnP6BEErCewhkR9TaqoKU) to view the correctly ordered files in a Google Drive.


![](https://github.com/stickmonster/Home_service_robot/blob/main/dropped%20off.JPG)



In this project, the aim was to build through a series of steps in which  a robot SLAM mapped an environment designed and realised in Gazebo, Ignition, then used Adaptive Monte-Carlo Localisation to follow manually placed 2d goals, before finally operating independently by following C++ programs. These programs would stipulate how the turtlebot will pick objects from the SLAM mapped Gazebo environment, and then add and delete markers to represent the process of the aforementioned automation.
All of these programs were to be tied together by a series of Shell Scripts, which would initiate the automation from a Linux shell.

![](https://github.com/stickmonster/Home_service_robot/blob/main/picked%20up%20home%20service.JPG)


Packages included in project:

Gazebo

Ubuntu Linux Shell

ROS kinetic Github sourced packages:

gmapping

turtlebot_teleop

turtlebot_rviz_launchers

turtlebot_gazebo

Directory tree

.
├── README.md

├── images

│   ├── ... ...

├── CMakeLists.txt

├── add_markers

│   ├── launch

│   │   └── home_service_rviz_config.launch

│   └── src

│       ├── add_markers.cpp

│       └── add_markers_test.cpp

│   ├──  ... ...
├── config

│   └── marker_config.yaml

├── map

│   ├── building

│   │   ├── ... ...

│   ├── home_service.world

│   ├── home_service_map.pgm

│   ├── home_service_map.yaml

├── pick_objects

│   └── src

│       ├── pick_objects.cpp

│       └── pick_objects_test.cpp

│   ├──  ... ...

├── rvizConfig

│   └── home_service.rviz

├── scripts

│   ├── add_marker.sh

│   ├── home_service.sh

│   ├── pick_objects.sh

│   ├── test_navigation.sh

│   └── test_slam.sh

├── slam_gmapping

│   ├── gmapping

│   |── ... ...

├── turtlebot

│   |── turtlebot_teleop

│   |── ... ...

├── turtlebot_interactions

│   |── turtlebot_rviz_launchers

│   |── ... ...

|── turtlebot_simulator

│   ├── turtlebot_gazebo

│   |── ... ...


Following a mkdir at the catkin level, the project was cloned from the following Github:

cd ~/catkin_ws/src/
git clone https://github.com/viks8dm/home-service-robot.git

and the dependency packages were installed:

git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git

The Gazebo environment was uploaded and SLAM mapped, after which the resulting map was tested using AMCL to check that the turtlebot robot could navigate it.

The C++ files were added (pick_objects.cpp and add_markers.cpp) and these all were tied together with  an installation initiated through shell scripts.

![](https://github.com/stickmonster/Home_service_robot/blob/main/success%20home%20service.JPG)

Success!

Thanks for reviewing this project,

Toby
