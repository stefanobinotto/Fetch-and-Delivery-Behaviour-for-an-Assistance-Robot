# Fetch-and-Delivery-Behaviour-for-an-Assistance-Robot

This repository contains the delivery of the final group project for the Intelligent Robotics course at UniPD - DEI 2022/23.

**Group 24**:
- Edoardo Bastianello, ID:2053077
- Stefano Binotto, ID: 2052421
- Gionata Grotto, ID: 2052418

## First assignment of the Project
Implement a routine that let Tiago navigate inside the environment, composed by two rooms with obstacles and a narrow space. 
Tiago has to navigate from a point A to a point B.
Once reached point B, it has to detect the obstacles, and print on the screen the position of the cylindric tables visible from that pose.
The static obstacles that are part of the map, e.g. the walls or the shelves, must not be detected as obstacles.

The assignment is described [here](https://github.com/stefanobinotto/Fetch-and-Delivery-Behaviour-for-an-Assistance-Robot/blob/main/Assignment_1.pdf).

Our implementation is described [here](https://github.com/stefanobinotto/Fetch-and-Delivery-Behaviour-for-an-Assistance-Robot/tree/main/Report%20Group%2024%20-%20Assignment_1).

Example of detection output:
<img src="https://github.com/stefanobinotto/Fetch-and-Delivery-Behaviour-for-an-Assistance-Robot/blob/main/Report%20Group%2024%20-%20Assignment_1/successful_partial_occlusion_example.PNG">

## Second assignment of the Project
Implement, in an assistance robot (namely a Tiago by PAL Robotics), a fetch and delivery behaviour for everyday objects.
In the simulated environment there are 7 objects on a pick-up table: a red cube, a green triangle and a blue hexagon, plus 4 gold hexagons.
The gold hexagons are obstacles, and Tiago must not collide with them (or with the table) while grasping the required objects.
Once an object is grasped it must be fetched and placed on the cylindrical table of the same colour.

To navigate in the environment and to move from the pick-up table to the delivery tables, you must use the navigation module you developed in the previous assignment.

The assignment is fulfilled if, in the end, the three coloured objects are delivered on top of the corresponding coloured table in the assigned sequence.

************************************************************************************************

### COMMANDS TO RUN THE PROGRAM
1. roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables
2. roslaunch tiago_iaslab_simulation apriltag.launch
3. roslaunch tiago_iaslab_simulation navigation.launch
4. rosrun tiago_iaslab_simulation human_node
5. rosrun tiago_iaslab_simulation robot_srv
6. rosrun tiago_iaslab_simulation detection_server
7. rosrun tiago_iaslab_simulation pick_server
8. rosrun tiago_iaslab_simulation place_server
9. rosrun tiago_iaslab_simulation client_human_navigate

### FILES IN THIS PROJECTS
**CPP FILES**:
- client_human_navigate.cpp 
- detection_server.cpp (used to detect the objects on the table)
- Human_node.cpp 
- Human.cpp
- pick_server.cpp (used to for the pick routine in front of the table)
- place_server.cpp (used for the place routine in front of the cylindrical tables)
- robot_srv.cpp (used for the navigation)
- utils.cpp (contains all the external functions used in other files)

**HEADER FILES**:
- Human.h
- utils.h (header file for utils.cpp file)
- variables.h (this file is inside the include directory and contains the values we set for the parameters/dimensions of objects/coordinates we used in the code)

**ACTION MESSAGE**: 
- Move.action (used for communication between client_human_navigate.cpp and robot_srv.cpp)

**SERVICE FILES**:
- srv_detect.srv (used for communication between client_human_navigate.cpp and detection_server.cpp)
- pickObj.srv (used for communication between client_human_navigate.cpp and pick_server.cpp)
- placeObj.srv (used for communication between client_human_navigate.cpp and place_server.cpp)
- Objs.srv (used for communication between client_human_navigate.cpp and Human_node.cpp)
