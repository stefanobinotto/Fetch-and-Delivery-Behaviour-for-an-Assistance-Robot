#ifndef VARIAB_ROS_PROJ_H
#define VARIABLE_ROS_PROJ_H

/*
    ******* VARIABLES USED IN MULTIPLE FILES *******
*/
//planning time for movit
const double PLANNING_TIME = 5.0;
//PI
const double PI = 3.14159;
//joint movement velocity
const double JOINT_MOVEMENT_VELOCITY = 1.0;
//gazebo origin coordinates
const double GAZEBO_COORD_X = -6.580434;
const double GAZEBO_COORD_Y = 1.369963;
//number of object that must be picked in this project
const int NUMBER_OBJECT_TO_PICK = 3;


/*
    ******* CLIENT_HUMAN_NAVIGATE *******
*/
//angle of the head to move the camera
const double CAMERA_ANGLE = -0.50;


/*
    ******* PICK_SERVER *******
*/
//offstet for cartesian path with respect to the z axis
const double CARTESIAN_HEXAGON_Z = -0.02;
const double CARTESIAN_TRIANGLE_Z = -0.085;
const double CARTESIAN_CUBE_Z = -0.085;
//size of closed gripper
const double CLOSED_GRIPPER_DIMENSION = 0.01;


/*
    ******* PLACE_SERVER *******
*/
//increment size on the z axis above the cylindrical table
const double PLACE_INCREMENT = 0.3;
//size of open gripper
const double OPEN_GRIPPER_SIZE = 0.4;


/*
    ******* UTILS *******
*/
//gripperControl function
//gripper velocity
const double GRIPPER_VELOCITY = 1.0;
//time gripper movement
const double GRIPPER_MOVEMENT_DURATION = 2.0;


//cartesianPath function
//velocity of robotic arm
const double ARM_VELOCITY = 0.1;
const double END_EFFECTOR_INTERP_RESOLUTION = 0.01;


//addCollisionObjects function (a lot of these variables are values taken from gazebo)
//increment variable
const double COLLISION_INCREMENT = 0.01;
//degree angle to rotate the triangle collision object
const int ANGLE_DEGREE_TRIANGLE = 35;
//height cone triangle
const double CONE_HEIGHT = 0.1;
//radius cone triangle
const double CONE_RADIUS = 0.04;
//height pickable hexagon
const double HEIGHT_HEXAGON_PICK = 0.15;
//radius pickable hexagon
const double RADIUS_HEXAGON_PICK = 0.04;
//height obstacle hexagon
const double HEIGHT_HEXAGON_OBSTACLE = 0.31;
//radius obstacle hexagon
const double RADIUS_HEXAGON_OBSTACLE = 0.05;
//height column table
const double HEIGHT_TABLE_COLUMN = 0.74;
//height base table
const double HEIGHT_TABLE_BASE = 0.04;
//height plane table
const double HEIGHT_TABLE_PLANE = 0.04;
//table x dimension
const double TABLE_X = 1.05;
//table y dimension
const double TABLE_Y = 1.05;
//table z dimension
const double TABLE_Z = 0.805;
//table position x map
const double TABLE_XMAP_POSITION = 1.2451;
//table position y map
const double TABLE_YMAP_POSITION = -1.613;


//addCylinderTablesCollisionObjects function
//cylindrical tables coordinates
const double BLUE_TABLE_X = 6.007146;
const double BLUE_TABLE_Y = 1.015966;
const double GREEN_TABLE_X = 5.007404;
const double GREEN_TABLE_Y = 1.015966;
const double RED_TABLE_X = 4.007396;
const double RED_TABLE_Y = 1.015966;


//computeFinalPickPose function
//offset of z pick pose of the objects
const double BLUE_Z_OFFSET = 0.18;
const double GREEN_Z_OFFSET = 0.3;
const double RED_Z_OFFSET = 0.3;

#endif