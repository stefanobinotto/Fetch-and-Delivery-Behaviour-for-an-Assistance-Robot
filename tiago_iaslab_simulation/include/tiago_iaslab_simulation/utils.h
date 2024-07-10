//Group 24
//Edoardo Bastianello, ID:2053077
//Stefano Binotto, ID: 2052421
//Gionata Grotto, ID: 2052418

#ifndef UTILSROS_H
#define UTILSROS_H

//action msg
#include "tiago_iaslab_simulation/MoveAction.h"

//services
#include "tiago_iaslab_simulation/pickObj.h"

//ros header files
#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//ros msgs
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/CollisionObject.h"

//standard library
#include <array>
#include <vector>
#include <string>

//******* NAVIGATION *******
//function that initialize the global poses that we assumed
std::array<tiago_iaslab_simulation::MoveGoal,6> initialize_global_pose();
//initialize poses of the 2 waypoints for navigation (see the report)
std::array<tiago_iaslab_simulation::MoveGoal,2> initialize_waypoints_pose();
//function to move the robot to the target pose using movebase
int move(tiago_iaslab_simulation::MoveGoal target_pose);
//callback that is called after a feedback is published and prints the feedback message
void feedbackCallback(const tiago_iaslab_simulation::MoveActionFeedback::ConstPtr &msg);
//callback that prints the final result (the centers of each obstacle)
void resultCallback(const tiago_iaslab_simulation::MoveActionResult::ConstPtr &msg);


//******* DETECTION AND COLLISION OBJECTS *******
//function that create the collision objects from the detections
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, const std::vector<apriltag_ros::AprilTagDetection>& detections);
//add cylindrical tables to the interface as collision objects
void addCylinderTablesCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
//function that convert from gazebo origin to map frame
geometry_msgs::Pose convertOriginToMap(const geometry_msgs::Pose& input_pose);


//******* PICK AND PLACE *******
//move to safe or intermediate configuration (see the report)
bool move_safe(std::string position);
//function to move the arm above the tag of the target object to pick
bool move_target(tiago_iaslab_simulation::pickObj::Request &req);
//compute cartesian path that moves the gripper of a quantity specified by the input parameters
void cartesianPath(double x_movement, double y_movement, double z_movement);
//function to open/close the gripper
void gripperControl(float position);
//function to move the arm above the cylindrical table
bool move_to_place(const geometry_msgs::PoseStamped& target_pose);
//function to compute the final pick pose to pick the object specified in the pose parameters
geometry_msgs::PoseStamped computeFinalPickPose(const geometry_msgs::Pose pose_target, const int id_obj);
//function to attach or detach an object to the arm (arm_7_link)
void AttachDetach(std::string service_name, std::string model2, std::string link2, ros::NodeHandle nh);

#endif
