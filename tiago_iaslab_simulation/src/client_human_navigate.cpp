//Group 24
//Edoardo Bastianello, ID:2053077
//Stefano Binotto, ID: 2052421
//Gionata Grotto, ID: 2052418

//services
#include "tiago_iaslab_simulation/Objs.h"
#include "tiago_iaslab_simulation/srv_detect.h"
#include "tiago_iaslab_simulation/pickObj.h"
#include "tiago_iaslab_simulation/placeObj.h"

//ros header files
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//ros msgs
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

//custom header files
#include <tiago_iaslab_simulation/utils.h>
#include <variables.h>

//main
int main(int argc, char **argv) {
    
    //initialize node A
    ros::init(argc, argv, "client_human");
    
    //get the global poses related to the 3 different IDs
    std::array<tiago_iaslab_simulation::MoveGoal, 6> global_poses = initialize_global_pose();

    //get poses of 2 waypoints for navigation (see report)
    std::array<tiago_iaslab_simulation::MoveGoal, 2> waypoints_poses = initialize_waypoints_pose();

    //user-defined head_controller action client to call head_controller
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_head("/head_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server head_controller to start.");
    ac_head.waitForServer();
    ROS_INFO("Action server head_controller started, sending goal.");

    //declaration of goal for head controller
    control_msgs::FollowJointTrajectoryGoal goal_head_controller;

    //the joint names to control the head
    goal_head_controller.trajectory.joint_names.push_back("head_1_joint");
    goal_head_controller.trajectory.joint_names.push_back("head_2_joint");

    //we use only one waypoint in this head trajectory
    goal_head_controller.trajectory.points.resize(1);

    //the trajectory point
    //positions
    goal_head_controller.trajectory.points[0].positions.resize(2);
    goal_head_controller.trajectory.points[0].positions[0] = 0.00;
    goal_head_controller.trajectory.points[0].positions[1] = CAMERA_ANGLE;

    //velocities
    goal_head_controller.trajectory.points[0].velocities.resize(2);
    goal_head_controller.trajectory.points[0].velocities[0] = 1.0;
    goal_head_controller.trajectory.points[0].velocities[1] = 1.0;

    //to be reached 2 second after starting along the trajectory
    goal_head_controller.trajectory.points[0].time_from_start = ros::Duration(2.0);

    goal_head_controller.trajectory.header.stamp = ros::Time::now();

    //send goal to head controller
    ac_head.sendGoal(goal_head_controller);
    ROS_INFO("Goal has been sent.");

    //wait for trajectory execution
    while(!(ac_head.getState().isDone()) && ros::ok()) {
        //sleep for two seconds until the head movement is done
        ros::Duration(2).sleep(); 
    }

    //link to the Objs.srv to get the random IDs
    ros::NodeHandle human_handle;
    ros::ServiceClient client_human = human_handle.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");

    //declare the custom service for the response with IDs to pick
    tiago_iaslab_simulation::Objs objs;
    objs.request.all_objs = true;
    objs.request.ready = true;

    //call the service for random IDs
    if (client_human.call(objs)) {
        //print the order of the IDs to move
        ROS_INFO("ID 1: [%d], ID2: [%d], ID3[%d]", (int)objs.response.ids[0], (int)objs.response.ids[1], (int)objs.response.ids[2]);
    } else {
        ROS_ERROR("Failed to call service IDs request");
        return 1;
    }
    ros::spinOnce();


    //client for the server implemented in the file detection_server.cpp to detect the objects on the table using AprilTag
    ros::NodeHandle n_detect;
    ros::ServiceClient client_detect = n_detect.serviceClient<tiago_iaslab_simulation::srv_detect>("detect_objects");

    //client for the server implemented in the file pick_server.cpp to pick the right object on the table
    ros::NodeHandle n_pick;
    ros::ServiceClient client_pick = n_pick.serviceClient<tiago_iaslab_simulation::pickObj>("pick_object");

    //client for the server implemented in the file place_server.cpp to place the object on the cylindrical table
    ros::NodeHandle n_place;
    ros::ServiceClient client_place = n_place.serviceClient<tiago_iaslab_simulation::placeObj>("place_object");

    //declare our custom service for the response
    tiago_iaslab_simulation::srv_detect srv_detect;
    tiago_iaslab_simulation::pickObj srv_pick;
    tiago_iaslab_simulation::placeObj srv_place; 


    //main loop for the navigation, detection and pick&place for each object IDs
    for (int i = 0; i < NUMBER_OBJECT_TO_PICK; i++) {
        //object to pick
        int ID_object = objs.response.ids[i];


        /*
            ******* NAVIGATION TO REACH THE TABLE *******
        */

        //firstly always navigate to waypoint #1 (see report)
        int debug = move(waypoints_poses[0]);
        if (debug==1) {
            //if robot can't reach the pose, return error
            return 1;
        }

        //if the robot has to pick the green triangle, also go to waypoint #2 (see report)
        if(ID_object == 2) {
            debug = move(waypoints_poses[1]);
            if (debug == 1) {
                //if robot can't reach the pose, return error
                return 1;
            }
        }

        //move to global pose in front of the table depending on the current object ID
        debug = move(global_poses[ID_object - 1]);
        if (debug==1) {
            //if robot can't reach the pose, return error
            return 1;
        }


        /*
            ******* DETECTION *******
        */

        //store the requested ID
        srv_detect.request.ID = (int)objs.response.ids[i];

        //wait for server for the detection
        ROS_INFO("Waiting for detection server to start.");
        client_detect.waitForExistence();
        ROS_INFO("Detection server started, sending goal.");

        //send a request with current object ID and get a vector of AprilTagDetection as response
        if (client_detect.call(srv_detect)) {
            ROS_INFO("Detection completed: %d objects detected", (int)srv_detect.response.collision_objects.size());
        } else {
            //detection failed
            ROS_ERROR("Detection error");
            return 1;
        }

        //create collision objects from detections
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        addCollisionObjects(planning_scene_interface, srv_detect.response.collision_objects);


        /*
            ******* PICK *******
        */

        //find the detection object corresponding to the current object ID
        int detections_index_current_object = -1;
        for (unsigned int detections_i = 0; detections_i < srv_detect.response.collision_objects.size(); detections_i++) {
            if (srv_detect.response.collision_objects[detections_i].id[0] == ID_object) {
                detections_index_current_object = detections_i;
            }
        }

        //stop the execution if the requested object ID hasn't been detected
        if (detections_index_current_object == -1) {
            ROS_ERROR("No object detected.");
            return 1;
        }

        //get a PoseStamped above the tag to simplify the pick
        geometry_msgs::PoseStamped goal_pose = computeFinalPickPose(srv_detect.response.collision_objects[detections_index_current_object].pose.pose.pose, ID_object);

        //goal pose for pick_server
        srv_pick.request.object_pose = goal_pose;

        //send the object ID of the object to pick
        std::string current_id = "cube";
        if (ID_object == 1) {
            current_id = "Hexagon";
        } else if (ID_object == 2) {
            current_id = "Triangle";
        }
        srv_pick.request.object_ID = current_id;

        //wait for server for the pick
        ROS_INFO("Waiting for pick server to start.");
        client_pick.waitForExistence();
        ROS_INFO("Pick server started, sending pose and object ID.");

        //call the pick server
        if (client_pick.call(srv_pick)) {
            if (srv_pick.response.finishPick) {
                ROS_INFO("PICK RESULT: SUCCESS");
            } else {
                ROS_ERROR("PICK RESULT: FAILED");
                return 1;
            }
        } else {
            //pick failed
            ROS_ERROR("Pick error");
            return 1;
        }


        /*
            ******* NAVIGATION TO REACH THE CORRESPONDING CYLINDRICAL TABLE *******
        */

        //if the robot has to place the green triangle, also go to waypoint #2 (see report)
        if(ID_object == 2) {
            debug = move(waypoints_poses[1]);
            if (debug == 1) {
                //if robot can't reach the pose, return error
                return 1;
            }
        }

        //if the robot has to place the green triangle, then go to waypoint #1 (see report)
        if(ID_object == 2) {
            debug = move(waypoints_poses[0]);
            if (debug == 1) {
                //if robot can't reach the pose, return error
                return 1;
            }
        }

        //move to corresponding cylinder
        debug = move(global_poses[ID_object + 3 - 1]);
        if (debug == 1) {
            //if robot can't reach the pose, return error
            return 1;
        }


        /*
            ******* PLACE *******
        */

        //add the cylindrical tables to the interface as collision objects
        addCylinderTablesCollisionObjects(planning_scene_interface);

        //wait for server for the place
        ROS_INFO("Waiting for place server to start.");
        client_place.waitForExistence();
        ROS_INFO("Place server started, sending object ID.");

        //object to place
        srv_place.request.object_ID = current_id;

        //call the place_server
        if (client_place.call(srv_place)) {
            if (srv_place.response.finishPlace) {
                ROS_INFO("PLACE RESULT: SUCCESS");
            } else {
                ROS_ERROR("PLACE RESULT: FAILED");
                return 1;
            }
        } else {
            //place failed
            ROS_ERROR("Place error");
            return 1;
        }
    }
    
    return 0;
}