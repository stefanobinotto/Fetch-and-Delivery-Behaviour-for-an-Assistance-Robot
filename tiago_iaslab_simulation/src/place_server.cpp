//Group 24
//Edoardo Bastianello, ID:2053077
//Stefano Binotto, ID: 2052421
//Gionata Grotto, ID: 2052418

//services
#include "tiago_iaslab_simulation/placeObj.h"

//ros header files
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//ros msgs
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/CollisionObject.h"

//custom header files
#include <tiago_iaslab_simulation/utils.h>
#include <variables.h>


//callback function to place the object on the relative cylindrical table
bool placeCallback(tiago_iaslab_simulation::placeObj::Request &req, tiago_iaslab_simulation::placeObj::Response &res) {
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //initialize response to false
    res.finishPlace = false;
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //get collision objects from PlanningSceneInterface
    std::map<std::string, moveit_msgs::CollisionObject> map_objs = planning_scene_interface.getObjects();
    //vector of the collision object of the cylindrical tables
    std::vector<std::string> coll_objects(map_objs.size());
    //collision object of current cylindrical table
    moveit_msgs::CollisionObject current_cylinder;

    //check what cylindrical table you want to consider
    std::string cylinder_to_consider;
    if (req.object_ID == "cube") {
        cylinder_to_consider = "place_table_r";
    } else if (req.object_ID == "Triangle") {
        cylinder_to_consider = "place_table_g";
    } else {
        cylinder_to_consider = "place_table_b";
    }

    //iterate for all the collision objects and save the current cylinder collision object
    for(std::map<std::string, moveit_msgs::CollisionObject>::iterator it = map_objs.begin(); it != map_objs.end(); ++it) {
        if (cylinder_to_consider == it->first) {
            current_cylinder = it->second;
        }
        coll_objects.push_back(it->first);
    }

    //create pose above the cylindrical table
    geometry_msgs::PoseStamped target_pose_stamped;
    target_pose_stamped.header.frame_id = current_cylinder.header.frame_id;
    target_pose_stamped.pose = current_cylinder.primitive_poses[0];
    target_pose_stamped.pose.position.z += (current_cylinder.primitives[0].dimensions[0]/2 + PLACE_INCREMENT);

    //release the hexagon slightly higher to ensure safety
    if (req.object_ID == "Hexagon") {
        target_pose_stamped.pose.position.z += 0.1;
    }

    /*
        For the following lines see the report for more details about each individual pose 
    */

    //move to the intermediate pose
    move_safe("intermediate");

    //move to the target pose
    move_to_place(target_pose_stamped);

    //open the gripper
    gripperControl(OPEN_GRIPPER_SIZE);
    
    //detach the object from the gripper
    std::string link = req.object_ID +"_link";
    AttachDetach("/link_attacher_node/detach", req.object_ID, link, nh);

    //move to the intermediate pose
    move_safe("intermediate");

    //move to the safe pose
    move_safe("safe");

    //remove all the cylinder tables collision objects from the scene
    planning_scene_interface.removeCollisionObjects(coll_objects);
    
    //set success to true into the response
    res.finishPlace = true;

    spinner.stop();

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "place_server");
    
    ros::CallbackQueue callback_n1;
    ros::NodeHandle n1;
    n1.setCallbackQueue(&callback_n1);
    
    ros::ServiceServer service = n1.advertiseService("place_object", placeCallback);
    ROS_INFO("Ready to place objects");

    while(ros::ok()) {
        callback_n1.callOne(ros::WallDuration(1));
    }
    return 0;
}