//Group 24
//Edoardo Bastianello, ID:2053077
//Stefano Binotto, ID: 2052421
//Gionata Grotto, ID: 2052418

//services
#include "tiago_iaslab_simulation/pickObj.h"

//ros header files
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//ros msgs
#include "moveit_msgs/CollisionObject.h"

//custom header files
#include <tiago_iaslab_simulation/utils.h>
#include <variables.h>

//callback to pick the current object ID
bool pickCallback(tiago_iaslab_simulation::pickObj::Request &req, tiago_iaslab_simulation::pickObj::Response &res) {
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //initialize response to false
    res.finishPick = false;

    /*
        For the following lines see the report for more details about each individual pose 
    */
    
    //move the robot arm to the safe position
    move_safe("safe");

    //move the robot arm to the intermediate position
    move_safe("intermediate");

    //move the robot arm above the selected object
    move_target(req);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //get all the collision objects from PlanningSceneInterface
    std::map<std::string, moveit_msgs::CollisionObject> map_objs = planning_scene_interface.getObjects();
    
    //vector of obstacles on the table
    std::vector<std::string> coll_objects(map_objs.size()-1);
    //collision object of current object to pick
    moveit_msgs::CollisionObject object_to_pick;
    //iterate all the collision object and save the collision object of the object to pick
    for(std::map<std::string, moveit_msgs::CollisionObject>::iterator it = map_objs.begin(); it != map_objs.end(); ++it) {
        if (it->first != req.object_ID) {
            coll_objects.push_back(it->first);
        } else {
            object_to_pick = it->second;
        }
    }

    //cartesian path to reach the object
    if(req.object_ID == "Hexagon") {
         cartesianPath(0.0, 0.0, CARTESIAN_HEXAGON_Z);
    }
    else if(req.object_ID == "Triangle") {
        cartesianPath(0.0, 0.0, CARTESIAN_TRIANGLE_Z);
    }
    else {
        cartesianPath(0.0, 0.0, CARTESIAN_CUBE_Z);
    }
    
    //remove object to pick from the collision objects of PlanningSceneInterface
    object_to_pick.operation = object_to_pick.REMOVE;
    planning_scene_interface.applyCollisionObject(object_to_pick);

    //create "object_to_pick" link
    std::string link = object_to_pick.id +"_link";

    //attachment of the "object_to_pick"
    AttachDetach("/link_attacher_node/attach", object_to_pick.id, link, nh);

    //close the gripper
    gripperControl(CLOSED_GRIPPER_DIMENSION);

    //return to safe with intermediate poses
    move_target(req);
    move_safe("intermediate");
    move_safe("safe");
    
    //remove all the collision objects from the PlanningSceneInterface
    planning_scene_interface.removeCollisionObjects(coll_objects);
    
    spinner.stop();

    //set success to true into the response
    res.finishPick = true;

    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_server");
    
    ros::CallbackQueue callback_n1;
    ros::NodeHandle n1;
    n1.setCallbackQueue(&callback_n1);

    ros::ServiceServer service = n1.advertiseService("pick_object", pickCallback);
    ROS_INFO("Ready to pick objects");

    while(ros::ok()) {
        callback_n1.callOne(ros::WallDuration(1));
    }
    return 0;
}