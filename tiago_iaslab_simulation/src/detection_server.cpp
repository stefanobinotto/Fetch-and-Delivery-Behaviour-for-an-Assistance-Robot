//Group 24
//Edoardo Bastianello, ID:2053077
//Stefano Binotto, ID: 2052421
//Gionata Grotto, ID: 2052418

//services
#include "tiago_iaslab_simulation/srv_detect.h"

//ros header files
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <tf2_ros/transform_listener.h>

//ros msgs
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


//callback function to detect objects on the table
void detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg, tiago_iaslab_simulation::srv_detect::Response &res) {

    //initialize vector of detected objects
    std::vector<apriltag_ros::AprilTagDetection> detected_objects(0);

    //create transform to convert camera frame to base_footprint
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("base_footprint", msg->header.frame_id, ros::Time(0), ros::Duration(3));
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Can't build tranformer object.");
        exit(0);
    }

    //for each detected object
    for (int i = 0; i < msg->detections.size(); i++) {

        //id of current detected object
        int current_ID = (int)msg->detections[i].id[0];

        //pose of current object
        geometry_msgs::Pose pose_ID = msg->detections[i].pose.pose.pose;

        //convert to robot "base_footprint" frame
        tf2::doTransform(pose_ID, pose_ID, transformStamped);

        //save the pose
        apriltag_ros::AprilTagDetection tmp = msg->detections[i];
        tmp.pose.pose.pose = pose_ID;

        //insert the detection into the detection array
        detected_objects.push_back(tmp);
    }

    //assign the detected objects to the response
    res.collision_objects = detected_objects;

    ROS_INFO("Detection completed. Results have been sent.");
}

//callback function
bool callbackfunction(tiago_iaslab_simulation::srv_detect::Request &req, tiago_iaslab_simulation::srv_detect::Response &res) {

    //callback queue and node
    ros::CallbackQueue callback_n1;
    ros::NodeHandle n1;
    n1.setCallbackQueue(&callback_n1);

    //subscribe to tag detections topic
    //res is passed as a reference to detectionCallback
    ros::Subscriber sub = n1.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections", 10000, boost::bind(detectionCallback, _1, boost::ref(res)));
    callback_n1.callOne(ros::WallDuration(1));
    sub.shutdown();

    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "detection_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("detect_objects", callbackfunction);
    ROS_INFO("Ready to detect objects");

    ros::spin();
    return 0;
}