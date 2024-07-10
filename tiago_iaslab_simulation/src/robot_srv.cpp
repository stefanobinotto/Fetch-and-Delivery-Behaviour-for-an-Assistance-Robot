//Group 24
//Edoardo Bastianello, ID:2053077
//Stefano Binotto, ID: 2052421
//Gionata Grotto, ID: 2052418

//custom msg
#include <tiago_iaslab_simulation/MoveAction.h>

//ros header files
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//ros msgs
#include <move_base_msgs/MoveBaseAction.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"

//opencv header file
#include "opencv2/core/core.hpp"

#include <time.h>

//time limit to reach the goal
#define MAX_TIME_TO_REACH_GOAL 200

//we set 0,15rad as the threshold to identify when the robot is turning or just moving basically straight
#define RADIUS_THRESHOLD_WHEN_TURNING 0.15

//diameter of movable obstacle is 0.36
//here we set it to 0.37 to accomodate for scan errors
#define DIAMETER_MOVABLE_OBJECT 0.37

//threshold to subdivide the detections of the "ranges" array in different sections
//each section is defined as the set of consequent detections with distance between each other <= THRESHOLD_DISTANCE
#define THRESHOLD_DISTANCE 0.25

class PlannerRobot
{
protected:
    //NodeHandle
    ros::NodeHandle nh_;
    //action server
    actionlib::SimpleActionServer<tiago_iaslab_simulation::MoveAction> as_;
    //action name
    std::string action_name_;
    //feedback
    tiago_iaslab_simulation::MoveFeedback feedback_;
    //result
    tiago_iaslab_simulation::MoveResult result_;

    //NodeHandles to subsbribe to "scan" topic and "nav_vel" topic
    ros::NodeHandle node_scan;
    ros::NodeHandle node_velocity;

public:

    PlannerRobot(std::string name): as_(nh_, name, boost::bind(&PlannerRobot::move_towards_goal, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    ~PlannerRobot(void){}

    //function that sends the goal to move_base and waits for the robot to reach the goal pose
    void move_towards_goal(const tiago_iaslab_simulation::MoveGoalConstPtr &goal)
    {
        //user-defined move_base action client to call move_base
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move("move_base", true);
        ROS_INFO("Waiting for action server move_base to start.");
        ac_move.waitForServer();
        ROS_INFO("Action server move_base started, sending goal.");
        
        //send goal pose to move_base
        move_base_msgs::MoveBaseGoal goal_move_base;
        goal_move_base.target_pose.header.frame_id = "map";
        goal_move_base.target_pose.header.stamp = ros::Time::now();
        goal_move_base.target_pose.pose.position = goal->Pose_B.position;
        goal_move_base.target_pose.pose.orientation = goal->Pose_B.orientation;
        ac_move.sendGoal(goal_move_base);
        ROS_INFO("Goal has been sent.");

        //publish feedback (robot is moving)
        feedback_.status = "The robot is moving";
        feedback_.head.stamp = ros::Time::now();
        as_.publishFeedback(feedback_);

        //keep track of the time
        time_t start_time = time(0);
        bool reached_goal_before_timeout = true;

        //queue for the subscription to "nav_vel" topic
        ros::CallbackQueue callback_velocity;
        node_velocity.setCallbackQueue(&callback_velocity);
        ros::Subscriber sub_velocity = node_velocity.subscribe("nav_vel", 10000, &PlannerRobot::velocityCallback, this);
        
        //callback to velocity (check if robot is moving or stopped moving)
        do {
            callback_velocity.callOne(ros::WallDuration(1));

            //stop if the timeout is exceeded
            if (static_cast<double>(difftime(time(0), start_time)) > MAX_TIME_TO_REACH_GOAL) {
                reached_goal_before_timeout = false;
            } 
        } while (ros::ok() && (ac_move.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) && reached_goal_before_timeout);
        
        //robot reached the goal position before timeout
        if (reached_goal_before_timeout)
        {
            //publish feedback (robot reached the goal)
            feedback_.status = "The robot reached the goal";
            feedback_.head.stamp = ros::Time::now();
            as_.publishFeedback(feedback_);

            //queue for the subscription to "scan" topic
            //ros::CallbackQueue callback_scan;
            //node_scan.setCallbackQueue(&callback_scan);
            //ros::Subscriber sub_scan = node_scan.subscribe("scan", 10000, &PlannerRobot::scanCallback, this);
            
            //callback_scan.callOne(ros::WallDuration(1));
            
            //the points are sent to the client
            as_.setSucceeded(result_);
            //ROS_INFO("The results have been sent to the client.");
        }
        else
        {
            //the robot didn't reach the goal before timeout
            ROS_INFO("The robot didn't reached the goal before timeout.");
            as_.setAborted(result_);
        }
    }

    //callback function for checking the velocity of the robot
    void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        //linear velocity
        geometry_msgs::Vector3 linear_velocity = msg->linear;
        float x_linear = linear_velocity.x;
        float y_linear = linear_velocity.y;
        float z_linear = linear_velocity.z;

        //angular velocity
        geometry_msgs::Vector3 angular_velocity = msg->angular;
        float z_angular = angular_velocity.z;       // the robot can only turn on the z dimension

        //check if the robot is moving, turning or has stoppped moving
        if(std::abs(z_angular)>RADIUS_THRESHOLD_WHEN_TURNING) {
            feedback_.status = "The robot is turning";
        } else if(x_linear != 0 || y_linear != 0 || z_linear != 0) {
            feedback_.status = "The robot is moving";
        } else {
            feedback_.status = "The robot is stopped";
        }

        //publish feedback (robot action/status)
        feedback_.head.stamp = ros::Time::now();
        as_.publishFeedback(feedback_);
    }

    //assignment1 callback. It isn't used in assignment2
    //scan callback that computes the centers of the movable obstabcles
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {   
        //publish feedback (the robot started the detection of the obstacles)
        feedback_.status = "The robot started the detection of the obstacles";
        feedback_.head.stamp = ros::Time::now();
        as_.publishFeedback(feedback_);

        //vector containing the points of each section
        std::vector<std::vector<cv::Point2f>> points_of_sections = subdivide_in_sections(msg);

        //vector containing the point of the movable obstacles
        std::vector<std::vector<cv::Point2f>> valid_sections = obstacles_finder(points_of_sections);

        //vector containing the centers of each movable obstacles
        std::vector<cv::Point2f> centers;
        centers.reserve(valid_sections.size());

        //compute centers of movable obstacles
        for (int i = 0; i < valid_sections.size(); i++) {
            centers.push_back(center_circle(valid_sections[i]));
        }

        //convert OpenCV points to "geometry_msgs/Point" and store them in the "result_" variable
        std::vector<geometry_msgs::Point> final_points(centers.size());
        for (int i = 0; i < centers.size(); i++) {
            geometry_msgs::Point tmp_point;
            tmp_point.x = centers[i].x;
            tmp_point.y = centers[i].y;
            tmp_point.z = 0;
            final_points[i] = tmp_point;
        }
        result_.obstacles = final_points;

        // publish feedback (the robot finished the detection of the obstacles)
        feedback_.status = "The robot finished the detection of the obstacles";
        feedback_.head.stamp = ros::Time::now();
        as_.publishFeedback(feedback_);
    }

    //subdivide the detections of the "ranges" array in differents sections depending on the distance between them
    //each section is defined as the set of consequent detections with distance between each other <= THRESHOLD_DISTANCE
    std::vector<std::vector<cv::Point2f>> subdivide_in_sections(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        //section that is being analyzed
        int current_section = 0;

        //angle increment of laser detections
        float increment = msg->angle_increment;

        //point in previous position in "ranges" array
        cv::Point2f previous_point;

        //vector containing the points of each section
        std::vector<std::vector<cv::Point2f>> points_of_sections;

        //subdivide the detections in different sections
        for (int i = 19; i < (msg->ranges.size() - 18); i++) //discard first 19 points and last 18 points since the laser hits the robot
        {
            //computation only for points in laser range
            if ((msg->ranges[i] <= msg->range_max) && (msg->ranges[i] >= msg->range_min))
            {
                // get angle
                float angle_rad = increment * i + msg->angle_min;

                float x = (msg->ranges[i]) * cos(angle_rad); // get x
                float y = (msg->ranges[i]) * sin(angle_rad); // get y

                // create a cv::point with this coordinates
                cv::Point2f point(x, y);

                //first point of ranges array
                if(i == 19)
                {
                    //add first point to vector of vector of points
                    std::vector<cv::Point2f> section_zero;
                    section_zero.push_back(point);
                    points_of_sections.push_back(section_zero);
                }
                else //check if the point belongs to the same section
                {
                    //compute distance from the point in the previous cell of ranges array
                    double distance_from_previous = cv::norm(point -  previous_point);
                    
                    //check if the point belongs to a different section
                    if(distance_from_previous > THRESHOLD_DISTANCE) //new section
                    {
                        //add point to the new section
                        current_section++; //update counter of sections
                        std::vector<cv::Point2f> new_section;
                        new_section.push_back(point);
                        points_of_sections.push_back(new_section);
                    }
                    else //same section
                    {
                        //add point to previous section
                        points_of_sections[current_section].push_back(point);
                    }
                }

                //update previous point
                previous_point = point;
            }
        }

        return points_of_sections;
    }

    //function to find the movable obstacles given all the different sections
    std::vector<std::vector<cv::Point2f>> obstacles_finder(const std::vector<std::vector<cv::Point2f>> &sections)
    {
        //radius of the movable ostacle
        float radius_obj = DIAMETER_MOVABLE_OBJECT/2;

        //vector containing movable obstacles
        std::vector<std::vector<cv::Point2f>> valid;

        //loop for each section
        for (int i = 0; i < sections.size(); i++) {

            //section to be evaluated
            std::vector<cv::Point2f> section = sections[i];

            //distance between first and last point of the section
            double length_of_section = cv::norm(section[0] -  section[section.size()-1]);

            if (length_of_section <= DIAMETER_MOVABLE_OBJECT && (section.size() >= 5)) {

                //center of the section to be evaluated
                cv::Point2f center = center_circle(section);
                
                bool flag = true;

                //loop for each point of the section
                for (int j = 0; j < section.size(); j++) {

                    //distance between the center of the section and the current point of the section
                    double distance = cv::norm(center-section[j]);
                    
                    //error of the radius to accomodate for measurement errors
                    const float ERROR_RADIUS = 0.02;

                    /*
                    if the distance between the center and the point is not close to the radius of the obstacle,
                    then pass to the next section
                    */
                    if (!((radius_obj-ERROR_RADIUS<=distance) && (distance<=radius_obj+ERROR_RADIUS))) { 
                        flag = false;
                        break;
                    }
                }
                //if flag == true, then it's an movable obstacle
                if (flag) {
                    //add current section to "valid" vector
                    valid.push_back(section);
                }
            }
        }

        return valid;
    }

    //function that returns the center of a circle using 3 of its points
    cv::Point2f center_circle(const std::vector<cv::Point2f> &section) 
    {
        int middle_element_index = section.size() / 2;
        int last_element_index = section.size() - 1;

        float x12 = section[0].x - section[middle_element_index].x;
        float x13 = section[0].x - section[last_element_index].x;

        float y12 = section[0].y - section[middle_element_index].y;
        float y13 = section[0].y - section[last_element_index].y;

        float x31 = section[last_element_index].x - section[0].x;
        float x21 = section[middle_element_index].x - section[0].x;

        float y31 = section[last_element_index].y - section[0].y;
        float y21 = section[middle_element_index].y - section[0].y;

        float sx13 = pow(section[0].x, 2) - pow(section[last_element_index].x, 2);
        float sy13 = pow(section[0].y, 2) - pow(section[last_element_index].y, 2);
        float sx21 = pow(section[middle_element_index].x, 2) - pow(section[0].x, 2);
        float sy21 = pow(section[middle_element_index].y, 2) - pow(section[0].y, 2);

        //y of the center
        float y_circle = - (sx13 * x12 + sy13 * x12 + sx21 * x13 + sy21 * x13) / (2 * (y31 * x12 - y21 * x13));
        //x of the center
        float x_circle = - (sx13 * y12 + sy13 * y12 + sx21 * y13 + sy21 * y13) / (2 * (x31 * y12 - x21 * y13));

        return cv::Point2f(x_circle, y_circle);
    }
};

//main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rob_srv");

    //server
    PlannerRobot srv("move");
    
    ros::spin();
    return 0;
}