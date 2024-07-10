//Group 24
//Edoardo Bastianello, ID:2053077
//Stefano Binotto, ID: 2052421
//Gionata Grotto, ID: 2052418

//custom msg
#include <tiago_iaslab_simulation/MoveAction.h>

//services
#include "tiago_iaslab_simulation/Objs.h"
#include "tiago_iaslab_simulation/pickObj.h"
#include "gazebo_ros_link_attacher/Attach.h"

//ros header files
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf/transform_broadcaster.h>

//ros msgs
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/CollisionObject.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

//custom header files
#include <tiago_iaslab_simulation/utils.h>
#include <variables.h>

//standard library
#include <array>

//******* NAVIGATION *******

//function that initialize the global poses that we assumed
/*
0 = pose to pick blue hexagon
1 = pose to pick green triangle
2 = pose to pick red cube
3 = pose to place blue hexagon on the cylindrical table
4 = pose to place green triangle on the cylindrical table
5 = pose to place red cube on the cylindrical table
*/
std::array<tiago_iaslab_simulation::MoveGoal,6> initialize_global_pose() {

    std::array<tiago_iaslab_simulation::MoveGoal,6> g_pose;

    // red table
    g_pose[2].Pose_B.position.x = 7.482;
    g_pose[2].Pose_B.position.y = -1.977;
    g_pose[2].Pose_B.position.z = 0;
    g_pose[2].Pose_B.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -(PI/180)*70); // -70 degrees with respect to the z axis

    // green table
    g_pose[1].Pose_B.position.x = 7.774;
    g_pose[1].Pose_B.position.y = -3.869;
    g_pose[1].Pose_B.position.z = 0;
    g_pose[1].Pose_B.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, (PI/180)*100); // 100 degrees with respect to the z axis

    // blue table
    g_pose[0].Pose_B.position.x = 8.005;
    g_pose[0].Pose_B.position.y = -1.973;
    g_pose[0].Pose_B.position.z = 0;
    g_pose[0].Pose_B.orientation.x = 0;
    g_pose[0].Pose_B.orientation.y = 0;
    g_pose[0].Pose_B.orientation.z = -0.745;
    g_pose[0].Pose_B.orientation.w = 0.667;

    // red cylindrical table
    g_pose[5].Pose_B.position.x = 10.5;
    g_pose[5].Pose_B.position.y = 0.30;
    g_pose[5].Pose_B.position.z = 0;
    g_pose[5].Pose_B.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -PI/2); // -90 degrees with respect to the z axis

    // green cylindrical table
    g_pose[4].Pose_B.position.x = 11.5;
    g_pose[4].Pose_B.position.y = 0.30;
    g_pose[4].Pose_B.position.z = 0;
    g_pose[4].Pose_B.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -PI/2); // -90 degrees with respect to the z axis

    // blue cylindrical table
    g_pose[3].Pose_B.position.x = 12.5;
    g_pose[3].Pose_B.position.y = 0.30;
    g_pose[3].Pose_B.position.z = 0;
    g_pose[3].Pose_B.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -PI/2); // -90 degrees with respect to the z axis

    return g_pose;
}

//initialize poses of the 2 waypoints for navigation (see the report)
std::array<tiago_iaslab_simulation::MoveGoal,2> initialize_waypoints_pose(){

    std::array<tiago_iaslab_simulation::MoveGoal,2> w_pose;

    //low waypoint 1
    w_pose[0].Pose_B.position.x = 8.787;
    w_pose[0].Pose_B.position.y = 0.259;
    w_pose[0].Pose_B.position.z = 0;
    w_pose[0].Pose_B.orientation.x = 0;
    w_pose[0].Pose_B.orientation.y = 0;
    w_pose[0].Pose_B.orientation.z = -0.833;
    w_pose[0].Pose_B.orientation.w = 0.553;

    //high waypoint 2
    w_pose[1].Pose_B.position.x = 8.787;
    w_pose[1].Pose_B.position.y = -4.1;
    w_pose[1].Pose_B.position.z = 0;
    w_pose[1].Pose_B.orientation.x = 0;
    w_pose[1].Pose_B.orientation.y = 0;
    w_pose[1].Pose_B.orientation.z = -0.833;
    w_pose[1].Pose_B.orientation.w = 0.553;
    return w_pose;
}

//function to move the robot to the target pose using movebase
int move(tiago_iaslab_simulation::MoveGoal target_pose) {

    actionlib::SimpleActionClient<tiago_iaslab_simulation::MoveAction> ac("move", true);
    ROS_INFO("Waiting for action server robot_srv to start.");
    ac.waitForServer();
    ROS_INFO("Action server robot_srv started, sending goal.");

    //send goal to the server
    ac.sendGoal(target_pose);
    ROS_INFO("Goal has been sent.");   

    //NodeHandles
    ros::NodeHandle n1;
    ros::NodeHandle n2;

    //queue for the feedback and result topics
    ros::CallbackQueue callback_n1;
    ros::CallbackQueue callback_n2;

    //link all NodeHandle to the relative queue
    n1.setCallbackQueue(&callback_n1);
    //n2.setCallbackQueue(&callback_n2);

    //subscribe to topic "move/feedback" and to topic "move/result"
    ros::Subscriber sub = n1.subscribe("move/feedback", 1000, feedbackCallback);
    //ros::Subscriber sub2 = n2.subscribe("move/result", 1000, resultCallback);

    //call the "move/feedback" topic until "SUCCEEDED" state occurs
    while(ros::ok() && (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)) {
        callback_n1.callOne(ros::WallDuration(1));

        //print error message if robot doesn't reach the final pose in time
        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
            ROS_ERROR("The robot didn't reach the goal pose in time");
            return 1;
        }
    }

    //forcing "move/feedback" subscription to close
    sub.shutdown();
    
    //call the "move/result" topic
    //callback_n2.callOne();

    ROS_INFO("Goal reached.");

    return 0;
}

//callback that is called after a feedback is published and prints the feedback message
void feedbackCallback(const tiago_iaslab_simulation::MoveActionFeedback::ConstPtr &msg) {
    ROS_INFO("FEEDBACK STATUS: %s", msg->feedback.status.c_str());
}

//callback that prints the final result (the centers of each obstacle)
void resultCallback(const tiago_iaslab_simulation::MoveActionResult::ConstPtr &msg) {
    //vector with the centers of each obstacle
    std::vector<geometry_msgs::Point> obstacles = msg->result.obstacles;

    //no obstacles detected
    if(obstacles.size() == 0)
    {
        ROS_INFO("No movable obstacles have been detected");
        return;
    }

    //print obstacles coordinates
    for(int i = 0; i < obstacles.size(); i++)
    {
        ROS_INFO("MOVABLE OBSTACLE NUMBER [%d]: x=%f y=%f z=%f", i + 1, obstacles[i].x, obstacles[i].y, obstacles[i].z);
    }
}






//******* DETECTION AND COLLISION OBJECTS *******

//function that create the collision objects from the detections
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, const std::vector<apriltag_ros::AprilTagDetection>& detections) {

    //set size of collision objects
    std::vector<moveit_msgs::CollisionObject> collision_objects(detections.size()+1);

    //main loop to create collision objects
    for (int i = 0; i < collision_objects.size()-1; i++) {
        
        //set header frame
        collision_objects[i].header.frame_id = "base_footprint";

        //set pose
        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0] = detections[i].pose.pose.pose;

        //set operation to do in the PlanningSceneInterface
        collision_objects[i].operation = collision_objects[i].ADD;

        if(detections[i].id[0] == 3) { //red cube

            
            collision_objects[i].id = "cube";

            //define the primitive and its dimensions (of the object) and enlarge them a little bit to ensure safety
            collision_objects[i].primitives.resize(1);
            collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
            collision_objects[i].primitives[0].dimensions.resize(3);
            collision_objects[i].primitives[0].dimensions[0] = 0.05 + COLLISION_INCREMENT;    //x
            collision_objects[i].primitives[0].dimensions[1] = 0.05 + COLLISION_INCREMENT;    //y
            collision_objects[i].primitives[0].dimensions[2] = 0.05 + COLLISION_INCREMENT;    //z
            
            //find the center of the cube
            collision_objects[i].primitive_poses[0].position.z -= (collision_objects[i].primitives[0].dimensions[2]/2);

        } else if(detections[i].id[0] == 2) { //green triangle

            //set name to the one used in gazebo
            collision_objects[i].id = "Triangle";

            //define the primitive and its dimensions (the dimensions are already enlarged)
            collision_objects[i].primitives.resize(1);
            collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].CONE;
            collision_objects[i].primitives[0].dimensions.resize(2);
            collision_objects[i].primitives[0].dimensions[0] = CONE_HEIGHT;  //cone_height
            collision_objects[i].primitives[0].dimensions[1] = CONE_RADIUS;  //cone_radius 

            //adjust the rotation of the collision object since the tag is tilted
            tf2::Quaternion q_orig, q_rot, q_new;
            tf2::convert(collision_objects[i].primitive_poses[0].orientation, q_orig);
            double r = 0.0;
            double p = (PI/180)*ANGLE_DEGREE_TRIANGLE; // 35 degrees
            double y = 0.0;
            q_rot.setRPY(r,p,y);
            q_new = q_rot * q_orig;
            q_new.normalize();
            tf2::convert(q_new, collision_objects[i].primitive_poses[0].orientation);
            
        } else if(detections[i].id[0] == 1) { //blue hexagon

            //set name to the one used in gazebo
            collision_objects[i].id = "Hexagon";    

            //define the primitive and its dimensions (the dimensions are already enlarged)
            collision_objects[i].primitives.resize(1);
            collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].CYLINDER;
            collision_objects[i].primitives[0].dimensions.resize(2);
            collision_objects[i].primitives[0].dimensions[0] = HEIGHT_HEXAGON_PICK + COLLISION_INCREMENT;    //cylinder_height
            collision_objects[i].primitives[0].dimensions[1] = RADIUS_HEXAGON_PICK;    //cylinder_radius = 0.0226

            //find the center of the hexagon
            collision_objects[i].primitive_poses[0].position.z -= (collision_objects[i].primitives[0].dimensions[0]/2);

        } else if(detections[i].id[0] == 4 || detections[i].id[0] == 5 || detections[i].id[0] == 6 || detections[i].id[0] == 7 ) { //hexagon obstacles

            //set name to the one used in gazebo
            collision_objects[i].id = "Obstacle" + std::to_string(detections[i].id[0]-4);

            //Define the primitive and its dimensions (of the object) and enlarge them a little bit to ensure safety
            collision_objects[i].primitives.resize(1);
            collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].CYLINDER;
            collision_objects[i].primitives[0].dimensions.resize(2);
            collision_objects[i].primitives[0].dimensions[0] = HEIGHT_HEXAGON_OBSTACLE + COLLISION_INCREMENT*2;     //cylinder_height
            collision_objects[i].primitives[0].dimensions[1] = RADIUS_HEXAGON_OBSTACLE + COLLISION_INCREMENT;       //cylinder_radius = 0.046

            //find the center of the hexagonal obstacle
            collision_objects[i].primitive_poses[0].position.z -= (collision_objects[i].primitives[0].dimensions[0]/2);

        }
    }
    //index of the collision object of the table
    int j = collision_objects.size()-1;

    //set name to the one used in gazebo
    collision_objects[j].id = "Table";

    //set frame
    collision_objects[j].header.frame_id = "base_footprint";
    
    //define the primitive and its dimensions of the table
    collision_objects[j].primitives.resize(1);
    collision_objects[j].primitives[0].type = collision_objects[j].primitives[0].BOX;
    collision_objects[j].primitives[0].dimensions.resize(3);

    collision_objects[j].primitives[0].dimensions[0] = TABLE_X; //x
    collision_objects[j].primitives[0].dimensions[1] = TABLE_Y; //y
    collision_objects[j].primitives[0].dimensions[2] = TABLE_Z; //z

    //create pose table
    geometry_msgs::Pose table_pose;
    table_pose.position.x = TABLE_XMAP_POSITION;
    table_pose.position.y = TABLE_YMAP_POSITION;
    
    //convert from gazebo origin to map frame
    table_pose = convertOriginToMap(table_pose);

    table_pose.position.z = HEIGHT_TABLE_COLUMN / 2;
    table_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    
    //convert from map frame to base_footprint
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("base_footprint", "map", ros::Time(0), ros::Duration(3));  // check if frame map is correct
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("Can't build tranformer object.");
        exit(0);
    }
    tf2::doTransform(table_pose, table_pose, transformStamped);

    collision_objects[j].primitive_poses.resize(1);
    collision_objects[j].primitive_poses[0] = table_pose;
    collision_objects[j].operation = collision_objects[j].ADD;
    
    //add all callision objects to the scene
    planning_scene_interface.addCollisionObjects(collision_objects);
}

//add cylindrical tables to the interface as collision objects
void addCylinderTablesCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {

    // number of collision objects equals number of tables
    std::vector<moveit_msgs::CollisionObject> collision_objects(NUMBER_OBJECT_TO_PICK); 
    
    //for each cylindrical table creates its collision objects
    for(int k = 0; k<3; k++) {
        
        //frame
        collision_objects[k].header.frame_id = "base_footprint";

        //define the primitive and its dimensions of the cilinder
        collision_objects[k].primitives.resize(1);
        collision_objects[k].primitives[0].type = collision_objects[k].primitives[0].CYLINDER;
        collision_objects[k].primitives[0].dimensions.resize(2);

        //pose of the cylinder
        geometry_msgs::Pose cylinder_pose;

        //set right position
        if(k==0) {
            collision_objects[k].id = "place_table_b";
            cylinder_pose.position.x = BLUE_TABLE_X;
            cylinder_pose.position.y = BLUE_TABLE_Y;
        } else if(k==1) {
            collision_objects[k].id = "place_table_g";
            cylinder_pose.position.x = GREEN_TABLE_X;
            cylinder_pose.position.y = GREEN_TABLE_Y;
        } else if(k==2) {
            collision_objects[k].id = "place_table_r";
            cylinder_pose.position.x = RED_TABLE_X;
            cylinder_pose.position.y = RED_TABLE_Y;
        }

        //convert from gazebo origin to map frame
        cylinder_pose = convertOriginToMap(cylinder_pose);

        //set center of the cylindrical table
        cylinder_pose.position.z = 0.345;

        //set dimensions of the collision object and enlarge it to ensure safety
        collision_objects[k].primitives[0].dimensions[0] = cylinder_pose.position.z * 2 + COLLISION_INCREMENT;
        collision_objects[k].primitives[0].dimensions[1] = 0.21 + COLLISION_INCREMENT*7;

        //orientation
        cylinder_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        //convert from map frame to base_footprint
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("base_footprint", "map", ros::Time(0), ros::Duration(3));
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("Can't build tranformer object.");
            exit(0);
        }
        tf2::doTransform(cylinder_pose, cylinder_pose, transformStamped);
        
        collision_objects[k].primitive_poses.resize(1);
        collision_objects[k].primitive_poses[0] = cylinder_pose;
        collision_objects[k].operation = collision_objects[k].ADD;
    }

    planning_scene_interface.addCollisionObjects(collision_objects);
}

//function that convert from gazebo origin to map frame
geometry_msgs::Pose convertOriginToMap(const geometry_msgs::Pose& input_pose) {

    geometry_msgs::Pose new_pose = input_pose;

    new_pose.position.x -= GAZEBO_COORD_X;
    new_pose.position.x -= 0.005;
    new_pose.position.y -= GAZEBO_COORD_Y;

    return new_pose;
}




//******* PICK AND PLACE *******

//move to safe or intermediate configuration (see the report)
bool move_safe(std::string position) {
    
    //setting target configuration
    std::map<std::string, double> target_position;

    if(position == "safe") {
        //safe pose is a configuration that keeps the arm close to the body (ideal for navigation)
        target_position["torso_lift_joint"] = 0.15; // like the initial configuration
        target_position["arm_1_joint"] = 0.262;
        target_position["arm_2_joint"] = -1.38;
        target_position["arm_3_joint"] = -0.59;
        target_position["arm_4_joint"] = 1.89;
        target_position["arm_5_joint"] = -1.54;
        target_position["arm_6_joint"] = 1.38;
        target_position["arm_7_joint"] = 0.314;
    } else if(position == "intermediate") {
        //intermediate pose is a configuration with the arm in a high position to decrease the possibilities of having a collision
        target_position["torso_lift_joint"] = 0.350;
        target_position["arm_1_joint"] = 0.80;
        target_position["arm_2_joint"] = 0.80;
        target_position["arm_3_joint"] = -3,02;
        target_position["arm_4_joint"] = 0.733;
        target_position["arm_5_joint"] = -0.21;
        target_position["arm_6_joint"] = 0.37;
        target_position["arm_7_joint"] = 0.91;
    } else {
        ROS_ERROR("The string position can only be ""safe"" or ""intermediate""");
    }

    //joints
    std::vector<std::string> torso_arm_joint_names;
    
    //select group of joints (both arm and torso)
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    
    //choose planner
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    //set parameters
    torso_arm_joint_names = group_arm_torso.getJoints();
    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(JOINT_MOVEMENT_VELOCITY);

    for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    {
        if (target_position.count(torso_arm_joint_names[i]) > 0)
        {
            //ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
            group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
        }
    }

    //plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    group_arm_torso.setPlanningTime(PLANNING_TIME);
    bool success = (bool)group_arm_torso.plan(my_plan);
    if (!success) {
        throw std::runtime_error("No plan found");
        ROS_INFO("PLANNING ERROR");
    } else {
        ROS_INFO("PLANNING SUCCESS");
    }
    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
    
    // Execute the plan
    //ros::Time start_fk = ros::Time::now();
    group_arm_torso.move();

    //ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_fk).toSec());

    return true;
}

//function to move the arm above the tag of the target object to pick
bool move_target(tiago_iaslab_simulation::pickObj::Request &req)
{
    //move both arm and torso
    moveit::planning_interface::MoveGroupInterface group_arm("arm_torso");

    //choose planner
    group_arm.setPlannerId("SBLkConfigDefault");

    //set reference frame (in this case will be base_footprint)
    group_arm.setPoseReferenceFrame(req.object_pose.header.frame_id);

    //goal pose
    geometry_msgs::PoseStamped goal_target_pose;
    goal_target_pose.header.frame_id = "base_footprint";
    goal_target_pose.pose.position.x = req.object_pose.pose.position.x;
    goal_target_pose.pose.position.y = req.object_pose.pose.position.y;
    goal_target_pose.pose.position.z = req.object_pose.pose.position.z;
    goal_target_pose.pose.orientation = req.object_pose.pose.orientation;

    //orient the gripper relative to the object orientation
    double roll_pose, pitch_pose, yaw_pose;
    tf::Quaternion orient(goal_target_pose.pose.orientation.x, goal_target_pose.pose.orientation.y, goal_target_pose.pose.orientation.z, goal_target_pose.pose.orientation.w);
    tf::Matrix3x3 m(orient);
    m.getRPY(roll_pose, pitch_pose, yaw_pose);
    goal_target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.0*yaw_pose, 0, 0);  
    
    //rotate the gripper by 90 degrees with respect to the y axis (vertical position of the gripper)
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(goal_target_pose.pose.orientation, q_orig);
    double r = 0.0;
    double p = PI/2;
    double y = 0.0;
    q_rot.setRPY(r,p,y);
    q_new = q_rot * q_orig;
    q_new.normalize();
    tf2::convert(q_new, goal_target_pose.pose.orientation);
    
    //set target
    group_arm.setPoseTarget(goal_target_pose);
    //set parameters
    group_arm.setStartStateToCurrentState();
    group_arm.setMaxVelocityScalingFactor(1.0);

    //plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_ik;
    group_arm.setPlanningTime(PLANNING_TIME);

    //check if plan is successfull
    bool success = bool(group_arm.plan(my_plan_ik));
    if (!success) {
        throw std::runtime_error("No plan found");
        ROS_INFO("PLANNING ERROR");
    } else {
        ROS_INFO("PLANNING SUCCESS");
    }
    ROS_INFO_STREAM("Plan found in " << my_plan_ik.planning_time_ << " seconds");

    //execute the plan
    moveit::planning_interface::MoveItErrorCode e = group_arm.move();
    if (!bool(e)) {
        throw std::runtime_error("Error executing plan");
    }

    return true;
}

//compute cartesian path that moves the gripper of a quantity specified by the input parameters
void cartesianPath(double x_movement, double y_movement, double z_movement) {
    
    //can move both arm and torso
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");

    //array with waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    
    //finalPose
    geometry_msgs::Pose finalPose = group_arm_torso.getCurrentPose().pose;
    finalPose.position.x += x_movement;
    finalPose.position.y += y_movement;
    finalPose.position.z += z_movement;

    //push final pose in the vector of waypoints
    waypoints.push_back(finalPose);

    //set velocity of robotic arm
    group_arm_torso.setMaxVelocityScalingFactor(ARM_VELOCITY);
    
    //jump threshold = 0 ---> disabled
    const double jump_threshold = 0.0;

    //end effector interpolation resolution
    const double eef_step = END_EFFECTOR_INTERP_RESOLUTION;

    moveit_msgs::RobotTrajectory trajectory;

    double fraction = group_arm_torso.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    //plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    //execute the trajectory
    group_arm_torso.execute(trajectory);
}

//function to open/close the gripper
void gripperControl(float position) {

    //user-defined gripper_controller action client to call gripper_controller
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_gripper("/gripper_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server gripper_controller to start.");
    ac_gripper.waitForServer();
    ROS_INFO("Action server gripper_controller started, sending goal.");

    //send goal pose to gripper_controller
    control_msgs::FollowJointTrajectoryGoal goal_gripper_controller;

    //joint names to control
    goal_gripper_controller.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal_gripper_controller.trajectory.joint_names.push_back("gripper_right_finger_joint");

    //we used only one waypoint in this goal trajectory
    goal_gripper_controller.trajectory.points.resize(1);

    //the trajectory point
    //positions
    goal_gripper_controller.trajectory.points[0].positions.resize(2);
    goal_gripper_controller.trajectory.points[0].positions[0] = position;
    goal_gripper_controller.trajectory.points[0].positions[1] = position;

    //velocities
    goal_gripper_controller.trajectory.points[0].velocities.resize(2);
    goal_gripper_controller.trajectory.points[0].velocities[0] = GRIPPER_VELOCITY;
    goal_gripper_controller.trajectory.points[0].velocities[1] = GRIPPER_VELOCITY;

    //to be reached 2 seconds after starting along the trajectory
    goal_gripper_controller.trajectory.points[0].time_from_start = ros::Duration(GRIPPER_MOVEMENT_DURATION);

    //header
    goal_gripper_controller.trajectory.header.stamp = ros::Time::now();

    //send Goal
    ac_gripper.sendGoal(goal_gripper_controller);
    ROS_INFO("Goal has been sent.");

    //wait for trajectory execution
    while(!(ac_gripper.getState().isDone()) && ros::ok()) {
        ros::Duration(GRIPPER_MOVEMENT_DURATION).sleep(); // sleep for two seconds until the gripper movement is done
    }
}

//function to move the arm above the cylindrical table
bool move_to_place(const geometry_msgs::PoseStamped& target_pose) {
    
    //move both arm and torso
    moveit::planning_interface::MoveGroupInterface group_arm("arm_torso");

    //choose planner
    group_arm.setPlannerId("SBLkConfigDefault");

    //set reference frame
    group_arm.setPoseReferenceFrame(target_pose.header.frame_id);

    //goal pose
    geometry_msgs::PoseStamped goal_target_pose = target_pose;    
    
    //set initial orientation
    goal_target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    
    //rotate the gripper by 90 degrees with respect to the y axis (vertical position of the gripper)
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(goal_target_pose.pose.orientation, q_orig);
    double r = 0.0;
    double p = 3.14159/2;
    double y = 0.0;
    q_rot.setRPY(r,p,y);
    q_new = q_rot * q_orig;
    q_new.normalize();
    tf2::convert(q_new, goal_target_pose.pose.orientation);
    
    //set target
    group_arm.setPoseTarget(goal_target_pose);

    //set parameters
    group_arm.setStartStateToCurrentState();
    group_arm.setMaxVelocityScalingFactor(JOINT_MOVEMENT_VELOCITY);

    //plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_ik;
    group_arm.setPlanningTime(PLANNING_TIME);

    //plan
    bool success = bool(group_arm.plan(my_plan_ik));
    if (!success) {
        throw std::runtime_error("No plan found");
        ROS_INFO("PLANNING ERROR");
    } else {
        ROS_INFO("PLANNING SUCCESS");
    }

    ROS_INFO_STREAM("Plan found in " << my_plan_ik.planning_time_ << " seconds");

    //execute
    moveit::planning_interface::MoveItErrorCode e = group_arm.move();
    if (!bool(e)) {
        throw std::runtime_error("Error executing plan");
    }

    return true;
}

//function to compute the final pick pose to pick the object specified in the pose parameters
geometry_msgs::PoseStamped computeFinalPickPose(const geometry_msgs::Pose pose_target, const int id_obj) {
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_footprint";
    goal_pose.pose.position = pose_target.position;
    goal_pose.pose.orientation = pose_target.orientation;

    //set a specific offset for each specific object to pick
    if (id_obj == 1) {
        //hexagon
        goal_pose.pose.position.z += BLUE_Z_OFFSET;
    } else if (id_obj == 2) {
        //triangle
        goal_pose.pose.position.z += GREEN_Z_OFFSET;
    } else {
        //cube
        goal_pose.pose.position.z += RED_Z_OFFSET;
    }

    return goal_pose;
}

//function to attach or detach an object to the arm (arm_7_link)
void AttachDetach(std::string service_name, std::string model2, std::string link2, ros::NodeHandle nh) {

    ros::ServiceClient client = nh.serviceClient<gazebo_ros_link_attacher::Attach>(service_name);
    gazebo_ros_link_attacher::Attach srv;

    //define the 2 objects to attach
    srv.request.model_name_1 = "tiago";
    srv.request.link_name_1 = "arm_7_link";
    srv.request.model_name_2 = model2;
    srv.request.link_name_2 = link2;

    if(client.call(srv)) {
        if (service_name == "/link_attacher_node/attach") {
            ROS_INFO("Attached the object to tiago arm");
        } else {
            ROS_INFO("Detached the object to tiago arm");
        }
    } else {
        if (service_name == "/link_attacher_node/attach") {
            ROS_ERROR("Attach error!");
        } else {
            ROS_ERROR("Detach error!");
        }
    }
}