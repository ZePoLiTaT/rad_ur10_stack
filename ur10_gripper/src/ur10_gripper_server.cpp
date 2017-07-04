#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "ros/ros.h"
#include "ur10_gripper_msgs/UR10.h"

#include <cmath>
#include <iostream>
#include <vector>

#define MOVETO 0
#define PUSH 1
#define TIMEDISTANCE 2
#define EXECUTE 3

std::vector<moveit::planning_interface::MoveGroup::Plan> plans;
int numPlans = 0;

double deg2rad(double d)
{
    double ratio = M_PI / 180.0;
    return d * ratio;
}

void setRPYGoal(geometry_msgs::Pose &goal, double roll, double pitch, double yaw)
{
    double y = yaw;
    double r = roll;
    double p = pitch;

    double t0 = std::cos(y * 0.5);
    double t1 = std::sin(y * 0.5);
    double t2 = std::cos(r * 0.5);
    double t3 = std::sin(r * 0.5);
    double t4 = std::cos(p * 0.5);
    double t5 = std::sin(p * 0.5);

    goal.orientation.w = t0 * t2 * t4 + t1 * t3 * t5;
    goal.orientation.x = t0 * t3 * t4 - t1 * t2 * t5;
    goal.orientation.y = t0 * t2 * t5 + t1 * t3 * t4;
    goal.orientation.z = t1 * t2 * t4 - t0 * t3 * t5;
    return;
}

bool moveTo(ur10_gripper_msgs::UR10::Request  &req,
            ur10_gripper_msgs::UR10::Response &res, 
            moveit::planning_interface::MoveGroup &group)
{
    if(req.pose.size() < 6)
    {
        ROS_ERROR("Invalid Request Parameters:");
        ROS_ERROR("Provide X, Y, Z, R, P, Y in pose vector");
        return false;
    }
    ROS_INFO("Move Request Recieved");
    ROS_INFO_STREAM("X: " << req.pose.at(0) << " Y: " << req.pose.at(1) 
                << " Z: " << req.pose.at(2) << " R: " << req.pose.at(3)
                << " P: " << req.pose.at(4) << " Y: " << req.pose.at(5));

    if(req.plan == true)
    {
        ROS_INFO("Planning Only Mode");
    }
    group.setPlanningTime(10.0);
    //group.setGoalPositionTolerance(0.1);
    geometry_msgs::Pose goal;

    goal.position.x = req.pose[0];
    goal.position.y = req.pose[1];
    goal.position.z = req.pose[2] - 1.43;
    setRPYGoal(goal, req.pose[3], req.pose[4], req.pose[5]);

    group.setPoseTarget(goal);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    ROS_INFO_STREAM("Planning motion");
    res.success = group.plan(my_plan);
    
    sleep(1.0);
    //Actually Move if not a planning only request
    if(res.success)
    {
        if (req.plan == false)
        {
            ROS_INFO_STREAM("Moving to planned pose");
            res.success = group.execute(my_plan);
            sleep(5.0);
        }
        /*
        else
        {
            plans.push_back(my_plan);
            std::cout << "Plan stored as plan #" << plans.size() - 1 << std::endl;
        }
        */
    }

    return true;
}

bool push(ur10_gripper_msgs::UR10::Request  &req,
          ur10_gripper_msgs::UR10::Response &res,
          moveit::planning_interface::MoveGroup &group)
{
    double leadDistance = 0.05;

    if(req.pose.size() < 5)
    {
        ROS_ERROR("Invalid Request Parameters:");
        ROS_ERROR("Provide X, Y, Z of object in pose vector"); 
        ROS_ERROR("followed by direction theta and distance d");
        return false;
    }
    ROS_INFO("Push Request Recieved");
    ROS_INFO_STREAM("X: " << req.pose.at(0) << " Y: " << req.pose.at(1) 
                << " Z: " << req.pose.at(2));
    ROS_INFO_STREAM("Push Angle = " << req.pose.at(3));
    ROS_INFO_STREAM("Push Distance = " << req.pose.at(3));
    
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(3);
    
    //Set object as collision

    //Move to Initial Pose
    geometry_msgs::Pose goal1;

    goal1.position.x = req.pose[0] - leadDistance * cos(req.pose[3]);
    goal1.position.y = req.pose[1] - leadDistance * sin(req.pose[3]);
    goal1.position.z = req.pose[2] - 1.43;
    setRPYGoal(goal1, 3.14, 0.0, req.pose[3]);

    group.setPoseTarget(goal1);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    ROS_INFO_STREAM("Planning motion to initial Pose");
    res.success = group.plan(my_plan);
    sleep(3.0);
    if(res.success)
    {
        res.success = group.execute(my_plan);
        ROS_INFO_STREAM("Moving to initial Pose");
    }
    sleep(7.0);

    //Remove collision

    //Move with constraint
    /*
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = group.getEndEffectorLink();
    ocm.orientation = goal1.orientation;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints pushConstraint;
    pushConstraint.orientation_constraints.push_back(ocm);
    group.setPathConstraints(pushConstraint);
    */

    if(!res.success)
    {
        return true;
    }
    geometry_msgs::Pose goal2;

    goal2.position.x = req.pose[0] + req.pose[4] * cos(req.pose[3]);
    goal2.position.y = req.pose[1] + req.pose[4] * sin(req.pose[3]);
    goal2.position.z = req.pose[2] - 1.43;
    setRPYGoal(goal2, 3.14, 0.0, req.pose[3]);

    group.setPoseTarget(goal2);

    moveit::planning_interface::MoveGroup::Plan my_plan2;
    ROS_INFO_STREAM("Planning push");
    res.success = group.plan(my_plan2);
    sleep(3.0);
    if(res.success)
    {
        res.success = group.execute(my_plan2);
        ROS_INFO_STREAM("Pushing");
    }
    sleep(5.0);

    group.clearPathConstraints();


    return true;
}


void printStart(moveit::planning_interface::MoveGroup &group)
{
    std::cout << std::endl;
    std::cout << "Start Pose" << std::endl;
    std::cout << "X: " << group.getCurrentPose().pose.position.x
              << " Y: " << group.getCurrentPose().pose.position.y
              << " Z: " << group.getCurrentPose().pose.position.z + 1.43 << std::endl;


    std::cout << "Qx: " << group.getCurrentPose().pose.orientation.x
              << " Qy: " << group.getCurrentPose().pose.orientation.y
              << " Qz: " << group.getCurrentPose().pose.orientation.z
              << " Qw: " << group.getCurrentPose().pose.orientation.w << std::endl;

    for (int i = 0; i < 6; ++i)
    {
        std::cout << group.getJoints().at(i) << ": " 
                  << group.getCurrentJointValues().at(i) << std::endl;
    }
    std::cout << std::endl;
}

void printGoal(geometry_msgs::Pose goal, moveit::planning_interface::MoveGroup &group,
                moveit::planning_interface::MoveGroup::Plan &plan)
{
    std::cout << "Goal Pose" << std::endl;
    std::cout << "X: " << goal.position.x
              << " Y: " << goal.position.y
              << " Z: " << goal.position.z + 1.43 << std::endl;


    std::cout << "Qx: " << goal.orientation.x
              << " Qy: " << goal.orientation.y
              << " Qz: " << goal.orientation.z
              << " Qw: " << goal.orientation.w << std::endl;

    std::vector<double> joints = plan.trajectory_.joint_trajectory.points.back().positions;
    for (int i = 0; i < 6; ++i)
    {
        std::cout << group.getJoints().at(i) << ": " << joints.at(i) << std::endl;
    }
    std::cout << std::endl;
    std::cout << "Time to Reach Goal State: "
              << plan.trajectory_.joint_trajectory.points.back().time_from_start 
              << " Seconds " << std::endl;
}

bool timeDistance(ur10_gripper_msgs::UR10::Request  &req,
                  ur10_gripper_msgs::UR10::Response &res,
                  moveit::planning_interface::MoveGroup &group)
{
    if(req.pose.size() < 6)
    {
        ROS_ERROR("Invalid Request Parameters:");
        ROS_ERROR("Provide X, Y, Z, R, P, Y in pose vector");
        return false;
    }
    ROS_INFO("Time and Distance Request Recieved");
    ROS_INFO_STREAM("X: " << req.pose.at(0) << " Y: " << req.pose.at(1) 
                << " Z: " << req.pose.at(2) << " R: " << req.pose.at(3)
                << " P: " << req.pose.at(4) << " Y: " << req.pose.at(5));
    
    group.setPlanningTime(10.0);
    //group.setGoalPositionTolerance(0.1);
    geometry_msgs::Pose goal;

    goal.position.x = req.pose[0];
    goal.position.y = req.pose[1];
    goal.position.z = req.pose[2] - 1.43;
    setRPYGoal(goal, req.pose[3], req.pose[4], req.pose[5]);

    group.setPoseTarget(goal);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    ROS_INFO_STREAM("Planning motion");
    res.success = group.plan(my_plan);

    plans.push_back(my_plan);

    printStart(group);
    printGoal(goal, group, my_plan);

    std::cout << "Plan stored as plan #" << plans.size()-1 << std::endl;

    sleep(1.0);

    return true;
}


bool execute(ur10_gripper_msgs::UR10::Request  &req,
             ur10_gripper_msgs::UR10::Response &res,
             moveit::planning_interface::MoveGroup &group)
{
    if(req.pose.size() < 1)
    {
        ROS_ERROR("Invalid Request Parameters:");
        ROS_ERROR("Provide plan number in pose vector");
        return false;
    }
    ROS_INFO("Execution Request Recieved");
    int plan = (int)req.pose.at(0);
    if(plan > plans.size()-1)
    {
        ROS_ERROR("Invalid plan number, no plan with that number stored");
        return false;
    }
    ROS_INFO_STREAM("Executing Plan " << plan);
    group.execute(plans.at(plan));
    
    sleep(5.0); 
}



bool action(ur10_gripper_msgs::UR10::Request  &req,
            ur10_gripper_msgs::UR10::Response &res)
{
    moveit::planning_interface::MoveGroup group("Arm");
    switch(req.request)
    {
        case MOVETO:
            moveTo(req, res, group);
            break;
        case PUSH:
            push(req, res, group);
            break;
        case TIMEDISTANCE:
            timeDistance(req, res, group);
            break;
        case EXECUTE:
            execute(req, res, group);
            break;
        default:
            ROS_ERROR("Invalid Request Recieved");
            break;
    }
    std::cout << std::endl;
    ROS_INFO("Ready to recieve commands");
    return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur10_server");
  	ros::NodeHandle n;
    
    moveit::planning_interface::MoveGroup group("Arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //Checks if the frame has been loaded to prevent collision
    if(planning_scene_interface.getKnownObjectNames().size() < 6)
    {
        ROS_ERROR("Collision Objects not loaded!");
        ros::shutdown();
        return 1;
    }
    std::cout << std::endl;
    ROS_INFO("Ready to recieve commands");

    ros::ServiceServer service = n.advertiseService("UR10", action);
    ros::AsyncSpinner spinner(1);
    spinner.start();
  	
    ros::Rate r(10);

    while (ros::ok()) 
    {
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();
  	return 0;
}
