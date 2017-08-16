#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <geometry_msgs/Point.h>
#include "ur10_gripper_msgs/UR10.h"
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <std_msgs/Float32.h>

ros::Publisher pub_dir;

double randF(double min, double max)
{
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
}

double semiRandomDirection(double x, double y, int ball = 0)
{
    int noL = 0;
    int noR = 0;
    int noU = 0;
    int noD = 0;
    double dir;
    if (x < 0.0)
    {
        //noR = 1;
    }
    else
    {
        //noL = 1;
    }

    if(noR == 1)
    {
        if(noU == 1)
        {
            //180 to 270
            dir = randF(3.1415, 4.7123);
        }
        else if(noD == 1)
        {
            //90 to 18
            dir = randF(1.5708, 3.1415);
        }
        else
        {
            //180 to 270
            dir = randF(1.5708, 4.7123);
        }
    }
    else if(noL == 1)
    {
        if(noU == 1)
        {
            //270 to 360
            dir = randF(4.7123, 6.283);
        }
        else if(noD == 1)
        {
            //0 to 90
            dir = randF(0.0, 1.5708);
        }
        else
        {
            //270 to 90
            dir = randF(0, 3.1415);
            if(dir > 1.5708)
            {
               dir += 3.1415; 
            }
        }
    }
    else
    {
        if(noU == 1)
        {
            //180 to 360
            dir = randF(3.1415, 6.283);
        }
        else if(noD == 1)
        {
            //0 to 180
            dir = randF(0.0, 3.1415);
        }
        else
        {
            //0 to 360
            dir = randF(0.0, 6.283);
        } 
    }
    std::cout << "Direction     Right   Left    Up    Down" << std::endl;
    std::cout << dir << " " << noR << " " << noL << " " << noU << " " << noD << std::endl;
    return dir;
}

void pushDuck(ros::NodeHandle n)
{
    geometry_msgs::Point::ConstPtr bd = ros::topic::waitForMessage<geometry_msgs::Point>("/blue_duck", n, ros::Duration(10));
    ros::ServiceClient client = n.serviceClient<ur10_gripper_msgs::UR10>("UR10");
    ur10_gripper_msgs::UR10 srv;
    srv.request.request = 1;
    srv.request.plan = true;
    srv.request.pose.push_back(bd->x);
    srv.request.pose.push_back(bd->y-0.07);
    srv.request.pose.push_back(bd->z+0.015);
    std_msgs::Float32 dir;
    dir.data = semiRandomDirection(bd->x, bd->y-0.07);
    srv.request.pose.push_back(dir.data);
    srv.request.pose.push_back(0.1);
    pub_dir.publish(dir);
    if(client.call(srv))
    {
        if(srv.response.success)
        {
            ROS_INFO("Pushed Object");
            pub_dir.publish(dir);
            return;
        }
        ROS_ERROR("Could not push object");
        return;
    }
    ROS_ERROR("Could not issue command to arm");
}

void pushBall(ros::NodeHandle n)
{
    geometry_msgs::Point::ConstPtr bd = ros::topic::waitForMessage<geometry_msgs::Point>("/red_ball", n, ros::Duration(10));
    ros::ServiceClient client = n.serviceClient<ur10_gripper_msgs::UR10>("UR10");
    ur10_gripper_msgs::UR10 srv;
    srv.request.request = 1;
    srv.request.plan = true;
    srv.request.pose.push_back(bd->x-0.01);
    srv.request.pose.push_back(bd->y-0.09);
    srv.request.pose.push_back(0.03);
    std_msgs::Float32 dir;
    dir.data = semiRandomDirection(bd->x, bd->y-0.05, 1);
    srv.request.pose.push_back(dir.data);
    srv.request.pose.push_back(0.075);
    pub_dir.publish(dir);
    if(client.call(srv))
    {
        if(srv.response.success)
        {
            ROS_INFO("Pushed Object");
            pub_dir.publish(dir);
            return;
        }
        ROS_ERROR("Could not push object");
        return;
    }
    ROS_ERROR("Could not issue command to arm");
}

void pushLego(ros::NodeHandle n)
{
    geometry_msgs::Point::ConstPtr bd = ros::topic::waitForMessage<geometry_msgs::Point>("/yellow_lego", n, ros::Duration(10));
    ros::ServiceClient client = n.serviceClient<ur10_gripper_msgs::UR10>("UR10");
    ur10_gripper_msgs::UR10 srv;
    srv.request.request = 1;
    srv.request.plan = true;
    srv.request.pose.push_back(bd->x-0.01);
    srv.request.pose.push_back(bd->y-0.09);
    srv.request.pose.push_back(0.04);
    std_msgs::Float32 dir;
    dir.data = semiRandomDirection(bd->x, bd->y-0.05, 1);
    srv.request.pose.push_back(dir.data);
    srv.request.pose.push_back(0.10);
    pub_dir.publish(dir);
    if(client.call(srv))
    {
        if(srv.response.success)
        {
            ROS_INFO("Pushed Object");
            pub_dir.publish(dir);
            return;
        }
        ROS_ERROR("Could not push object");
        return;
    }
    ROS_ERROR("Could not issue command to arm");
}

void pushDuckOnLego(ros::NodeHandle n)
{
    geometry_msgs::Point::ConstPtr bd = ros::topic::waitForMessage<geometry_msgs::Point>("/blue_duck", n, ros::Duration(10));
    ros::ServiceClient client = n.serviceClient<ur10_gripper_msgs::UR10>("UR10");
    ur10_gripper_msgs::UR10 srv;
    srv.request.request = 1;
    srv.request.plan = true;
    srv.request.pose.push_back(bd->x-0.01);
    srv.request.pose.push_back(bd->y-0.05);
    srv.request.pose.push_back(0.135);
    std_msgs::Float32 dir;
    dir.data = semiRandomDirection(bd->x, bd->y-0.05, 1);
    srv.request.pose.push_back(dir.data);
    srv.request.pose.push_back(0.10);
    pub_dir.publish(dir);
    if(client.call(srv))
    {
        if(srv.response.success)
        {
            ROS_INFO("Pushed Object");
            pub_dir.publish(dir);
            return;
        }
        ROS_ERROR("Could not push object");
        return;
    }
    ROS_ERROR("Could not issue command to arm");
}

void pushBallOnLego(ros::NodeHandle n)
{
    geometry_msgs::Point::ConstPtr bd = ros::topic::waitForMessage<geometry_msgs::Point>("/red_ball", n, ros::Duration(10));
    ros::ServiceClient client = n.serviceClient<ur10_gripper_msgs::UR10>("UR10");
    ur10_gripper_msgs::UR10 srv;
    srv.request.request = 1;
    srv.request.plan = true;
    srv.request.pose.push_back(bd->x-0.01);
    srv.request.pose.push_back(bd->y-0.05);
    srv.request.pose.push_back(0.13);
    std_msgs::Float32 dir;
    dir.data = semiRandomDirection(bd->x, bd->y-0.05, 1);
    srv.request.pose.push_back(dir.data);
    srv.request.pose.push_back(0.10);
    pub_dir.publish(dir);
    if(client.call(srv))
    {
        if(srv.response.success)
        {
            ROS_INFO("Pushed Object");
            pub_dir.publish(dir);
            return;
        }
        ROS_ERROR("Could not push object");
        return;
    }
    ROS_ERROR("Could not issue command to arm");
}



int main(int argc, char** argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "ur10_trials");
    ros::NodeHandle n;
    
    if(argc != 2)
    {
        ROS_ERROR("Invalid arguments, pass trial type 1, 2, 3 etc as single argument");
    }
    pub_dir = n.advertise<std_msgs::Float32>("push_direction", 1);
    int trial = atoi(argv[1]);
    switch(trial)
    {
        case 1:
            pushDuck(n);
            break;
        case 2: 
            pushBall(n);
            break;
        case 3:
            pushLego(n);
            break;
        case 4:
            pushDuckOnLego(n);
            break;
        case 5:
            pushBallOnLego(n);
            break;
        default:
            break;
    }
    //ros::spin();

}
