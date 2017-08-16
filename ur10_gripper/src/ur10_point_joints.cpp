#include "ros/ros.h"
#include "ur10_gripper_msgs/UR10.h"
#include <cstdlib>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur10_point_joints");
    ros::NodeHandle n;
    if(argc != 5)
    {
        ROS_ERROR("Invalid arguments, pass filename followed by X, Y and Z of object position");
        return -1;
    }
    float obj_X = atof(argv[2]);
    float obj_Y = atof(argv[3]);
    float obj_Z = atof(argv[4]);
    ros::ServiceClient client = n.serviceClient<ur10_gripper_msgs::UR10>("UR10");
    ur10_gripper_msgs::UR10 srv;
    srv.request.request = 2;
    srv.request.plan = false;

    std::ifstream file;
    file.open(argv[1]);
    float x, y, z;
    while(file >> y)
    {
        y *= -1;
        file >> x;
        file >> z;
        float dX = x - obj_X;
        float dY = y - obj_Y;
        float dZ = z - obj_Z;
        float roll = 3.14;
        float yaw = atan2(dY, dX) + 3.14;
        float pitch = atan2(dZ, sqrt(dX * dX + dY * dY)) + 0.393;
        std::cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " ";
        std::vector<float> pose;
        pose.push_back(x);
        pose.push_back(y);
        pose.push_back(z);
        pose.push_back(roll);
        pose.push_back(pitch);
        pose.push_back(yaw);
        srv.request.pose = pose;
        if(client.call(srv))
        {
            if(srv.response.success)
            {
                std::cout << srv.response.joints[0] << " " << srv.response.joints[1] 
                          << " " << srv.response.joints[2] << " " << srv.response.joints[3] 
                          << " " << srv.response.joints[4] << " " << srv.response.joints[5] << std::endl;
            }
            else
            {
                std::cout <<  "X X X X X X" << std::endl; 
            }
        }
        sleep(5.0);
    }
}




