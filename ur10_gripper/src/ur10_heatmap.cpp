#include "ros/ros.h"
#include "ur10_gripper_msgs/UR10.h"
#include <cstdlib>
#include <vector>
#include <cmath>

int heatmap[41][41][4] = {0};

void print()
{
    for(int k = 0; k < 4; ++k)
    {
        std::cout << "Layer " << k+1 << " : z = " << 0.01 * (k + 3) << std::endl;
        for(int i = 0; i < 40; ++i)
        {
            for(int j = 0; j < 40; ++j)
            {
                std::cout << heatmap[i][j][k] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

void printLayer(int k)
{
    std::cout << "Layer " << k << " : z = " << 0.01 * (k+2) << std::endl;
    for(int i = 0; i < 40; ++i)
    {
        for(int j = 0; j < 40; ++j)
        {
            std::cout << heatmap[i][j][k-1] << " ";
        }
        std::cout << std::endl;
     }
    std::cout << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur10_heatmap");
    ros::NodeHandle n;

    int X = -20;
    int Y = -20;
    int Z = 1;
    if(argc == 4)
    {
        X = atoi(argv[1]);
        Y = atoi(argv[2]);
        Z = atoi(argv[3]);
    }
    std::cout << "Starting at X = " << X << " Y = " << Y << " Z = " << Z << std::endl;
    ros::ServiceClient client = n.serviceClient<ur10_gripper_msgs::UR10>("UR10");
    ur10_gripper_msgs::UR10 srv;
    for(int k = Z; k < 5; k++)
    {
        for(int i = X; i <= 20; i++)
        {
            for(int j = Y; j <= 20; j++)
            {
                int count = 0;
                double x = (double) i / 20.0;
                double y = (double) j / 20.0;
                double z = (double) (k+2) / 100.0;
                int initFail = 0;
                for(double m = 0; m < (2 * M_PI); m += (M_PI/16.0))
                {
                    if(initFail >= 3)
                    {
                        break;
                    }
                    std::vector<float> pose;
                    pose.push_back(x-0.01);
                    pose.push_back(y-0.09);
                    pose.push_back(z);
                    pose.push_back(m);
                    pose.push_back(0.1);

                    srv.request.request = 1;
                    srv.request.pose = pose;
                    srv.request.plan = true;
                    if (client.call(srv))
                    {
                        if(srv.response.success)
                        {
                            count++;
                        }
                        else if(srv.response.code == 1)
                        {
                            initFail++;
                        }
                    }
                }

                heatmap[i+20][j+20][k-1] = count;
                std::cout << x << ", " << y << ", " << z << " has heat count of " << count << std::endl;
            }
            printLayer(k);
        }
    }
    print();
}

