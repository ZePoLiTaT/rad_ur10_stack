#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/topic.h>
#include <geometry_msgs/Point.h>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <std_msgs/Float32.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char** argv)
{
    double x0, y0, x1, y1, dir;
    dir = 0;
    for(int i = 1; i < argc; ++i)
    {
        int first = 1;
        rosbag::Bag bag;
        bag.open(argv[i], rosbag::bagmode::Read);
        
        std::vector<std::string> topics;
        topics.push_back("/blue_duck");
        topics.push_back("/push_direction");
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        
        foreach(rosbag::MessageInstance const m, view)
        {
            if (m.getTopic() == "/blue_duck")
            {
                geometry_msgs::Point::ConstPtr position = m.instantiate<geometry_msgs::Point>();
                if(position != NULL)
                {
                    if(first == 1)
                    {
                        first = 0;
                        x0 = position -> x;
                        y0 = position -> y;
                    }
                    x1 = position -> x;
                    y1 = position -> y;
                }
            }
            else if (m.getTopic() == "/push_direction")
            {
                std_msgs::Float32::ConstPtr direction = m.instantiate<std_msgs::Float32>();
                if(direction != NULL)
                {
                    dir = direction->data;
                }
            }
        }
        bag.close();
        std::cout << argv[i] << " " << x0 << ", " << y0 << ", " << x1 << ", " << y1 << ", " << dir << std::endl;
    }
}
