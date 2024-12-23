#include "ros/ros.h"
#include "navigation_pkg/Coord2d.h"

#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
    // Initilize a new ros node
    ros::init(argc, argv, "navi_node");

    // Create a ros nodehandle
    ros::NodeHandle n;

    // Create a publisher, has a topic named "navi_topic", message type is navigation_pkg::Coord2d
    ros::Publisher navi_publisher = n.advertise<navigation_pkg::Coord2d>("navi_topic", 100);

    // Set the loop frequency
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        // Initialize message in the type of navigation_pkg::Coord2d
        navigation_pkg::Coord2d message;
        message.x = count++;
        message.y = 0;

        // Check the message content before publishing
        ROS_INFO_STREAM("Publishing: '"<<message.x<<", "<<message.y<<"'");

        // Publish the message
        navi_publisher.publish(message);

        // Sleep based on the set frequency
        loop_rate.sleep();
    }

    return 0;
}