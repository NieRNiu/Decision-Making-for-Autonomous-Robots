#include "ros/ros.h"
#include "navigation_pkg/Coord2d.h"

using namespace std;

// Define callback function, will enter the function once received message from subscribed topic
void CoordCallback(const navigation_pkg::Coord2d::ConstPtr& message)
{
    // print out the received message
    ROS_INFO_STREAM("Subsribing: '"<<message->x<<", "<<message->y<<"'");
}

int main(int argc, char **argv)
{
    // Initilize a new ros node
    ros::init(argc, argv, "navi_subscriber");

    // Create a ros nodehandle
    ros::NodeHandle n;

    // Create a subscriber, has a topic named "navi_topic", has a callback function CoordCallback
    ros::Subscriber navi_sub = n.subscribe("navi_topic", 1000, CoordCallback);

    // enter a loop, waiting for callbacks
    ros::spin();

    return 0;
}