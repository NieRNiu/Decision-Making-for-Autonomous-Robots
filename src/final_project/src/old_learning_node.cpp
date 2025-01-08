#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <final_project/UpdateObjectList.h>
#include <final_project/SetInitTiagoPose.h>
//add the new libraries for the new service
#include <final_project/GetSceneObjectList.h>
#include <gazebo_msgs/ModelStates.h>

class LearningNode {
private:

    // Subscribers to receive data from Percept and Reasoning nodes
    ros::Subscriber percept_sub_;
    ros::Subscriber reasoning_sub_;

    // Publishers to send data to Tiago Control Node or Reasoning Node
    ros::Publisher learning_to_control_pub_;
    ros::Publisher learning_to_reasoning_pub_;

    // Service client for potential interaction with Percept Node
    ros::ServiceClient percept_client_;

    // Buffer variables for data processing
    std::string current_object_;
    geometry_msgs::Pose object_pose_;

public:

    LearningNode(ros::NodeHandle& nh) {
        // Initialize subscribers
        percept_sub_ = nh_.subscribe("/percept_node/new_object", 10, &LearningNode::perceptCallback, this);
        reasoning_sub_ = nh_.subscribe("/reasoning_node/knowledge_update", 10, &LearningNode::reasoningCallback, this);

        // Initialize publishers
        learning_to_control_pub_ = nh_.advertise<std_msgs::String>("/learning_node/control_command", 10);
        learning_to_reasoning_pub_ = nh_.advertise<std_msgs::String>("/learning_node/assert_knowledge", 10);

        // Initialize service client for Percept Node
        percept_client_ = nh_.serviceClient<world_percept_assig4::UpdateObjectList>("update_object_list");

        ROS_INFO("Learning Node initialized and ready.");
    }

    ~LearningNode() {}

private:
    // Callback for data from the Percept Node
    void perceptCallback(const world_percept_assig4::UpdateObjectList::ConstPtr& msg) {
        ROS_INFO("Received data from Percept Node: Object [%s]", msg->object_name.c_str());

        // Process data from Percept Node
        current_object_ = msg->object_name;
        object_pose_ = msg->object_pose;

        // Forward information to Reasoning Node
        std_msgs::String knowledge_update_msg;
        knowledge_update_msg.data = "Assert: " + current_object_;
        learning_to_reasoning_pub_.publish(knowledge_update_msg);

        // Optional interaction with Tiago Control Node
        std_msgs::String control_msg;
        control_msg.data = "Navigate to object: " + current_object_;
        learning_to_control_pub_.publish(control_msg);
    }

    // Callback for data from the Reasoning Node
    void reasoningCallback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Received data from Reasoning Node: %s", msg->data.c_str());

        // Process knowledge updates and decide actions
        if (msg->data.find("Asserted") != std::string::npos) {
            ROS_INFO("Knowledge successfully updated in Reasoning Node.");
        }
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "learning_node");
    ros::NodeHandle nh;

    LearningNode myLearningNode(nh);

    ros::spin();

    return 0;
}
