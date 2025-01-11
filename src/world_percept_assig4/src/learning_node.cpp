#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <world_percept_assig4/UpdateObjectList.h>
#include <world_percept_assig4/SetInitTiagoPose.h>
//add the new libraries for the new service
#include <world_percept_assig4/GetSceneObjectList.h>
#include <gazebo_msgs/ModelStates.h>
#include <world_percept_assig4/DecideObject.h>
#include <boost/python.hpp>

class LearningNode 
{
private:

    std::string srv_update_obj_name_;        ///< service name for new obj
    ros::ServiceServer update_obj_list_srv_; // Advertise service to update obj list
    std::vector<std::pair<std::string, geometry_msgs::Pose>> map_objs_; ///< List of objects in the scene

    ros::Timer tf_timer_;                 ///< Timer to run a parallel process
    tf::TransformBroadcaster broadcaster_; ///< TF broadcaster variable

    ros::ServiceServer get_scene_obj_srv_; // Advertise service to send the pose of a target object in the scene
    std::string srv_get_scene_name_;       ///< service name to send a target object in the scene

    std::map<std::string, int> name2id; ///< Map object names to index in the object list
    std::map<int, std::string> id2name; ///< Map index in the object list to object names

    std::string srv_assert_knowledge_name_; 
    std::vector<std::string> v_seen_obj_resoning_;  
    ros::ServiceClient client_reasoning_; 

    ros::ServiceServer decide_object_srv_; // Service for deciding which object to choose
    std::string srv_decide_object_name_;  ///< Name of the decision-making service


public:

    LearningNode(ros::NodeHandle& nh) {
        
        ROS_WARN_STREAM("Created world map");

        srv_update_obj_name_="update_object_list";

        update_obj_list_srv_ = nh.advertiseService(srv_update_obj_name_, &LearningNode::srv_update_callback, this);

        // publish the current position
        tf_timer_ = nh.createTimer(ros::Duration(1), &LearningNode::tf_timer_callback, this);
    
        //Advertising the new service
        srv_get_scene_name_ = "get_scene_object_list";
        get_scene_obj_srv_ = nh.advertiseService(srv_get_scene_name_, &LearningNode::srv_get_scene_obj_callback, this);

        v_seen_obj_resoning_.push_back("tiago");
        v_seen_obj_resoning_.push_back("ground_plane");
        srv_assert_knowledge_name_ = "assert_knowledge";

        client_reasoning_ = nh.serviceClient<world_percept_assig4::UpdateObjectList>(srv_assert_knowledge_name_);
        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_assert_knowledge_name_.c_str());
        bool service_found2 = ros::service::waitForService(srv_assert_knowledge_name_, ros::Duration(30.0)); // You can adjust the timeout as needed
        if(!service_found2)
        {
            ROS_ERROR("Failed to call service %s", srv_assert_knowledge_name_.c_str());
            exit;
        }
        ROS_INFO_STREAM("Connected to service: "<<srv_assert_knowledge_name_);
        
        srv_decide_object_name_ = "decide_object";
        decide_object_srv_ = nh.advertiseService(srv_decide_object_name_, &LearningNode::srv_decide_object_callback, this);
        ROS_INFO("Decision-making service [%s] initialized.", srv_decide_object_name_.c_str());
    }

    ~LearningNode() 
    {

    };

private:
    
/**
   * @brief Callback function for the service that adds objects to the map_objects list
   *
   * @param Request requested object to be added to the list
   * @param Respose response from the service when the object has been added (true/false)
*/
bool srv_update_callback(world_percept_assig4::UpdateObjectList::Request  &req,
         world_percept_assig4::UpdateObjectList::Response &res)
{
    ROS_INFO_STREAM("Got new object: "<<req.object_name);
    ROS_INFO_STREAM("Object Pose: "<<req.object_pose);

    //Push the information that is obtained from the client via the request variables
    //TODO: Use the correct variables here 
    map_objs_.push_back(std::make_pair(req.object_name, req.object_pose)); 
    name2id[req.object_name] = map_objs_.size()-1;

    for (size_t i = 0; i < map_objs_.size(); i++)
    {
        ROS_INFO_STREAM(map_objs_.at(i).first<<": "<<map_objs_.at(i).second);
    }

    res.confirmation = true;
    return res.confirmation;
}

/**
     * @brief Callback function for the service that sends the info of a target object in the scene. This service could be use, for example, by the controller node to find the target position
     *
     * @param Request name of the requested object. Use "all" to get the list of all the available objects.
     * @param Respose response is a list of objects with name and pose.
     */
    bool srv_get_scene_obj_callback(world_percept_assig4::GetSceneObjectList::Request &req,
                                    world_percept_assig4::GetSceneObjectList::Response &res)
    {
        ROS_INFO_STREAM("Target object: " << req.object_name);

        // If the requested object name is "all", we should send all the available objects

        if (req.object_name == "all")
        {
            //TODO A04.T01: copy all the pairs (names,poses) into the object list (0.5 pts)
            for (auto &&pairs : map_objs_)
            {
               res.objects.name.push_back(pairs.first);
               res.objects.pose.push_back(pairs.second);
            }

            res.obj_found = true;
            return res.obj_found;
        }

        // Verify if the target name exists in the list
        auto it = name2id.find(req.object_name);

        if (it != name2id.end())
        {
            // The target object name is in the list
            // Get the index in the pair list
            int target_idx = name2id[req.object_name];

            //TODO: A04.T01: Push the information that is obtained about the name and pose of the object found in the list in the response variable "res" (0.5pts)
            res.objects.name.push_back(map_objs_[target_idx].first);
            res.objects.pose.push_back(map_objs_[target_idx].second);

            res.obj_found = true;
        }
        else
        {
            // target object is not in the list
            res.obj_found = false;

            std::stringstream s;

            s<<"The target object [" << req.object_name << "] is not in the list. \nAvailable Objects";

            // copy all the pairs (names,poses) into the object list
            for (auto &&pairs : map_objs_)
            {
                s<<pairs.first<<"\n";
            }

            res.message = s.str();

            ROS_ERROR_STREAM("The target object [" << req.object_name << "] is not in the list");
        }

        return res.obj_found;
    }

    bool srv_decide_object_callback(world_percept_assig4::DecideObject::Request &req,
                                       world_percept_assig4::DecideObject::Response &res)
    {
        ROS_INFO("Received decision request with conditions:");
        ROS_INFO("Taste Preference: %s", req.taste_preference.c_str());
        ROS_INFO("Prefers Alcohol: %s", req.prefers_alcohol ? "Yes" : "No");
        ROS_INFO("Budget: %s", req.budget.c_str());
        ROS_INFO("Gender: %s", req.gender.c_str());
        ROS_INFO("Age: %d", req.age);

        // 构造命令行字符串，调用 Python 脚本并传递参数
        std::string command = "python3 /home/user/exchange/SSY236_group8/src/world_percept_assig4/src/decision_tree_predict.py '" + req.taste_preference + "' " + 
                              std::to_string(req.prefers_alcohol) + " '" + req.budget + "' '" + 
                              req.gender + "' " + std::to_string(req.age);
        
        // 获取 Python 脚本输出
        std::string result = exec(command.c_str());

        res.selected_object = result;
        res.success = true;
        ROS_INFO("Decision made: %s", res.selected_object.c_str());

        return true;
    }

    // Function to execute a system command and return the result as a string
    std::string exec(const char* cmd) {
        char buffer[128];
        std::string result = "";
        ROS_INFO_STREAM("Executing command: " << cmd);  // 打印命令以调试
        FILE* fp = popen(cmd, "r");
        if (fp == nullptr) {
            ROS_ERROR("Failed to run command");
            return "";
        }
        while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
            result += buffer;
        }
        fclose(fp);
        ROS_INFO_STREAM("Python script output: " << result);  // 打印输出以调试
        return result;
    }



void tf_timer_callback(const ros::TimerEvent& e)
{
    std::vector<geometry_msgs::TransformStamped> v_ts;

    // Get the current time
    ros::Time aux_time = ros::Time::now();
    
    for (size_t i = 0; i < map_objs_.size(); i++)
    {
        geometry_msgs::TransformStamped ts;
        
        std::string object_name = map_objs_.at(i).first ; //TODO: use the correct variable 
        geometry_msgs::Pose obj_pose = map_objs_.at(i).second; //TODO: use the correct variable 

        // TF object to populate our TF message
        tf2::Transform tf;

        //TODO: use the correct variable to define the right object position (0.5 pts)
        tf.setOrigin(tf2::Vector3(obj_pose.position.x,obj_pose.position.y,obj_pose.position.z)); 

        //TODO: use the correct variable to define the right object orientation (0.5 pts)
        tf.setRotation(tf2::Quaternion(obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z,obj_pose.orientation.w));

        // Transform the TF object to TF message
        ts.transform = tf2::toMsg(tf);

        // Set the reference frame for the TF (parent link)
        //TODO: Define the right reference frame 
        ts.header.frame_id = "world";
        // Set the time stamp for the message
        ts.header.stamp = aux_time;
        // Set the name for the TF
        ts.child_frame_id = object_name;

         //// To visualize objects in Rviz we need to publish its corresponding TF
        // Create TF msg
        v_ts.push_back(ts);
    }

    //Once all the information of the TFs is obtained, then we broadcast this data to Rviz
    broadcaster_.sendTransform(v_ts);

}

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "learning_node");
    ros::NodeHandle nh;

    LearningNode myLearningNode(nh);

    ROS_INFO("LearningNode is running...");
    ros::spin();

    return 0;
}
