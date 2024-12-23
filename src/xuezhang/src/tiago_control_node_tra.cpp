#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <world_percept_assig3/GetSceneObjectList.h>
#include <world_percept_assig3/GotoObject.h>

//#include <world_percept_assig3/LoadKnowledge.h>

using namespace std;

class TiagoControlNode
{
private:
    //subscriber
    ros::Subscriber sub_gazebo_data_;     ///< Subscriber gazebo model_states
    std::string subs_topic_name_;         ///< Subscriber gazebo model_states topic name

    //publisher
    ros::Timer tf_timer_;             ///< Timer to run a parallel process
    ros::Publisher vel_pub;           ///pusblisher of velocity

    //client
    std::string srv_get_scene_name_; ///< get scene object list topic name
    ros::ServiceClient client_scene_list_; ///< Client to request the object list in the map generator node

    //service
    std::string srv_goto_obj_name_; /// srv name of goto object
    ros::ServiceServer goto_obj_srv_; /// service of goto object

    //General info
    geometry_msgs::Pose tiago_pose_;
    geometry_msgs::Pose obj_pose_;
    std::string obj_name_ = "";

public:

    TiagoControlNode(ros::NodeHandle &nh)
    {
        ROS_WARN_STREAM("Created tiago control");

        //subscriber
        subs_topic_name_ = "/gazebo/model_states";
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &TiagoControlNode::topic_callback, this);

        //publisher
        tf_timer_ = nh.createTimer(ros::Duration(1), &TiagoControlNode::tf_timer_callback, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/key_vel", 1000);

        //client
        srv_get_scene_name_ = "get_scene_object_list";
        client_scene_list_ = nh.serviceClient<world_percept_assig3::GetSceneObjectList>(srv_get_scene_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_get_scene_name_.c_str());
        bool service_found = ros::service::waitForService(srv_get_scene_name_, ros::Duration(30.0)); // You can adjust the timeout as needed
        if (!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_get_scene_name_.c_str());
            exit;
        }
        ROS_INFO_STREAM("Connected to service: " << srv_get_scene_name_);

        //server
        srv_goto_obj_name_ = "goto_object";
        goto_obj_srv_ = nh.advertiseService(srv_goto_obj_name_, &TiagoControlNode::srv_goto_obj_callback, this);
    };

    ~TiagoControlNode(){

    };

private:
    /**
     * @brief Callback function to receive the Gazebo Model State topic
     *
     * @param msg message with the current Gazebo model state
     */
    void topic_callback(const gazebo_msgs::ModelStates::ConstPtr &msg){
        // DONE - update tiago pos
        auto it = std::find(msg->name.begin(), msg->name.end(), "tiago");
        if (it != msg->name.end())
        {
            // Calculate the index
            int index = std::distance(msg->name.begin(), it);
            tiago_pose_ = msg->pose.at(index);
        }
    }

    void tf_timer_callback(const ros::TimerEvent &e){
        // DONE - get values and calculate twist of tiago
        // target_w??  tiago_w??
        // check TWIST include
        if(obj_name_ == ""){
            return;
        }
        geometry_msgs::Twist tiago_twist_cmd;
        Eigen::Matrix2d Rtiago_w = q2Rot2D(tiago_pose_.orientation);

        Eigen::Matrix2d Rw_tiago = Rtiago_w.inverse();

        Eigen::Vector2d tiago_w;
        tiago_w << tiago_pose_.position.x, tiago_pose_.position.y;
        Eigen::Vector2d target_w;
        target_w << obj_pose_.position.x, obj_pose_.position.y;
        Eigen::Vector2d Dpose_w = target_w - tiago_w;

        Eigen::Vector2d Dpose_tiago = Rw_tiago * Dpose_w;
        double d = (Dpose_tiago.norm() <= 1.3) ? 0.0 : Dpose_tiago.norm();

        double theta = std::atan2(Dpose_tiago(1), Dpose_tiago(0));
        double Kwz = 1.1, Kvx = 0.9;
        tiago_twist_cmd.linear.x = Kvx * d;
        tiago_twist_cmd.angular.z = Kwz * theta;
        vel_pub.publish(tiago_twist_cmd);

        ROS_INFO_STREAM("Send command " << tiago_twist_cmd);
    }

    void get_object_info(const std::string obj){
        //DONE request object info

        world_percept_assig3::GetSceneObjectList srv;
        srv.request.object_name = obj;

        if (client_scene_list_.call(srv)){
            ROS_INFO_STREAM("Object list " << (int)srv.response.obj_found);

            if (srv.response.obj_found) {
                ROS_INFO_STREAM("Object found: " <<srv.response.objects);
                obj_name_ = srv.response.objects.name[0];
                obj_pose_ = srv.response.objects.pose[0];
            }
            else {
                ROS_INFO_STREAM("Internal error object found" << srv.response.message);
            }
        }
        else{
            ROS_ERROR_STREAM("Failed to call service " << srv_get_scene_name_);
        }
    }

    bool srv_goto_obj_callback(world_percept_assig3::GotoObject::Request &req,
                               world_percept_assig3::GotoObject::Response &res){
        //DONE - Implement logic to move to obj
        get_object_info(req.obj);
        res.confirmation = true;
                                
        return res.confirmation;
    }

    Eigen::Matrix2d q2Rot2D(const geometry_msgs::Quaternion &quaternion){
        Eigen::Quaterniond eigenQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        Eigen::Matrix2d rotationMatrix = eigenQuaternion.toRotationMatrix().block(0, 0, 2, 2);
        return rotationMatrix;
    }

}; //class KnowledgeNode

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tiago_control_node");

  ros::NodeHandle nh;   
  
  TiagoControlNode myTiagoControlNode(nh);

  ros::spin();

  
  return 0;
}