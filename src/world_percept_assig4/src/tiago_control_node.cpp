#include "ros/ros.h"
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <vector>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <gazebo_msgs/ModelStates.h>
#include <world_percept_assig4/GetSceneObjectList.h>
#include <world_percept_assig4/GotoObject.h>

using namespace std;

struct TrajectoryPoint {
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    Eigen::Vector2d acceleration;
    double time;
};

std::pair<std::vector<double>, std::vector<double>> generateTrajectory2D(const TrajectoryPoint& start, const TrajectoryPoint& end) {
    Eigen::MatrixXd A(6, 6);
    Eigen::VectorXd bx(6);
    Eigen::VectorXd by(6);
    
    // Populate the system of equations for a 5th-order polynomial
    A << 1, start.time, pow(start.time, 2), pow(start.time, 3), pow(start.time, 4), pow(start.time, 5),
         1, end.time, pow(end.time, 2), pow(end.time, 3), pow(end.time, 4), pow(end.time, 5),
         0, 1, 2 * start.time, 3 * pow(start.time, 2), 4 * pow(start.time, 3), 5 * pow(start.time, 4),
         0, 1, 2 * end.time, 3 * pow(end.time, 2), 4 * pow(end.time, 3), 5 * pow(end.time, 4),
         0, 0, 2, 6 * start.time, 12 * pow(start.time, 2), 20 * pow(start.time, 3),
         0, 0, 2, 6 * end.time, 12 * pow(end.time, 2), 20 * pow(end.time, 3);
    
    bx << start.position.x(), end.position.x(), start.velocity.x(), 
         end.velocity.x(), start.acceleration.x(), end.acceleration.x();

    by << start.position.y(), end.position.y(), start.velocity.y(), 
         end.velocity.y(), start.acceleration.y(), end.acceleration.y();

    // Solve for coefficients
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(by);
    return {
        std::vector<double>(coeffs_x.data(), coeffs_x.data() + coeffs_x.size()),
        std::vector<double>(coeffs_y.data(), coeffs_y.data() + coeffs_y.size())
    };
}

class Tiagocontrol
{
private:

    // Subscriber to the topic /gazebo/model_states
    std::string subs_topic_name_; 
    ros::Subscriber sub_gazebo_data_;

    // Publisher that will send the twist information of the robot
    ros::Publisher velocity_publisher_;

    // Client that will be inside the callback function of the subscriber
    std::string srv_get_scene_name_; 
    ros::ServiceClient client_object_list_;

    // Service that will calculate the linear and angular velocity
    std::string srv_calculate_velocity_name_;
    ros::ServiceServer calculate_velocity_srv_;   

    // Tiago pose and target object data
    geometry_msgs::Pose tiago_pose_; 
    geometry_msgs::Pose obj_pose_; 
    std::string obj_name_; 

public:

    Tiagocontrol(ros::NodeHandle &nh)
    {
        // Create Subscriber
        subs_topic_name_ = "/gazebo/model_states";
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &Tiagocontrol::subscriber_callback, this);

        // Create Publisher
        velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("/key_vel", 100);

        // Create client
        srv_get_scene_name_ = "get_scene_object_list";
        client_object_list_ = nh.serviceClient<world_percept_assig4::GetSceneObjectList>(srv_get_scene_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_get_scene_name_.c_str());
        bool service_found = ros::service::waitForService(srv_get_scene_name_, ros::Duration(300.0)); // You can adjust the timeout as needed

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_get_scene_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: "<<srv_get_scene_name_);

        // Create service
        srv_calculate_velocity_name_ = "goto_object";
        calculate_velocity_srv_ = nh.advertiseService(srv_calculate_velocity_name_, &Tiagocontrol::service_callback, this);
    };

    ~Tiagocontrol(){

    };


private:

    bool service_callback(world_percept_assig4::GotoObject::Request &req,
                          world_percept_assig4::GotoObject::Response &res)
    {
        
        world_percept_assig4::GetSceneObjectList srv;
        srv.request.object_name = req.obj;

        if (client_object_list_.call(srv)) {
            if (srv.response.obj_found) {
                obj_name_ = srv.response.objects.name[0];
                obj_pose_ = srv.response.objects.pose[0];
                ROS_INFO_STREAM("Target object set: " << obj_name_);
                res.confirmation = true;
            } else {
                ROS_WARN_STREAM("Object not found: " << req.obj);
                res.confirmation = false;
            }
        } else {
            ROS_ERROR_STREAM("Failed to call service " << srv_get_scene_name_);
            res.confirmation = false;
        }
        return res.confirmation;

    }

    void subscriber_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        // Find tiago in the list of models
        auto it = std::find(msg->name.begin(), msg->name.end(), "tiago");
        if (it != msg->name.end()) {
            int index = std::distance(msg->name.begin(), it);
            tiago_pose_ = msg->pose.at(index);
        }

        // If no target is set, do nothing
        if (obj_name_.empty()) {
            return;
        }

        geometry_msgs::Twist tiago_twist_cmd;
        Eigen::Matrix2d Rtiago_w = q2Rot2D(tiago_pose_.orientation);
        Eigen::Matrix2d Rw_tiago = Rtiago_w.inverse();

        //Eigen::Vector2d tiago_w(tiago_pose_.position.x, tiago_pose_.position.y);
        //Eigen::Vector2d target_w(obj_pose_.position.x, obj_pose_.position.y);
        Eigen::Vector2d tiago_w;
        tiago_w << tiago_pose_.position.x, tiago_pose_.position.y;
        Eigen::Vector2d target_w;
        target_w << obj_pose_.position.x, obj_pose_.position.y;

        Eigen::Vector2d Dpose_w = target_w - tiago_w;

        Eigen::Vector2d Dpose_tiago = Rw_tiago * Dpose_w;
        double d = (Dpose_tiago.norm() <= 1.3)? 0.0: Dpose_tiago.norm();
        double theta = std:: atan2(Dpose_tiago(1), Dpose_tiago(0));
        double Kwz = 1.1;
        double Kvx = 0.1;

        // Set velocity
        tiago_twist_cmd.linear.x = Kvx * d;
        tiago_twist_cmd.angular.z = Kwz * theta;

        velocity_publisher_.publish(tiago_twist_cmd);
        ROS_INFO_STREAM("Publishing velocity command: " << tiago_twist_cmd);

        // Define trajectory points for example
        TrajectoryPoint start = {{tiago_w[0], tiago_w[1]}, {0.0, 0.0}, {0.0, 0.0}, 0.0};
        TrajectoryPoint end = {{target_w[0], target_w[1]}, {0.0, 0.0}, {0.0, 0.0}, 2.0};

        // Generate trajectory
        auto coeffs = generateTrajectory2D(start, end);
        ROS_INFO_STREAM("Generated Trajectory Coefficients:");
        // Iterate over the x-coefficients
        ROS_INFO_STREAM("X Coefficients:");
        for (size_t i = 0; i < coeffs.first.size(); ++i) {
            ROS_INFO_STREAM("Coeff_x " << i << ": " << coeffs.first[i]);
        }

        // Iterate over the y-coefficients
        ROS_INFO_STREAM("Y Coefficients:");
        for (size_t i = 0; i < coeffs.second.size(); ++i) {
            ROS_INFO_STREAM("Coeff_y " << i << ": " << coeffs.second[i]);
        }


    }

    Eigen::Matrix2d q2Rot2D(const geometry_msgs::Quaternion &quaternion)
    {
        Eigen::Quaterniond eigenQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        Eigen::Matrix2d rotationMatrix = eigenQuaternion.toRotationMatrix().block(0,0,2,2);
        return rotationMatrix;
    }

}; 


int main(int argc, char **argv)
{

    ros::init(argc, argv, "tiago_control_node");

    ros::NodeHandle nh;

    Tiagocontrol myTiagocontrol(nh);

    ros::spin();

    return 0;
}