#include <memory>
#include <functional>
#include <filesystem>
#include <fstream>
#include <vector>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "robomaster_interface/srv/takeoff.hpp"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/twist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class DroneServiceNode : public rclcpp::Node
{
public:
    DroneServiceNode() : Node("drone_service_node"), _state(Idle)
    {        
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, 
            std::bind(&DroneServiceNode::odometry_callback, this, _1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, 
            std::bind(&DroneServiceNode::cmd_vel_callback, this, _1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel_sim", 10);
        _curret_pose = geometry_msgs::msg::Pose();
        takeoff_srv = create_service<robomaster_interface::srv::Takeoff>(
            "takeoff", 
            std::bind(&DroneServiceNode::handle_takeoff, this, _1, _2));
        land_srv = create_service<std_srvs::srv::Trigger>(
            "land", 
            std::bind(&DroneServiceNode::handle_land, this, _1, _2));
        enable_motors_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "enable_velocity_controller", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(), "Drone service started");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(60),
            std::bind(&DroneServiceNode::timer_callback, this));
    }
    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Hovering = 3, // Hover vz = 0.008
        Landing = 4,
    };

private:
    std::string selectRandomFile(const std::string& dir_path)
    {
        std::vector<std::string> bin_files;
        
        if (!std::filesystem::exists(dir_path)) {
            RCLCPP_WARN(this->get_logger(), "Directory does not exist: %s", dir_path.c_str());
            return "";
        }
        
        // Collect all .bin files
        for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
            if (entry.path().extension() == ".bin") {
                bin_files.push_back(entry.path().string());
            }
        }
        
        if (bin_files.empty()) {
            RCLCPP_WARN(this->get_logger(), "No .bin files found in: %s", dir_path.c_str());
            return "";
        }
        
        // Select random file
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, bin_files.size() - 1);
        
        return bin_files[dis(gen)];
    }
    void loadFlightData()
    {
        try {
            std::string package_dir = ament_index_cpp::get_package_share_directory("robomaster_gz"); // Replace with your package name
            std::string takeoff_file = selectRandomFile(package_dir + "/data/takeoff_data");
            std::string landing_file = selectRandomFile(package_dir + "/data/landing_data");
            

            // Load the selected files
            if (!takeoff_file.empty()) {
                takeoff_data = readBinaryFile(takeoff_file);
                RCLCPP_INFO(this->get_logger(), "Loaded takeoff file: %s (%zu rows)", 
                           std::filesystem::path(takeoff_file).filename().c_str(), takeoff_data.size());
            }
            
            if (!landing_file.empty()) {
                landing_data = readBinaryFile(landing_file);
                RCLCPP_INFO(this->get_logger(), "Loaded landing file: %s (%zu rows)", 
                           std::filesystem::path(landing_file).filename().c_str(), landing_data.size());
            }
                       
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load flight data: %s", e.what());
        }
    }
    
    std::vector<std::array<float, 3>> readBinaryFile(const std::string& filename)
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", filename.c_str());
            return {};
        }
        
        // Read shape (2 int32: rows, cols)
        int32_t rows, cols;
        file.read(reinterpret_cast<char*>(&rows), sizeof(int32_t));
        file.read(reinterpret_cast<char*>(&cols), sizeof(int32_t));
        
        if (cols != 3) {
            RCLCPP_ERROR(this->get_logger(), "Expected 3 columns, got %d in file %s", cols, filename.c_str());
            return {};
        }
        
        // Read data
        std::vector<std::array<float, 3>> trajectory(rows);
        for (int i = 0; i < rows; ++i) {
            double vx, vy, vz;
            file.read(reinterpret_cast<char*>(&vx), sizeof(double));
            file.read(reinterpret_cast<char*>(&vy), sizeof(double));
            file.read(reinterpret_cast<char*>(&vz), sizeof(double));
            
            trajectory[i] = {static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz)};
        }
        
        return trajectory;
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _curret_pose = msg->pose.pose;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = _curret_pose;

        geometry_msgs::msg::TransformStamped msgTFRigidBodies;
        msgTFRigidBodies.header = msg->header;
        msgTFRigidBodies.child_frame_id = this->get_namespace();
        msgTFRigidBodies.header.frame_id = "world";
        msgTFRigidBodies.transform.translation.x = _curret_pose.position.x;
        msgTFRigidBodies.transform.translation.y = _curret_pose.position.y;
        msgTFRigidBodies.transform.translation.z = _curret_pose.position.z;
        msgTFRigidBodies.transform.rotation.x = _curret_pose.orientation.x;
        msgTFRigidBodies.transform.rotation.y = _curret_pose.orientation.y;
        msgTFRigidBodies.transform.rotation.z = _curret_pose.orientation.z;
        msgTFRigidBodies.transform.rotation.w = _curret_pose.orientation.w;

        pose_pub_->publish(pose_stamped);
        tf_broadcaster_->sendTransform(msgTFRigidBodies);
    }

    void handle_takeoff(
        const std::shared_ptr<robomaster_interface::srv::Takeoff::Request> request,
        std::shared_ptr<robomaster_interface::srv::Takeoff::Response> response)
    {
        if (_state != Idle) {
            RCLCPP_WARN(this->get_logger(), "Drone is not in idle state, cannot take off");
            return;
        }
        loadFlightData();
        if (takeoff_data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No takeoff data available");
            return;
        }
        enable_motors.data = true; // Enable motors during takeoff
        enable_motors_pub_->publish(enable_motors);
        _state = TakingOff;
        RCLCPP_INFO(this->get_logger(), "Drone is taking off");
        response->success = true;
    }
    void handle_land(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (_state != Automatic && _state != Hovering && _curret_pose.position.z < 0.13) {
            RCLCPP_WARN(this->get_logger(), "Drone is not in automatic or hovering state, cannot land");
            return;
        }
        _state = Landing;
        RCLCPP_INFO(this->get_logger(), "Drone is landing");
        response->success = true;
        response->message = "Landing initiated";
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        _cmd_vel_usr = *msg;
        if (_state == Hovering)
        {
            _state = Automatic; // Set state to Automatic when cmd_vel is received
        }
    }
    
    void timer_callback()
    {
        //RCLCPP_INFO(this->get_logger(), "Current state: %d", static_cast<int>(_state));
        if (_state == Idle)
        {
            enable_motors.data = false; // Disable motors in Idle state
            enable_motors_pub_->publish(enable_motors);
            _cmd_vel_usr.linear.x = 0.0;
            _cmd_vel_usr.linear.y = 0.0;
            _cmd_vel_usr.linear.z = 0.0; 
            cmd_vel_pub_->publish(_cmd_vel_usr);
        }
        else if (_state == TakingOff)
        {
            if (takeoff_data.size() > 0 && _curret_pose.position.z < 0.6) {
                auto& data = takeoff_data.front();
                _cmd_vel_usr.linear.x = data[0];
                _cmd_vel_usr.linear.y = data[1];
                _cmd_vel_usr.linear.z = data[2]*0.9; // Adjust takeoff speed
                cmd_vel_pub_->publish(_cmd_vel_usr);
                takeoff_data.erase(takeoff_data.begin());
            } else {
                _state = Hovering;
                RCLCPP_INFO(this->get_logger(), "Drone has taken off and is hovering");
            }
        }
        else if (_state == Hovering)
        {
            _cmd_vel_usr.linear.x = 0.0;
            _cmd_vel_usr.linear.y = 0.0;
            _cmd_vel_usr.linear.z = 0.008; // Maintain altitude
            cmd_vel_pub_->publish(_cmd_vel_usr);
        }
        else if (_state == Landing)
        {
            if (landing_data.size() > 0) {
                auto& data = landing_data.front();
                _cmd_vel_usr.linear.x = data[0];
                _cmd_vel_usr.linear.y = data[1];
                _cmd_vel_usr.linear.z = data[2]*1.4; // Adjust landing speed
                cmd_vel_pub_->publish(_cmd_vel_usr);
                landing_data.erase(landing_data.begin());
            } 
            else if (_curret_pose.position.z > 0.15) {
                _cmd_vel_usr.linear.z = -0.17; // Descend
                cmd_vel_pub_->publish(_cmd_vel_usr);
            }
            else {
                _state = Idle;
                RCLCPP_INFO(this->get_logger(), "Drone has landed");
            }
        }
        else if (_state == Automatic)
        {
            if (_cmd_vel_usr.linear.z == 0.0) {
                _cmd_vel_usr.linear.z = 0.0072; // Maintain altitude
            }
            _cmd_vel_usr.linear.x*=-1;
            _cmd_vel_usr.linear.y*=-1;
            _cmd_vel_usr.linear.z*=0.6; // Adjust speed
            cmd_vel_pub_->publish(_cmd_vel_usr);
        }
    }

    geometry_msgs::msg::Pose _curret_pose;
    geometry_msgs::msg::Twist _cmd_vel_usr;
    rclcpp::Service<robomaster_interface::srv::Takeoff>::SharedPtr takeoff_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    State _state = Idle;
    std_msgs::msg::Bool enable_motors;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_motors_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<std::array<float, 3>> takeoff_data;  // [row_index][vx,vy,vz]
    std::vector<std::array<float, 3>> landing_data;  // [row_index][vx,vy,vz]
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}