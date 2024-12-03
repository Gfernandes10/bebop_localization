#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <cstdlib> // For getenv
#include <ctime>   // For time functions
#include <iomanip> // For std::put_time
#include <boost/filesystem.hpp> // For filesystem operations
#include "../ROSUtilities/csv_logger.h"

class bebop_localization
{
public:    
    // Class members
    geometry_msgs::Quaternion quat;
    // geometry_msgs::PoseStamped pose;
    nav_msgs::Odometry odometry, filtered_pose, pose;
    ros::Publisher filtered_pose_pub_;
    ros::Subscriber odom_sub;
    ros::Subscriber natnet_sub;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd P_pred_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd x_pred_;
    Eigen::VectorXd z_;
    Eigen::VectorXd x_;
    Eigen::VectorXd x_odom_;
    std::unique_ptr<CSVLogger> csv_logger_filtered_pose_;
    std::unique_ptr<CSVLogger> csv_logger_odometry_;
    std::unique_ptr<CSVLogger> csv_logger_ground_truth_;
    std::ostringstream timestamp;
    std::string odom_topic = "bebop2/odometry_sensor1/odometry";
    std::string natnet_topic = "bebop2/ground_truth/odometry";
    std::string filtered_pose_topic = "/filtered_pose";
    

    bebop_localization(ros::NodeHandle& nh)
    {
        // Initialize Kalman filter matrices
        P_ = Eigen::MatrixXd::Zero(12, 12);
        P_pred_ = Eigen::MatrixXd::Zero(12, 12);
        F_ = Eigen::MatrixXd::Identity(12, 12);
        H_ = Eigen::MatrixXd::Zero(6, 12);
        H_(0, 0) = 1;
        H_(1, 4) = 1;
        H_(2, 8) = 1;
        H_(3, 6) = 1;
        H_(4, 2) = 1;
        H_(5, 10) = 1;
        Q_ = Eigen::MatrixXd::Identity(12, 12) * 0.2;
        R_ = Eigen::MatrixXd::Identity(6, 6);
        x_pred_ = Eigen::VectorXd::Zero(12);
        x_odom_ = Eigen::VectorXd::Zero(12);
        z_ = Eigen::VectorXd::Zero(6);
        // Initialize loggers
        std::vector<std::string> header = std::vector<std::string>{"timestamp", 
                                                                    "topic", 
                                                                    "x", 
                                                                    "dx", 
                                                                    "theta", 
                                                                    "dtheta", 
                                                                    "y", 
                                                                    "dy", 
                                                                    "roll_angle", 
                                                                    "droll", 
                                                                    "z", 
                                                                    "dz", 
                                                                    "yaw_angle", 
                                                                    "dyaw",
                                                                    "qx",
                                                                    "qy",
                                                                    "qz",
                                                                    "qw"};
        csv_logger_filtered_pose_ = std::make_unique<CSVLogger>("mybebop_ws", "bebop_localization", "filtered_pose", header);
        csv_logger_odometry_ = std::make_unique<CSVLogger>("mybebop_ws", "bebop_localization", "odometry", header);
        csv_logger_ground_truth_ = std::make_unique<CSVLogger>("mybebop_ws", "bebop_localization", "ground_truth", header);

        // Initialize parameters
        initializeParameters(nh);

        // Initialize publishers
        // joy_pub_ = nh.advertise<sensor_msgs::Joy>("joy", 10);
        filtered_pose_pub_ = nh.advertise<nav_msgs::Odometry>(filtered_pose_topic, 10);


        // Initialize subscribers
        odom_sub = nh.subscribe(odom_topic, 10, &bebop_localization::odomCallback, this);
        natnet_sub = nh.subscribe(natnet_topic, 10, &bebop_localization::natnetCallback, this);
        
        // Get current time
        std::time_t now = std::time(nullptr);
        std::tm* now_tm = std::localtime(&now);        
        timestamp << std::put_time(now_tm, "%Y%m%d_%H%M%S");


        // Initialize services
        // service_ = nh.advertiseService("service_name", &bebop_localization::serviceCallback, this);

        // Initialize timers
        // timer_ = nh.createTimer(ros::Duration(1.0), &bebop_localization::timerCallback, this);
    }
        ~bebop_localization()
    {
    }

    void spin()
    {
        ros::spin();
    }

private:
    // Function to initialize parameters
    void initializeParameters(ros::NodeHandle& nh)
    {
        // int default_value = 0; // Define default_value with the same type as param_value_
        // nh.param("param_name", param_value_, default_value);
        // Add more parameters as needed
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ROS_INFO("Received Odometry message");
        // Process Odometry messages here
        odometry = *msg;
        tf::Quaternion quaternion;      
        tf::quaternionMsgToTF(odometry.pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        double x = odometry.pose.pose.position.x;
        double dx = odometry.twist.twist.linear.x;
        double theta = pitch;
        double dtheta = odometry.twist.twist.angular.y;
        double y = odometry.pose.pose.position.y;
        double dy = odometry.twist.twist.linear.y;
        double roll_angle = roll;
        double droll = odometry.twist.twist.angular.x;
        double z = odometry.pose.pose.position.z;
        double dz = odometry.twist.twist.linear.z;
        double yaw_angle = yaw;
        double dyaw = odometry.twist.twist.angular.z;
        x_odom_ << x, dx, theta, dtheta, y, dy, roll_angle, droll, z, dz, yaw_angle, dyaw;
        x_pred_ = F_ * x_odom_;
        P_pred_ = F_ * P_ * F_.transpose() + Q_;

        updateKalmanFilter(z_, x_pred_,P_pred_);
        
        std::vector<std::variant<std::string, double>> data_odom = {std::to_string(ros::Time::now().toSec()), 
                                                                odom_topic, 
                                                                x, 
                                                                dx, 
                                                                theta, 
                                                                dtheta, 
                                                                y, 
                                                                dy, 
                                                                roll_angle, 
                                                                droll, 
                                                                z, 
                                                                dz, 
                                                                yaw_angle, 
                                                                dyaw,
                                                                quaternion.x(),
                                                                quaternion.y(),
                                                                quaternion.z(),
                                                                quaternion.w()};
        csv_logger_odometry_->writeCSV(data_odom);
    }
    // void natnetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)    
    void natnetCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ROS_INFO("Received Pose message");
        // // Process Pose messages here
        pose = *msg;
        tf::Quaternion quaternion;
        tf::quaternionMsgToTF(pose.pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        z_(0) = pose.pose.pose.position.x;
        z_(1) = pose.pose.pose.position.y;
        z_(2) = pose.pose.pose.position.z;
        z_(3) = roll;
        z_(4) = pitch;
        z_(5) = yaw;
        std::vector<std::variant<std::string, double>> data_ground_truth = {std::to_string(ros::Time::now().toSec()), 
                                                                natnet_topic, 
                                                                pose.pose.pose.position.x, 
                                                                0.0, 
                                                                pitch, 
                                                                0.0,
                                                                pose.pose.pose.position.y, 
                                                                0.0,
                                                                roll, 
                                                                0.0, 
                                                                pose.pose.pose.position.z, 
                                                                0.0,   
                                                                yaw, 
                                                                0.0,
                                                                quaternion.x(),
                                                                quaternion.y(),
                                                                quaternion.z(),
                                                                quaternion.w()};
        csv_logger_ground_truth_->writeCSV(data_ground_truth);
    }

    void updateKalmanFilter(Eigen::VectorXd z, Eigen::VectorXd x_pred, Eigen::MatrixXd P_pred)
    {
        Eigen::MatrixXd K = P_pred * H_.transpose() * (H_ * P_pred * H_.transpose() + R_).inverse();
        x_ = x_pred + K * (z - H_ * x_pred);
        P_ = (Eigen::MatrixXd::Identity(12, 12) - K * H_) * P_pred;
        filtered_pose.pose.pose.position.x = x_(0);
        filtered_pose.pose.pose.position.y = x_(4);
        filtered_pose.pose.pose.position.z = x_(8);
        filtered_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(x_(6), x_(2), x_(10));
        filtered_pose_pub_.publish(filtered_pose);
        std::vector<std::variant<std::string, double>> data_filtered = {std::to_string(ros::Time::now().toSec()), 
                                                                filtered_pose_topic, 
                                                                x_(0), 
                                                                x_(1), 
                                                                x_(2), 
                                                                x_(3), 
                                                                x_(4), 
                                                                x_(5), 
                                                                x_(6), 
                                                                x_(7), 
                                                                x_(8), 
                                                                x_(9), 
                                                                x_(10), 
                                                                x_(11),
                                                                filtered_pose.pose.pose.orientation.x,
                                                                filtered_pose.pose.pose.orientation.y,
                                                                filtered_pose.pose.pose.orientation.z,
                                                                filtered_pose.pose.pose.orientation.w};
        csv_logger_filtered_pose_->writeCSV(data_filtered);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_localization");
    ros::NodeHandle nh;

    bebop_localization node(nh);
    node.spin();

    return 0;
}