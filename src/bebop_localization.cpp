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
    nav_msgs::Odometry odometry, filtered_pose, filtered_pose_est;
    geometry_msgs::PoseStamped pose;
    ros::Publisher filtered_pose_pub_;
    ros::Publisher filtered_pose_pub_est_;
    ros::Subscriber odom_sub;
    ros::Subscriber natnet_sub;
    ros::Timer timer_;
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
    std::unique_ptr<CSVLogger> csv_logger_filtered_pose_est_;
    std::unique_ptr<CSVLogger> csv_logger_odometry_;
    std::unique_ptr<CSVLogger> csv_logger_ground_truth_;
    std::ostringstream timestamp;
    std::string odom_topic = "bebop2/odometry_sensor1/odometry";
    std::string natnet_topic = "/natnet_ros/Bebop1/pose";
    std::string filtered_pose_topic = "/filtered_pose";
    std::string filtered_pose_est_topic = "/filtered_pose_est";
    //Estimated Kalman Parameters
    bool use_odom = false;
    Eigen::MatrixXd F_est;
    Eigen::MatrixXd G_est;
    Eigen::MatrixXd H_est;
    Eigen::MatrixXd Q_est;
    Eigen::MatrixXd R_est;
    Eigen::MatrixXd x_est;
    Eigen::MatrixXd P_est;

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
        x_ = Eigen::VectorXd::Zero(12);

        //Estimated Kalman Filter Parameters
        double Ts = 0.05;
        double sigma_a = 0.1;
        initializeEstimatedKalmanParameters(Ts, sigma_a);

        // Initialize loggers
        const char* workspace_name = std::getenv("MY_WORKSPACE_NAME");
        if (workspace_name == nullptr)
        {
            ROS_ERROR("Environment variable MY_WORKSPACE_NAME is not set.");
            return;
        }
        std::vector<std::string> header = std::vector<std::string>{"timestamp", 
                                                                    "topic", 
                                                                    "x", 
                                                                    "dx", 
                                                                    "theta", 
                                                                    "dtheta", 
                                                                    "y", 
                                                                    "dy", 
                                                                    "roll", 
                                                                    "droll", 
                                                                    "z", 
                                                                    "dz", 
                                                                    "yaw", 
                                                                    "dyaw",
                                                                    "qx",
                                                                    "qy",
                                                                    "qz",
                                                                    "qw"};
        csv_logger_filtered_pose_ = std::make_unique<CSVLogger>(workspace_name, "bebop_localization", "filtered_pose", header);
        csv_logger_filtered_pose_est_ = std::make_unique<CSVLogger>(workspace_name, "bebop_localization", "filtered_pose_est", header);
        csv_logger_odometry_ = std::make_unique<CSVLogger>(workspace_name, "bebop_localization", "odometry", header);
        csv_logger_ground_truth_ = std::make_unique<CSVLogger>(workspace_name, "bebop_localization", "ground_truth", header);

        // Initialize parameters
        initializeParameters(nh);

        // Initialize publishers
        // joy_pub_ = nh.advertise<sensor_msgs::Joy>("joy", 10);
        filtered_pose_pub_ = nh.advertise<nav_msgs::Odometry>(filtered_pose_topic, 10);
        filtered_pose_pub_est_ = nh.advertise<nav_msgs::Odometry>(filtered_pose_est_topic, 10);

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
        timer_ = nh.createTimer(ros::Duration(Ts), &bebop_localization::timerCallback, this);
        ROS_INFO("Timer initialized with Ts = %f", Ts);
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
    void initializeEstimatedKalmanParameters(double Ts, double sigma)
    {
        F_est = Eigen::MatrixXd::Identity(12, 12);
        G_est = Eigen::MatrixXd(12, 6);
        H_est = Eigen::MatrixXd::Zero(6, 12);
        Q_est = Eigen::MatrixXd::Zero(12, 12);
        R_est = Eigen::MatrixXd::Identity(6, 6);
        x_est = Eigen::VectorXd::Zero(12);
        P_est = Eigen::MatrixXd::Identity(12, 12);
        // Initialize F_ matrix
        for (int i = 0; i < 6; ++i) {
            F_est(2 * i, 2 * i + 1) = Ts;
        }

        // Initialize G_ matrix
        for (int i = 0; i < 6; ++i) {
            G_est(2 * i, i) = 0.5 * Ts * Ts;
            G_est(2 * i + 1, i) = Ts;
        }
        // Initialize H_ matrix
        H_est(0, 0) = 1;
        H_est(1, 4) = 1;
        H_est(2, 8) = 1;
        H_est(3, 6) = 1;
        H_est(4, 2) = 1;
        H_est(5, 10) = 1;
        // Initialize Q_ matrix
        Q_est = G_est * G_est.transpose() * sigma * sigma;

        // Initialize R_ matrix
        R_est = Eigen::MatrixXd::Identity(6, 6) * 0.1; // Exemplo de valor para R_
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
        double pitch_ = pitch;
        double dpitch = odometry.twist.twist.angular.y;
        double y = odometry.pose.pose.position.y;
        double dy = odometry.twist.twist.linear.y;
        double roll_ = roll;
        double droll = odometry.twist.twist.angular.x;
        double z = odometry.pose.pose.position.z;
        double dz = odometry.twist.twist.linear.z;
        double yaw_ = yaw;
        double dyaw = odometry.twist.twist.angular.z;
        x_odom_ << x, dx, pitch_, dpitch, y, dy, roll_, droll, z, dz, yaw_, dyaw;
        x_pred_ = F_ * x_odom_;
        P_pred_ = F_ * P_ * F_.transpose() + Q_;        
        updateKalmanFilter(z_, x_pred_,P_pred_);
        
        std::vector<std::variant<std::string, double>> data_odom = {std::to_string(ros::Time::now().toSec()), 
                                                                odom_topic, 
                                                                x, 
                                                                dx, 
                                                                pitch_, 
                                                                dpitch, 
                                                                y, 
                                                                dy, 
                                                                roll_, 
                                                                droll, 
                                                                z, 
                                                                dz, 
                                                                yaw_, 
                                                                dyaw,
                                                                quaternion.x(),
                                                                quaternion.y(),
                                                                quaternion.z(),
                                                                quaternion.w()};
        csv_logger_odometry_->writeCSV(data_odom);
    }
    // void natnetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)    
    void natnetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_INFO("Received Pose message");
        // // Process Pose messages here
        pose = *msg;
        tf::Quaternion quaternion;
        
        tf::quaternionMsgToTF(pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double z = pose.pose.position.z;
        z_(0) = x;
        z_(1) = y;
        z_(2) = z;
        z_(3) = roll;
        z_(4) = pitch;
        z_(5) = yaw;



        std::vector<std::variant<std::string, double>> data_ground_truth = {std::to_string(ros::Time::now().toSec()), 
                                                                natnet_topic, 
                                                                pose.pose.position.x, 
                                                                0.0, 
                                                                pitch, 
                                                                0.0,
                                                                pose.pose.position.y, 
                                                                0.0,
                                                                roll, 
                                                                0.0, 
                                                                pose.pose.position.z, 
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
        filtered_pose.twist.twist.linear.x = x_(1);
        filtered_pose.twist.twist.linear.y = x_(5);
        filtered_pose.twist.twist.linear.z = x_(9);
        filtered_pose.twist.twist.angular.x = x_(7);
        filtered_pose.twist.twist.angular.y = x_(3);
        filtered_pose.twist.twist.angular.z = x_(11);        
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
    void timerCallback(const ros::TimerEvent&)
    {
        // Chama a função para realizar o filtro de Kalman e publicar a pose
        applyKalmanFilterAndPublish();
    }
    void applyKalmanFilterAndPublish()
    {
        // Predição do estado
        Eigen::VectorXd x_pred_est = F_est * x_est;
        Eigen::MatrixXd P_pred_est = F_est * P_est* F_est.transpose() + Q_est;

        // Atualização do filtro de Kalman
        Eigen::MatrixXd K = P_pred_est * H_est.transpose() * (H_est * P_pred_est * H_est.transpose() + R_est).inverse();
        x_est = x_pred_est + K * (z_ - H_est * x_pred_est);
        P_est = (Eigen::MatrixXd::Identity(12, 12) - K * H_est) * P_pred_est;

        // Estimação das velocidades
        double x = x_(0);
        double dx = x_(1);
        double pitch = x_(2);
        double dpitch = x_(3);
        double y = x_(4);
        double dy = x_(5);
        double roll = x_(6);
        double droll = x_(7);
        double z = x_(8);
        double dz = x_(9);
        double yaw = x_(10);
        double dyaw = x_(11);

        // Publica a pose filtrada
        filtered_pose_est.pose.pose.position.x = x;
        filtered_pose_est.pose.pose.position.y = y;
        filtered_pose_est.pose.pose.position.z = z;
        filtered_pose_est.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        filtered_pose_est.twist.twist.linear.x = dx;
        filtered_pose_est.twist.twist.linear.y = dy;
        filtered_pose_est.twist.twist.linear.z = dz;
        filtered_pose_est.twist.twist.angular.x = droll;
        filtered_pose_est.twist.twist.angular.y = dpitch;
        filtered_pose_est.twist.twist.angular.z = dyaw;
        filtered_pose_pub_est_.publish(filtered_pose_est);

        // Log dos dados filtrados
        std::vector<std::variant<std::string, double>> data_filtered_est = {std::to_string(ros::Time::now().toSec()), 
                                                                filtered_pose_est_topic, 
                                                                x, 
                                                                dx, 
                                                                pitch, 
                                                                dpitch, 
                                                                y, 
                                                                dy, 
                                                                roll, 
                                                                droll, 
                                                                z, 
                                                                dz, 
                                                                yaw, 
                                                                dyaw,
                                                                filtered_pose_est.pose.pose.orientation.x,
                                                                filtered_pose_est.pose.pose.orientation.y,
                                                                filtered_pose_est.pose.pose.orientation.z,
                                                                filtered_pose_est.pose.pose.orientation.w};
        csv_logger_filtered_pose_est_->writeCSV(data_filtered_est);
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