#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class OdometryToPose
{
public:
    OdometryToPose()
    {
        // Inicializa o publisher para o tópico PoseStamped
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/natnet_ros/Bebop1/pose", 10);

        // Inicializa o subscriber para o tópico Odometry
        odom_sub_ = nh_.subscribe("bebop2/ground_truth/odometry", 10, &OdometryToPose::odomCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Subscriber odom_sub_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Cria uma mensagem PoseStamped
        geometry_msgs::PoseStamped pose_msg;

        // Copia o cabeçalho da mensagem Odometry para a mensagem PoseStamped
        pose_msg.header = msg->header;

        // Copia a pose da mensagem Odometry para a mensagem PoseStamped
        pose_msg.pose = msg->pose.pose;
        
        // Publica a mensagem PoseStamped
        pose_pub_.publish(pose_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_to_pose");

    OdometryToPose odometry_to_pose;

    ros::spin();

    return 0;
}