#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped odomPose;
geometry_msgs::Twist Tiago_vel;

std::vector<geometry_msgs::PoseStamped> global_planner_path;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    odomPose.pose = msg->pose.pose;
    odomPose.header = msg->header;
    odomPose.header.frame_id = "map";
    ROS_INFO("Odometry callback");
}

void handle_goal(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    goal = *msg;
    goal.header.frame_id = "map";
    ROS_INFO("Goal passed successfully");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "navi");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    tf2_ros::Buffer buffer(ros::Duration(10));

    tf2_ros::TransformListener tf(buffer);
    costmap_2d::Costmap2DROS global_costmap("global_costmap", buffer);

    // global_planner::GlobalPlanner global_planner("global_planner", global_costmap.getCostmap(), "map");

    ros::Subscriber goalSubscriber = n.subscribe(
        "/move_base_simple/goal", 1000, handle_goal);
    ros::Subscriber odometrySubscriber = n.subscribe(
        "/mobile_base_controller/odom", 1000, odom_callback);
    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("nav_vel", 1000);

    int count = 0;
    while (ros::ok())
    {

        ROS_INFO("Planning");

        velocityPublisher.publish(Tiago_vel);

        // std_msgs::String msg;

        // std::stringstream ss;
        // ss << "hello world " << count;
        // msg.data = ss.str();

        // ROS_INFO("%s", msg.data.c_str());

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}