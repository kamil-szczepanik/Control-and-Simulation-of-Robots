#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_ros/buffer.h"
#include "global_planner/planner_core.h"
#include "base_local_planner/trajectory_planner_ros.h"
#include "rotate_recovery/rotate_recovery.h"
#include "dwa_local_planner/dwa_planner_ros.h"
#include <dwa_local_planner/DWAPlannerConfig.h>

geometry_msgs::PoseStamped goal_pose;
geometry_msgs::PoseStamped start_pose;
std::vector<geometry_msgs::PoseStamped> global_plan;
geometry_msgs::Twist vel_msg;

bool goal_reached = true;
bool new_goal = false;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    goal_pose = *pose_msg;
    goal_pose.header.frame_id = "map";
    new_goal = true;
	ROS_INFO("New goal set to: ( x: %f, y: %f )", goal_pose.pose.position.x, goal_pose.pose.position.y);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navi");
    ros::NodeHandle node;
    ros::Rate rate(10);

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    ros::Subscriber goal_sub = node.subscribe("/move_base_simple/goal", 1000, goalCallback);
	ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("key_vel", 1000);

    costmap_2d::Costmap2DROS global_costmap("global_costmap", buffer);
    costmap_2d::Costmap2DROS local_costmap("local_costmap", buffer);

    global_planner::GlobalPlanner global_planner("global_planner", global_costmap.getCostmap(), "map");
    base_local_planner::TrajectoryPlannerROS local_planner;
    local_planner.initialize("local_planner", &buffer, &local_costmap);

    rotate_recovery::RotateRecovery rr;
    rr.initialize("recovery_behaviour", &buffer, &global_costmap, &local_costmap);

    while(ros::ok()) {
        if(new_goal) {
            global_costmap.getRobotPose(start_pose);
            global_planner.makePlan(start_pose, goal_pose, global_plan);
            local_planner.setPlan(global_plan);

            new_goal = false;
            goal_reached = false;
        }
        if(!goal_reached) {
            if(local_planner.computeVelocityCommands(vel_msg)) {
                ROS_INFO("Navigating to goal - linear: %f, angular %f", vel_msg.linear.x, vel_msg.angular.z);
                vel_pub.publish(vel_msg);
            }
            else {
                ROS_INFO("Recovery behaviour");
                rr.runBehavior();
            }
            if(local_planner.isGoalReached()) {
                ROS_INFO("Goal reached !");
                goal_reached = true;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}