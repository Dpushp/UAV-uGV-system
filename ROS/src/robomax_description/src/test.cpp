#include <ros/ros.h>
#include <queue> 
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <robomax_description/stateHandler.h>
#include <robomax_description/waypointsArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "diff_drive/GoToPoseAction.h"
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


int main(int argc, char const *argv[])
{
    geometry_msgs::TransformStamped transform;
    geometry_msgs::PoseStamped initpose,mission;
    initpose.pose.position.x = 10.0;
    initpose.pose.position.y = -90.0;
    initpose.pose.position.z = 0.0;
    initpose.pose.orientation.x = 0.0;
    initpose.pose.orientation.y = 0.0;
    initpose.pose.orientation.z = 0.0;
    initpose.pose.orientation.w = 1.0;
    
    transform.header.frame_id = "base_footprint";
    transform.header.stamp = ros::Time(0);
    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 1.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.70;
    transform.transform.rotation.w = 0.70;
    tf2::doTransform(initpose, mission, transform); 
    std::cout << "mission.header.frame_id    :  " << mission.header.frame_id << std::endl;
    std::cout << "mission.pose.position.x    :  " << mission.pose.position.x << std::endl;
    std::cout << "mission.pose.position.y    :  " << mission.pose.position.y << std::endl;
    std::cout << "mission.pose.position.z    :  " << mission.pose.position.z << std::endl;
    std::cout << "mission.pose.orientation.x :  " << mission.pose.orientation.x << std::endl;
    std::cout << "mission.pose.orientation.y :  " << mission.pose.orientation.y << std::endl;
    std::cout << "mission.pose.orientation.z :  " << mission.pose.orientation.z << std::endl;
    std::cout << "mission.pose.orientation.w :  " << mission.pose.orientation.w << std::endl;

    return 0;
}
