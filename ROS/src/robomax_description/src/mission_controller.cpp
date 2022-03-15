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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::queue <geometry_msgs::PoseStamped> missionQueue;
nav_msgs::Odometry current_position;
bool controlThread = true;

std::string previous_control = "none";

// CANCEL
// STOP PAUSE
// START
// none

void waypoint_executor(int x)
{
    actionlib::SimpleActionClient<diff_drive::GoToPoseAction> actionClient("diff_drive_go_to_goal", true);
    // MoveBaseClient actionClient("diff_drive_go_to_goal", true);
    
    while(!actionClient.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }    
    ros::Time prev = ros::Time::now();

    while(ros::ok())
    {
        if (controlThread == true)
        {
            if (missionQueue.size()>0)
            {
                ROS_INFO("Executor : Running Mission %d",missionQueue.size());
                //debug missionqueue front
                geometry_msgs::PoseStamped tempStore;
                tempStore = missionQueue.front();
                if (tempStore.pose.orientation.z == 0.00 && tempStore.pose.orientation.w == 0.00)
                {
                    sleep(tempStore.pose.orientation.x);
                    if (missionQueue.size() > 0)
                            missionQueue.pop();
                } 
                else
                {
                    diff_drive::GoToPoseGoal goal;
                    goal.pose.header.frame_id = tempStore.header.frame_id;
                    goal.pose.header.stamp = ros::Time::now();
                    goal.pose.pose.position.x = tempStore.pose.position.x;
                    goal.pose.pose.position.y = tempStore.pose.position.y;
                    goal.pose.pose.orientation.z = tempStore.pose.orientation.z;
                    goal.pose.pose.orientation.w = tempStore.pose.orientation.w;
                    ROS_INFO("Sending Goal");
                    actionClient.sendGoal(goal);

                    actionClient.waitForResult();

                    if(actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("Executor: Mission %d completed",missionQueue.size());
                        if (missionQueue.size() > 0)
                            missionQueue.pop();
                    }
                    else
                    {
                        ROS_INFO("Executor: Mission %d did not complete",missionQueue.size());
                        controlThread = false;
                        previous_control = "stop";
                        ROS_INFO("Executor: Stopped",missionQueue.size());
                    }
                    
                }
            }
            else {
                usleep(500000);// no delay needed
                ROS_INFO("Executor : No missions to execute");
            }
        }
    }
}



bool controller_state_handler(robomax_description::stateHandler::Request &req,
                              robomax_description::stateHandler::Response &res,
                              ros::Publisher &cancelPulbisher){

        if (previous_control != req.state.data)
        {
            ROS_INFO("Current state : %s .Change  controller state to : %s",previous_control.c_str(),req.state.data.c_str());
            previous_control = req.state.data;

            if(previous_control == "clear")
            {
                controlThread = false;
                if (missionQueue.size() > 0)
                {
                    ROS_INFO("Flushing all current missions. Total : %d",missionQueue.size());
                    while(missionQueue.size() > 0)
                        missionQueue.pop();
                        ROS_INFO("Removing all missions");
                }
            }

            if(previous_control == "stop")
            {
                controlThread = false;
                ROS_INFO("Executor running : Stop");
            }

            if(previous_control == "start")
            {
                controlThread = true;
            }

            if (previous_control == "cancel")
            {
                controlThread = false;
                ROS_INFO("Executor : Cancelling current mission");
                actionlib_msgs::GoalID temp;
                cancelPulbisher.publish(temp);

            }
            //kill current mission 
            res.status.data = true;
        }
        else
        {
            res.status.data = false;   
        }
    return true;
}


bool controller_digester (robomax_description::waypointsArray::Request &req,
                          robomax_description::waypointsArray::Response &res)
{ 
    ROS_INFO("Got new mission ");
    if(req.maneuver.data == "grab")
    {
        ROS_INFO("grab maneuver");
        geometry_msgs::PoseStamped mission;
        mission.header.frame_id="base_footprint";
        mission.pose.position.x = 0.0;
        mission.pose.orientation.z = 1.0;
        mission.pose.orientation.w = 0.0;
        missionQueue.push(mission);
        mission.pose.position.x = -0.2;
        mission.pose.orientation.z = 0.0;
        mission.pose.orientation.w = 1.0;
        missionQueue.push(mission);
        mission.pose.position.x = 0.2;
        mission.pose.orientation.z = 0.0;
        mission.pose.orientation.w = 1.0;
        missionQueue.push(mission);
    }
    else if (req.maneuver.data == "search")
    {
        geometry_msgs::PoseStamped mission;
        mission.header.frame_id="base_footprint";
        // 1 - point
        mission.pose.position.x = 1.0;
        mission.pose.orientation.z = 0.0;
        mission.pose.orientation.w = 1.0;
        missionQueue.push(mission);
        // 2 - point
        mission.pose.position.x = 1.0;
        mission.pose.orientation.z = 0.707;
        mission.pose.orientation.w = 0.707;
        missionQueue.push(mission);
        // 3 - point
        mission.pose.position.x = 1.0;
        mission.pose.orientation.z = 0.707;
        mission.pose.orientation.w = 0.707;
        missionQueue.push(mission);
        // 4 - point
        mission.pose.position.x = 1.0;
        mission.pose.orientation.z = 0.707;
        mission.pose.orientation.w = 0.707;
        missionQueue.push(mission);
        // 5 - point
        mission.pose.position.x = 2.0;
        mission.pose.orientation.z = 0.707;
        mission.pose.orientation.w = 0.707;
        missionQueue.push(mission);
        // 6 - point
        mission.pose.position.x = 1.0;
        mission.pose.orientation.z = 0.707;
        mission.pose.orientation.w = 0.707;
        missionQueue.push(mission);
        // 7 - point
        mission.pose.position.x = 1.0;
        mission.pose.orientation.z = 0.707;
        mission.pose.orientation.w = 0.707;
        missionQueue.push(mission);
        // 8 - point
        mission.pose.position.x = 1.0;
        mission.pose.orientation.z = 0.0;
        mission.pose.orientation.w = 1.0;
        missionQueue.push(mission);
    }
    else if (req.maneuver.data == "test")
    {
        geometry_msgs::TransformStamped transform;
        geometry_msgs::PoseStamped initpose,mission;
        initpose.header = current_position.header;
        initpose.pose = current_position.pose.pose;
        transform.header.frame_id = "base_footprint";
        transform.header.stamp = ros::Time(0);
        transform.transform.translation.x = 1.0;
        transform.transform.rotation.w = 1.0;
        tf2::doTransform(initpose, mission, transform); 
        missionQueue.push(mission);
        transform.header.frame_id = "base_footprint";
        transform.header.stamp = ros::Time(0);
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 1.0;
        transform.transform.rotation.w = 1.0;
        tf2::doTransform(initpose, mission, transform); 
        missionQueue.push(mission);

    }
    else
    {
        missionQueue.push(req.goal);
    }
    return true;
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_position.child_frame_id = msg->child_frame_id;
    current_position.header = msg->header;
    current_position.pose = msg->pose;
    current_position.twist = msg->twist;
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle node;
    sleep(2);
    ros::Subscriber odomsub = node.subscribe("odom", 1, chatterCallback);
    ros::Publisher move_base_cancel_publisher = node.advertise<actionlib_msgs::GoalID>("/diff_drive_go_to_goal/cancel", 1);
    ros::ServiceServer waypoints_digester = node.advertiseService("mission_digester", controller_digester);
    ros::ServiceServer state_handler = node.advertiseService<robomax_description::stateHandler::Request,robomax_description::stateHandler::Response>("mission_state",boost::bind(controller_state_handler,_1,_2,boost::ref(move_base_cancel_publisher)));
    ROS_INFO("Mission controller started");
    std::thread thread1(waypoint_executor,1);
    ros::spin();
}