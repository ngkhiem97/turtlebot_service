#include <ros/ros.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <std_msgs/String.h>
#include <turtlebot_service/RobotDetail.h>
#include <turtlebot_service/RobotList.h>

#define TIME_STR_LENGTH 20

ros::Publisher robot_list_pub;

std::vector<turtlebot_service::RobotDetail> registered_robots;

std::string getTimeStr(){
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    char buffer[256];
    std::strftime(&buffer[0], 256, "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return buffer;
}

bool existRobotId(std::string message) {
    std::vector<turtlebot_service::RobotDetail>::iterator it;
    for (it = registered_robots.begin(); it != registered_robots.end(); ++it) {
        if (it->robot_id == message) return true;
    }

    return false;
}

void registerCallback(const std_msgs::String::ConstPtr &msg) {
    if (!existRobotId(msg->data.c_str())) {
        turtlebot_service::RobotDetail robot_detail;
        robot_detail.robot_id   = msg->data.c_str();
        robot_detail.created_at = getTimeStr();
        registered_robots.push_back(robot_detail);
    }
}

void removeCallback(const std_msgs::String::ConstPtr& msg) {
    std::vector<turtlebot_service::RobotDetail>::iterator it;
    for (it = registered_robots.begin(); it != registered_robots.end(); ++it) {
        if (it->robot_id == msg->data.c_str()) {
            it = registered_robots.erase(it);
            break;
        }
    }
}

void msgPub()
{
    turtlebot_service::RobotList list;

    std::vector<turtlebot_service::RobotDetail>::iterator it;
    for(it = registered_robots.begin(); it != registered_robots.end(); ++it) {
        list.robots.push_back(*it);
    }

    robot_list_pub.publish(list);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_manager");
    ros::NodeHandle nh;

    robot_list_pub = nh.advertise<turtlebot_service::RobotList>("robot_list", 10);

    ros::Subscriber rg = nh.subscribe("register", 10, registerCallback);
    ros::Subscriber rm = nh.subscribe("remove", 10, removeCallback);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        msgPub();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}