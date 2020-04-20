#include <ros/ros.h>
#include <vector>
#include <std_msgs/String.h>
#include <fstream>
#include <signal.h>
#include <string.h>
#include "turtlebot_service/MapStatus.h"
#include "turtlebot_service/MapList.h"

std::vector<turtlebot_service::MapDetail> maps;

ros::Publisher map_list_pub;

// Load all saved map data and store it in "maps" variable
// for later pubish to "map_manager/list"
void loadMapData() {
    ROS_INFO("START LOAD MAP LIST FROM FILE!!!!");
    std::ifstream database("/home/khiem/data/map-data.db");

    std::string robot_id, file_name, time_stamp;
    while (database >> robot_id >> file_name >> time_stamp) {

        turtlebot_service::MapDetail map;
        map.robot_id   = robot_id;
        map.file_name  = file_name;
        map.time_stamp = time_stamp;

        maps.push_back(map);
    }

    database.close();
}

// --- CALLBACK FUNCTION ON MESSAGE RECEIVE ---

// call baack "map_export/status" topic
void exportCallback(const turtlebot_service::MapStatus::ConstPtr &msg) {
    if (strcmp(msg->status.c_str(), "Success") == 0) {

        // Append current maps vector
        maps.push_back(msg->detail);
    }
}

// call baack "map_export/delete" topic
void deleteCallback(const std_msgs::String::ConstPtr &msg) {
    std::vector<turtlebot_service::MapDetail>::iterator it;
    for (it = maps.begin(); it != maps.end(); ++it) {

        // Delete map with file name
        if (it->file_name == msg->data.c_str()) {
            it = maps.erase(it);
            break;
        }
    }
}

// --- END CALLBACK FUNCTION ---

// Publish all current maps data to "map_manager/list"
void msgPub() {
    turtlebot_service::MapList list;

    std::vector<turtlebot_service::MapDetail>::iterator it;
    for(it = maps.begin(); it != maps.end(); ++it) {
        list.maps.push_back(*it);
    }

    map_list_pub.publish(list);
}

// Reload maps from current map file
void reload() {
    std::ifstream database("/home/khiem/data/map-data.db");

    std::vector<turtlebot_service::MapDetail>::iterator it;
    for (it = maps.begin(); it != maps.end(); ++it) {

        // Loop through file to see any differences
        std::string robot_id, file_name, time_stamp;
        while (database >> robot_id >> file_name >> time_stamp) {

            if (strcmp(it->robot_id.c_str(), robot_id.c_str()) != 0 && strcmp(it->time_stamp.c_str(), time_stamp.c_str()) != 0) {
                maps.push_back(*it);
            }
        }
    }

    database.close();
}

//  Write all current maps data back to storage
void writeMapData() {
    ROS_INFO("START WRITING MAP LIST TO FILE!!!!");

    reload();

    // Delete old data by opening in truncate mode
    std::ofstream database;
    database.open("/home/khiem/data/map-data.db", std::ofstream::out | std::ofstream::trunc);

    std::vector<turtlebot_service::MapDetail>::iterator it;
    for (it = maps.begin(); it != maps.end(); ++it) {
        database << it->robot_id.c_str() << " " << it->file_name.c_str() << " " << it->time_stamp.c_str() << std::endl;
    }

    database.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_manager");
    ros::NodeHandle nh;

    loadMapData();

    map_list_pub = nh.advertise<turtlebot_service::MapList>("map_manager/list", 10);

    ros::Subscriber map_status = nh.subscribe("map_export/status", 10, exportCallback);
    ros::Subscriber map_delete = nh.subscribe("map_manager/delete", 10, deleteCallback);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        msgPub();
        writeMapData();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}