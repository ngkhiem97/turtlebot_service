#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlebot_service/MapStatus.h"

ros::Publisher status_pub;

std::string getTimeStamp() {
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    char buffer[256];
    std::strftime(&buffer[0], 256, "%Y_%m_%d_%H_%M_%S", std::localtime(&now));
    return buffer;
}

void registerCallback(const std_msgs::String::ConstPtr &msg) {
  std::string robot_id   = msg->data.c_str();
  std::string time_stamp = getTimeStamp();
  std::string file_name  = "map_"+robot_id+"_"+time_stamp;

  // Execute system command
  int status = system((std::string("rosrun map_server map_saver map:=/turtlebot_service/robot/")+robot_id+
  "/map -f ~/catkin_ws/src/collada_web_server/map/"+file_name+
  " && convert ~/catkin_ws/src/collada_web_server/map/"+file_name+".pgm"
  " ~/catkin_ws/src/collada_web_server/map/"+file_name+".jpg").c_str());
  
  // Publish map exporting status
  turtlebot_service::MapStatus statusMsg;
  if (status < 0) {
    statusMsg.status  = "Error";
    statusMsg.message = "Can not export";

    status_pub.publish(statusMsg);
  } else {
    statusMsg.status     = "Success";
    statusMsg.message    = "Export successfully";
    statusMsg.detail.robot_id   = robot_id;
    statusMsg.detail.file_name  = file_name;
    statusMsg.detail.time_stamp = time_stamp;

    status_pub.publish(statusMsg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_export");
  ros::NodeHandle nh;

  status_pub = nh.advertise<turtlebot_service::MapStatus>("map_export/status", 10);

  ros::Subscriber rg = nh.subscribe("map_export/request", 10, registerCallback);

  ros::spin();
  return 0;
}
