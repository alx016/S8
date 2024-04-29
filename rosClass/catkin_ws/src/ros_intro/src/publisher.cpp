#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Int32>("/numbers", 10);
  ros::Rate rate(1);  // 1 Hz

  while (ros::ok())
  {
    std_msgs::Int32 msg;
    msg.data = 42; // Example number to publish
    ROS_INFO("Publishing: %d", msg.data);
    pub.publish(msg);
    rate.sleep();
  }

  return 0;
}
