#include <ros/ros.h>


int main(int argc, char **argv){

    //argc : El número de argumentos pasados al programa desde la linea de comandos
    //argv : Array de cadenas que contiene los argumentos pasados al programa desde
    // la línea de comandos. 

    ros::init(argc, argv, "cpp_node");

    ros::NodeHandle nh;


    ROS_INFO("Node has been started");

    ros::Duration(1.0).sleep();

    ros::Rate rate(10);

    while (ros::ok()){
        ROS_INFO("Hello");
        rate.sleep();
    }

    ROS_INFO("Exit Now");

}
