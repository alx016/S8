#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

void main_process(cv::Mat& image){
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    cv::equalizeHist(gray_image, gray_image);

    cv::imshow("Equalized Image CPP", gray_image);
    cv::waitKey(1);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        //Convert ROS Image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::imshow("Image Subscriber CPP", cv_ptr->image);
        cv::waitKey(1);
        main_process(cv_ptr -> image);
    } catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "image_subscriber_node_cpp");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, imageCallback);

    ros::spin();
    return 0;
}