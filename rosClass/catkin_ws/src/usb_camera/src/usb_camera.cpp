#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

void find_devices(){
    for (int i=0; i<5; i++){
        cv::VideoCapture cap_test(i);
        if (cap_test.isOpened()){
            std::cout << "Device " <<i<< " is available."<<
            std::endl;
        }
        cap_test.release();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh("~");

    //Initialize image transport and publisher
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

    find_devices();

    int deviceID, width, height;
    nh.getParam("/camera/deviceID", deviceID);
    nh.getParam("/camera/width", width);
    nh.getParam("/camera/height", height);

    //Initialize OpenCV video capture
    cv::VideoCapture cap(0); //Open the default camera
    if (!cap.isOpened()){
        ROS_ERROR("Failed to open web_cam");
        return -1;
    }

    int frameWidth = width;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
    std::cout<<"Set Frame Width: "<<frameWidth <<std::endl;

    int frameHeight = height;
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
    std::cout<<"Set Frame Height: "<<frameHeight<<std::endl;

    std::cout <<"Current camera properties: "<<std::endl;

    std::cout<<"Device ID: "<< deviceID<<std::endl;

    frameWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    std::cout << "Frame Width:" << frameWidth <<std::endl;

    frameHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Frame Height: " <<frameHeight <<std::endl;

    double frameRate = cap.get(cv::CAP_PROP_FPS);
    std::cout << "Frame Rate: " << frameRate << std::endl;

    while(ros::ok()){
        cv::Mat frame;
        cap>>frame; //Capture frame from camera

        if(!frame.empty()){

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
        
        ros::spinOnce();
    }

    return 0;
}