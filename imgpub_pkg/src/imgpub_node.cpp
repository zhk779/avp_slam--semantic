#include <iostream>
#include <string>
#include <sstream>
using namespace std;

// OpenCV includes
#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_color");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    /**************ROS与Opencv图像转换***********************/
    String imgs[] = {
        // "/home/snow/Documents/workspace/slam_ws/src/imgpub_pkg/img/underground_1.jpg",
        // "/home/snow/Documents/workspace/slam_ws/src/imgpub_pkg/img/underground_2.jpg",
        // "/home/snow/Documents/workspace/slam_ws/src/imgpub_pkg/img/underground_3.jpg",
        // "/home/snow/Documents/workspace/slam_ws/src/imgpub_pkg/img/underground_4.jpg",
        // "/home/snow/Documents/workspace/slam_ws/src/imgpub_pkg/img/underground_5.jpg",
        "/home/snow/Documents/workspace/slam_ws/src/imgpub_pkg/img/ADAS_EYES_360_VIEW.png",
    };
    int i = 0;
    ros::Rate loop_rate(5);
    while (nh.ok())
    {
        Mat image = imread(imgs[i++], cv::IMREAD_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        if(i == 3)
        {
            i = 0;
        }
        i = 0;
        loop_rate.sleep();
    }
    return 0;
}
