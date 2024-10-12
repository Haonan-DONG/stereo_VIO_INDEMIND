#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        // change ros image msg into cv::Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image_cv(msg->height, msg->width, CV_8UC1, (void *)msg->data.data());
        ROS_INFO("Saving image '%s'.", std::to_string(msg->header.stamp.toSec()).c_str());
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    // init ros
    ros::init(argc, argv, "stereo_vio_sub");
    ros::NodeHandle nh;

    // create a sub_crite
    ros::Subscriber image_sub_ = nh.subscribe("/imsee/image/left", 10, imageCallback);

    // 循环等待回调函数执行
    ros::spin();

    return 0;
}