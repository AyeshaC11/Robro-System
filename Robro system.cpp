#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


cv::Point line_start(100, 200); 
cv::Point line_end(300, 200);   

void processImage(cv::Mat& image, ros::Publisher& count_pub) 
{
   
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    
    cv::GaussianBlur(grayImage, grayImage, cv::Size(5, 5), 0);

    
    cv::Mat binaryImage;
    cv::threshold(grayImage, binaryImage, 100, 255, cv::THRESH_BINARY);

    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    cv::drawContours(image, contours, -1, cv::Scalar(0, 255, 0), 2);

    
    int object_count = 0;
    for (const auto& contour : contours) 
    {
        cv::Rect boundingBox = cv::boundingRect(contour);
        cv::Point centroid(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
        int lineY = line_start.y; 
        if (centroid.y > lineY) 
        {
           
            object_count++;
        }
    }

   
    std_msgs::Int32 count_msg;
    count_msg.data = object_count;
    count_pub.publish(count_msg);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& count_pub) 
{
    try 
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        processImage(image, count_pub);

    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;

    ros::Publisher count_pub = nh.advertise<std_msgs::Int32>("/object_count", 1); // Create publisher for object count
    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, boost::bind(imageCallback, _1, count_pub));

    ros::spin();

    return 0;
}