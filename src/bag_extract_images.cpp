#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "extract_images_bag");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    std::string bag_file, image_dir, left_image_topic, right_image_topic;
    ROS_FATAL_STREAM_COND(!nh_private.getParam("bag", bag_file),
                          "Bag file path not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("image_dir", image_dir),
                          "Destination Image dir not set from params");

    ROS_FATAL_STREAM_COND(!nh_private.getParam("left", left_image_topic),
                          "Left image topic not set from params");

    float scale = 1.0;
    bool compressed = false;
    bool stereo = false;

    if (nh_private.hasParam("scale"))
        nh_private.getParam("scale", scale);

    if (nh_private.hasParam("compressed"))
        nh_private.getParam("compressed", compressed);

    if (nh_private.hasParam("stereo"))
        nh_private.getParam("stereo", stereo);

    std::string left_image_dir = image_dir + "/left";
    std::string right_image_dir = image_dir + "/right";

    if (!fs::exists(left_image_dir))
    {
        fs::create_directories(left_image_dir);
    }
    if (compressed)
        left_image_topic = left_image_topic + "/compressed";
    if (stereo)
    {
        ROS_FATAL_STREAM_COND(!nh_private.getParam("right", right_image_topic),
                              "right image topic not set from params");

        if (!fs::exists(right_image_dir))
            fs::create_directories(right_image_dir);

        if (compressed)
            right_image_topic = right_image_topic + "/compressed";
    }

    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    rosbag::View view_left(bag, rosbag::TopicQuery(left_image_topic));
    rosbag::View view_right(bag, rosbag::TopicQuery(right_image_topic));

    cv::Mat image;
    std::string stamp;

    rosbag::View::iterator it_left = view_left.begin();
    if (compressed)
    {
        sensor_msgs::CompressedImageConstPtr image_msg = it_left->instantiate<sensor_msgs::CompressedImage>();
        image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    else
    {
        sensor_msgs::ImageConstPtr image_msg = it_left->instantiate<sensor_msgs::Image>();
        image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    }

    int new_width = static_cast<int>(image.size().width * scale);
    int new_height = static_cast<int>(image.size().height * scale);

    std::string image_list_file = image_dir + "/image_list.txt";
    std::ofstream image_list(image_list_file);
    image_list << "timestamp[s], filename" << std::endl;

    for (auto it = view_left.begin(); it != view_left.end(); ++it)
    {
        std::stringstream ss;
        std::string stamp;
        if (compressed)
        {
            sensor_msgs::CompressedImageConstPtr image_msg = it->instantiate<sensor_msgs::CompressedImage>();
            image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            ss.precision(9);
            ss << std::fixed << image_msg->header.stamp;
            stamp = std::to_string(image_msg->header.stamp.toNSec());
        }
        else
        {
            sensor_msgs::ImageConstPtr image_msg = it->instantiate<sensor_msgs::Image>();
            image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            ss.precision(9);
            ss << std::fixed << image_msg->header.stamp;
            stamp = std::to_string(image_msg->header.stamp.toNSec());
        }
        std::string filename = left_image_dir + "/" + ss.str() + ".png";
        image_list << ss.str() << "," << stamp << ".png" << std::endl;

        // cv::imshow("image", image);
        // cv::waitKey(1);

        cv::resize(image, image, cv::Size(new_width, new_height));
        cv::imwrite(filename, image);
    }

    for (auto it = view_right.begin(); it != view_right.end(); ++it)
    {
        std::stringstream ss;
        std::string stamp;
        if (compressed)
        {
            sensor_msgs::CompressedImageConstPtr image_msg = it->instantiate<sensor_msgs::CompressedImage>();
            image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            stamp = std::to_string(image_msg->header.stamp.toNSec());
            ss.precision(9);
            ss << std::fixed << image_msg->header.stamp;
        }
        else
        {
            sensor_msgs::ImageConstPtr image_msg = it->instantiate<sensor_msgs::Image>();
            image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            stamp = std::to_string(image_msg->header.stamp.toNSec());
            ss.precision(9);
            ss << std::fixed << image_msg->header.stamp;
        }

        std::string filename = right_image_dir + "/" + ss.str() + ".png";
        cv::resize(image, image, cv::Size(new_width, new_height));
        cv::imwrite(filename, image);
    }

    bag.close();
    cv::destroyAllWindows();

    return 0;
}
