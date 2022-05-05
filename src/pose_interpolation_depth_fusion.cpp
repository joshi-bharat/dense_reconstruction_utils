#include "Utils.h"
#include "PoseInterpolator.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <fstream>

#include <geometry_msgs/Pose.h>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "interpolator");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    std::string image_list, trajectory_file, output_file;
    ROS_FATAL_STREAM_COND(!nh_private.getParam("image_list_file", image_list),
                          "Image list file path not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("trajectory_file", trajectory_file),
                          "VIO trajectory file not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("output_file", output_file),
                          "VIO trajectory file not set from params");

    std::vector<std::uint64_t> camera_timestamps;
    std::vector<std::string> original_stamps;
    Utils::getImageStampsImageList(image_list, camera_timestamps, original_stamps);
    ROS_INFO_STREAM("Got " << camera_timestamps.size() << " images\n");

    std::unique_ptr<PoseInterpolator> pose_intr = std::make_unique<PoseInterpolator>(trajectory_file);
    pose_intr->loadTrajectory();

    std::ofstream output_stream(output_file, std::ios::out);

    geometry_msgs::Pose pose;
    for (int i = 0; i < camera_timestamps.size(); ++i)
    {
        std::uint64_t stamp = camera_timestamps.at(i);
        std::string original_stamp = original_stamps.at(i);
        // std::cout << std::setprecision(16) << "Finding pose at: " << ((double)stamp) / 1000000000 << std::endl;
        if (pose_intr->getPose(stamp, pose))
        {
            // output_stream << original_stamp << " " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " "
            //               << pose.orientation.w << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << std::endl;

            output_stream << original_stamp << pose.position.x << " " << pose.position.y << " " << pose.position.z << " "
                          << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " "
                          << pose.orientation.w << " "<<  std::endl;
        }
        else
        {
            ROS_WARN_STREAM("Pose not found at: " << original_stamp << std::endl);
        }
    }

    output_stream.close();
    return 0;
    // ' std::vector<std::uint64_t>
}