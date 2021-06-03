#include "Utils.h"
#include "PoseInterpolator.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "interpolator");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    std::string data_folder, trajectory_file;
    ROS_FATAL_STREAM_COND(!nh_private.getParam("dataset_path", data_folder),
                          "Stereo dataset folder path not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("trajectory_file", trajectory_file),
                          "VIO trajectory file not set from params");

    std::string image_folder = data_folder + "/" + "left";
    std::vector<std::uint64_t> camera_timestamps;
    Utils::getImageStamps(image_folder, camera_timestamps);
    ROS_INFO_STREAM("Got " << camera_timestamps.size() << " images\n");

    std::unique_ptr<PoseInterpolator> pose_intr = std::make_unique<PoseInterpolator>(trajectory_file);
    pose_intr->loadTrajectory();

    geometry_msgs::Pose pose;
    for (int i = 0; i < camera_timestamps.size(); ++i)
    {
        std::uint64_t stamp = camera_timestamps.at(i);
        std::cout << std::setprecision(16) << "Finding pose at: " << ((double)stamp) / 1000000000 << std::endl;
        if (pose_intr->getPose(stamp, pose))
        {
            ROS_INFO_STREAM("Pose: " << pose << std::endl);
        }
        else
        {
            ROS_WARN_STREAM("Pose not found\n");
        }
    }

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
    // ' std::vector<std::uint64_t>
}