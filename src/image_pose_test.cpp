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

    std::string trajectory_file;
    double stamp;
    ROS_FATAL_STREAM_COND(!nh_private.getParam("stamp", stamp),
                          "Image stamp required");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("trajectory_file", trajectory_file),
                          "VIO trajectory file not set from params");

    std::unique_ptr<PoseInterpolator> pose_intr = std::make_unique<PoseInterpolator>(trajectory_file);
    pose_intr->loadTrajectory();

    geometry_msgs::Pose pose;

    std::uint64_t nano_stamp = (std::uint64_t)(stamp * 1000000000);
    std::cout << std::setprecision(16) << "Finding pose at: " << ((double)nano_stamp) / 1000000000 << std::endl;
    if (pose_intr->getPose(nano_stamp, pose))
    {
        ROS_INFO_STREAM("Pose: " << pose << std::endl);
    }
    else
    {
        ROS_WARN_STREAM("Pose not found\n");
    }

    ros::shutdown();

    return 0;
}