#include <fstream>
#include <string>

#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <ros/package.h>

#include <Eigen/Core>

void getTransforms(const std::string &file, std::vector<std::pair<double, tf::Transform>> &transforms,
                   char separator = ' ', bool skip_firstline = true)
{
    std::ifstream fin(file.c_str());

    ROS_FATAL_STREAM_COND(!fin.is_open(), "Cannot open file: " << file << '\n');

    // Skip the first line, containing the header.
    std::string line;
    if (skip_firstline)
        std::getline(fin, line);

    while (std::getline(fin, line))
    {
        double timestamp = 0;
        std::vector<double> data_raw;
        for (size_t i = 0u; i < 9; i++)
        {
            int idx = line.find_first_of(separator);
            if (i == 0u)
            {
                timestamp = std::stold(line.substr(0, idx));
            }
            else
            {
                data_raw.push_back(std::stod(line.substr(0, idx)));
            }
            line = line.substr(idx + 1);
        }

        tf::Transform tf;
        tf.setOrigin(tf::Vector3(data_raw[0], data_raw[1], data_raw[2]));

        // Quaternion x y z w.

        // Sanity check.
        Eigen::Vector4d q(data_raw[3], data_raw[4], data_raw[5], data_raw[6]);

        tf.setRotation(tf::Quaternion(q(0), q(1), q(2), q(3)));

        transforms.emplace_back(std::make_pair(timestamp, tf));
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "correct_traj");
    ros::NodeHandle nh;

    std::string pkg_path = ros::package::getPath("iccv_utils");

    std::string file = pkg_path + "/traj/svin_without_loop_imu_frame.txt";
    // std::string file = pkg_path + "/traj/svin_loop_left_cam_frame.txt";

    std::vector<std::pair<double, tf::Transform>>
        transforms;
    getTransforms(file, transforms);

    ROS_INFO_STREAM("Got " << transforms.size() << " transforms from file.\n");

    tf::Transform t_imu_cam;
    t_imu_cam.setOrigin(tf::Vector3(0.09571982853264846, -0.002254291217617223, 0.024317486464850345));
    t_imu_cam.setRotation(tf::Quaternion(0.003271266547824, 0.013954930694373, -0.999890495190098, 0.003681895298451));

    tf::Transform t_wf_cf;
    t_wf_cf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    t_wf_cf.setRotation(tf::Quaternion(0.0, 0.707106781186548, -0.707106781186548, 0.0));

    tf::Transform t_cf_wf = t_wf_cf.inverse();

    std::string f = pkg_path + "/traj/svin_no_loop.txt";
    std::ofstream fout(f, std::ios::out);
    fout << "timestamp px py pz qx qy qz qw" << std::endl;
    fout.close();

    std::ofstream fapp(f, std::ios::app);
    fapp.setf(std::ios::fixed, std::ios::floatfield);
    fapp.precision(9);

    for (std::uint32_t i = 0; i < transforms.size(); ++i)
    {
        std::pair<double, tf::Transform> stamp_tf_pair = transforms.at(i);
        double stamp = stamp_tf_pair.first;

        tf::Transform t_w_imu = stamp_tf_pair.second;
        tf::Transform t_w_cam = t_w_imu * t_imu_cam;

        // tf::Transform t_w_cam = stamp_tf_pair.second;

        tf::Transform t_cf_cam = t_cf_wf * t_w_cam;

        geometry_msgs::Transform transform;
        tf::transformTFToMsg(t_cf_cam, transform);

        geometry_msgs::Vector3 trans = transform.translation;
        geometry_msgs::Quaternion quat = transform.rotation;

        fapp << stamp << " ";
        fapp << trans.x << " " << trans.y << " " << trans.z << " ";
        fapp << quat.x << " " << quat.y << " " << quat.z << " " << quat.w << std::endl;
    }

    fapp.close();
}