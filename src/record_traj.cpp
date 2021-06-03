#include <chrono>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr &pose_stamped);

// tf::Transform t_w_c, t_c_w;

static std::string getTimeStr()
{
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    char s[100];
    std::strftime(s, sizeof(s), "%Y_%m_%d_%H_%M_%S", std::localtime(&now));
    return s;
}

std::string filename;

int main(int argc, char *argv[])
{

    // t_w_c.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    // t_w_c.setRotation(tf::Quaternion(0.0, 0.707106781186548, -0.707106781186548, 0.0));

    // t_c_w = t_c_w.inverse();
    std::string pkg_path = ros::package::getPath("iccv_utils");
    filename = pkg_path + "/okvis_" + getTimeStr() + ".txt";

    std::ofstream fout(filename, std::ios::out);
    fout << "timestamp px py pz qx qy qz qw" << std::endl;
    fout.close();

    ros::init(argc, argv, "tf_listner");
    ros::NodeHandle nh;

    std::string transform_topic = "/okvis_node/okvis_transform";
    ros::Subscriber sub = nh.subscribe(transform_topic, 1000, transformCallback);

    while (ros::ok())
    {
        ros::spinOnce();
    }
}

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr &pose_msg)
{
    std::ofstream fout(filename, std::ios::app);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(9);

    ros::Time time = pose_msg->header.stamp;
    double stamp = time.toSec();
    geometry_msgs::Vector3 trans = pose_msg->transform.translation;
    geometry_msgs::Quaternion quat = pose_msg->transform.rotation;

    fout << stamp << " ";
    fout << trans.x << " " << trans.y << " " << trans.z << " ";
    fout << quat.x << " " << quat.y << " " << quat.z << " " << quat.w << std::endl;
    fout.close();
    // tf::Transform
}