#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr &pose_stamped);

tf::Transform t_w_c, t_c_w;

int main(int argc, char *argv[])
{
    t_w_c.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    t_w_c.setRotation(tf::Quaternion(0.0, 0.707106781186548, -0.707106781186548, 0.0));

    t_c_w = t_c_w.inverse();

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
    ros::Time time = pose_msg->header.stamp;
    tf::Transform transform;
    tf::transformMsgToTF(pose_msg->transform, transform);

    tf::Transform 
}