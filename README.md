# Utils package for ICCV Reconstruction Paper

## Install the package

To install package follow the create catkin workspace folder

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone git@github.com:joshi-bharat/iccv_utils.git
cd ~/catkin_ws
catkin_make
```

## Extract bag file into synchronized stereo images

Launch the stereo sync script. Need to pass the folder to extract images as argument to bag file or change the default arg in bag file. There is also option to scale the output image just in case.

```bash
roslaunch iccv_utils stereo_sync.launch
```
The extracted folder structure look like

```
+-- _config.yml
+-- root_dataset_folder
    +-- left
    |   +--data
    |      +-- *.png 
    |   +--data.csv
    +-- right
    |   +--data
    |      +-- *.png 
    |   +--data.csv
```
data.csv contains the list of images; images are named as timestamp from bag file.

Play the bag file with --clock arguments. Since the images are large and need to be synchronized, I recommend playing bag file at 0.5x speed.

```bash
rosbag play bagfile --clock -r 0.5
```

## Pose Estimate of images

We provide pose estimate for images in left camera reference frame. The [traj folder](https://github.com/joshi-bharat/iccv_utils/tree/main/traj) contains two files: svin_no_loop.txt and svin_loop.txt. svin_no_loop.txt contains trajectoty from Visual Inertial Odometry without loop closure and the trajectory drift after some time. svin_loop.txt contains trajectory obtained after loop closure. Since, the images are obtained using appoximate time synchronization they do not match with time stamp from trajectory file.

Actaully, the loop closure trajectory only contains pose of keyframes. So, exact pose of image pose can be found by interpolation.

I have implemented an example of such interpolation as orientation interpolation is not trivial.
Pose interpolation using quaterion [slerp](http://docs.ros.org/en/jade/api/tf/html/c++/classtf_1_1Quaternion.html#affa098b16b0091af8b71bfb533b5494a) can be tested using

```bash
roslaunch iccv_utils pose_interpolation_test.launch
```

More details on implementation can be found at [PoseInterpolator.cpp](https://github.com/joshi-bharat/iccv_utils/blob/main/src/PoseInterpolator.cpp) based on [slerp implemenation of ROS tf2 library.](http://docs.ros.org/en/jade/api/tf/html/c++/classtf_1_1Quaternion.html#affa098b16b0091af8b71bfb533b5494a)
