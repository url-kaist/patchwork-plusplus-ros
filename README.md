# patchwork-plusplus-ros

This is ROS package of Patchwork++, which is a fast and robust ground segmentation method.

<p align="center"><img src=pictures/patchwork++.gif alt="animated" /></p>

> If you are not familiar with ROS, please visit the [original repository][patchwork++link].

> If you follow the [repository][patchwork++link], you can run Patchwork++ in Python and C++ easily.

[patchwork++link]: https://github.com/url-kaist/patchwork-plusplus

## :open_file_folder: What's in this repositpory

* ROS based Patchwork source code ([patchworkpp.hpp][codelink])
* Demo launch file ([demo.launch][launchlink]) with sample rosbag file. You can execute Patchwork++ simply!

[codelink]: https://github.com/seungjae24/patchwork-plusplus-ros/blob/master/include/patchworkpp/patchworkpp.hpp
[launchlink]: https://github.com/seungjae24/patchwork-plusplus-ros/blob/master/launch/demo.launch

## :package: Prerequisite packages
You may need to install ROS, PCL, Eigen, ...

## :gear: How to build Patchwork++
To build Patchwork++, you can follow below codes.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin build # or catkin_make
```

## :runner: To run the demo codes
There is a demo which executes Patchwork++ with sample rosbag file. You can download sample file with the following command.

```bash
$ wget https://www.dropbox.com/s/oel7o0azosm0m46/kitti_00_sample.bag
```

> The rosbag file is based on the [KITTI][kittilink] dataset. The bin files are merged into the rosbag file format.

> The sample file contains LiDAR sensor data only.

[kittilink]: http://www.cvlibs.net/datasets/kitti/raw_data.php

Then, you can run demo as follows.

```bash
# Start Patchwork++
$ roslaunch patchworkpp demo.launch
# Start the bag file
$ rosbag play kitti_00_sample.bag
```

## :postbox: Contact
If you have any question, don't be hesitate let us know!

* [Seungjae Lee][sjlink] :envelope: (sj98lee at kaist.ac.kr)
* [Hyungtae Lim][htlink] :envelope: (shapelim at kaist.ac.kr)

[sjlink]: https://github.com/seungjae24
[htlink]: https://github.com/LimHyungTae
