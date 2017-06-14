/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Veľas (ivelas@fit.vutbr.cz)
 * Date: 13/03/2017
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <signal.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/KittiUtils.h>

using namespace std;
using namespace pcl;
using namespace ros;
using namespace tf;
using namespace but_velodyne;

const string NODE_NAME = "velodyne_imu_bag_parser";

class Parser {
private:
  int cloud_counter;
  bool is_first;
  tf::TransformListener tf_listener;
  StampedTransform first_transform;
  ofstream out_poses;

public:
  string out_dir, frame_id;

  Parser(const string &frame_id_, const string &out_dir_) :
    frame_id(frame_id_),
    out_dir(out_dir_),
    cloud_counter(0),
    is_first(true),
    out_poses((out_dir_ + "/poses.txt").c_str(), ios_base::out) {
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    VelodynePointCloud cloud;
    fromROSMsg(*msg, cloud);
    cloud.setImageLikeAxisFromKitti();
    string filename = out_dir + "/" + KittiUtils::getKittiFrameName(cloud_counter++, ".pcd");
    ROS_INFO_STREAM("Saving " << filename);
    io::savePCDFileBinary(filename, cloud);

    tf::StampedTransform transform;
    tf_listener.waitForTransform(frame_id, "imu_base", ros::Time::now(), ros::Duration(0.01));
    tf_listener.lookupTransform(frame_id, "imu_base", ros::Time(0), transform);
    if(is_first) {
      first_transform = transform;
      is_first = false;
    }
    Eigen::Affine3d t;
    tf::transformTFToEigen(first_transform.inverse()*transform, t);
    Eigen::Matrix4f t_float = cloud.getAxisCorrection() * t.cast<float>().matrix() * cloud.getAxisCorrection().inverse();
    KittiUtils::save_kitti_pose(Eigen::Affine3f(t_float), out_poses);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, NODE_NAME, ros::init_options::AnonymousName);
  NodeHandle nh("~");

  string out_dir(".");
  nh.getParam("out_dir", out_dir);
  string topic("/velodyne_points");
  nh.getParam("topic_name", topic);
  string frame_id("velodyne");
  nh.getParam("frame_id", frame_id);

  Parser parser(frame_id, out_dir);
  ros::Subscriber pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 10, &Parser::callback, &parser);
  spin();

  return EXIT_SUCCESS;
}
