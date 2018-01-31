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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

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
  ofstream out_poses, out_times;
  static const int MIN_REQ_POINTS = 1000;

public:
  string out_dir, frame_id_1, frame_id_2;

  Parser(const string &frame_id_1_, const string &frame_id_2_, const string &out_dir_) :
    frame_id_1(frame_id_1_),
    frame_id_2(frame_id_2_),
    out_dir(out_dir_),
    cloud_counter(0),
    is_first(true),
    out_poses((out_dir_ + "/poses.txt").c_str(), ios_base::out),
    out_times((out_dir_ + "/times.txt").c_str(), ios_base::out) {
  }

  void findSaveTransform(const Eigen::Matrix4f &axis_correction) {
    tf::StampedTransform transform;
    tf_listener.waitForTransform(frame_id_1, "imu_base", ros::Time(0), ros::Duration(0.01));
    tf_listener.lookupTransform(frame_id_1, "imu_base", ros::Time(0), transform);
    if(is_first) {
      first_transform = transform;
      is_first = false;
    }
    Eigen::Affine3d t;
    tf::transformTFToEigen(first_transform.inverse()*transform, t);
    Eigen::Matrix4f t_float = axis_correction * t.cast<float>().matrix() * axis_correction.inverse();
    KittiUtils::save_kitti_pose(Eigen::Affine3f(t_float), out_poses);
  }

  void saveCloud(const VelodynePointCloud &cloud, int sensor_id) {
    stringstream filename;
    filename << out_dir << "/" << KittiUtils::getKittiFrameName(cloud_counter, ".") << sensor_id << ".pcd";
    ROS_INFO_STREAM("Saving " << filename.str());
    io::savePCDFileBinary(filename.str(), cloud);
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& msg1, const sensor_msgs::PointCloud2ConstPtr& msg2) {
    VelodynePointCloud cloud1, cloud2;
    fromROSMsg(*msg1, cloud1);
    fromROSMsg(*msg2, cloud2);

    cloud1.removeNanPoints();
    cloud2.removeNanPoints();

    if(cloud1.size() > MIN_REQ_POINTS && cloud2.size() > MIN_REQ_POINTS) {
      cloud1.setImageLikeAxisFromKitti();
      cloud2.setImageLikeAxisFromKitti();

      saveCloud(cloud1, 1);
      saveCloud(cloud2, 2);
      cloud_counter++;

      findSaveTransform(cloud1.getAxisCorrection());
      out_times << fixed << msg1->header.stamp.toSec() << " " << fixed << msg2->header.stamp.toSec() << endl;
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, NODE_NAME, ros::init_options::AnonymousName);
  NodeHandle nh("~");

  string out_dir(".");
  nh.getParam("out_dir", out_dir);
  string topic_1("/velodyne_points");
  nh.getParam("topic_name", topic_1);
  string topic_2("/velodyne2/velodyne_points");
  nh.getParam("topic_name", topic_2);
  string frame_id_1("velodyne");
  nh.getParam("frame_id", frame_id_1);
  string frame_id_2("velodyne2");
  nh.getParam("frame_id", frame_id_2);

  message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne1_sub(nh, topic_1, 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne2_sub(nh, topic_2, 10);

  Parser parser(frame_id_1, frame_id_2, out_dir);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne1_sub, velodyne2_sub);
  sync.registerCallback(boost::bind(&Parser::callback, &parser, _1, _2));

  spin();

  return EXIT_SUCCESS;
}
