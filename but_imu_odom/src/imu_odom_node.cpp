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
 * Date: 04/03/2017
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

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace ros;
using namespace tf;

class AccelerationIntegration {
public:
  AccelerationIntegration(void) {
    prev_acc.setZero();
    prev_vel.setZero();
    prev_time = -1.0;
  }

  Vector3 integrate(const Vector3 &new_acc, double new_time) {
    Vector3 new_vel; new_vel.setZero();
    Vector3 delta_position; delta_position.setZero();

    if(prev_time > 0) {
      double delta_time = new_time - prev_time;
      new_vel = prev_vel + (new_acc + prev_acc) / 2.0 * delta_time;
      delta_position = (new_vel + prev_vel) / 2.0 * delta_time;
    }

    prev_acc = new_acc;
    prev_vel = new_vel;
    prev_time = new_time;
    return delta_position;
  }

private:
  Vector3 prev_acc;
  Vector3 prev_vel;
  double prev_time;
};

class ImuOdometry {
public:
  ImuOdometry(NodeHandle &nh) :
    pose_publisher(nh.advertise<geometry_msgs::PoseStamped>("pose", 10)),
    gravity_publisher(nh.advertise<geometry_msgs::PoseStamped>("pose", 10)),
    raw_imu_subscriber(nh.subscribe<sensor_msgs::Imu>("/raw_imu", 10, &ImuOdometry::process, this)),
    need_msgs_to_init(MSGS_TO_INIT),
    gravity(0, 0, 0),
    init_orientation(0, 0, 0, 1.0) {
    position.setZero();
  }

  void process(const sensor_msgs::Imu::ConstPtr& imuIn) {
    Quaternion orientation;
    quaternionMsgToTF(imuIn->orientation, orientation);
    Vector3 raw_acceleration;
    vector3MsgToTF(imuIn->linear_acceleration, raw_acceleration);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = imuIn->header.frame_id;
    pose_msg.header.stamp = imuIn->header.stamp;
    tf::quaternionTFToMsg(orientation, pose_msg.pose.orientation);

    if(need_msgs_to_init != 0) {
      // calibration - estimation of the initial orientation and gravity vector
      static const float factor = 1.0/MSGS_TO_INIT;
      init_orientation = orientation;
      gravity += raw_acceleration*factor;
      need_msgs_to_init--;
    } else {
      Matrix3x3 relative_orientation(init_orientation.inverse()*orientation);
      Vector3 relative_gravity = relative_orientation.inverse() * gravity;
      Vector3 acceleration = raw_acceleration - relative_gravity;
      Vector3 translation = integration.integrate(acceleration, imuIn->header.stamp.toSec());
      position += Matrix3x3(orientation) * translation;
      pointTFToMsg(position, pose_msg.pose.position);
    }

    pose_publisher.publish(pose_msg);
  }
private:
  Publisher pose_publisher, gravity_publisher;
  Subscriber raw_imu_subscriber;

  Vector3 gravity;
  Quaternion init_orientation;
  static const int MSGS_TO_INIT = 30;
  int need_msgs_to_init;

  AccelerationIntegration integration;
  Point position;
};

int main(int argc, char** argv) {
  init(argc, argv, "but_imu_odom");
  NodeHandle nh("~");

  ImuOdometry odometry(nh);
  spin();

  return EXIT_SUCCESS;
}

