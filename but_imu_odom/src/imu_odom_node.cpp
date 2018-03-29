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

#include <vector>
#include <math.h>
#include <numeric>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace ros;
using namespace tf;
using namespace std;

static const float G = 9.81;
static const float TO_DEG = 180.0/M_PI;

void print(const string &name, const Vector3 &value) {
  cerr << name << ": [" << value.x() << ", " << value.y() << ", " << value.z() << "]" << endl;
}

class ImuOdometry {
public:
  ImuOdometry(NodeHandle &nh) :
    pose_publisher(nh.advertise<geometry_msgs::PoseStamped>("pose", 10)),
    raw_imu_subscriber(nh.subscribe<sensor_msgs::Imu>("/imu", 10, &ImuOdometry::process, this)),
    position(0.0, 0.0, 0.0),
    first_time(0.0) {
  }

  void process(const sensor_msgs::Imu::ConstPtr& imuIn) {
    Quaternion orientation;
    quaternionMsgToTF(imuIn->orientation, orientation);
    Vector3 raw_acceleration;
    vector3MsgToTF(imuIn->linear_acceleration, raw_acceleration);
    double time = imuIn->header.stamp.toSec();

    if(first_time < 0.0001) {
      first_time = time;
    }
    cout << time-first_time << " " <<
        raw_acceleration.x()/G << " " << raw_acceleration.y()/G << " " << raw_acceleration.z()/G << " " <<
        imuIn->angular_velocity.x*TO_DEG << " " << imuIn->angular_velocity.y*TO_DEG << " " << imuIn->angular_velocity.z*TO_DEG << endl;

    Vector3 calibrated_acceleration = Matrix3x3(orientation.inverse()) * raw_acceleration;

    if(last_accelerations.size() > 100) {
      Vector3 avg_gravity(0.0, 0.0, 0.0);
      avg_gravity = accumulate(last_accelerations.begin(), last_accelerations.end(), avg_gravity);
      avg_gravity /= last_accelerations.size();

      Vector3 acceleration = calibrated_acceleration - avg_gravity;
      position += acceleration * pow(time-first_time, 2.0);

      /*print("raw", raw_acceleration);
      print("calib", calibrated_acceleration);
      print("gravity", avg_gravity);
      print("acc", acceleration);
      cerr << "--------------------------------------------------------------------------------" << endl;*/

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = imuIn->header.frame_id;
      pose_msg.header.stamp = imuIn->header.stamp;
      tf::quaternionTFToMsg(orientation, pose_msg.pose.orientation);
      pointTFToMsg(position, pose_msg.pose.position);

      pose_publisher.publish(pose_msg);
      last_accelerations.erase(last_accelerations.begin());
    }
    last_accelerations.push_back(calibrated_acceleration);
  }
private:
  Publisher pose_publisher;
  Subscriber raw_imu_subscriber;

  vector<Vector3> last_accelerations;
  double first_time;

  Point position;
};

int main(int argc, char** argv) {
  init(argc, argv, "but_imu_odom");
  NodeHandle nh("~");

  ImuOdometry odometry(nh);
  spin();

  return EXIT_SUCCESS;
}

