#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "sensor_msgs/Imu.h"
#include <math.h>


double prev_imu_ping;
double interval = 0.2; // seconds

double v_x, v_y, v_z;
double start_time = 0;

int v_flag = 0;

sensor_msgs::Imu remove_gravity(const sensor_msgs::Imu::ConstPtr &msg, double g) {
/*
 * subtracts the acceleration due to gravity from
 * the odometry message so that it
 * can be used in dead reckoning
 */

  sensor_msgs::Imu no_g;

  no_g.linear_acceleration.x = msg->linear_acceleration.x;
  no_g.linear_acceleration.y = msg->linear_acceleration.y;
  no_g.linear_acceleration.z = msg->linear_acceleration.z;

  tf::Quaternion quat(msg->orientation.x,
                      msg->orientation.y,
                      msg->orientation.z,
                      msg->orientation.w);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  no_g.linear_acceleration.x -= g*cos(roll);
  no_g.linear_acceleration.y -= g*cos(pitch);
  no_g.linear_acceleration.z -= g*cos(yaw);

  return no_g;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
/*
 *Call back for the IMU topic
 */
  double curr_time = ros::Time::now().toSec();
  double delta = curr_time - prev_imu_ping;
  prev_imu_ping = curr_time;
  sensor_msgs::Imu no_g_data = remove_gravity(msg, 9.82);

  if (start_time == 0) {
    v_x = 0;
    v_y = 0;
    v_z = 0;
    start_time = ros::Time::now().toSec();
    v_flag = 0;
  }

  if (ros::Time::now().toSec() - start_time >= interval) {
    start_time = 0;
    v_flag = 1;
  }

  v_x += delta*no_g_data.linear_acceleration.x;
  v_y += delta*no_g_data.linear_acceleration.y;
  v_z += delta*no_g_data.linear_acceleration.z;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dead_reckoning_node");
  ros::NodeHandle nh;


  std::string imu_topic;

  if (nh.hasParam("/imu_topic")) {
    nh.getParam("/imu_topic", imu_topic);
  } else {
    imu_topic = "/imu/data";
  }
  ros::Publisher vel_pos_publisher = nh.advertise<nav_msgs::Odometry>("/pos_vel", 100);
  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, imu_cb);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.child_frame_id = "base_link";

  while(1) {
    if (v_flag == 1) {
      odom_msg.twist.twist.linear.x = v_x;
      odom_msg.twist.twist.linear.y = v_y;
      odom_msg.twist.twist.linear.z = v_z;
      vel_pos_publisher.publish(odom_msg);
    }

    ros::spinOnce();
  }
  return 0;
}

