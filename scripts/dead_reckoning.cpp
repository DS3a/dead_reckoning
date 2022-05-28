#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "LinearMath/btMatrix3x3.h"
#include "sensor_msgs/Imu.h"
#include <math.h>


double prev_imu_ping;
double interval = 0.2; // seconds

double v_x, v_y, v_z;
double start_time = 0;

int v_flag = 0;

sensor_msgs::Imu remove_gravity(sensor_msgs::Imu &msg, double g) {
/*
 * subtracts the acceleration due to gravity from
 * the odometry message so that it
 * can be used in dead reckoning
 */

  sensor_msgs::Imu no_g = msg;

  tf::Quaternion quat;
  quat.x = msg.orientation.x;
  quat.y = msg.orientation.y;
  quat.z = msg.orientation.z;
  quat.w = msg.orientation.w;

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  no_g.linear_acceleration.x -= g*cos(roll);
  no_g.linear_acceleration.y -= g*cos(pitch);
  no_g.linear_acceleration.z -= g*cos(yaw);

  return no_g;
}

void imu_cb(sensor_msgs::Imu &msg) {
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

// ros::Publisher 

  while(1) {

    ros::spinOnce();
  }
  return 0;
}

