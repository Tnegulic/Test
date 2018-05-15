#ifndef TURTLEBOT3_EXAMPLE06_SEGWAY_CONFIG_H
#define TURTLEBOT3_EXAMPLE06_SEGWAY_CONFIG_H
//ros basic
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <IMU.h>
#include <Filters.h>

#include "turtlebot3_segway_motor_driver.h"

#define DEBUG

#define PWM_LIMIT                       885
#define CONTOL_PERIOD                   7000     // in microseconds

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

void startDynamixelControlInterrupt();
void imuInit();
void getAngle(float angle[3]);
void controlSegway();

#endif // TURTLEBOT3_EXAMPLE06_SEGWAY_CONFIG_H
