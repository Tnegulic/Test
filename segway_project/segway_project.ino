#include "turtlebot3_segway.h"

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
cIMU IMU;

/*******************************************************************************
* Declaration for Filter
*******************************************************************************/
float filterFrequency = 1.0;    // Hz
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

float angle[3] = {0.0,0.0,0.0}; //roll, pitch, yaw

// initial angle offset
float angle_offset = 10.18;

// keyboard input
char keyboard;

// Set PID gain
float p_gain = 1600.0;
float i_gain = 4200.0;
float d_gain = 120.0;

float control_output = 0.0;

void setup()
{
  // Setting for Dynamixel motors
  motor_driver.init();
  nh.initNode();
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);

  // Initialization IMU
  imuInit();
}

void loop()
{
  if (IMU.update() > 0){
    getAngle(angle);

    imu_msg.header.stamp    = nh.now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = IMU.gyroData[0];
    imu_msg.angular_velocity.y = IMU.gyroData[1];
    imu_msg.angular_velocity.z = IMU.gyroData[2];
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration.x = IMU.accData[0];
    imu_msg.linear_acceleration.y = IMU.accData[1];
    imu_msg.linear_acceleration.z = IMU.accData[2];
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation.w = IMU.quat[0];
    imu_msg.orientation.x = IMU.quat[1];
    imu_msg.orientation.y = IMU.quat[2];
    imu_msg.orientation.z = IMU.quat[3];

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;

    imu_pub.publish(&imu_msg);
  
  
  }

/*******************************************************************************
* Get angle from IMU
*******************************************************************************/
void getAngle(float angle[3])
{
  float roll, pitch, yaw;

  roll  = IMU.rpy[0];
  pitch = IMU.rpy[1];
  yaw   = IMU.rpy[2];

  angle[0] = lowpassFilter.input(pitch) + angle_offset;
  angle[1] = pitch + angle_offset;
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTOL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlSegway);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Initialization of IMU
*******************************************************************************/
void imuInit()
{
  IMU.begin();

  IMU.SEN.acc_cali_start();
  while( IMU.SEN.acc_cali_get_done() == false )
  {
    IMU.update();
  }

  // Start Dynamixel Control Interrupt
  startDynamixelControlInterrupt();
}

/*******************************************************************************
* Control segway PWM
*******************************************************************************/
void controlSegway(void)
{
  bool dxl_comm_result = false;
  static float control_input = 0.0;

  static float cur_error = 0.0, pre_error = 0.0, integral = 0.0, derivative = 0.0;
  static float diff_time = 0.007;

  static int16_t cnt = 0;           // timer counter

  cur_error  = control_input - angle[0];
  integral   = integral + (cur_error * diff_time);
  derivative = (cur_error - pre_error) / diff_time;

  if (cnt > 500)
  {
    integral = 0.0;
    cnt = 0;
  }
  else
  {
    cnt++;
  }

  control_output = p_gain * cur_error +
                   i_gain * integral  +
                   d_gain * derivative;

  if (control_output >= PWM_LIMIT)
  {
    control_output = PWM_LIMIT;
  }
  else if (control_output <= (-1) * PWM_LIMIT)
  {
    control_output = (-1) * PWM_LIMIT;
  }
  pre_error = cur_error;
  control_output = (-1) * control_output;
  dxl_comm_result = motor_driver.controlMotor((int64_t)control_output, (int64_t)control_output);
  if (dxl_comm_result == false)
    return;
}
