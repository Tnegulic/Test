#include "turtlebot3_segway_motor_driver.h"

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  left_wheel_id_(DXL_LEFT_ID),
  right_wheel_id_(DXL_RIGHT_ID)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  closeDynamixel();
}

bool Turtlebot3MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    //ERROR_PRINT("Port is opened");
  }
  else
  {
    //ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    //ERROR_PRINT("Baudrate is set");
  }
  else
  {
   // ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }
  int8_t dxl_comm_result_;
  dxl_comm_result_ = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_ID, ADDR_X_PWM, LEN_X_PWM);
  dxl_comm_result_ = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_ID, ADDR_X_PWM, LEN_X_PWM);

  // Enable Dynamixel Torque
  setTorque(left_wheel_id_, true);
  setTorque(right_wheel_id_, true);

  groupSyncWritePWM_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_PWM, LEN_X_GOAL_PWM);

  return true;
}

bool Turtlebot3MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}

void Turtlebot3MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(left_wheel_id_, false);
  setTorque(right_wheel_id_, false);

  // Close port
  portHandler_->closePort();
}

bool Turtlebot3MotorDriver::controlMotor(int64_t left_wheel_value, int64_t right_wheel_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWritePWM_->addParam(left_wheel_id_, (uint8_t*)&left_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePWM_->addParam(right_wheel_id_, (uint8_t*)&right_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWritePWM_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWritePWM_->clearParam();
  return true;
}
