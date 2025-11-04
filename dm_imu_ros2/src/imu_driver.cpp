#include "dm_imu/imu_driver.hpp"
#include "dm_imu/bsp_crc.h"
#include <cmath>

namespace dmbot_serial
{

ssize_t DmImu::serial_write(const uint8_t* data, size_t len)
{
  return write(serial_fd, data, len);
}

ssize_t DmImu::serial_read(uint8_t* data, size_t len)
{
  size_t total_read = 0;
  while (total_read < len && rclcpp::ok())
  {
    ssize_t n = read(serial_fd, data + total_read, len - total_read);
    if (n > 0) {
      total_read += n;
    } else if (n < 0) {
      RCLCPP_ERROR(this->get_logger(), "Read error");
      return -1;
    }
  }
  return total_read;
}

bool DmImu::serial_is_open()
{
  return serial_fd >= 0;
}

DmImu::DmImu() : Node("dm_imu_node"), serial_fd(-1)
{
  this->declare_parameter("port", "/dev/ttyACM0");
  this->declare_parameter("baud", 921600);
  
  this->get_parameter("port", imu_serial_port);
  this->get_parameter("baud", imu_serial_baud);
  
  imu_msg.header.frame_id = "imu_link";
  
  init_imu_serial();
  
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  
  enter_setting_mode();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  turn_on_accel();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  turn_on_gyro();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  turn_on_euler();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  turn_off_quat();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  set_output_1000HZ();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  save_imu_para();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  exit_setting_mode();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  tcflush(serial_fd, TCIOFLUSH);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  rec_thread = std::thread(&DmImu::get_imu_data_thread, this);
  
  RCLCPP_INFO(this->get_logger(), "DM IMU initialized successfully");
}

DmImu::~DmImu()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down DM IMU");
  stop_thread_ = true;
  
  if (rec_thread.joinable()) {
    rec_thread.join();
  }
  
  if (serial_is_open()) {
    close(serial_fd);
  }
}

void DmImu::init_imu_serial()
{
  serial_fd = open(imu_serial_port.c_str(), O_RDWR | O_NOCTTY);
  
  if (serial_fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open IMU serial port: %s", imu_serial_port.c_str());
    throw std::runtime_error("Serial port initialization failed");
  }
  
  struct termios options;
  tcgetattr(serial_fd, &options);
  
  cfsetispeed(&options, B921600);
  cfsetospeed(&options, B921600);
  
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag |= CREAD | CLOCAL;
  
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_oflag &= ~OPOST;
  
  options.c_cc[VMIN] = 1;
  options.c_cc[VTIME] = 10;
  
  tcsetattr(serial_fd, TCSANOW, &options);
  tcflush(serial_fd, TCIOFLUSH);
  
  RCLCPP_INFO(this->get_logger(), "IMU Serial Port opened: %s @ 921600", imu_serial_port.c_str());
}

void DmImu::enter_setting_mode()
{
  uint8_t txbuf[4] = {0xAA, 0x06, 0x01, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::turn_on_accel()
{
  uint8_t txbuf[4] = {0xAA, 0x01, 0x14, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::turn_on_gyro()
{
  uint8_t txbuf[4] = {0xAA, 0x01, 0x15, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::turn_on_euler()
{
  uint8_t txbuf[4] = {0xAA, 0x01, 0x16, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::turn_off_quat()
{
  uint8_t txbuf[4] = {0xAA, 0x01, 0x07, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::set_output_1000HZ()
{
  uint8_t txbuf[5] = {0xAA, 0x02, 0x01, 0x00, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::save_imu_para()
{
  uint8_t txbuf[4] = {0xAA, 0x03, 0x01, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::exit_setting_mode()
{
  uint8_t txbuf[4] = {0xAA, 0x06, 0x00, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::restart_imu()
{
  uint8_t txbuf[4] = {0xAA, 0x00, 0x00, 0x0D};
  for (int i = 0; i < 5; i++) {
    serial_write(txbuf, sizeof(txbuf));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::get_imu_data_thread()
{
  int error_num = 0;
  
  while (rclcpp::ok() && !stop_thread_)
  {
    if (!serial_is_open()) {
      RCLCPP_WARN(this->get_logger(), "IMU serial port is not open");
      continue;
    }
    
    ssize_t n = serial_read((uint8_t*)(&receive_data.FrameHeader1), 4);
    if (n != 4) continue;
    
    if (receive_data.FrameHeader1 == 0x55 && 
        receive_data.flag1 == 0xAA && 
        receive_data.slave_id1 == 0x01 && 
        receive_data.reg_acc == 0x01)
    {
      n = serial_read((uint8_t*)(&receive_data.accx_u32), 53);
      if (n != 53) continue;
      
      if (Get_CRC16((uint8_t*)(&receive_data.FrameHeader1), 16) == receive_data.crc1)
      {
        data.accx = *((float*)(&receive_data.accx_u32));
        data.accy = *((float*)(&receive_data.accy_u32));
        data.accz = *((float*)(&receive_data.accz_u32));
      }
      
      if (Get_CRC16((uint8_t*)(&receive_data.FrameHeader2), 16) == receive_data.crc2)
      {
        data.gyrox = *((float*)(&receive_data.gyrox_u32));
        data.gyroy = *((float*)(&receive_data.gyroy_u32));
        data.gyroz = *((float*)(&receive_data.gyroz_u32));
      }
      
      if (Get_CRC16((uint8_t*)(&receive_data.FrameHeader3), 16) == receive_data.crc3)
      {
        data.roll = *((float*)(&receive_data.roll_u32));
        data.pitch = *((float*)(&receive_data.pitch_u32));
        data.yaw = *((float*)(&receive_data.yaw_u32));
      }
      
      imu_msg.header.stamp = this->now();
      
      tf2::Quaternion q;
      q.setRPY(data.roll * M_PI / 180.0, data.pitch * M_PI / 180.0, data.yaw * M_PI / 180.0);
      
      imu_msg.orientation.x = q.x();
      imu_msg.orientation.y = q.y();
      imu_msg.orientation.z = q.z();
      imu_msg.orientation.w = q.w();
      
      imu_msg.angular_velocity.x = data.gyrox;
      imu_msg.angular_velocity.y = data.gyroy;
      imu_msg.angular_velocity.z = data.gyroz;
      
      imu_msg.linear_acceleration.x = data.accx;
      imu_msg.linear_acceleration.y = data.accy;
      imu_msg.linear_acceleration.z = data.accz;
      
      imu_pub_->publish(imu_msg);
      
      error_num = 0;
    }
    else
    {
      error_num++;
      if (error_num > 1200) {
        RCLCPP_WARN(this->get_logger(), "Failed to get correct IMU data");
        error_num = 0;
      }
    }
  }
}

}
