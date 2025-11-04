#ifndef _IMU_DRIVER_HPP_
#define _IMU_DRIVER_HPP_

#include <iostream>
#include <thread>
#include <memory>
#include <atomic>
#include <array>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dmbot_serial
{
#pragma pack(1)
  typedef struct
  {
    uint8_t FrameHeader1;
    uint8_t flag1;
    uint8_t slave_id1;
    uint8_t reg_acc;
    uint32_t accx_u32;
    uint32_t accy_u32;
    uint32_t accz_u32;
    uint16_t crc1;
    uint8_t FrameEnd1;

    uint8_t FrameHeader2;
    uint8_t flag2;
    uint8_t slave_id2;
    uint8_t reg_gyro;
    uint32_t gyrox_u32;
    uint32_t gyroy_u32;
    uint32_t gyroz_u32;
    uint16_t crc2;
    uint8_t FrameEnd2;

    uint8_t FrameHeader3;
    uint8_t flag3;
    uint8_t slave_id3;
    uint8_t reg_euler;
    uint32_t roll_u32;
    uint32_t pitch_u32;
    uint32_t yaw_u32;
    uint16_t crc3;
    uint8_t FrameEnd3;
  } IMU_Receive_Frame;
#pragma pack()

  typedef struct
  {
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
    float roll;
    float pitch;
    float yaw;
  } IMU_Data;

  class DmImu : public rclcpp::Node
  {
  public:
    DmImu();
    ~DmImu();
    
  private:
    void init_imu_serial(); 
    void get_imu_data_thread();
    
    void enter_setting_mode();
    void turn_on_accel();
    void turn_on_gyro();
    void turn_on_euler();
    void turn_off_quat();
    void set_output_1000HZ();
    void save_imu_para();
    void exit_setting_mode();
    void restart_imu();
    
    // 串口操作
    ssize_t serial_write(const uint8_t* data, size_t len);
    ssize_t serial_read(uint8_t* data, size_t len);
    bool serial_is_open();
    
    int imu_serial_baud;
    std::string imu_serial_port;
    int serial_fd;  // 串口文件描述符
    std::thread rec_thread;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    sensor_msgs::msg::Imu imu_msg;
    
    std::atomic<bool> stop_thread_{false};
    IMU_Receive_Frame receive_data{};
    IMU_Data data{};
  };
}
#endif

