#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath> // for std::isinf

#include "PDD_Include.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#define BUFFER_SIZE 256
#define PUBLISH_TIME_MS 1
#define WRITE_TIME_MS 50
#define ELIMINATE_INF true
#define DVL_IP "192.168.1.100"
#define DVL_READ_PORT 1034
#define DVL_WRITE_PORT 1033

bool firstEnsemble = false;

class DvlPublisher : public rclcpp::Node
{
  public:
    DvlPublisher() : Node("dvl_publisher")
    {
      publisher_point_ = this->create_publisher<geometry_msgs::msg::Point>("/dvl/point", 10);
      publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/dvl/twist", 10);
      subscriber_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/ahrs/quaternion",
            10,
            std::bind(&DvlPublisher::ahrs_callback, this, std::placeholders::_1));
      publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_TIME_MS),
            std::bind(&DvlPublisher::publish_timer_callback, this));
      write_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(WRITE_TIME_MS),
            std::bind(&DvlPublisher::write_timer_callback, this));
      init_ports();
      init_dvl();
      init_decoder();

      RCLCPP_INFO(this->get_logger(), "Running DVL measurements");
    }

    ~DvlPublisher()
    {
      delete decoder_;
      decoder_ = nullptr;
      delete[] buffer_;
      buffer_ = nullptr;
      close(fd_read_);
      close(fd_write_);
    }

  private:
    void init_ports()
    {
      fd_read_ = socket(AF_INET, SOCK_STREAM, 0);
      if (fd_read_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error creating socket (DVL read)");
        return;
      }

      // Set up server address
      server_addr_.sin_family = AF_INET;
      server_addr_.sin_port = htons(DVL_READ_PORT);
      if (inet_pton(AF_INET, DVL_IP, &server_addr_.sin_addr) <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid address/ Address not supported (DVL read)");
        close(fd_read_);
        return;
      }

      // Connect to DVL read
      if (connect(fd_read_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed (DVL read)");
        close(fd_read_);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Connected to DVL Output at %s:%d", DVL_IP, DVL_READ_PORT);

      fd_write_ = socket(AF_INET, SOCK_STREAM, 0);
      if (fd_write_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error creating socket (DVL write)");
        return;
      }

      // Set up server address
      server_addr_.sin_family = AF_INET;
      server_addr_.sin_port = htons(DVL_WRITE_PORT);
      if (inet_pton(AF_INET, DVL_IP, &server_addr_.sin_addr) <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid address/ Address not supported (DVL write)");
        close(fd_write_);
        return;
      }

      // Connect to DVL write
      if (connect(fd_write_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed (DVL write)");
        close(fd_write_);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Connected to DVL Command at %s:%d", DVL_IP, DVL_WRITE_PORT);
    }

    void init_decoder()
    {
      tdym::PDD_SetInvalidValue(INFINITY); //force invalid data to be infinity
      tdym::PDD_InitializeDecoder(decoder_); //init decoder
    }

    void init_dvl()
    {
      dprintf(fd_write_, "===\r");
      sleep(1);
      dprintf(fd_write_, "CR1\r");
      sleep(1);
      dprintf(fd_write_, "EX11111\r");
      sleep(1);
      dprintf(fd_write_, "EA-13500\r");
      sleep(1);
      dprintf(fd_write_, "BJ100100000\r");
      sleep(1);
      dprintf(fd_write_, "CS\r");
      sleep(1);
      ready_ = true;
    }

    void publish_timer_callback()
    {
      std::memset(buffer_, 0, BUFFER_SIZE);
      int data = read(fd_read_, buffer_, sizeof(buffer_));

      if (data > 0) {
        tdym::PDD_AddDecoderData(decoder_, buffer_, (int)data);
        int found = 0;
        do
        {
          found = tdym::PDD_GetPD0Ensemble(decoder_, &ens_);
          if (found)
          {
            tdym::PDD_VariableLeader* varLeader = ens_.varLeader[0];
            if (varLeader) {
              RCLCPP_INFO(this->get_logger(), "HPR: %d, %d, %d", varLeader->heading, varLeader->pitch, varLeader->roll);
            }

            tdym::PDD_BTHighResVelocity* hiResVelocity = ens_.btHiResVel;
            if (!hiResVelocity) continue;

            int32_t *velocity = hiResVelocity->velocity;
            if (ELIMINATE_INF){
              if(std::isinf(velocity[0]) || std::isinf(velocity[1]) || std::isinf(velocity[2])){
                continue; //skip if any are infinity!
              }
            }

            geometry_msgs::msg::Twist twistMessage;
            twistMessage.linear.x = double(velocity[1])/100000;
            twistMessage.linear.y = double(velocity[0])/100000;
            twistMessage.linear.z = double(velocity[2])/100000;
            publisher_twist_->publish(twistMessage);

            int32_t *distance = hiResVelocity->distMadeGood;

            geometry_msgs::msg::Point pointMessage;
            pointMessage.x = double(distance[1])/100000;
            pointMessage.y = double(distance[0])/100000;
            pointMessage.z = double(distance[2])/100000;
            publisher_point_->publish(pointMessage);

            if (firstEnsemble == false) {
              RCLCPP_INFO(this->get_logger(), "DVL node is publishing succesfully");
              firstEnsemble = true; 
            }
          }
        } while (found > 0);
      }
    }

    void write_timer_callback() {
      if (!ready_) {
        return;
      }
      switch (write_state_) {
        case 0:
          dprintf(fd_write_, "EH%d,1\r", yaw_);
          break;
        case 1:
          dprintf(fd_write_, "EP%+d,%+d,1\r", pitch_, roll_);
          break;
      }
      write_state_ = (write_state_ + 1) % 2;
    }
    
    void ahrs_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg){
      // Conversion to Euler Angles from Quaternion
      // roll (x-axis rotation)
      double sinr_cosp = 2 * (msg->w * msg->x + msg->y * msg->z);
      double cosr_cosp = 1 - 2 * (msg->x * msg->x + msg->y * msg->y);
      double roll = atan2(sinr_cosp, cosr_cosp);

      // pitch (y-axis rotation)
      double sinp = 2 * (msg->w * msg->y - msg->z * msg->x);
      double pitch;
      if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
        pitch = asin(sinp);

      // yaw (z-axis rotation)
      double siny_cosp = 2 * (msg->w * msg->z + msg->x * msg->y);
      double cosy_cosp = 1 - 2 * (msg->y * msg->y + msg->z * msg->z);
      double yaw = atan2(siny_cosp, cosy_cosp);

      int yaw_centi = yaw * 18000 / M_PI;
      if(yaw_centi < 0){
          yaw_centi += 36000;
      }
      yaw_ = yaw_centi;

      int pitch_centi = pitch * 18000 / M_PI;
      int roll_centi = roll * 18000 / M_PI;
      pitch_ = pitch_centi;
      roll_ = roll_centi;
    }

  //Variables
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_point_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscriber_;
  int fd_read_;
  int fd_write_;
  struct sockaddr_in server_addr_;
  unsigned char* buffer_ = new unsigned char[BUFFER_SIZE];
  tdym::PDD_Decoder* decoder_ = new tdym::PDD_Decoder();
  tdym::PDD_PD0Ensemble ens_;
  int yaw_ = 0;
  int pitch_ = 0;
  int roll_ = 0;
  int write_state_ = 0;
  bool ready_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlPublisher>());
  rclcpp::shutdown();
  return 0;
}

