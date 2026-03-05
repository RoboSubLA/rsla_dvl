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
#include "geometry_msgs/msg/quaternion.hpp"

#define BUFFER_SIZE 256
#define PUBLISH_TIME_MS 1
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
      publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/dvl/point", 10);
      subscriber_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/ahrs/quaternion",
            10,
            std::bind(&DvlPublisher::ahrs_callback, this, std::placeholders::_1));
      timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_TIME_MS),
            std::bind(&DvlPublisher::timer_callback, this));
      init_ports();
      init_dvl();
      init_decoder();

      RCLCPP_INFO(this->get_logger(), "Running DVL measurements");
    }

    ~DvlPublisher()
    {
      delete decoder;
      decoder = nullptr;
      delete[] buffer;
      buffer = nullptr;
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
      tdym::PDD_InitializeDecoder(decoder); //init decoder
    }

    void init_dvl()
    {
      dprintf(fd_write_, "===\r");
      dprintf(fd_write_, "CR1\r");
      dprintf(fd_write_, "EA+04500\r");
      dprintf(fd_write_, "BJ100100000\r");
      dprintf(fd_write_, "CS\r");
    }

    void timer_callback()
    {
      std::memset(buffer, 0, BUFFER_SIZE);
      int data = read(fd_read_, buffer, sizeof(buffer));

      if (data > 0) {
        tdym::PDD_AddDecoderData(decoder, buffer, (int)data);
        int found = 0;
        do
        {
          found = tdym::PDD_GetPD0Ensemble(decoder, &ens);
          if (found)
          {
            tdym::PDD_BTHighResVelocity* hiResVelocity = ens.btHiResVel;
            if (!hiResVelocity) continue;

            int32_t *distance = hiResVelocity->distMadeGood;
            if (ELIMINATE_INF){
              if(std::isinf(distance[0]) || std::isinf(distance[1]) || std::isinf(distance[2])){
                continue; //skip if any are infinity!
              }
            }

            geometry_msgs::msg::Point pointMessage;
            pointMessage.x = double(distance[0])/100000;
            pointMessage.y = double(distance[1])/100000;
            pointMessage.z = double(distance[2])/100000;

            publisher_->publish(pointMessage);
            if (firstEnsemble == false){
              RCLCPP_INFO(this->get_logger(), "DVL node is publishing succesfully");
              firstEnsemble = true; 
            }
          }
        } while (found > 0);
      }
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

      dprintf(fd_write_, "EH%d,1\r", (int)(yaw * 18000 / M_PI));
      dprintf(fd_write_, "EP%d,%d,1\r", (int)(pitch * 18000 / M_PI), (int)(roll * 18000 / M_PI));
    }

  //Variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscriber_;
  int fd_read_;
  int fd_write_;
  struct sockaddr_in server_addr_;
  unsigned char* buffer = new unsigned char[BUFFER_SIZE];
  tdym::PDD_Decoder* decoder = new tdym::PDD_Decoder();
  tdym::PDD_PD0Ensemble ens;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlPublisher>());
  rclcpp::shutdown();
  return 0;
}
