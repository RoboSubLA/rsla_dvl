#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath> // for std::isinf

#include "PDD_Include.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

#define BUFFER_SIZE 256
#define PUBLISH_TIME_MS 1
#define ELIMINATE_INF true
#define DVL_IP "192.168.1.100"
#define DVL_PORT 1034

bool firstEnsemble = false;

class DvlPublisher : public rclcpp::Node
{
  public:
    DvlPublisher() : Node("dvl_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/dvl/point", 10);
      timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_TIME_MS),
            std::bind(&DvlPublisher::timer_callback, this));
      init_port();
      init_decoder();

      RCLCPP_INFO(this->get_logger(), "Running DVL measurements");
    }

    ~DvlPublisher()
    {
      delete decoder;
      decoder = nullptr;
      delete[] buffer;
      buffer = nullptr;
      close(fd_);
    }

  private:
    void init_port()
    {
      fd_ = socket(AF_INET, SOCK_STREAM, 0);
      if (fd_ < 0) {
          RCLCPP_ERROR(this->get_logger(), "Error creating socket");
          return;
      }

      // Set up server address
      server_addr_.sin_family = AF_INET;
      server_addr_.sin_port = htons(DVL_PORT);
      if (inet_pton(AF_INET, DVL_IP, &server_addr_.sin_addr) <= 0) {
          RCLCPP_ERROR(this->get_logger(), "Invalid address/ Address not supported");
          close(fd_);
          return;
      }

      // Connect to DVL
      if (connect(fd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
          RCLCPP_ERROR(this->get_logger(), "Connection failed");
          close(fd_);
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Connected to DVL at %s:%d", DVL_IP, DVL_PORT);
    }

    void init_decoder()
    {
      tdym::PDD_SetInvalidValue(INFINITY); //force invalid data to be infinity
      tdym::PDD_InitializeDecoder(decoder); //init decoder
    }

    void timer_callback()
    {
      std::memset(buffer, 0, BUFFER_SIZE);
      int data = read(fd_, buffer, sizeof(buffer));

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

  //Variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  int fd_;
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
