#include <memory>
#include <string>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include "can_plugins2/msg/frame.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "../include/can_utils.hpp"


using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

struct BIDNumber
{
  uint32_t solenoidValve      = 0x100;
  uint32_t centerWheel        = 0x110;
  uint32_t centerSteering     = 0x210;
  uint32_t lowerLeftWheel     = 0x120;
  uint32_t lowerLeftSteering  = 0x220;
  uint32_t lowerRightWheel    = 0x130;
  uint32_t lowerRightSteering = 0x230;
};

class pubsub : public rclcpp::Node
{
  public:
    pubsub() : Node("steering4_node"), count_(0)
    {
      publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
      subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&pubsub::joy_callback, this, _1));
    }

  private:
    BIDNumber bidNumber;
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    size_t count_;
    int count = 0;
};

void pubsub::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {  
    RCLCPP_INFO(this->get_logger(), "I heard:");

    if(msg->buttons[3] == true)
    {
      publisher_->publish(get_frame(pubsub::bidNumber.centerWheel,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftWheel,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerRightWheel,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.centerSteering,static_cast<uint8_t>(3)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftSteering,static_cast<uint8_t>(3)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerRightSteering,static_cast<uint8_t>(3)));
    }

    if(msg->buttons[2] == true)
    {
      publisher_->publish(get_frame(pubsub::bidNumber.centerWheel,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftWheel,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerRightWheel,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(pubsub::bidNumber.centerSteering,static_cast<uint8_t>(6)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftSteering,static_cast<uint8_t>(6)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerRightSteering,static_cast<uint8_t>(6)));
    }

    if(msg->buttons[1] == true)
    {
      publisher_->publish(get_frame(pubsub::bidNumber.centerWheel,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftWheel,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerRightWheel,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.centerSteering,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftSteering,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerRightSteering,static_cast<uint8_t>(0)));
    }

    float x= -(msg->axes[0]);
    float y=  (msg->axes[1]);
    float theta = acosf(x/sqrt(x*x+y*y));
/*
    if(msg->buttons[4]==true){
      publisher_->publish(get_frame((pubsub::bidNumber.upperRightWheel+1), M_PI));
      publisher_->publish(get_frame((pubsub::bidNumber.upperLeftWheel+1), M_PI));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftWheel+1), M_PI));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerRightwheel+1), M_PI));
      //wheel速度制御

      publisher_->publish(get_frame((pubsub::bidNumber.upperRightsteering+1), M_PI/4));
      publisher_->publish(get_frame((pubsub::bidNumber.upperLeftsteering+1), M_PI*3/4));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftsteering+1), M_PI*5/4));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerRightsteering+1), M_PI*7/4));
      //steering制御
    }
    //↑左回転
    else if(msg->buttons[5]==true){
      publisher_->publish(get_frame((pubsub::bidNumber.upperRightWheel+1), -M_PI));
      publisher_->publish(get_frame((pubsub::bidNumber.upperLeftWheel+1), -M_PI));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftWheel+1), -M_PI));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerRightwheel+1), -M_PI));
      //wheel速度制御

      publisher_->publish(get_frame((pubsub::bidNumber.upperRightsteering+1), M_PI/4));
      publisher_->publish(get_frame((pubsub::bidNumber.upperLeftsteering+1), M_PI*3/4));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftsteering+1), M_PI*5/4));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerRightsteering+1), M_PI*7/4));
      //steering制御
    }
    //右回転
*/
    if(x>0.0005 || y>0.0005){
      publisher_->publish(shirasu_frame((pubsub::bidNumber.centerWheel+1), M_PI*2.0f*(x*x+y*y)));
      publisher_->publish(shirasu_frame((pubsub::bidNumber.lowerLeftWheel+1), M_PI*2.0f*(x*x+y*y)));
      publisher_->publish(shirasu_frame((pubsub::bidNumber.lowerRightWheel+1), M_PI*2.0f*(x*x+y*y)));
      //wheel速度制御

      publisher_->publish(shirasu_frame((pubsub::bidNumber.centerSteering+1), theta-M_PI/2));
      publisher_->publish(shirasu_frame((pubsub::bidNumber.lowerLeftSteering+1), theta-M_PI/2));
      publisher_->publish(shirasu_frame((pubsub::bidNumber.lowerRightSteering+1), theta-M_PI/2));
      //steering制御

      RCLCPP_INFO(this->get_logger(), "Publishing:bokuha warukunai!");
      count = 0;
    }
    else{
      if(count == 0){
        publisher_->publish(shirasu_frame((pubsub::bidNumber.centerWheel+1), 0.0f));
        publisher_->publish(shirasu_frame((pubsub::bidNumber.lowerLeftWheel+1), 0.0f));
        publisher_->publish(shirasu_frame((pubsub::bidNumber.lowerRightWheel+1), 0.0f));
        count = 1;
      }
    }
  }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pubsub>());
  rclcpp::shutdown();
  return 0;
}