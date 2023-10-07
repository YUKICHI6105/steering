#include <memory>
#include <string>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include "can_plugins2/msg/frame.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "can_plugins2/msg/robomas_frame.hpp"
#include "can_plugins2/msg/robomas_target.hpp"
#include "../include/can_utils.hpp"
#include "../include/robomas_utils.hpp"


using std::placeholders::_1;

struct BIDNumber
{
  uint32_t solenoidValve      = 0x100;
  uint32_t upperRightwheel    = 0x110;
  uint32_t upperRightsteering = 0x210;
  uint32_t upperLeftwheel     = 0x120;
  uint32_t upperLeftsteering  = 0x220;
  uint32_t lowerRightwheel    = 0x130;
  uint32_t lowerRightsteering = 0x230;
  uint32_t lowerLeftwheel     = 0x140;
  uint32_t lowerLeftsteering  = 0x240;
};

class pubsub : public rclcpp::Node
{
  public:
    pubsub() : Node("steering4_node"), count_(0)
    {
      publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
      robomas_pub_ = this->create_publisher<can_plugins2::msg::RobomasFrame>("robomaster", 10);
      robomas_pub1_ = this->create_publisher<can_plugins2::msg::RobomasTarget>("robomas_target1", 10);
      robomas_pub2_ = this->create_publisher<can_plugins2::msg::RobomasTarget>("robomas_target2", 10);
      robomas_pub3_ = this->create_publisher<can_plugins2::msg::RobomasTarget>("robomas_target3", 10);
      robomas_pub4_ = this->create_publisher<can_plugins2::msg::RobomasTarget>("robomas_target4", 10);
      subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&pubsub::joy_callback, this, _1));
    }

  private:
    BIDNumber bidNumber;
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
    rclcpp::Publisher<can_plugins2::msg::RobomasFrame>::SharedPtr robomas_pub_;
    rclcpp::Publisher<can_plugins2::msg::RobomasTarget>::SharedPtr robomas_pub1_;
    rclcpp::Publisher<can_plugins2::msg::RobomasTarget>::SharedPtr robomas_pub2_;
    rclcpp::Publisher<can_plugins2::msg::RobomasTarget>::SharedPtr robomas_pub3_;
    rclcpp::Publisher<can_plugins2::msg::RobomasTarget>::SharedPtr robomas_pub4_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    size_t count_;
    int count = 0;
};

void pubsub::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {  
    RCLCPP_INFO(this->get_logger(), "I heard:");

    if(msg->buttons[3]==true)
    {
      //publisher_->publish(get_frame(pubsub::bidNumber.upperLeftwheel,static_cast<uint8_t>(0)));
      //publisher_->publish(get_frame(pubsub::bidNumber.upperRightwheel,static_cast<uint8_t>(0)));
      //publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftwheel,static_cast<uint8_t>(0)));
      //publisher_->publish(get_frame(pubsub::bidNumber.lowerRightwheel,static_cast<uint8_t>(0)));
      // publisher_->publish(get_frame(pubsub::bidNumber.upperLeftsteering,static_cast<uint8_t>(3)));
      // publisher_->publish(get_frame(pubsub::bidNumber.upperRightsteering,static_cast<uint8_t>(3)));
      // publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftsteering,static_cast<uint8_t>(3)));
      // publisher_->publish(get_frame(pubsub::bidNumber.lowerRightsteering,static_cast<uint8_t>(3)));
    }

    if(msg->buttons[2]==true)
    {
      //publisher_->publish(get_frame(pubsub::bidNumber.upperLeftwheel,static_cast<uint8_t>(5)));
      //publisher_->publish(get_frame(pubsub::bidNumber.upperRightwheel,static_cast<uint8_t>(5)));
      //publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftwheel,static_cast<uint8_t>(5)));
      //publisher_->publish(get_frame(pubsub::bidNumber.lowerRightwheel,static_cast<uint8_t>(5)));
      robomas_pub_->publish(get_robomas_frame(0,1,50,30.0,22.0,5.0,1000.0));
      robomas_pub_->publish(get_robomas_frame(1,1,50,30.0,22.0,5.0,1000.0));
      robomas_pub_->publish(get_robomas_frame(2,1,50,30.0,22.0,5.0,1000.0));
      robomas_pub_->publish(get_robomas_frame(3,1,50,30.0,22.0,5.0,1000.0));
      publisher_->publish(get_frame(pubsub::bidNumber.upperLeftsteering,static_cast<uint8_t>(6)));
      publisher_->publish(get_frame(pubsub::bidNumber.upperRightsteering,static_cast<uint8_t>(6)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftsteering,static_cast<uint8_t>(6)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerRightsteering,static_cast<uint8_t>(6)));
    }

    if(msg->buttons[1]==true)
    {
      // publisher_->publish(get_frame(pubsub::bidNumber.upperLeftwheel,static_cast<uint8_t>(0)));
      // publisher_->publish(get_frame(pubsub::bidNumber.upperRightwheel,static_cast<uint8_t>(0)));
      // publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftwheel,static_cast<uint8_t>(0)));
      // publisher_->publish(get_frame(pubsub::bidNumber.lowerRightwheel,static_cast<uint8_t>(0)));
      robomas_pub_->publish(get_robomas_frame(0,0,50,30.0,22.0,5.0,1000.0));
      robomas_pub_->publish(get_robomas_frame(1,0,50,30.0,22.0,5.0,1000.0));
      robomas_pub_->publish(get_robomas_frame(2,0,50,30.0,22.0,5.0,1000.0));
      robomas_pub_->publish(get_robomas_frame(3,0,50,30.0,22.0,5.0,1000.0));
      publisher_->publish(get_frame(pubsub::bidNumber.upperLeftsteering,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.upperRightsteering,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftsteering,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(pubsub::bidNumber.lowerRightsteering,static_cast<uint8_t>(0)));
    }

    float x= -(msg->axes[0]);
    float y=  (msg->axes[1]);
    float theta = std::atan2(y, x) * 5 / 2;
  
    if(msg->buttons[4]==true){
      // publisher_->publish(get_frame((pubsub::bidNumber.upperRightwheel+1), M_PI));
      // publisher_->publish(get_frame((pubsub::bidNumber.upperLeftwheel+1), M_PI));
      // publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftwheel+1), M_PI));
      // publisher_->publish(get_frame((pubsub::bidNumber.lowerRightwheel+1), M_PI));
      robomas_pub1_->publish(get_robomas_target(M_PI));
      robomas_pub2_->publish(get_robomas_target(M_PI));
      robomas_pub3_->publish(get_robomas_target(M_PI));
      robomas_pub4_->publish(get_robomas_target(M_PI));
      //wheel速度制御

      publisher_->publish(get_frame((pubsub::bidNumber.upperRightsteering+1), M_PI/4));
      publisher_->publish(get_frame((pubsub::bidNumber.upperLeftsteering+1), M_PI*3/4));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftsteering+1), M_PI*5/4));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerRightsteering+1), M_PI*7/4));
      //steering制御
    }
    //↑左回転
    else if(msg->buttons[5]==true){
      // publisher_->publish(get_frame((pubsub::bidNumber.upperRightwheel+1), -M_PI));
      // publisher_->publish(get_frame((pubsub::bidNumber.upperLeftwheel+1), -M_PI));
      // publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftwheel+1), -M_PI));
      // publisher_->publish(get_frame((pubsub::bidNumber.lowerRightwheel+1), -M_PI));
      robomas_pub1_->publish(get_robomas_target(-M_PI));
      robomas_pub2_->publish(get_robomas_target(-M_PI));
      robomas_pub3_->publish(get_robomas_target(-M_PI));
      robomas_pub4_->publish(get_robomas_target(-M_PI));
      //wheel速度制御

      publisher_->publish(get_frame((pubsub::bidNumber.upperRightsteering+1), M_PI/4));
      publisher_->publish(get_frame((pubsub::bidNumber.upperLeftsteering+1), M_PI*3/4));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftsteering+1), M_PI*5/4));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerRightsteering+1), M_PI*7/4));
      //steering制御
    }
    //右回転

    if((x>0.01 || y>0.01) || (x<-0.01 || y<-0.01)){
      // publisher_->publish(get_frame((pubsub::bidNumber.upperRightwheel+1), M_PI*2.0f*(x*x+y*y)));
      // publisher_->publish(get_frame((pubsub::bidNumber.upperLeftwheel+1), M_PI*2.0f*(x*x+y*y)));
      // publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftwheel+1), M_PI*2.0f*(x*x+y*y)));
      // publisher_->publish(get_frame((pubsub::bidNumber.lowerRightwheel+1), M_PI*2.0f*(x*x+y*y)));
      robomas_pub1_->publish(get_robomas_target(M_PI*2.0f*(x*x+y*y)*100));
      robomas_pub2_->publish(get_robomas_target(M_PI*2.0f*(x*x+y*y)*100));
      robomas_pub3_->publish(get_robomas_target(M_PI*2.0f*(x*x+y*y)*100));
      robomas_pub4_->publish(get_robomas_target(M_PI*2.0f*(x*x+y*y)*100));
      //wheel速度制御

      publisher_->publish(get_frame((pubsub::bidNumber.upperRightsteering+1), theta-M_PI/2));
      publisher_->publish(get_frame((pubsub::bidNumber.upperLeftsteering+1), theta-M_PI/2));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerLeftsteering+1), theta-M_PI/2));
      publisher_->publish(get_frame((pubsub::bidNumber.lowerRightsteering+1), theta-M_PI/2));
      //steering制御

      RCLCPP_INFO(this->get_logger(), "Publishing:bokuha warukunai!");
      count = 1;
    }else{
      if(count == 1){
        robomas_pub1_->publish(get_robomas_target(0));
        robomas_pub2_->publish(get_robomas_target(0));
        robomas_pub3_->publish(get_robomas_target(0));
        robomas_pub4_->publish(get_robomas_target(0));
        //wheel速度制御
        count = 0;
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