#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "stewart_interfaces/msg/legs_len_vel.hpp"
#include "stewart_interfaces/msg/legs_len.hpp"
#include "canbus.h"

using std::placeholders::_1;

class Actuator : public rclcpp::Node
{
    public:
        Actuator()
        : Node("actuator"), can(new CANbus)
        {
            ref_subscriber_ = this->create_subscription<stewart_interfaces_pkg::msg::LegsLenVel>(
                "leg_ref", 1, std::bind(&Actuator::actuate_callback, this, _1));
            
            RCLCPP_INFO(this->get_logger(), "Actuator Node running! Subscribed to 'leg_ref' topic.");

            feedback_publisher_ = this->create_publisher<stewart_interfaces_pkg::msg::LegsLen>(
                "leg_feedback", 10);
            
            RCLCPP_INFO(this->get_logger(), "Publishing feedback-data on 'leg_feedback' topic.");
        }

        ~Actuator()
        {
            delete can;
        }

    private:
        void actuate_callback(const stewart_interfaces_pkg::msg::LegsLenVel::SharedPtr msg) const
        {
            float len[6];
            float vel[6];

            for (uint8_t i = 0; i < 6; i ++)
            {
                len[i] = msg->leg[i].length;
                vel[i] = msg->leg[i].velocity;
            }
            
            float* feedback_ptr = can->send_data(len, vel);
            
            auto feedback_msg = stewart_interfaces_pkg::msg::LegsLen();

            for (uint8_t i = 0; i < 6; i++)
            {
                feedback_msg.leg[i].length = *(feedback_ptr + i);
            }

            feedback_publisher_->publish(feedback_msg);
        }
        CANbus* can;
        rclcpp::Subscription<stewart_interfaces_pkg::msg::LegsLenVel>::SharedPtr ref_subscriber_;
        rclcpp::Publisher<stewart_interfaces_pkg::msg::LegsLen>::SharedPtr feedback_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Actuator>());
    rclcpp::shutdown();
    return 0;
}



