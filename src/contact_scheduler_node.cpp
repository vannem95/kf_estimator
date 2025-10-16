#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <vector>
#include <chrono>
#include <numeric>

using namespace std::chrono_literals;

// Define constants matching the EKF node's expected input
const int N_FEET = 8;
// Foot order assumed based on EKF header: 
// 0: BL-Front, 1: BL-Rear, 2: BR-Front, 3: BR-Rear, 
// 4: FL-Front, 5: FL-Rear, 6: FR-Front, 7: FR-Rear

class ContactSchedulerNode : public rclcpp::Node
{
public:
    ContactSchedulerNode() : Node("contact_scheduler_node")
    {
        // 1. Publisher setup
        contact_pub_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("contact_schedule", 10);

        // 2. Timer setup (e.g., publish at 10 Hz)
        double publish_rate_hz = 200.0;
        double tumble_frequency_hz = 0.5; // Gait cycle (front-to-rear-to-front) completes once every 2 seconds
        
        publish_period_ = 1.0 / publish_rate_hz;
        phase_switch_period_ = 1.0 / (2.0 * tumble_frequency_hz); // Switch contact every 1 second

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_period_),
            std::bind(&ContactSchedulerNode::publish_contact_state, this));
            
        current_time_ = 0.0;
        is_front_contact_phase_ = true; // Start with all front wheels in contact
        RCLCPP_INFO(this->get_logger(), "Contact Scheduler Node started at %.1f Hz. Simulating Shin Tumbling Gait.", publish_rate_hz);
    }

private:
    void publish_contact_state()
    {
        auto message = std_msgs::msg::Int8MultiArray();
        message.data.resize(N_FEET);
        
        // Check if it's time to switch the gait phase
        if (current_time_ >= phase_switch_period_) {
            is_front_contact_phase_ = !is_front_contact_phase_;
            current_time_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Switching phase. Front Wheels Contact: %s", is_front_contact_phase_ ? "True" : "False");
        }
        
        // Shin Tumbling Gait: Alternates contact between all front wheels and all rear wheels
        
        for (int i = 0; i < N_FEET; ++i) {
            // Even indices (0, 2, 4, 6) correspond to the Front Wheels
            bool is_front_wheel = (i % 2 == 0);
            
            if (is_front_contact_phase_) {
                // Phase 1: Front wheels TRUE (in contact), Rear wheels FALSE (in flight)
                message.data[i] = is_front_wheel;
            } else {
                // Phase 2: Rear wheels TRUE (in contact), Front wheels FALSE (in flight)
                message.data[i] = !is_front_wheel;
            }
        }

        contact_pub_->publish(message);
        current_time_ += publish_period_;
    }

    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr contact_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double current_time_;
    double publish_period_;
    double phase_switch_period_;
    bool is_front_contact_phase_; // Tracks whether the front wheels or rear wheels are constrained
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContactSchedulerNode>());
    rclcpp::shutdown();
    return 0;
}
