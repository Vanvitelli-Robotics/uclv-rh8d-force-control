#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <stdexcept>

class ProportionalController : public rclcpp::Node
{
public:
    double gain_;
    double desired_force_;
    double measured_force_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr desired_force_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measured_force_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr result_pub_;

    ProportionalController()
        : Node("proportional_controller"),
          gain_(this->declare_parameter<double>("gain", 1.0))
    {
        if (gain_ < 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'gain' must be non-negative. Exiting...");
            throw std::invalid_argument("Parameter 'gain' must be non-negative");
        }

        desired_force_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/desired_force", 10, std::bind(&ProportionalController::desired_force_callback, this, std::placeholders::_1));

        measured_force_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/measured_force", 10, std::bind(&ProportionalController::measured_force_callback, this, std::placeholders::_1));

        result_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/result_proportional_controller", 10);
    }

private:
    void desired_force_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (std::isnan(msg->data))
        {
            RCLCPP_WARN(this->get_logger(), "Received NaN in /desired_force. Ignoring...");
            return;
        }
        desired_force_ = msg->data;
        compute_and_publish_result();
    }

    void measured_force_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (std::isnan(msg->data))
        {
            RCLCPP_WARN(this->get_logger(), "Received NaN in /measured_force. Ignoring...");
            return;
        }
        measured_force_ = msg->data;
        compute_and_publish_result();
    }

    void compute_and_publish_result()
    {
        try
        {
            if (desired_force_ && measured_force_)
            {
                double error = desired_force_ - measured_force_;
                double result = gain_ * error;

                auto result_msg = std_msgs::msg::Float64();
                result_msg.data = result;

                result_pub_->publish(result_msg);
                RCLCPP_INFO(this->get_logger(), "Published result: %f", result);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Both /desired_force and /measured_force need to be received before computing the result.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Exception caught while computing result: %s", e.what());
            throw;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto proportional_controller_node = std::make_shared<ProportionalController>();
        rclcpp::spin(proportional_controller_node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
