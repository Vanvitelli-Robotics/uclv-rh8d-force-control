#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp" 
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"


class SensorStateSubscriber : public rclcpp::Node
{
public:
    SensorStateSubscriber()
        : Node("sensor_state_subscriber"), node_activated_(false)
    {
        sensor_state_subscription_ = this->create_subscription<sensor_msgs::msg::SensorState>(
            "sensor_state", 10, std::bind(&SensorStateSubscriber::sensor_state_callback, this, std::placeholders::_1));

        desired_forces_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/cmd/desired_forces", 10, std::bind(&SensorStateSubscriber::desired_forces_callback, this, std::placeholders::_1));

        activation_service_ = this->create_service<std_srvs::srv::Trigger>(
            "activate_node", std::bind(&SensorStateSubscriber::activate_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void sensor_state_callback(const sensor_msgs::msg::SensorState::SharedPtr msg)
    {
        if (node_activated_)
        {
            RCLCPP_INFO(this->get_logger(), "Node is active, processing sensor data...");

            for (size_t i = 0; i < msg->forces.size(); ++i)
            {
                double x = msg->forces[i].x;
                double y = msg->forces[i].y;
                RCLCPP_INFO(this->get_logger(), "Force ID: %u, x: %f, y: %f", msg->ids[i], x, y);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Node is inactive. Waiting for activation...");
        }
    }

    void desired_forces_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (node_activated_)
        {
            RCLCPP_INFO(this->get_logger(), "Node is active, processing desired forces...");

            for (size_t i = 0; i < msg->data.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "Desired force ID: %u, value: %f", msg->layout.dim[0].size + i, msg->data[i]);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Node is inactive. Waiting for activation...");
        }
    }

    void activate_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        node_activated_ = !node_activated_;
        response->success = true;
        response->message = node_activated_ ? "Node activated!" : "Node deactivated!";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::SensorState>::SharedPtr sensor_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_forces_subscription_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr activation_service_;

    bool node_activated_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorStateSubscriber>());
    rclcpp::shutdown();
    return 0;
}
