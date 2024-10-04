#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include <cmath>
#include <memory>

using std::placeholders::_1;

class ForceNorm : public rclcpp::Node
{
public:
    std::string sensor_state_topic_;
    std::string measured_norm_topic_;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr sensor_state_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_topic_pub_;

    ForceNorm()
    : Node("force_norm"),
      sensor_state_topic_(this->declare_parameter<std::string>("sensor_state_topic", std::string())),
      measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", std::string()))
    {   
        check_parameters();
        
        sensor_state_sub_= this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            sensor_state_topic_, 10, std::bind(&ForceNorm::sensorStateCallback, this, _1));

        measured_norm_topic_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10);
    }

private:


void check_parameters()
    {
        auto check_string_parameter = [this](const std::string &param_name, const std::string &value)
        {
            if (value.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid value.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };

        check_string_parameter("measured_norm_topic", measured_norm_topic_);
        check_string_parameter("sensor_state_topic",sensor_state_topic_);

    
        RCLCPP_INFO(this->get_logger(), "All required parameters are set correctly.");
    }


    void sensorStateCallback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        // Create a message for the calculated norms
        uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped norm_msg;
        norm_msg.header.stamp = this->now();  // Set the current time stamp
        norm_msg.ids = msg->ids;  // Copy sensor IDs
        norm_msg.data.resize(msg->forces.size());  // Resize norms vector to match the size of forces

        // Calculate the norm of each 3D force
        for (size_t i = 0; i < msg->forces.size(); ++i)
        {
            // Compute the 3D force norm
            norm_msg.data[i] = std::sqrt(std::pow(msg->forces[i].x, 2) + 
                                          std::pow(msg->forces[i].y, 2) + 
                                          std::pow(msg->forces[i].z, 2)) / 1000.0;

            // Log the calculated norm for debugging purposes
            // RCLCPP_INFO(this->get_logger(), "Calculated norm for sensor ID: %d, Norm: %f", msg->ids[i], norm_msg.data[i]);
        }

        // Publish the calculated norms
        measured_norm_topic_pub_->publish(norm_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto force_norm_node = std::make_shared<ForceNorm>();
    rclcpp::spin(force_norm_node);
    rclcpp::shutdown();
    return 0;
}
