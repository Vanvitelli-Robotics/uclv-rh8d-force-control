#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include <stdexcept>
#include <vector>

/*
    devo fare un mapping tra gli id che ho dato ai sensori e quelli dei motori 
    perchè la mano deve essere inizializzata con gli id che mi servono
*/

class ProportionalController : public rclcpp::Node
{
public:
    double gain_;
    std::vector<int64_t> motor_ids_;
    uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors desired_forces_;
    uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors measured_forces_;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr sensor_state_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr desired_forces_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr result_pub_;

    ProportionalController()
        : Node("proportional_controller"),
          gain_(this->declare_parameter<double>("gain", 1.0)),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>()))
    {
        if (gain_ < 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'gain' must be non-negative. Exiting...");
            throw std::invalid_argument("Parameter 'gain' must be non-negative");
        }

        if (motor_ids_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_ids' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_ids' is empty or not set");
        }

        sensor_state_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            "/sensor_state", 10, std::bind(&ProportionalController::sensor_state_callback, this, std::placeholders::_1));

        desired_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            "/desired_forces", 10, std::bind(&ProportionalController::desired_forces_callback, this, std::placeholders::_1));

        result_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            "/result_proportional_controller", 1);
    }

private:
    void sensor_state_callback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        measured_forces_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received measured forces:");
        for (size_t i = 0; i < msg->forces.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Motor ID: %ld, Force: (%f, %f, %f)",
                        msg->ids[i], msg->forces[i].x, msg->forces[i].y, msg->forces[i].z);
        }
        compute_and_publish_result();
    }

    void desired_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        desired_forces_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received desired forces:");
        for (size_t i = 0; i < msg->forces.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Motor ID: %ld, Force: (%f, %f, %f)",
                        msg->ids[i], msg->forces[i].x, msg->forces[i].y, msg->forces[i].z);
        }
        compute_and_publish_result();
    }

    void compute_and_publish_result()
    {
        if (desired_forces_.forces.size() != measured_forces_.forces.size())
        {
            RCLCPP_FATAL(this->get_logger(), "Mismatch in size of desired and measured forces vectors.");
            return;
        }

        uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors result_msg;
        result_msg.header.stamp = this->now();

        size_t num_forces = measured_forces_.forces.size();

        result_msg.forces.resize(num_forces);
        result_msg.ids.resize(num_forces);

        for (size_t i = 0; i < num_forces; ++i)
        {
            try
            {
                // Compute error
                double error_x = desired_forces_.forces[i].x - measured_forces_.forces[i].x;
                double error_y = desired_forces_.forces[i].y - measured_forces_.forces[i].y;
                double error_z = desired_forces_.forces[i].z - measured_forces_.forces[i].z;

                // Compute result using the gain
                result_msg.forces[i].x = gain_ * error_x;
                result_msg.forces[i].y = gain_ * error_y;
                result_msg.forces[i].z = gain_ * error_z;

                // Copy ids
                result_msg.ids[i] = measured_forces_.ids[i];

                RCLCPP_INFO(this->get_logger(), "Published result for motor %ld: (%f, %f, %f)",
                            result_msg.ids[i], result_msg.forces[i].x, result_msg.forces[i].y, result_msg.forces[i].z);
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(this->get_logger(), "Exception caught while computing result for motor %ld: %s",
                             measured_forces_.ids[i], e.what());
                throw;
            }
        }

        result_pub_->publish(result_msg);
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