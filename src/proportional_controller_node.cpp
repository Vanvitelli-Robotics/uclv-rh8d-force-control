#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/sensors_norm.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_error.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp"
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <algorithm>

// Definizione della classe per il controller proporzionale
class ProportionalController : public rclcpp::Node
{
public:
    double gain_; // Valore del guadagno di controllo proporzionale
    std::vector<int64_t> motor_ids_; // Lista degli ID dei motori gestiti dal controller

    uclv_seed_robotics_ros_interfaces::msg::SensorsNorm desired_norm_forces_;
    uclv_seed_robotics_ros_interfaces::msg::SensorsNorm measured_norm_forces_;

    bool desired_norm_forces_received_ = false;
    bool measured_norm_forces_received_ = false;

    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr desired_norm_forces_sub_;

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorError>::SharedPtr error_pub_;

    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;

    ProportionalController()
        : Node("proportional_controller"),
          gain_(this->declare_parameter<double>("gain", 1.0)), // Inizializza il guadagno dal parametro ROS
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

        initialize_motor_to_sensor_map();

        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            "norm_forces", 10, std::bind(&ProportionalController::measured_norm_forces_callback, this, std::placeholders::_1));

        desired_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            "/cmd/desired_norm_forces", 10, std::bind(&ProportionalController::desired_norm_forces_callback, this, std::placeholders::_1));

        error_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorError>(
            "/result_proportional_controller", 10);

        set_gain_service_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SetGain>(
            "set_gain", std::bind(&ProportionalController::set_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void initialize_motor_to_sensor_map()
    {
        motor_to_sensor_map_[35] = {0};
        motor_to_sensor_map_[36] = {1};
        motor_to_sensor_map_[37] = {2};
        motor_to_sensor_map_[38] = {3, 4};
    }

    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::SensorsNorm::SharedPtr msg)
    {
        measured_norm_forces_ = *msg;
        measured_norm_forces_received_ = true;
        compute_and_publish_error();
    }

    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::SensorsNorm::SharedPtr msg)
    {
        desired_norm_forces_ = *msg;
        desired_norm_forces_received_ = true;
        compute_and_publish_error();
    }

    void set_gain_callback(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Request> request,
                           std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Response> response)
    {
        if (request->gain < 0.0)
        {
            response->success = false;
            response->message = "Gain must be non-negative.";
            RCLCPP_WARN(this->get_logger(), "Attempted to set a negative gain.");
        }
        else
        {
            gain_ = request->gain;
            response->success = true;
            response->message = "Gain updated successfully.";
            RCLCPP_INFO(this->get_logger(), "Gain updated to: %f", gain_);
        }
    }

    void compute_and_publish_error()
    {
        if (!desired_norm_forces_received_ || !measured_norm_forces_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Desired or measured forces data not received yet.");
            return;
        }

        uclv_seed_robotics_ros_interfaces::msg::MotorError error_msg;
        error_msg.header.stamp = this->now();

        for (int64_t motor_id : motor_ids_)
        {
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id);
            if (sensor_ids_iter == motor_to_sensor_map_.end())
            {
                RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue;
            }

            const auto &sensor_ids = sensor_ids_iter->second;

            for (int64_t sensor_id : sensor_ids)
            {
                auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
                auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.ids.end(), sensor_id);

                if (measured_force_iter == measured_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in measured forces", sensor_id);
                    continue;
                }
                if (desired_force_iter == desired_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in desired forces", sensor_id);
                    continue;
                }

                size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
                size_t desired_id = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

                double measured_norm = measured_norm_forces_.norms[measured_id];
                double desired_norm = desired_norm_forces_.norms[desired_id]; 

                double error = desired_norm - measured_norm;

                RCLCPP_INFO(this->get_logger(), "Error for Sensor ID: %ld - Error: %f", sensor_id, error);

                error_msg.motor_ids.push_back(motor_id);
                error_msg.errors.push_back(gain_ * error);
            }
        }

        error_pub_->publish(error_msg);

        desired_norm_forces_received_ = false;
        measured_norm_forces_received_ = false;
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
