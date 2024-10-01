#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <algorithm>

class ProportionalController : public rclcpp::Node
{
public:
    double gain_;                                     // Proportional gain value for the controller
    std::vector<int64_t> motor_ids_;                  // List of motor IDs managed by the controller
    std::vector<std::string> motor_sensor_mappings_;  // Mapping motor ID - sensors ID
    std::vector<std::string> sensor_weight_mappings_; // Mapping sensor ID - weight
    std::string measured_norm_topic_;     // Name of the topic for measured normalized forces
    std::string desired_norm_topic_;      // Name of the topic for desired normalized forces
    std::string measured_velocity_topic_; // Name of the topic for publishing errors
    std::string set_gain_service_name_;   // Name of the service to set gain
    std::string node_service_name_; // Name of the service to activate the controller

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_forces_;  // Message for desired normalized forces
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_; // Message for measured normalized forces

    bool desired_norm_forces_received_ = false;  // Flag indicating if desired forces data has been received
    bool measured_norm_forces_received_ = false; // Flag indicating if measured forces data has been received
    bool controller_activated_ = false;          // Flag to check if the controller is activated

    // Mapping of motor IDs to their corresponding sensor IDs
    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;
    // Mapping of sensor IDs to their corresponding weights
    std::unordered_map<int64_t, std::vector<double>> sensor_to_weight_map_;

    // ROS 2 subscriptions to receive sensor data
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_forces_sub_;

    // ROS 2 publisher to publish motor errors
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_velocity_pub_;

    // ROS 2 service to handle requests for setting the gain value
    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;

    // Service to activate the controller
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_controller_service_;

    // Constructor for initializing the node
    ProportionalController()
        : Node("proportional_controller"),
          gain_(this->declare_parameter<double>("gain", 1.0)),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),
          motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>())),
          sensor_weight_mappings_(this->declare_parameter<std::vector<std::string>>("sensor_weight_mappings", std::vector<std::string>())),
          measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "norm_forces")),
          desired_norm_topic_(this->declare_parameter<std::string>("desired_norm_topic", "/cmd/desired_norm_forces")),
          measured_velocity_topic_(this->declare_parameter<std::string>("measured_velocity_topic", "measured_velocity")),
          set_gain_service_name_(this->declare_parameter<std::string>("set_gain_service_name", "set_gain")),
          node_service_name_(this->declare_parameter<std::string>("node_service_name", "activate_controller"))
    {
        // Initialize mappings
        initialize_motor_to_sensor_map();
        initialize_sensor_to_weight_map();

        // Create subscription to measured normalized forces
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10, std::bind(&ProportionalController::measured_norm_forces_callback, this, std::placeholders::_1));

        // Create subscription to desired normalized forces
        desired_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            desired_norm_topic_, 10, std::bind(&ProportionalController::desired_norm_forces_callback, this, std::placeholders::_1));

        // Create publisher for motor errors
        measured_velocity_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_velocity_topic_, 10);

        // Create service for setting the gain
        set_gain_service_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SetGain>(
            set_gain_service_name_, std::bind(&ProportionalController::set_gain_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create service to activate the controller
        activate_controller_service_ = this->create_service<std_srvs::srv::SetBool>(
            node_service_name_, std::bind(&ProportionalController::activate_controller_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void activate_controller_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        controller_activated_ = request->data;
        response->success = true;
        response->message = controller_activated_ ? "Controller activated" : "Controller deactivated";
        RCLCPP_INFO(this->get_logger(), response->message.c_str());
    }

    // Callback function for receiving measured normalized forces
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        if (!controller_activated_)
        {
            RCLCPP_WARN(this->get_logger(), "Controller is not activated. Waiting for activation...");
            return;
        }
        measured_norm_forces_ = *msg;
        measured_norm_forces_received_ = true;
        compute_and_publish_error();
    }

    // Callback function for receiving desired normalized forces
    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        if (!controller_activated_)
        {
            RCLCPP_WARN(this->get_logger(), "Controller is not activated. Waiting for activation...");
            return;
        }
        desired_norm_forces_ = *msg;
        desired_norm_forces_received_ = true;
        compute_and_publish_error();
    }

    // Callback function for setting the gain value
    void set_gain_callback(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Request> request,
                           std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Response> response)
    {
        if (request->gain < 0.0) // Check if the requested gain is negative
        {
            response->success = false;
            response->message = "Gain must be non-negative.";
            RCLCPP_WARN(this->get_logger(), "Attempted to set a negative gain.");
        }
        else // Update the gain value if it is valid
        {
            gain_ = request->gain;
            response->success = true;
            response->message = "Gain updated successfully.";
            RCLCPP_INFO(this->get_logger(), "Gain updated to: %f", gain_);
        }
    }

    // Function to compute and publish the motor errors
    void compute_and_publish_error()
    {
        // Initialize the motor error message
        uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped velocity_msg;
        velocity_msg.header.stamp = this->now(); // Set the timestamp for the message

        // Iterate over each motor ID to compute the error
        for (int64_t motor_id : motor_ids_)
        {
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id); // Find the associated sensor IDs for the motor
            if (sensor_ids_iter == motor_to_sensor_map_.end())          // Check if the motor has a valid mapping
            {
                RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Ok mapping for motor ID: %ld", motor_id);
            }

            const auto &sensor_ids = sensor_ids_iter->second; // Get the associated sensor IDs for the motor
            double total_weighted_error = 0.0;                // Sum of weighted errors for the motor
            double total_weights = 0.0;                       // Sum of sensor weights

            // Iterate over each sensor ID to compute the weighted error
            for (int64_t sensor_id : sensor_ids)
            {
                // Find the weight(s) associated with the sensor
                auto weight_iter = sensor_to_weight_map_.find(sensor_id);
                if (weight_iter == sensor_to_weight_map_.end()) // Check if the sensor has a valid weight mapping
                {
                    RCLCPP_ERROR(this->get_logger(), "No mapping found for sensor ID: %ld", sensor_id);
                    continue;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Ok mapping for sensor ID: %ld", sensor_id);
                }

                const auto &weights = weight_iter->second;

                // Find the index of the sensor ID in the measured and desired forces
                auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
                auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.ids.end(), sensor_id);

                // Check if the sensor ID is found in the measured and desired forces
                if (desired_force_iter == desired_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in norm forces desired", sensor_id);
                    continue;
                }
                else if (measured_force_iter == measured_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in norm forces measured", sensor_id);
                    continue;
                }

                // Compute the indices of the sensor ID in the measured and desired forces
                size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
                size_t desired_id = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

                // Retrieve the measured and desired normalized forces
                double measured_norm = measured_norm_forces_.data[measured_id];
                double desired_norm = desired_norm_forces_.data[desired_id];

                // Compute the weighted error for each weight associated with the sensor
                for (const auto &weight : weights)
                {
                    double error = (desired_norm - measured_norm) * weight; // Weighted error
                    total_weighted_error += error;
                    total_weights += weight; // Sum up the weights
                }
            }

            // Calculate the weighted average error for the motor
            if (total_weights > 0.0)
            {
                double average_error = total_weighted_error / total_weights;
                velocity_msg.ids.push_back(motor_id);
                velocity_msg.data.push_back(gain_ * average_error); // Apply the proportional gain to the average error
                RCLCPP_INFO(this->get_logger(), "Published error for motor ID: %ld - Error: %f", motor_id, average_error);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Total weight is zero for motor ID: %ld", motor_id);
            }
        }

        // Publish the computed motor errors
        measured_velocity_pub_->publish(velocity_msg);

        // Reset the flags for receiving new data
        // desired_norm_forces_received_ = false;
        measured_norm_forces_received_ = false;
    }

template <typename KeyType, typename ValueType>
    void initialize_map_from_mappings(
        const std::vector<std::string> &mappings,
        std::unordered_map<KeyType, std::vector<ValueType>> &map,
        const std::string &map_type)
    {
        if (mappings.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is empty or not set. Exiting...", map_type.c_str());
            throw std::runtime_error("Parameter '" + map_type + "' is empty or not set");
        }

        for (const auto &mapping : mappings)
        {
            std::istringstream iss(mapping);
            KeyType key;
            std::vector<ValueType> values;
            char delimiter;

            if (!(iss >> key >> delimiter))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid mapping format: %s", mapping.c_str());
                continue;
            }

            ValueType value;
            while (iss >> value)
            {
                values.push_back(value);
                iss >> delimiter; // consume the comma if present
            }

            if (values.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No values found for key: %ld", static_cast<int64_t>(key));
                continue;
            }

            map[key] = values;
            RCLCPP_INFO(this->get_logger(), "Mapped key %ld to values: %s",
                        static_cast<int64_t>(key), [&values]()
                                                   {
                        std::ostringstream oss;
                        for (size_t i = 0; i < values.size(); ++i)
                        {
                            if (i > 0)
                                oss << ", ";
                            oss << values[i];
                        }
                        return oss.str(); }().c_str());
        }

        if (map.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "No valid mappings were created for '%s'. Exiting...", map_type.c_str());
            throw std::runtime_error("No valid mappings were created for '" + map_type + "'");
        }
    }

    void initialize_motor_to_sensor_map()
    {
        initialize_map_from_mappings(motor_sensor_mappings_, motor_to_sensor_map_, "motor_sensor_mappings");
    }

    void initialize_sensor_to_weight_map()
    {
        initialize_map_from_mappings(sensor_weight_mappings_, sensor_to_weight_map_, "sensor_weight_mappings");
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
