#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <sstream>

using std::placeholders::_1;

class Close : public rclcpp::Node
{
public:
    std::string measured_norm_topic_;                // Topic for normalized forces
    std::string node_service_name_;                  // Name for the start/stop service
    std::string measured_velocity_topic_;            // Topic for measured velocity
    std::vector<int64_t> motor_ids_;                 // Motor IDs for the velocities
    double threshold_;                               // Threshold for forces
    double initial_velocity_;                        // Initial velocity value
    std::vector<std::string> motor_sensor_mappings_; // Mapping motor ID - sensors ID
    std::string proportional_service_name_;          // Name for the proportional controller service
    std::string integrator_service_name_;            // Nome del servizio per l'integratore

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_velocity_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr node_service_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr proportional_service_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr integrator_service_client_;

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_;

    bool measured_norm_forces_received_ = false;
    bool service_close_actived_ = false;

    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;

    Close()
        : Node("close_node"),
          measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "")),
          measured_velocity_topic_(this->declare_parameter<std::string>("measured_velocity_topic", "")),
          motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>())),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),
          threshold_(this->declare_parameter<double>("threshold", 0.0)),
          initial_velocity_(this->declare_parameter<double>("initial_velocity", 0.0)),
          node_service_name_(this->declare_parameter<std::string>("node_service_name", "")),
          proportional_service_name_(this->declare_parameter<std::string>("proportional_service_name", "")),
          integrator_service_name_(this->declare_parameter<std::string>("integrator_service_name", ""))

    {

        initialize_motor_to_sensor_map();

        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10, std::bind(&Close::measured_norm_forces_callback, this, std::placeholders::_1));

        node_service_ = this->create_service<std_srvs::srv::SetBool>(
            node_service_name_, std::bind(&Close::service_activate_callback, this, _1, std::placeholders::_2));

        measured_velocity_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_velocity_topic_, 10);

        proportional_service_client_ = this->create_client<std_srvs::srv::SetBool>(proportional_service_name_);

        integrator_service_client_ = this->create_client<std_srvs::srv::SetBool>(integrator_service_name_);
    }

private:
    template <typename KeyType, typename ValueType>
    void
    initialize_map_from_mappings(
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

    void publish_motor_velocity(const std::vector<int64_t>& motor_ids, double velocity)
{
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped velocity_msg;


    for (int64_t motor_id : motor_ids_)
    {
        velocity_msg.ids.push_back(motor_id);
    }
    velocity_msg.data.push_back(velocity);

    RCLCPP_INFO(this->get_logger(), "Publishing velocity: %ld", velocity);
    measured_velocity_pub_->publish(velocity_msg);
}


    void service_activate_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            RCLCPP_INFO(this->get_logger(), "Service called to activate node.");
            service_close_actived_ = true;

            if (!integrator_service_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_ERROR(this->get_logger(), "Integrator service not available after waiting.");
                response->success = false;
                response->message = "Integrator service unavailable. Failed to activate node.";
                return;
            }

            activate_integrator_service();
            publish_motor_velocity(motor_ids_, initial_velocity_);
            response->success = true;
            response->message = "Node activated successfully - Integration is active.";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Service called with deactivate command.");
            service_close_actived_ = false;

            
            if (!proportional_service_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_ERROR(this->get_logger(), "Proportional controller service not available after waiting.");
                response->success = false;
                response->message = "Proportional controller service unavailable. Failed to deactivate node.";
                return;
            }

            

            deactivate_integrator_service();

            response->success = true;
            response->message = "Node deactivated successfully.";
        }
    }

    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        if (service_close_actived_)
        {
            measured_norm_forces_ = *msg;
            measured_norm_forces_received_ = true;

            RCLCPP_INFO(this->get_logger(), "Normalized forces received and node is activated.");

            process_norm_forces();
        }
    }

    void activate_integrator_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        auto result = integrator_service_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request sent to activate the integrator node.");
    }

    void deactivate_integrator_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;

        auto result = integrator_service_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request sent to activate the integrator node.");
    }

    void activate_proportional_controller()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        proportional_service_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request sent to activate the proportional node.");
    }

    void deactivate_proportional_controller()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;

        proportional_service_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request sent to deactivate the proportional node.");
    }


    void process_norm_forces()
    {
        bool all_above_threshold = true;
        bool stop_motor = false;

        for (int64_t motor_id : motor_ids_)
        {
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id);
            if (sensor_ids_iter == motor_to_sensor_map_.end())
            {
                RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue;
            }
            const auto &sensor_ids = sensor_ids_iter->second;
            bool partial_above_threshold = false;

            for (int64_t sensor_id : sensor_ids)
            {
                auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
                if (measured_force_iter == measured_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in norm forces measured", sensor_id);
                    continue;
                }

                size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
                double measured_norm = measured_norm_forces_.data[measured_id];

                RCLCPP_INFO(this->get_logger(), "Measured norm for sensor ID %ld: %f", sensor_id, measured_norm);

                partial_above_threshold = partial_above_threshold || (measured_norm >= threshold_);


            }
            all_above_threshold = all_above_threshold && partial_above_threshold;
        }

        if (all_above_threshold)
        {
            RCLCPP_INFO(this->get_logger(), "Stopping close node and activating proportional controller");
            service_close_actived_ = false;

            activate_proportional_controller();
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto close_node = std::make_shared<Close>();
        rclcpp::spin(close_node);
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
