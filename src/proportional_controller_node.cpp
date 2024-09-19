
#include "rclcpp/rclcpp.hpp" // Include the main header for ROS 2 C++ client library
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp" // Include custom service type for setting the gain value
#include <stdexcept>                                          // Include standard library for handling exceptions
#include <unordered_map>                                      // Include standard library for using hash maps
#include <vector>                                             // Include standard library for using vectors
#include <algorithm>                                          // Include standard library for common algorithms like std::find

// Definition of the ProportionalController class, which inherits from rclcpp::Node
class ProportionalController : public rclcpp::Node
{
public:
    double gain_;                                     // Proportional gain value for the controller
    std::vector<int64_t> motor_ids_;                  // List of motor IDs managed by the controller
    std::vector<std::string> motor_sensor_mappings_;  // Mapping motor ID - sensors ID
    std::vector<std::string> sensor_weight_mappings_; // Mapping sensor ID - weight

    std::string measured_norm_topic_;       // Name of the topic for measured normalized forces
    std::string desired_norm_topic_;        // Name of the topic for desired normalized forces
    std::string proportional_result_topic_; // Name of the topic for publishing errors
    std::string set_gain_service_name_;     // Name of the service to set gain

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_forces_;  // Message for desired normalized forces
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_; // Message for measured normalized forces

    bool desired_norm_forces_received_ = false;  // Flag indicating if desired forces data has been received
    bool measured_norm_forces_received_ = false; // Flag indicating if measured forces data has been received

    // Mapping of motor IDs to their corresponding sensor IDs
    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;
    // Mapping of sensor IDs to their corresponding weights
    std::unordered_map<int64_t, std::vector<double>> sensor_to_weight_map_;

    // ROS 2 subscriptions to receive sensor data
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_forces_sub_;

    // ROS 2 publisher to publish motor errors
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr error_pub_;

    // ROS 2 service to handle requests for setting the gain value
    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;

    // Constructor for initializing the node
    ProportionalController()
        : Node("proportional_controller"),                                                                // Initialize the node with the name "proportional_controller"
          gain_(this->declare_parameter<double>("gain", 1.0)),                                            // Initialize gain from the ROS parameter (default: 1.0)
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())), // Initialize motor IDs from the ROS parameter
          motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>())),
          sensor_weight_mappings_(this->declare_parameter<std::vector<std::string>>("sensor_weight_mappings", std::vector<std::string>())),
          measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "norm_forces")),
          desired_norm_topic_(this->declare_parameter<std::string>("desired_norm_topic", "/cmd/desired_norm_forces")),
          proportional_result_topic_(this->declare_parameter<std::string>("proportional_result_topic_", "result_proportional_controller")),
          set_gain_service_name_(this->declare_parameter<std::string>("set_gain_service_name", "set_gain"))
    {
        // Check if the gain is non-negative; terminate if not
        if (gain_ < 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'gain' must be non-negative. Exiting...");
            throw std::invalid_argument("Parameter 'gain' must be non-negative");
        }

        // Check if motor IDs have been provided; terminate if not
        if (motor_ids_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_ids' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_ids' is empty or not set");
        }

        // Initialize the mapping between motors and their associated sensors
        initialize_motor_to_sensor_map();
        initialize_sensor_to_weight_map();

        // Create subscription to measured normalized forces
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10, std::bind(&ProportionalController::measured_norm_forces_callback, this, std::placeholders::_1));

        // Create subscription to desired normalized forces
        desired_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            desired_norm_topic_, 10, std::bind(&ProportionalController::desired_norm_forces_callback, this, std::placeholders::_1));

        // Create publisher for motor errors
        error_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            proportional_result_topic_, 10);

        // Create service for setting the gain
        set_gain_service_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SetGain>(
            set_gain_service_name_, std::bind(&ProportionalController::set_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void initialize_motor_to_sensor_map()
    {

        if (motor_sensor_mappings_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_sensor_mappings_' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_sensor_mappings_' is empty or not set");
        }

        for (const auto &mapping : motor_sensor_mappings_)
        {
            std::istringstream iss(mapping);
            int64_t motor_id;
            std::vector<int64_t> sensor_ids;
            char delimiter;

            if (!(iss >> motor_id >> delimiter))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid mapping format: %s", mapping.c_str());
                continue;
            }

            int64_t sensor_id;
            while (iss >> sensor_id)
            {
                sensor_ids.push_back(sensor_id);
                iss >> delimiter; // consume the comma if present
            }

            if (sensor_ids.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No sensor IDs found for motor ID: %ld", motor_id);
                continue;
            }

            motor_to_sensor_map_[motor_id] = sensor_ids;
            RCLCPP_INFO(this->get_logger(), "Mapped motor ID %ld to sensor IDs: %s",
                        motor_id, [&sensor_ids]()
                                  {
                            std::ostringstream oss;
                            for (size_t i = 0; i < sensor_ids.size(); ++i) {
                                if (i > 0) oss << ", ";
                                oss << sensor_ids[i];
                            }
                            return oss.str(); }().c_str());
        }

        if (motor_to_sensor_map_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "No valid motor-to-sensor mappings were created. Exiting...");
            throw std::runtime_error("No valid motor-to-sensor mappings were created");
        }
    }

    void initialize_sensor_to_weight_map()
    {

        if (sensor_weight_mappings_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'sensor_weight_mappings_' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'sensor_weight_mappings_' is empty or not set");
        }

        for (const auto &mapping : sensor_weight_mappings_)
        {
            std::istringstream iss(mapping);
            int64_t sensor_id;
            std::vector<double> weights;
            char delimiter;

            if (!(iss >> sensor_id >> delimiter))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid mapping format: %s", mapping.c_str());
                continue;
            }

            double weight;
            while (iss >> weight)
            {
                weights.push_back(weight);
                iss >> delimiter; // consume the comma if present
            }

            if (weights.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No weight found for sensor ID: %ld", sensor_id);
                continue;
            }

            sensor_to_weight_map_[sensor_id] = weights;
            RCLCPP_INFO(this->get_logger(), "Mapped sensor ID %ld to weights: %s",
                        sensor_id, [&weights]()
                                   {
                            std::ostringstream oss;
                            for (size_t i = 0; i < weights.size(); ++i) {
                                if (i > 0) oss << ", ";
                                oss << weights[i];
                            }
                            return oss.str(); }().c_str());
        }

        if (sensor_to_weight_map_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "No valid motor-to-sensor mappings were created. Exiting...");
            throw std::runtime_error("No valid motor-to-sensor mappings were created");
        }
    }

    // Callback function for receiving measured normalized forces
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_norm_forces_ = *msg;          // Store the received message
        measured_norm_forces_received_ = true; // Set the flag indicating that data has been received
        compute_and_publish_error();           // Compute and publish the motor errors
    }

    // Callback function for receiving desired normalized forces
    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        desired_norm_forces_ = *msg;          // Store the received message
        desired_norm_forces_received_ = true; // Set the flag indicating that data has been received
        compute_and_publish_error();          // Compute and publish the motor errors
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
        // VIENE VISUALIZZATO SEMPRE QUESTO, PER IL MOMENTO LO TOLGO
        // Check if both desired and measured forces data have been received
        // if (!desired_norm_forces_received_ || !measured_norm_forces_received_)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Desired or measured forces data not received yet.");
        //     return;
        // }

        // Initialize the motor error message
        uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped error_msg;
        error_msg.header.stamp = this->now(); // Set the timestamp for the message
        double error;
        // Iterate over each motor ID to compute the error
        for (int64_t motor_id : motor_ids_)
        {
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id); // Find the associated sensor IDs for the motor
            if (sensor_ids_iter == motor_to_sensor_map_.end())          // Check if the motor has a valid mapping
            {
                RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue;
            }

            const auto &sensor_ids = sensor_ids_iter->second; // Get the associated sensor IDs for the motor

            // Iterate over each sensor ID to compute the error
            for (int64_t sensor_id : sensor_ids)
            {
                // QUI DEVO TROVARE IL PESO/PESI ASSOCIATI A QUEL SENSORE
                auto weight_iter = sensor_to_weight_map_.find(sensor_id);
                if (weight_iter == sensor_to_weight_map_.end()) // Check if the sensor has a valid mapping
                {
                    RCLCPP_ERROR(this->get_logger(), "No mapping found for snesor ID: %ld", sensor_id);
                    continue;
                }
                // HO TROVATO I PESI ASSOCIATI AI SENSORI
                const auto &weights = weight_iter->second;

                // double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
                // double avg = sum/weights.size();

                // Find the index of the sensor ID in the measured forces message
                auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
                auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.ids.end(), sensor_id);

                // Check if the sensor ID is found in the measured forces
                if (measured_force_iter == measured_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in measured forces", sensor_id);
                    continue;
                }
                // Check if the sensor ID is found in the desired forces
                if (desired_force_iter == desired_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in desired forces", sensor_id);
                    continue;
                }

                // Compute the indices of the sensor ID in the measured and desired forces
                size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
                size_t desired_id = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

                // Retrieve the measured and desired normalized forces
                double measured_norm = measured_norm_forces_.data[measured_id];
                double desired_norm = desired_norm_forces_.data[desired_id];

                for (const auto &weight : weights)
                {
                    // Compute the error for the sensor
                    error = (desired_norm * weight - measured_norm)/(weights.size());
                }

                RCLCPP_INFO(this->get_logger(), "Error for Sensor ID: %ld - Error: %f", sensor_id, error);
            }
            // TODO: devi fare la media tra i valore degli ultimi due sensori
            //      perché stanno su un solo motore.
            //      Dopo che hai fatto la media, fuori dal for dei sensori, metti
            //      i push_back.
            // Add the motor ID and the computed error to the error message
            error_msg.ids.push_back(motor_id);
            error_msg.data.push_back(gain_ * error); // Apply the proportional gain to the error
            
        }
            // Publish the computed motor errors
            error_pub_->publish(error_msg);
        // Reset the flags for receiving new data
        desired_norm_forces_received_ = false;
        measured_norm_forces_received_ = false;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 client library
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

/*

#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp"
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <sstream> // Per l'analisi dei mapping
#include <utility> // Per std::pair

// Definition of the ProportionalController class
class ProportionalController : public rclcpp::Node
{
public:
    double gain_;
    std::vector<int64_t> motor_ids_;
    std::vector<std::string> motor_sensor_mappings_;

    std::vector<int64_t> sensor_idSS;

    std::string measured_norm_topic_;
    std::string desired_norm_topic_;
    std::string proportional_result_topic_;
    std::string set_gain_service_name_;

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_forces_;
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_;

    bool desired_norm_forces_received_ = false;
    bool measured_norm_forces_received_ = false;

    // Mappa dei pesi dei sensori associati ai motori
    // std::unordered_map<int64_t, std::unordered_map<int64_t, double>> motor_sensor_weights_map_;
    // std::unordered_map<int64_t, std::unordered_map<std::vector<int64_t>, std::vector<double>>> motor_sensor_weights_map_;

    // ROS 2 subscriptions to receive sensor data
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_forces_sub_;

    // ROS 2 publisher to publish motor errors
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr error_pub_;

    // ROS 2 service to handle requests for setting the gain value
    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;

    ProportionalController()
        : Node("proportional_controller"),
          gain_(this->declare_parameter<double>("gain", 1.0)),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),
          motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>())),
          measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "norm_forces")),
          desired_norm_topic_(this->declare_parameter<std::string>("desired_norm_topic", "/cmd/desired_norm_forces")),
          proportional_result_topic_(this->declare_parameter<std::string>("proportional_result_topic_", "result_proportional_controller")),
          set_gain_service_name_(this->declare_parameter<std::string>("set_gain_service_name", "set_gain"))
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

        initialize_motor_sensor_weights_map();

        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10, std::bind(&ProportionalController::measured_norm_forces_callback, this, std::placeholders::_1));

        desired_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            desired_norm_topic_, 10, std::bind(&ProportionalController::desired_norm_forces_callback, this, std::placeholders::_1));

        error_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            proportional_result_topic_, 10);

        set_gain_service_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SetGain>(
            set_gain_service_name_, std::bind(&ProportionalController::set_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void initialize_motor_sensor_weights_map()
    {
        if (!motor_sensor_mappings_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "OK, not empty");
        }

        std::string delimiter = ":";
        size_t pos = 0;
        std::string token;
        std::vector<std::string> tokens;

        for (const auto& element1 : motor_sensor_mappings_)
        {
            RCLCPP_INFO(this->get_logger(), "Element1, %s", element1.c_str());
        }

        while ((pos = element1.find(delimiter)) != std::string::npos)
        {
            token = s.substr(0, pos);
            tokens.push_back(token);
            s.erase(0, pos + delimiter.length());
        }
        ///////////////

        // if (motor_sensor_mappings_.empty())
        // {
        //     RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_sensor_mappings_' is empty or not set. Exiting...");
        //     throw std::runtime_error("Parameter 'motor_sensor_mappings_' is empty or not set");
        // }

        // for (const auto &mapping : motor_sensor_mappings_)
        // {
        //     std::istringstream iss(mapping);
        //     int64_t motor_id;
        //     int64_t sensor_id;
        //     double weight;
        //     char delimiter;

        //     if (!(iss >> motor_id >> delimiter))
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "Invalid mapping format: %s", mapping.c_str());
        //         continue;
        //     }

        //     std::unordered_map<int64_t, double> sensor_weight_map;

        //     while (iss >> motor_id >> delimiter >> sensor_id >> weight)
        //     {
        //         sensor_weight_map[sensor_id] = weight;
        //         iss >> delimiter; // Consuma un char se presente
        //     }

        //     if (sensor_weight_map.empty())
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "No sensor IDs found for motor ID: %ld", motor_id);
        //         continue;
        //     }

        //     motor_sensor_weights_map_[motor_id] = sensor_weight_map;

        //     // RCLCPP_INFO(this->get_logger(), "Mapped motor ID %ld to sensor IDs %ld with weights", motor_id, sensor_id);
        // }

        // if (motor_sensor_weights_map_.empty())
        // {
        //     RCLCPP_FATAL(this->get_logger(), "No valid motor-to-sensor mappings with weights were created. Exiting...");
        //     throw std::runtime_error("No valid motor-to-sensor mappings with weights were created");
        // }
    }

    // double get_sensor_weight(int64_t motor_id, int64_t sensor_id)
    // {
    //     auto motor_iter = motor_sensor_weights_map_.find(motor_id);
    //     if (motor_iter != motor_sensor_weights_map_.end())
    //     {
    //         auto sensor_iter = motor_iter->second.find(sensor_id);
    //         if (sensor_iter != motor_iter->second.end())
    //         {
    //             return sensor_iter->second;
    //         }
    //     }
    //     return 1.0; // Peso di default se il sensore non è trovato
    // }

    // void compute_and_publish_error()
    // {
    //     if (!desired_norm_forces_received_ || !measured_norm_forces_received_)
    //     {
    //         RCLCPP_WARN(this->get_logger(), "Desired or measured forces data not received yet.");
    //         return;
    //     }

    //     uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped error_msg;
    //     error_msg.header.stamp = this->now();

    //     for (int64_t motor_id : motor_ids_)
    //     {
    //         auto sensor_ids_iter = motor_sensor_weights_map_.find(motor_id);
    //         if (sensor_ids_iter == motor_sensor_weights_map_.end())
    //         {
    //             RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
    //             continue;
    //         }

    //         const auto &sensor_ids = sensor_ids_iter->second;

    //         for (auto it = sensor_ids.begin(); it != sensor_ids.end(); it++)
    //         {
    //             sensor_idSS.push_back(it->first);
    //         }

    //         double weighted_error_sum = 0.0;
    //         double total_weight = 0.0;

    //         for (int64_t sensor_id : sensor_idSS)
    //         {
    //             auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
    //             auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.ids.end(), sensor_id);

    //             if (measured_force_iter == measured_norm_forces_.ids.end())
    //             {
    //                 RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in measured forces", sensor_id);
    //                 continue;
    //             }
    //             if (desired_force_iter == desired_norm_forces_.ids.end())
    //             {
    //                 RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in desired forces", sensor_id);
    //                 continue;
    //             }

    //             size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
    //             size_t desired_id = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

    //             double measured_norm = measured_norm_forces_.data[measured_id];
    //             double desired_norm = desired_norm_forces_.data[desired_id];

    //             double error = desired_norm - measured_norm;

    //             double weight = get_sensor_weight(motor_id, sensor_id);
    //             weighted_error_sum += weight * error;
    //             total_weight += weight;

    //             RCLCPP_INFO(this->get_logger(), "Error for Sensor ID: %ld - Error: %f, Weight: %f", sensor_id, error, weight);
    //         }

    //         double average_error = total_weight > 0.0 ? weighted_error_sum / total_weight : 0.0;

    //         error_msg.ids.push_back(motor_id);
    //         error_msg.data.push_back(gain_ * average_error);
    //     }

    //     error_pub_->publish(error_msg);

    //     desired_norm_forces_received_ = false;
    //     measured_norm_forces_received_ = false;
    // }

    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_norm_forces_ = *msg;
        measured_norm_forces_received_ = true;
        // compute_and_publish_error();
    }

    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        desired_norm_forces_ = *msg;
        desired_norm_forces_received_ = true;
        // compute_and_publish_error();
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
*/