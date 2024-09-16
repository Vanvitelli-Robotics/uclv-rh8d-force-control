/*
#include "rclcpp/rclcpp.hpp"  // Include the main header for ROS 2 C++ client library
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp"  // Include custom service type for setting the gain value
#include <stdexcept>  // Include standard library for handling exceptions
#include <unordered_map>  // Include standard library for using hash maps
#include <vector>  // Include standard library for using vectors
#include <algorithm>  // Include standard library for common algorithms like std::find

// Definition of the ProportionalController class, which inherits from rclcpp::Node
class ProportionalController : public rclcpp::Node
{
public:
    double gain_; // Proportional gain value for the controller
    std::vector<int64_t> motor_ids_; // List of motor IDs managed by the controller
    std::vector<std::string> motor_sensor_mappings_; // Mapping motor ID - Sensors

    std::string measured_norm_topic_;  // Name of the topic for measured normalized forces
    std::string desired_norm_topic_;   // Name of the topic for desired normalized forces
    std::string proportional_result_topic_;      // Name of the topic for publishing errors
    std::string set_gain_service_name_; // Name of the service to set gain

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_forces_;  // Message for desired normalized forces
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_; // Message for measured normalized forces

    bool desired_norm_forces_received_ = false;  // Flag indicating if desired forces data has been received
    bool measured_norm_forces_received_ = false; // Flag indicating if measured forces data has been received

    // Mapping of motor IDs to their corresponding sensor IDs
    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;

    // ROS 2 subscriptions to receive sensor data
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_forces_sub_;

    // ROS 2 publisher to publish motor errors
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr error_pub_;

    // ROS 2 service to handle requests for setting the gain value
    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;

    // Constructor for initializing the node
    ProportionalController()
        : Node("proportional_controller"),  // Initialize the node with the name "proportional_controller"
          gain_(this->declare_parameter<double>("gain", 1.0)), // Initialize gain from the ROS parameter (default: 1.0)
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),  // Initialize motor IDs from the ROS parameter
          motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>())),
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

        for (const auto& mapping : motor_sensor_mappings_)
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
                        motor_id, [&sensor_ids]() {
                            std::ostringstream oss;
                            for (size_t i = 0; i < sensor_ids.size(); ++i) {
                                if (i > 0) oss << ", ";
                                oss << sensor_ids[i];
                            }
                            return oss.str();
                        }().c_str());
        }

        if (motor_to_sensor_map_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "No valid motor-to-sensor mappings were created. Exiting...");
            throw std::runtime_error("No valid motor-to-sensor mappings were created");
        }
    }

    // Callback function for receiving measured normalized forces
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_norm_forces_ = *msg;  // Store the received message
        measured_norm_forces_received_ = true;  // Set the flag indicating that data has been received
        compute_and_publish_error();  // Compute and publish the motor errors
    }

    // Callback function for receiving desired normalized forces
    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        desired_norm_forces_ = *msg;  // Store the received message
        desired_norm_forces_received_ = true;  // Set the flag indicating that data has been received
        compute_and_publish_error();  // Compute and publish the motor errors
    }

    // Callback function for setting the gain value
    void set_gain_callback(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Request> request,
                           std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Response> response)
    {
        if (request->gain < 0.0)  // Check if the requested gain is negative
        {
            response->success = false;
            response->message = "Gain must be non-negative.";
            RCLCPP_WARN(this->get_logger(), "Attempted to set a negative gain.");
        }
        else  // Update the gain value if it is valid
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
        // Check if both desired and measured forces data have been received
        if (!desired_norm_forces_received_ || !measured_norm_forces_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Desired or measured forces data not received yet.");
            return;
        }

        // Initialize the motor error message
        uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped error_msg;
        error_msg.header.stamp = this->now();  // Set the timestamp for the message

        // Iterate over each motor ID to compute the error
        for (int64_t motor_id : motor_ids_)
        {
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id);  // Find the associated sensor IDs for the motor
            if (sensor_ids_iter == motor_to_sensor_map_.end())  // Check if the motor has a valid mapping
            {
                RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue;
            }

            const auto &sensor_ids = sensor_ids_iter->second;  // Get the associated sensor IDs for the motor

            // Iterate over each sensor ID to compute the error
            for (int64_t sensor_id : sensor_ids)
            {
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

                // Compute the error for the sensor
                double error = desired_norm - measured_norm;

                RCLCPP_INFO(this->get_logger(), "Error for Sensor ID: %ld - Error: %f", sensor_id, error);
                
                // TODO: devi fare la media tra i valore degli ultimi due sensori 
                //      perché stanno su un solo motore.
                //      Dopo che hai fatto la media, fuori dal for dei sensori, metti
                //      i push_back.
                // Add the motor ID and the computed error to the error message
                error_msg.ids.push_back(motor_id);
                error_msg.data.push_back(gain_ * error);  // Apply the proportional gain to the error
                
                
            }
            
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
    rclcpp::init(argc, argv);  // Initialize the ROS 2 client library
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


#include "rclcpp/rclcpp.hpp"  // Include the main header for ROS 2 C++ client library
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp"  // Include custom service type for setting the gain value
#include <stdexcept>  // Include standard library for handling exceptions
#include <unordered_map>  // Include standard library for using hash maps
#include <vector>  // Include standard library for using vectors
#include <algorithm>  // Include standard library for common algorithms like std::find
#include <sstream>  // Include standard library for parsing strings

// Definition of the ProportionalController class, which inherits from rclcpp::Node
class ProportionalController : public rclcpp::Node
{
public:
    double gain_; // Proportional gain value for the controller
    std::vector<int64_t> motor_ids_; // List of motor IDs managed by the controller
    std::vector<std::string> motor_sensor_mappings_; // Mapping motor ID - Sensors

    std::string measured_norm_topic_;  // Name of the topic for measured normalized forces
    std::string desired_norm_topic_;   // Name of the topic for desired normalized forces
    std::string proportional_result_topic_;      // Name of the topic for publishing errors
    std::string set_gain_service_name_; // Name of the service to set gain

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_forces_;  // Message for desired normalized forces
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_; // Message for measured normalized forces

    bool desired_norm_forces_received_ = false;  // Flag indicating if desired forces data has been received
    bool measured_norm_forces_received_ = false; // Flag indicating if measured forces data has been received

    // Mapping of motor IDs to their corresponding sensor IDs and weights
    std::unordered_map<int64_t, std::unordered_map<int64_t, double>> motor_sensor_weights_map_;

    // ROS 2 subscriptions to receive sensor data
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_forces_sub_;

    // ROS 2 publisher to publish motor errors
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr error_pub_;

    // ROS 2 service to handle requests for setting the gain value
    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;


    // Constructor for initializing the node
    ProportionalController()
        : Node("proportional_controller"),  // Initialize the node with the name "proportional_controller"
          gain_(this->declare_parameter<double>("gain", 1.0)), // Initialize gain from the ROS parameter (default: 1.0)
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),  // Initialize motor IDs from the ROS parameter
          motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>())),
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

    private:

    // Metodo per inizializzare la mappa dei pesi dei sensori
    void initialize_motor_sensor_weights_map()
    {
        if (motor_sensor_mappings_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_sensor_mappings_' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_sensor_mappings_' is empty or not set");
        }

        for (const auto& mapping : motor_sensor_mappings_)
        {
            std::istringstream iss(mapping);
            int64_t motor_id;
            std::vector<std::pair<int64_t, double>> sensor_weight_pairs; // Paia di sensore e peso
            char delimiter;

            if (!(iss >> motor_id >> delimiter))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid mapping format: %s", mapping.c_str());
                continue;
            }

            int64_t sensor_id;
            double weight = 1.0;  // Default weight
            while (iss >> sensor_id)
            {
                sensor_weight_pairs.emplace_back(sensor_id, weight);
                iss >> delimiter; // Consuma la virgola se presente
            }

            if (sensor_weight_pairs.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No sensor IDs found for motor ID: %ld", motor_id);
                continue;
            }

            std::unordered_map<int64_t, double> sensor_weight_map;
            for (const auto& pair : sensor_weight_pairs)
            {
                sensor_weight_map[pair.first] = pair.second;
            }

            motor_sensor_weights_map_[motor_id] = sensor_weight_map;
            RCLCPP_INFO(this->get_logger(), "Mapped motor ID %ld to sensor IDs and weights", motor_id);
        }

        if (motor_sensor_weights_map_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "No valid motor-to-sensor mappings with weights were created. Exiting...");
            throw std::runtime_error("No valid motor-to-sensor mappings with weights were created");
        }
    }

    // Metodo per ottenere il peso di un sensore dato un motor_id e sensor_id
    double get_sensor_weight(int64_t motor_id, int64_t sensor_id)
    {
        auto motor_iter = motor_sensor_weights_map_.find(motor_id);
        if (motor_iter != motor_sensor_weights_map_.end())
        {
            auto sensor_iter = motor_iter->second.find(sensor_id);
            if (sensor_iter != motor_iter->second.end())
            {
                return sensor_iter->second;
            }
        }
        return 1.0;  // Peso di default se il sensore non è trovato
    }


void compute_and_publish_error()
{
    // Controlla se i dati delle forze desiderate e misurate sono stati ricevuti.
    if (!desired_norm_forces_received_ || !measured_norm_forces_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Desired or measured forces data not received yet.");
        return;  // Esce dalla funzione se i dati non sono ancora arrivati
    }

    // Inizializza il messaggio di errore che verrà pubblicato
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped error_msg;
    error_msg.header.stamp = this->now();  // Imposta il timestamp del messaggio

    // Itera su ciascun motore ID per calcolare l'errore
    for (int64_t motor_id : motor_ids_)
    {
        // Trova i sensori associati a questo motore
        auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id);
        if (sensor_ids_iter == motor_to_sensor_map_.end())  // Se non esiste una mappatura per il motore
        {
            RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
            continue;
        }

        // Recupera i sensori associati a questo motore
        const auto &sensor_ids = sensor_ids_iter->second;

        // Inizializza variabili per calcolare la media pesata degli errori dei sensori
        double weighted_error_sum = 0.0;  // Somma degli errori pesati
        double total_weight = 0.0;        // Somma dei pesi

        // Itera su ciascun sensore associato a questo motore
        for (int64_t sensor_id : sensor_ids)
        {
            // Trova l'indice del sensore nel messaggio delle forze misurate
            auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
            auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.end(), sensor_id);

            // Se il sensore non è trovato nel messaggio delle forze misurate, segnala l'errore
            if (measured_force_iter == measured_norm_forces_.ids.end())
            {
                RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in measured forces", sensor_id);
                continue;  // Passa al sensore successivo
            }

            // Se il sensore non è trovato nel messaggio delle forze desiderate, segnala l'errore
            if (desired_force_iter == desired_norm_forces_.ids.end())
            {
                RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in desired forces", sensor_id);
                continue;  // Passa al sensore successivo
            }

            // Calcola gli indici del sensore nelle liste delle forze misurate e desiderate
            size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
            size_t desired_id = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

            // Recupera le forze misurate e desiderate per il sensore corrente
            double measured_norm = measured_norm_forces_.data[measured_id];
            double desired_norm = desired_norm_forces_.data[desired_id];

            // Calcola l'errore del sensore come la differenza tra forza desiderata e misurata
            double sensor_error = desired_norm - measured_norm;

            // Recupera il peso del sensore
            double weight = get_sensor_weight(motor_id, sensor_id);

            // Somma l'errore pesato
            weighted_error_sum += sensor_error * weight;
            total_weight += weight;  // Somma il peso attuale

            RCLCPP_INFO(this->get_logger(), "Sensor ID: %ld - Error: %f, Weight: %f", sensor_id, sensor_error, weight);
        }

        // Se ci sono sensori validi, calcola la media pesata
        if (total_weight > 0.0)
        {
            double motor_error = weighted_error_sum / total_weight;  // Media pesata dell'errore

            // Applica il guadagno proporzionale e aggiungi il risultato al messaggio di errore
            error_msg.ids.push_back(motor_id);  // Aggiunge l'ID del motore
            error_msg.data.push_back(gain_ * motor_error);  // Aggiunge l'errore proporzionale calcolato
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No valid sensor data for motor ID: %ld", motor_id);
        }
    }

    // Pubblica il messaggio degli errori dei motori
    error_pub_->publish(error_msg);

    // Resetta i flag per ricevere nuovi dati
    desired_norm_forces_received_ = false;
    measured_norm_forces_received_ = false;
}




// Main function to run the ROS node
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<ProportionalController>());  // Start the controller node
    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;
}

