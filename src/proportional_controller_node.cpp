#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/sensors_norm.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_stamped.hpp"
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <algorithm> // Necessario per std::find

class ProportionalController : public rclcpp::Node
{
public:
    double gain_; // Valore di guadagno per il controllo proporzionale
    std::vector<int64_t> motor_ids_; // Lista degli ID dei motori gestiti dal controller

    // Messaggi per memorizzare le forze desiderate e misurate
    uclv_seed_robotics_ros_interfaces::msg::SensorsNorm desired_norm_forces_;
    uclv_seed_robotics_ros_interfaces::msg::SensorsNorm measured_norm_forces_;
    
    // Flag per tracciare se le forze desiderate e misurate sono state ricevute
    bool desired_norm_forces_received_ = false;
    bool measured_norm_forces_received_ = false;

    // Mappa per associare i motori ai loro sensori corrispondenti
    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;

    // Sottoscrizioni per ricevere lo stato del sensore e le forze desiderate
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr desired_norm_forces_sub_;

    // Publisher per pubblicare il risultato del controllo proporzionale
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64Stamped>::SharedPtr result_pub_;

    ProportionalController()
        : Node("proportional_controller"),
          gain_(this->declare_parameter<double>("gain", 1.0)),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>()))
    {
        // Controllo che il parametro di guadagno sia impostato correttamente (non negativo)
        if (gain_ < 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'gain' must be non-negative. Exiting...");
            throw std::invalid_argument("Parameter 'gain' must be non-negative");
        }

        // Assicurarsi che la lista degli ID dei motori non sia vuota
        if (motor_ids_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_ids' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_ids' is empty or not set");
        }

        // Inizializza la mappa tra motori e sensori
        initialize_motor_to_sensor_map();

        // Sottoscrivi ai topic per ricevere gli stati dei sensori e le forze desiderate
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            "norm_forces", 1, std::bind(&ProportionalController::measured_norm_forces_callback, this, std::placeholders::_1));

        desired_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            "/cmd/desired_norm_forces", 1, std::bind(&ProportionalController::desired_norm_forces_callback, this, std::placeholders::_1));

        // Crea publisher per pubblicare i risultati del controllo
        result_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64Stamped>(
            "/result_proportional_controller", 1);
    }

private:
    // Funzione per inizializzare la mappa tra motori e sensori
    void initialize_motor_to_sensor_map()
    {
        motor_to_sensor_map_[35] = {0};
        motor_to_sensor_map_[36] = {1};
        motor_to_sensor_map_[37] = {2};
        motor_to_sensor_map_[38] = {3, 4};
    }

    // Callback per ricevere i messaggi sullo stato dei sensori
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::SensorsNorm::SharedPtr msg)
    {
        measured_norm_forces_ = *msg; // Aggiorna le forze misurate
        measured_norm_forces_received_ = true;
        compute_and_publish_result(); // Prova a calcolare e pubblicare il risultato del controllo
    }

    // Callback per ricevere i messaggi sulle forze desiderate
    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::SensorsNorm::SharedPtr msg)
    {
        desired_norm_forces_ = *msg; // Aggiorna le forze desiderate
        desired_norm_forces_received_ = true;
        compute_and_publish_result(); // Prova a calcolare e pubblicare il risultato del controllo
    }

    // Funzione per calcolare il risultato del controllo e pubblicarlo
    void compute_and_publish_result()
    {
        // Verifica se i dati delle forze desiderate o misurate sono mancanti
        if (!desired_norm_forces_received_ || !measured_norm_forces_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Desired or measured forces data not received yet.");
            return;
        }

        // Itera sugli ID dei motori gestiti dal controller
        for (int64_t motor_id : motor_ids_)
        {
            // Trova l'ID del motore nella mappa dei sensori
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id);
            if (sensor_ids_iter == motor_to_sensor_map_.end())
            {
                RCLCPP_FATAL(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue;
            }

            // Ottieni gli ID dei sensori associati
            const auto &sensor_ids = sensor_ids_iter->second;

            // Itera sugli ID dei sensori associati al motore
            for (int64_t sensor_id : sensor_ids)
            {
                // Trova l'indice dell'ID sensore nelle liste delle forze desiderate e misurate
                auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
                auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.ids.end(), sensor_id);

                // Verifica se l'ID del sensore si trova sia nelle liste delle forze misurate che desiderate
                if (measured_force_iter == measured_norm_forces_.ids.end())
                {
                    RCLCPP_FATAL(this->get_logger(), "Sensor ID: %ld not found in measured forces", sensor_id);
                    continue;
                }
                if (desired_force_iter == desired_norm_forces_.ids.end())
                {
                    RCLCPP_FATAL(this->get_logger(), "Sensor ID: %ld not found in desired forces", sensor_id);
                    continue;
                }

                // Ottieni gli indici per le forze misurate e desiderate
                size_t measured_idx = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
                size_t desired_idx = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

                // Estrai le norme misurate e desiderate
                double measured_norm = measured_norm_forces_.norm[measured_idx].data; // Estrai il campo `data`
                double desired_norm = desired_norm_forces_.norm[desired_idx].data; // Estrai il campo `data`

                // Calcola l'errore tra le norme desiderate e misurate
                double error = desired_norm - measured_norm;

                RCLCPP_INFO(this->get_logger(), "Error for Sensor ID: %ld - Error: %f", sensor_id, error);

                // Crea un messaggio per il risultato del controllo proporzionale
                uclv_seed_robotics_ros_interfaces::msg::Float64Stamped result_msg;
                result_msg.header.stamp = this->now(); // Aggiungi il timestamp al messaggio
                result_msg.data = gain_ * error; // Applica il controllo proporzionale all'errore

                // Log del risultato computato
                RCLCPP_INFO(this->get_logger(), "Computed result for Sensor ID: %ld - Norm: %f", sensor_id, result_msg.data);

                // Pubblica il risultato del controllo
                result_pub_->publish(result_msg);
            }
        }

        // Reimposta i flag per garantire che i dati siano freschi per il prossimo calcolo
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
