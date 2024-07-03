#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_velocities.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>
#include <vector>
#include <unordered_map>

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;
    std::vector<int64_t> motor_ids_;

    bool initial_condition_received_;

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;
    std::vector<float> velocities_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorVelocities>::SharedPtr velocity_value_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    EulerIntegrator()
        : Node("euler_integrator"),
          initial_condition_received_(false)
    {
        dt_ = this->declare_parameter<double>("dt", double());
        motor_ids_ = this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>());

        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "startstop", std::bind(&EulerIntegrator::service_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));

        velocity_value_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorVelocities>(
            "/cmd/velocity_value", 10,
            std::bind(&EulerIntegrator::velocity_callback, this, std::placeholders::_1));

        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "desired_position", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&EulerIntegrator::integrate, this));

        timer_->cancel();
    }

private:
    void integrate()
    {
        for (size_t i = 0; i < motor_ids_.size(); i++)
        {
            // cerca in motor_positions_, che proviene dal wait_for_message della service_callback,
            // il motor_ids_[i] cioè il singolo motore presente nella lista dei motori passati
            // come parametro. 
            // Se la ricerca è ok, it contiene l'id
            auto it = std::find(motor_positions_.ids.begin(), motor_positions_.ids.end(), motor_ids_[i]);
            if (it != motor_positions_.ids.end()) // se non è stato trovato, l'iteratore ritorna l'ultima posizione del vettore
            {
                // mi devo trovare l'indice del motor_ids_[i] all'interno di motor_positions_ (perchè questo potrebbe contenere comunque
                // tutti gli id dei motori perché viene inizializzato quando lancio la mano)
                size_t index = std::distance(motor_positions_.ids.begin(), it);
                // velocities_ proviene dalla velocity_callback del subscriber per la velocità
                motor_positions_.positions[index] += dt_ * velocities_[index];
            }
        }
        
        desired_position_pub_->publish(motor_positions_);
    }

    void velocity_callback(const uclv_seed_robotics_ros_interfaces::msg::MotorVelocities::SharedPtr msg)
    {
        velocities_.clear();
        // per ogni id nel messaggio inviato al topic /cmd/velocity_value
        for (size_t i = 0; i < msg->ids.size(); i++)
        {
            // cerco in motor_ids_, cioè il vettore passato come parametro che contiene gli id dei motori che mi interessano, 
            // l'id presente nel messaggio inviato al topic
            auto it = std::find(motor_ids_.begin(), motor_ids_.end(), msg->ids[i]);
            if (it != motor_ids_.end()) // se non è stato trovato, l'iteratore ritorna l'ultima posizione del vettore
            {
                velocities_.push_back(msg->velocities[i]);
            }
        }
    }

    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            if (timer_->is_canceled())
            {
                // Wait for the initial condition
                if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                        motor_positions_, this->shared_from_this(), "/motor_state", std::chrono::seconds(1)))
                {
                    initial_condition_received_ = true;
                    // resize del vettore delle velocità e V = 0
                    velocities_.resize(motor_ids_.size(), 0.0);

                    uclv_seed_robotics_ros_interfaces::msg::MotorPositions filtered_positions;
                    // per ogni motore all'interno del vettore passato come parametro
                    for (size_t i = 0; i < motor_ids_.size(); i++)
                    {
                        
                        auto it = std::find(motor_positions_.ids.begin(), motor_positions_.ids.end(), motor_ids_[i]);
                        if (it != motor_positions_.ids.end())
                        {
                            size_t index = std::distance(motor_positions_.ids.begin(), it);
                            filtered_positions.ids.push_back(motor_positions_.ids[index]);
                            filtered_positions.positions.push_back(motor_positions_.positions[index]);
                        }
                    }
                    motor_positions_ = filtered_positions;

                    for (size_t i = 0; i < motor_positions_.positions.size(); i++)
                    {
                        RCLCPP_INFO(this->get_logger(), "ID: %d, Initial Position: %f", motor_positions_.ids[i], motor_positions_.positions[i]);
                    }

                    timer_->reset();
                    response->success = true;
                    response->message = "Timer started.";
                    RCLCPP_INFO(this->get_logger(), "Timer started.");
                }
                else
                {
                    response->success = false;
                    response->message = "Failed to receive initial motor positions.";
                    RCLCPP_WARN(this->get_logger(), "Failed to receive initial motor positions.");
                }
            }
            else
            {
                response->success = false;
                response->message = "Timer is already running.";
                RCLCPP_WARN(this->get_logger(), "Timer is already running.");
            }
        }
        else
        {
            if (!timer_->is_canceled())
            {
                timer_->cancel();
                response->success = true;
                response->message = "Timer stopped.";
                RCLCPP_INFO(this->get_logger(), "Timer stopped.");
            }
            else
            {
                response->success = true;
                response->message = "Timer was not running.";
                RCLCPP_WARN(this->get_logger(), "Timer is not running.");
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto euler_integrator_node = std::make_shared<EulerIntegrator>();
        rclcpp::spin(euler_integrator_node);
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
