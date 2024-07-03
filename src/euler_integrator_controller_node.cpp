#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_velocities.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>
#include <vector>

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;
    std::vector<int64_t> motor_ids_;

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;
    std::vector<float> desired_velocity_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorVelocities>::SharedPtr desired_velocity_sub;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    EulerIntegrator()
        : Node("euler_integrator"),
          dt_(this->declare_parameter<double>("dt", 0.1)),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>()))
    {
        desired_velocity_.reserve(motor_ids_.size()); 
        
        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "startstop", std::bind(&EulerIntegrator::service_callback, this, std::placeholders::_1, std::placeholders::_2));

        desired_velocity_sub = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorVelocities>(
            "/cmd/desired_velocity", 10, std::bind(&EulerIntegrator::desired_velocity_callback, this, std::placeholders::_1));

        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "desired_position", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), std::bind(&EulerIntegrator::integrate, this));

        
        // eccezione se motor_ids_ è vuoto e si deve incazzare

        
        


        timer_->cancel();
    }

private:
    void integrate()
    {
        // for (size_t i = 0; i < motor_ids_.size(); i++)
        // {
        //     // cerca in motor_positions_, che proviene dal wait_for_message della service_callback,
        //     // il motor_ids_[i] cioè il singolo motore presente nella lista dei motori passati
        //     // come parametro.
        //     // Se la ricerca è ok, it contiene l'id
        //     auto it = std::find(motor_positions_.ids.begin(), motor_positions_.ids.end(), motor_ids_[i]);
        //     if (it != motor_positions_.ids.end()) // se non è stato trovato, l'iteratore ritorna l'ultima posizione del vettore
        //     {
        //         // mi devo trovare l'indice del motor_ids_[i] all'interno di motor_positions_ (perchè questo potrebbe contenere comunque
        //         // tutti gli id dei motori perché viene inizializzato quando lancio la mano)
        //         size_t index = std::distance(motor_positions_.ids.begin(), it);
        //         // desired_velocity_ proviene dalla desired_velocity_callback del subscriber per la velocità
        //         motor_positions_.positions[index] += dt_ * desired_velocity_[index];
        //     }
        // }
        for (size_t i = 0; i < motor_positions_.ids.size(); i++)
        {
            motor_positions_.positions[i] += dt_ * desired_velocity_[i]; // QUI BISOGNA AGGIUNGERE L'ERRORE
        }
        

        desired_position_pub_->publish(motor_positions_);
    }

    void desired_velocity_callback(const uclv_seed_robotics_ros_interfaces::msg::MotorVelocities::SharedPtr msg)
    {
        desired_velocity_.clear();
        for (size_t i = 0; i < motor_ids_.size(); i++)
        {
            auto it = std::find(msg->ids.begin(), msg->ids.end(), motor_ids_[i]);
            if (it != msg->ids.end())
            {
                desired_velocity_.push_back(msg->velocities[i]);
            }
            else
            {
                // ERRORE SE QUEL msg->ids NON SI TROVA NELLA LISTA DEI MOTORI PASSATI AL LAUNCH FILE DELLA MANO
                // SI DEVE INCAZZARE
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
                // metti eccezione wait for message se non arriva il message
                if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                        motor_positions_, this->shared_from_this(), "/motor_state", std::chrono::seconds(1)))
                {
                    desired_velocity_.resize(motor_ids_.size(), 0.0);

                    uclv_seed_robotics_ros_interfaces::msg::MotorPositions filtered_positions;
                    for (size_t i = 0; i < motor_ids_.size(); i++)
                    {
                        auto it = std::find(motor_positions_.ids.begin(), motor_positions_.ids.end(), motor_ids_[i]);
                        if (it != motor_positions_.ids.end())
                        {
                            size_t index = std::distance(motor_positions_.ids.begin(), it);
                            filtered_positions.ids.push_back(motor_positions_.ids[index]);
                            filtered_positions.positions.push_back(motor_positions_.positions[index]);
                        }
                        else{
                            // ERRORE!
                            return;
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
                    return;
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