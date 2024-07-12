#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <vector>
#include <stdexcept>

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;
    std::vector<int64_t> motor_ids_;
    uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr proportional_result_;
    bool proportional_result_received_;

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr proportional_result_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    EulerIntegrator()
        : Node("euler_integrator"),
          dt_(this->declare_parameter<double>("dt", 0.1)),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),
          proportional_result_received_(false)
    {
        if (motor_ids_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_ids' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_ids' is empty or not set");
        }

        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "startstop", std::bind(&EulerIntegrator::service_callback, this, std::placeholders::_1, std::placeholders::_2));

        proportional_result_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            "/result_proportional_controller", 1, std::bind(&EulerIntegrator::proportional_result_callback, this, std::placeholders::_1));

        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "/desired_position", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), std::bind(&EulerIntegrator::integrate, this));

        timer_->cancel();
    }

private:
    void integrate()
    {
        if (proportional_result_received_)
        {
            for (size_t i = 0; i < motor_positions_.ids.size(); i++)
            {
                // RCLCPP_INFO(this->get_logger(), "Before Integration - Motor ID: %ld, Position: %f, Force Z: %f",
                //             motor_positions_.ids[i], motor_positions_.positions[i], proportional_result_->forces[i].z);


                // questo fa l'integrazione su motor positions positions ma io la devo fare su solo quelli che stanno 
                // nella mappa motore-sensore


                motor_positions_.positions[i] += dt_ * proportional_result_->forces[i].z; 
            //     RCLCPP_INFO(this->get_logger(), "After Integration - Motor ID: %ld, New Position: %f",
            //                 motor_positions_.ids[i], motor_positions_.positions[i]);
            }

            desired_position_pub_->publish(motor_positions_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Proportional result not received yet.");
        }
    }

    void proportional_result_callback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        proportional_result_ = msg;
        proportional_result_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Received proportional result:");
        for (size_t i = 0; i < msg->forces.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Sensor ID: %ld, Force: (%f, %f, %f)",
                        msg->ids[i], msg->forces[i].x, msg->forces[i].y, msg->forces[i].z);
        }
    }

    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            if (timer_->is_canceled())
            {
                if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                        motor_positions_, this->shared_from_this(), "/motor_state", std::chrono::seconds(1)))
                {
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
                        else
                        {
                            RCLCPP_FATAL(this->get_logger(), "Motor ID %ld from parameter 'motor_ids' not found in received motor positions. Exiting...", motor_ids_[i]);
                            throw std::runtime_error("Motor ID from parameter 'motor_ids' not found in received motor positions");
                        }
                    }
                    motor_positions_ = filtered_positions;

                    // for (size_t i = 0; i < motor_positions_.positions.size(); i++)
                    // {
                    //     RCLCPP_INFO(this->get_logger(), "ID: %ld, Initial Position: %f", motor_positions_.ids[i], motor_positions_.positions[i]);
                    // }

                    timer_->reset();
                    response->success = true;
                    response->message = "Timer started.";
                    RCLCPP_INFO(this->get_logger(), "Timer started.");
                }
                else
                {
                    RCLCPP_FATAL(this->get_logger(), "Failed to receive initial motor positions. Exiting...");
                    throw std::runtime_error("Failed to receive initial motor positions");
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