#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <vector>
#include <stdexcept>
#include <algorithm>

class EulerIntegratorController : public rclcpp::Node
{
public:
    double dt_;
    std::vector<int64_t> motor_ids_;
    std::vector<int64_t> motor_thresholds_;
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr measured_velocity_;
    bool measured_velocity_received_;

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr integrator_service_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_velocity_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string measured_velocity_topic_;
    std::string desired_position_topic_;
    std::string integrator_service_name_;
    std::string motor_state_topic_;

    EulerIntegratorController()
        : Node("euler_integrator_controller"),
        measured_velocity_received_(false),
        dt_(this->declare_parameter<double>("dt", 0.0)),
        motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),
        motor_thresholds_(this->declare_parameter<std::vector<int64_t>>("motor_thresholds", std::vector<int64_t>())),
        measured_velocity_topic_(this->declare_parameter<std::string>("measured_velocity_topic", "")),
        desired_position_topic_(this->declare_parameter<std::string>("desired_position_topic", "")),
        integrator_service_name_(this->declare_parameter<std::string>("integrator_service_name", "")),
        motor_state_topic_(this->declare_parameter<std::string>("motor_state_topic", ""))
    {
        check_parameters();

        integrator_service_ = create_service<std_srvs::srv::SetBool>(
            integrator_service_name_, std::bind(&EulerIntegratorController::service_callback, this, std::placeholders::_1, std::placeholders::_2));

        measured_velocity_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_velocity_topic_, 1, std::bind(&EulerIntegratorController::measured_velocity_callback, this, std::placeholders::_1));

        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            desired_position_topic_, 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), std::bind(&EulerIntegratorController::integrate, this));

        timer_->cancel();
    }

private:


void check_parameters()
    {
        auto check_string_parameter = [this](const std::string &param_name, const std::string &value)
        {
            if (value.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid value.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };


        auto check_double_parameter = [this](const std::string &param_name, const double &value)
        {
            if (value == 0.0)
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid vector.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };

        auto check_vector_int_parameter = [this](const std::string &param_name, const std::vector<int64_t> &value)
        {
            if (value.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid vector.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };

        auto check_vector_string_parameter = [this](const std::string &param_name, const std::vector<std::string> &value)
        {
            if (value.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid vector.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };
        
        check_double_parameter("dt", dt_);
        check_vector_int_parameter("motor_ids", motor_ids_);
        check_vector_int_parameter("motor_thresholds", motor_thresholds_);
        check_string_parameter("measured_velocity_topic", measured_velocity_topic_);
        check_string_parameter("desired_position_topic", desired_position_topic_);
        check_string_parameter("integrator_service_name", integrator_service_name_);
    
        RCLCPP_INFO(this->get_logger(), "All required parameters are set correctly.");
    }





    void integrate()
    {
        if (measured_velocity_received_)
        {
            for (size_t i = 0; i < motor_positions_.ids.size(); i++)
            {
                auto it = std::find(measured_velocity_->ids.begin(), measured_velocity_->ids.end(), motor_positions_.ids[i]);
                if (it != measured_velocity_->ids.end())
                {
                    motor_positions_.header.stamp = rclcpp::Clock{}.now();  // Get current time
                    size_t index = std::distance(measured_velocity_->ids.begin(), it);

                    motor_positions_.positions[i] += dt_ * measured_velocity_->data[index];

                    // Anti-WindUp
                    if (motor_positions_.positions[i] > motor_thresholds_[1])
                    {
                        motor_positions_.positions[i] = motor_thresholds_[1];
                    } else if (motor_positions_.positions[i] < motor_thresholds_[0])
                    {
                        motor_positions_.positions[i] = motor_thresholds_[0];
                    }
                }
            }

            desired_position_pub_->publish(motor_positions_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Proportional result not received yet.");
        }
    }

    void measured_velocity_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_velocity_ = msg;
        measured_velocity_received_ = true;
    }

    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            if (timer_->is_canceled())
            {
                // Wait for initial motor positions
                if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                        motor_positions_, this->shared_from_this(), motor_state_topic_, std::chrono::seconds(1)))
                {
                    uclv_seed_robotics_ros_interfaces::msg::MotorPositions filtered_positions;

                    for (int64_t motor_id : motor_ids_)
                    {
                        auto it = std::find(motor_positions_.ids.begin(), motor_positions_.ids.end(), motor_id);
                        if (it != motor_positions_.ids.end())
                        {
                            size_t index = std::distance(motor_positions_.ids.begin(), it);
                            filtered_positions.ids.push_back(motor_positions_.ids[index]);
                            filtered_positions.positions.push_back(motor_positions_.positions[index]);
                        }
                        else
                        {
                            RCLCPP_FATAL(this->get_logger(), "Motor ID %ld from parameter 'motor_ids' not found in received motor positions. Exiting...", motor_id);
                            throw std::runtime_error("Motor ID from parameter 'motor_ids' not found in received motor positions");
                        }
                    }
                    motor_positions_ = filtered_positions;

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


                desired_position_pub_->publish(motor_positions_);
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
        auto euler_integrator_controller_node = std::make_shared<EulerIntegratorController>();
        rclcpp::spin(euler_integrator_controller_node);
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
