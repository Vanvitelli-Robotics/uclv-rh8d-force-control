#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"

#include <memory>
#include <vector>

using std::placeholders::_1;

class Open : public rclcpp::Node
{
public:
    std::string integrator_service_name_;
    std::string proportional_service_name_;
    std::string calibrate_service_name_;
    std::string desired_position_topic_;
    std::string open_service_name_;

    std::vector<int64_t> motor_ids_;
    std::vector<double> motor_positions_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr open_service_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr motor_position_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr integrator_service_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr proportional_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;

    rclcpp::TimerBase::SharedPtr calibration_timer_;

    Open()
        : Node("open"),
          integrator_service_name_(this->declare_parameter<std::string>("integrator_service_name", "")),
          proportional_service_name_(this->declare_parameter<std::string>("proportional_service_name", "")),
          calibrate_service_name_(this->declare_parameter<std::string>("calibrate_service_name", "")),
          desired_position_topic_(this->declare_parameter<std::string>("desired_position_topic", "")),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),
          motor_positions_(this->declare_parameter<std::vector<double>>("motor_position", std::vector<double>())),
          open_service_name_(this->declare_parameter<std::string>("open_service_name", ""))
    {

        check_parameters();

        motor_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            desired_position_topic_, 10);

        open_service_ = this->create_service<std_srvs::srv::SetBool>(
            open_service_name_, std::bind(&Open::service_callback, this, _1, std::placeholders::_2));

        integrator_service_ = this->create_client<std_srvs::srv::SetBool>(integrator_service_name_);
        proportional_service_ = this->create_client<std_srvs::srv::SetBool>(proportional_service_name_);
        calibrate_service_ = this->create_client<std_srvs::srv::Trigger>(calibrate_service_name_);
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

        auto check_vector_int_parameter = [this](const std::string &param_name, const std::vector<int64_t> &value)
        {
            if (value.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid vector.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };

        auto check_vector_double_parameter = [this](const std::string &param_name, const std::vector<double> &value)
        {
            if (value.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid vector.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };

        check_string_parameter("integrator_service_name", integrator_service_name_);
        check_string_parameter("proportional_service_name", proportional_service_name_);
        check_string_parameter("calibrate_service_name", calibrate_service_name_);
        check_string_parameter("desired_position_topic", desired_position_topic_);
        check_string_parameter("open_service_name", open_service_name_);
        check_vector_int_parameter("motor_ids", motor_ids_);
        check_vector_double_parameter("motor_positions", motor_positions_);

        RCLCPP_INFO(this->get_logger(), "All required parameters are set correctly.");
    }

    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            RCLCPP_INFO(this->get_logger(), "Stopping integrator and proportional controller, setting motor positions.");

            if (integrator_service_->wait_for_service(std::chrono::seconds(1)))
            {
                auto integrator_req = std::make_shared<std_srvs::srv::SetBool::Request>();
                integrator_req->data = false;
                integrator_service_->async_send_request(integrator_req);
                RCLCPP_INFO(this->get_logger(), "Integrator service stopped.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Integrator service not available.");
                response->success = false;
                response->message = "Failed to stop integrator.";
                return;
            }

            if (proportional_service_->wait_for_service(std::chrono::seconds(1)))
            {
                auto proportional_req = std::make_shared<std_srvs::srv::SetBool::Request>();
                proportional_req->data = false;
                proportional_service_->async_send_request(proportional_req);
                RCLCPP_INFO(this->get_logger(), "Proportional controller service stopped.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Proportional controller service not available.");
                response->success = false;
                response->message = "Failed to stop proportional controller.";
                return;
            }

            publish_motor_position(motor_ids_, motor_positions_);

            calibration_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&Open::calibrate, this));

            response->success = true;
            response->message = "Motors set to specified positions, integrator and proportional stopped.";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Service called with deactivate command, no actions taken.");
            response->success = false;
            response->message = "Service called with deactivate command.";
        }
    }

    void publish_motor_position(const std::vector<int64_t> motor_ids, const std::vector<double> positions)
    {
        uclv_seed_robotics_ros_interfaces::msg::MotorPositions msg;
        msg.header.stamp = rclcpp::Clock{}.now();
        for (int64_t motor_id : motor_ids)
        {
            msg.ids.push_back(motor_id);
        }
        for (auto pos : positions)
        {
            msg.positions.push_back(pos);
        }

        motor_position_pub_->publish(msg);
    }

    void calibrate()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = calibrate_service_->async_send_request(request);

        calibration_timer_->cancel();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto open = std::make_shared<Open>();
        rclcpp::spin(open);
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
