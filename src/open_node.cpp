#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
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
    std::string motor_position_topic_;
    std::string node_service_name_;

    std::vector<int64_t> motor_ids_;
    std::vector<double> motor_positions_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr motor_position_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr integrator_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr proportional_client_;

    Open()
        : Node("open_node"),
          integrator_service_name_(this->declare_parameter<std::string>("integrator_service_name", "startstop")),
          proportional_service_name_(this->declare_parameter<std::string>("proportional_service_name", "activate_controller")),
          motor_position_topic_(this->declare_parameter<std::string>("motor_position_topic", "desired_position")),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{})),
          motor_positions_(this->declare_parameter<std::vector<double>>("motor_positions", std::vector<double>{})),
          node_service_name_(this->declare_parameter<std::string>("node_service_name", "close"))

    {

        // Create the publisher for motor positions
        motor_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            motor_position_topic_, 10);

        start_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
            node_service_name_, std::bind(&Open::service_callback, this, _1, std::placeholders::_2));

        // Clients for integrator and proportional controller
        integrator_client_ = this->create_client<std_srvs::srv::SetBool>(integrator_service_name_);
        proportional_client_ = this->create_client<std_srvs::srv::SetBool>(proportional_service_name_);
    }

private:
    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            RCLCPP_INFO(this->get_logger(), "Stopping integrator and proportional controller, setting motor positions.");

            // Stop integrator
            if (integrator_client_->wait_for_service(std::chrono::seconds(1)))
            {
                auto integrator_req = std::make_shared<std_srvs::srv::SetBool::Request>();
                integrator_req->data = false;
                integrator_client_->async_send_request(integrator_req);
                RCLCPP_INFO(this->get_logger(), "Integrator service stopped.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Integrator service not available.");
                response->success = false;
                response->message = "Failed to stop integrator.";
                return;
            }

            // Stop proportional controller
            if (proportional_client_->wait_for_service(std::chrono::seconds(1)))
            {
                auto proportional_req = std::make_shared<std_srvs::srv::SetBool::Request>();
                proportional_req->data = false;
                proportional_client_->async_send_request(proportional_req);
                RCLCPP_INFO(this->get_logger(), "Proportional controller service stopped.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Proportional controller service not available.");
                response->success = false;
                response->message = "Failed to stop proportional controller.";
                return;
            }

            // Publish motor positions
            publish_initial_velocity();

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

    void publish_initial_velocity()
    {

        uclv_seed_robotics_ros_interfaces::msg::MotorPositions msg;
        // Set motor IDs and initial velocity
        for (int64_t motor_id : motor_ids_)
        {
            msg.ids.push_back(motor_id);
            RCLCPP_INFO(this->get_logger(), "motor id: %d", motor_id);
        }
        for (auto pos : motor_positions_)
        {
            msg.positions.push_back(pos);
            RCLCPP_INFO(this->get_logger(), "pos id: %d", pos);
        }

        // RCLCPP_INFO(this->get_logger(), "Publishing initial velocity: %ld", initial_velocity_);
        motor_position_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto close_node = std::make_shared<Open>();
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
