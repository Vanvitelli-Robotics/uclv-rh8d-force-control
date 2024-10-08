#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/slipping_avoidance.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TaskNode : public rclcpp::Node
{
public:
    std::vector<double> desired_norm_data_;
    std::vector<int64_t> desired_norm_ids_;

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
    rclcpp::Client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>::SharedPtr slipping_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibrate_client_;

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_msg_;

    TaskNode()
        : Node("task_node"),
          desired_norm_data_(this->declare_parameter<std::vector<double>>("desired_norm_data", {0.3, 0.3, 0.3, 0.3, 0.3})),
          desired_norm_ids_(this->declare_parameter<std::vector<int64_t>>("desired_norm_ids", {0, 1, 2, 3, 4}))
    {
        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            "/cmd/norm_forces", 10);

        close_client_ = this->create_client<std_srvs::srv::SetBool>("close");
        slipping_client_ = this->create_client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>("slipping_activation");
        open_client_ = this->create_client<std_srvs::srv::SetBool>("open");
        calibrate_client_ = this->create_client<std_srvs::srv::Trigger>("calibrate_sensors");
    }

    void run()
    {

        calibrate();
        std::this_thread::sleep_for(std::chrono::seconds{2});

        publish_desired_norm_forces();

        std::cout << "Press for close..." << std::endl;
        std::cin.get();
        call_close_service();

        std::cout << "Press for active slipping..." << std::endl;
        std::cin.get();
        call_slipping_service();

        std::cout << "Press for deactivate slipping..." << std::endl;
        std::cin.get();
        call_slipping_service();

        std::cout << "Press for open..." << std::endl;
        std::cin.get();
        call_open_service();
    }

private:
    void publish_desired_norm_forces()
    {
        for (size_t i = 0; i < desired_norm_ids_.size(); i++)
        {
            desired_norm_msg_.ids.push_back(desired_norm_ids_[i]);
            desired_norm_msg_.data.push_back(desired_norm_data_[i]);
        }
        desired_norm_publisher_->publish(desired_norm_msg_);
    }

    void call_close_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        close_client_->async_send_request(request);
    }

    void call_slipping_service()
    {
        auto request = std::make_shared<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance::Request>();
        request->data = desired_norm_msg_.data;
        request->ids = desired_norm_msg_.ids;
        slipping_client_->async_send_request(request);
    }

    void call_open_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        open_client_->async_send_request(request);
    }

    void calibrate()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = calibrate_client_->async_send_request(request);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskNode>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
