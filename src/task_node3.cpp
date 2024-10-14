#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "uclv_dynamixel_utils/colors.hpp"

using std::placeholders::_1;


class TaskNode3 : public rclcpp::Node
{
public:
    std::vector<double> desired_norm_data_;
    std::vector<int64_t> desired_norm_ids_;
    double norm_threshold_;
    bool check_norm_forces_;

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibrate_client_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr norm_force_subscriber_;

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_msg_;

    TaskNode3()
        : Node("task_node3"),
            // ball & bottle
        //   desired_norm_data_(this->declare_parameter<std::vector<double>>("desired_norm_data", {0.5, 0.5, 0.5, 0.3, 0.3})),
            // sponge
        desired_norm_data_(this->declare_parameter<std::vector<double>>("desired_norm_data", {0.2, 0.2, 0.2, 0.2, 0.2})),

          desired_norm_ids_(this->declare_parameter<std::vector<int64_t>>("desired_norm_ids", {0, 1, 2, 3, 4})),
          norm_threshold_(this->declare_parameter<double>("norm_threshold", 0.3)), // 0.5 for bottle and ball
          check_norm_forces_(false)
    {
        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            "/cmd/norm_forces", 10);

        close_client_ = this->create_client<std_srvs::srv::SetBool>("close");
        open_client_ = this->create_client<std_srvs::srv::SetBool>("open");
        calibrate_client_ = this->create_client<std_srvs::srv::Trigger>("calibrate_sensors");

        norm_force_subscriber_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            "/state/norm_forces", 10, std::bind(&TaskNode3::check_and_handle_norm_forces, this, _1));
    }

    void run()
    {   
        std::this_thread::sleep_for(std::chrono::seconds{2});
        calibrate();
        std::cout << "Calibrate " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds{2});

        publish_desired_norm_forces();
        std::cout << "Publish desired norm forces " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;

        std::cout << "Press for close..." << std::endl;
        std::cin.get();
        call_close_service();

        std::cout << "Press Enter to start checking norm forces..." << std::endl;
        std::cin.get(); 
        check_norm_forces_ = true;

    }

private:
    void publish_desired_norm_forces()
    {
        desired_norm_msg_.header.stamp = rclcpp::Clock{}.now();
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

    void call_open_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        open_client_->async_send_request(request);
    }

    void calibrate()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        calibrate_client_->async_send_request(request);
    }

    void check_and_handle_norm_forces(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        if (!check_norm_forces_)
        {
            return;
        }

        double total_force_norm = 0.0;
        for (size_t i = 0; i < msg->data.size(); i++)
        {
            total_force_norm += msg->data[i];
            std::cout << "Norm force for sensor " << msg->ids[i] << ": " << msg->data[i] << std::endl;
        }

        double average_force_norm = total_force_norm / msg->data.size();
        std::cout << "Average norm force: " << average_force_norm << std::endl;

        if (average_force_norm > norm_threshold_)
        {
            std::cout << WARN_COLOR << "Average threshold exceeded. Activating open service." << CRESET << std::endl;
            call_open_service();
        }
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskNode3>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
