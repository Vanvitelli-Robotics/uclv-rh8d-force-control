#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/slipping_avoidance.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TaskNode : public rclcpp::Node
{
public:
    TaskNode() : Node("task_node")
    {
        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            "/cmd/desired_norm_forces", 10);

        close_client_ = this->create_client<std_srvs::srv::SetBool>("/close");
        slipping_client_ = this->create_client<std_srvs::srv::SetBool>("/slipping");
        open_client_ = this->create_client<std_srvs::srv::SetBool>("/open");

        // // Avvia il task
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1), std::bind(&TaskNode::publish_desired_norm_forces, this));
    }

    void run() {
        publish_desired_norm_forces();
        std::cout << "Continue..." << std::endl;
        std::cin.get();
        call_close_service();
        std::cout << "Continue..." << std::endl;
        std::cin.get();
        call_open_service();
    }

private:
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr slipping_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_desired_norm_forces()
    {
        uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped msg;
        msg.data = {0.3, 0.3, 0.3, 0.3, 0.3};
        msg.ids = {0, 1, 2, 3, 4};

        desired_norm_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Pubblicato desired_norm_forces");
    }

    void call_close_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        close_client_->async_send_request(request);
    }


    // void call_slipping_service()
    // {
    //     auto request = std::make_shared<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>();
    //     request->data = {0.3, 0.3, 0.3, 0.3, 0.3};
    //     request->ids = {0, 1, 2, 3, 4};

    //     slipping_client_->async_send_request(request);
    // }



    void call_open_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        open_client_->async_send_request(request);
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
