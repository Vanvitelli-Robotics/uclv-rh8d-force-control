#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TaskNode : public rclcpp::Node
{
public:
    TaskNode() : Node("task_node")
    {
        // Crea un publisher per il topic '/cmd/desired_norm_forces'
        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            "/cmd/desired_norm_forces", 10);

        // Crea client per i servizi richiesti
        close_client_ = this->create_client<std_srvs::srv::SetBool>("/close");
        slipping_client_ = this->create_client<std_srvs::srv::SetBool>("/slipping");
        open_client_ = this->create_client<std_srvs::srv::SetBool>("/open");

        // Avvia il task
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TaskNode::publish_desired_norm_forces, this));
    }

private:
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr slipping_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_desired_norm_forces()
    {
        // Passo 1: Pubblica il messaggio su '/cmd/desired_norm_forces'
        uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped msg;
        msg.data = {0.3, 0.3, 0.3, 0.3, 0.3};
        msg.ids = {0, 1, 2, 3, 4};

        desired_norm_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Pubblicato desired_norm_forces");

        // Attendi prima di richiamare il servizio
        auto timer_after_publish = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&TaskNode::call_close_service, this));
    }

    void call_close_service()
    {
        // Passo 2: Richiama il servizio /close
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        close_client_->async_send_request(request, std::bind(&TaskNode::handle_close_response, this, _1));
    }

    void handle_close_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        if (future.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Servizio /close chiamato con successo");
            call_slipping_service();
        }
    }

    void call_slipping_service()
    {
        // Passo 3: Richiama il servizio /slipping
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        slipping_client_->async_send_request(request, std::bind(&TaskNode::handle_slipping_response, this, _1));
    }

    void handle_slipping_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        if (future.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Servizio /slipping chiamato con successo");
            call_open_service();
        }
    }

    void call_open_service()
    {
        // Passo 4: Richiama il servizio /open
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        open_client_->async_send_request(request, std::bind(&TaskNode::handle_open_response, this, _1));
    }

    void handle_open_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        if (future.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Servizio /open chiamato con successo. Task completato!");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
