#include <rclcpp/rclcpp.hpp>
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class Task : public rclcpp::Node
{
public:
    Task() : Node("task")
    {
        // Publisher per desired_position
        motor_positions_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>("desired_position", 10);

        // Publisher per desired_norm_forces
        norm_forces_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>("/cmd/desired_norm_forces", 10);

        // Service client per /startstop
        startstop_client_ = this->create_client<std_srvs::srv::SetBool>("/startstop");
    }

    void run()
    {
        // Attendi interazione utente per l'inizio
        std::cout << "Premi Invio per pubblicare la posizione iniziale dei motori..." << std::endl;
        std::cin.get();
        init_position();

        std::cout << "Premi Invio per pubblicare le norme desiderate..." << std::endl;
        std::cin.get();
        set_norms();

        std::cout << "Premi Invio per avviare il servizio start/stop..." << std::endl;
        std::cin.get();
        start_service();

        std::cout << "Aspettando 15 secondi prima di fermare il servizio..." << std::endl;
        stop_service_timer_ = this->create_wall_timer(15s, std::bind(&OrchestratorNode::stop_service, this));
    }

private:
    void init_position()
    {
        // Pubblica la posizione iniziale
        auto motor_positions_msg = uclv_seed_robotics_ros_interfaces::msg::MotorPositions();
        motor_positions_msg.ids = {31, 32, 33, 34, 35, 36, 37, 38};
        motor_positions_msg.positions = {100, 2000, 2000, 3000, 100, 100, 100, 100};
        motor_positions_pub_->publish(motor_positions_msg);
        RCLCPP_INFO(this->get_logger(), "Init position published.");
    }

    void set_norms()
    {
        // Pubblica le norme desiderate
        auto norm_forces_msg = uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped();
        norm_forces_msg.data = {0.6, 0.6, 0.6, 0.6, 0.6};
        norm_forces_msg.ids = {0, 1, 2, 3, 4};
        norm_forces_pub_->publish(norm_forces_msg);
        RCLCPP_INFO(this->get_logger(), "Desired force norms published.");
    }

    void start_service()
    {
        // Inizia il servizio
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        auto result = startstop_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Service started.");
    }

    void stop_service()
    {
        // Ferma il servizio
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        auto result = startstop_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Service stopped.");

        init_position();
    }

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr motor_positions_pub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr norm_forces_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr startstop_client_;

    rclcpp::TimerBase::SharedPtr stop_service_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Task>();

    node->run();  // Esegue il ciclo con interazione dell'utente

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
