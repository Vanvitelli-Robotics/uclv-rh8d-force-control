#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/slipping_avoidance.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TaskNode : public rclcpp::Node
{
public:

std::vector<double> desired_norm_data_;
std::vector<uint16_t> desired_norm_ids_;

rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
    rclcpp::Client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>::SharedPtr slipping_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;
    
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_msg_;


    TaskNode() 
: Node("task_node"),
desired_norm_data_(this->declare_parameter<std::vector<double>>("desired_norm_data", {0.3, 0.3, 0.3, 0.3, 0.3})),
        desired_norm_ids_(this->declare_parameter<std::vector<uint16_t>>("desired_norm_ids", {0, 1, 2, 3, 4}))
    {
        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            "/cmd/desired_norm_forces", 10);

        close_client_ = this->create_client<std_srvs::srv::SetBool>("/close");
        slipping_client_ = this->create_client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>("/slipping");
        open_client_ = this->create_client<std_srvs::srv::SetBool>("/open");

    }

    void run() {
        publish_desired_norm_forces();
        std::cout << "Continue..." << std::endl;
        std::cin.get();
        
        call_close_service();
        std::cout << "Continue..." << std::endl;
        std::cin.get();
        
        call_slipping_service();
        std::cout << "Continue..." << std::endl;
        std::cin.get();
        
        call_open_service();
    }

private:
    
    void publish_desired_norm_forces()
    {
        this->get_parameter("desired_norm_data", desired_norm_msg_.data);
        this->get_parameter("desired_norm_ids", desired_norm_msg_.ids);

        desired_norm_publisher_->publish(desired_norm_msg_);
        RCLCPP_INFO(this->get_logger(), "Pubblicato desired_norm_forces");
    }

    void call_close_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        close_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Richiesta servizio close inviata");
    }

    void call_slipping_service()
    {
        auto request = std::make_shared<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance::Request>();
        request->data = desired_norm_msg_.data;
        request->ids = desired_norm_msg_.ids; 

        slipping_client_->async_send_request(request); 
        RCLCPP_INFO(this->get_logger(), "Richiesta servizio slipping inviata");
    }

    void call_open_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        open_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Richiesta servizio open inviata");
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




#include "rclcpp/rclcpp.hpp"
// #include "std_srvs/srv/set_bool.hpp"
// #include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
// #include "uclv_seed_robotics_ros_interfaces/srv/slipping_avoidance.hpp"

// using std::placeholders::_1;
// using std::placeholders::_2;

// class TaskNode : public rclcpp::Node
// {
// public:
//     std::vector<double> desired_norm_data_;
//     std::vector<uint16_t> desired_norm_ids_;

//     rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
//     rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
//     rclcpp::Client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>::SharedPtr slipping_client_;
//     rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;

//     uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_msg_;

//     TaskNode()
//         : Node("task_node"),
//           desired_norm_data_(this->declare_parameter<std::vector<double>>("desired_norm_data", {0.3, 0.3, 0.3, 0.3, 0.3})),
//           desired_norm_ids_(this->declare_parameter<std::vector<uint16_t>>("desired_norm_ids", {0, 1, 2, 3, 4}))
//     {
//         desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
//             "/cmd/desired_norm_forces", 10);

//         close_client_ = this->create_client<std_srvs::srv::SetBool>("/close");
//         slipping_client_ = this->create_client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>("/slipping");
//         open_client_ = this->create_client<std_srvs::srv::SetBool>("/open");
//     }

//     void run()
//     {
//         publish_desired_norm_forces();
//         RCLCPP_INFO(this->get_logger(), "Premi INVIO per continuare...");
//         std::cin.get();

//         if (!call_close_service()) return;
//         RCLCPP_INFO(this->get_logger(), "Premi INVIO per continuare...");
//         std::cin.get();

//         if (!call_slipping_service()) return;
//         RCLCPP_INFO(this->get_logger(), "Premi INVIO per continuare...");
//         std::cin.get();

//         if (!call_open_service()) return;
//     }

// private:
//     void publish_desired_norm_forces()
//     {
//         desired_norm_msg_.data = desired_norm_data_; 
//         desired_norm_msg_.ids = desired_norm_ids_;   

//         desired_norm_publisher_->publish(desired_norm_msg_);
//         RCLCPP_INFO(this->get_logger(), "Pubblicato desired_norm_forces");
//     }


//     bool call_close_service()
//     {
//         if (!close_client_->wait_for_service(std::chrono::seconds(5))) {
//             RCLCPP_ERROR(this->get_logger(), "Servizio close non disponibile.");
//             return false;
//         }

//         auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
//         request->data = true;

//         auto future = close_client_->async_send_request(request);
//         if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
//             RCLCPP_ERROR(this->get_logger(), "Timeout per il servizio close.");
//             return false;
//         }

//         if (future.get()->success) {
//             RCLCPP_INFO(this->get_logger(), "Servizio close eseguito con successo.");
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Errore nell'esecuzione del servizio close.");
//             return false;
//         }
//         return true;
//     }

//     bool call_slipping_service()
//     {
//         if (!slipping_client_->wait_for_service(std::chrono::seconds(5))) {
//             RCLCPP_ERROR(this->get_logger(), "Servizio slipping non disponibile.");
//             return false;
//         }

//         auto request = std::make_shared<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance::Request>();
//         request->data = desired_norm_msg_.data;
//         request->ids = desired_norm_msg_.ids;

//         auto future = slipping_client_->async_send_request(request);
//         if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
//             RCLCPP_ERROR(this->get_logger(), "Timeout per il servizio slipping.");
//             return false;
//         }

//         if (future.get()->success) {
//             RCLCPP_INFO(this->get_logger(), "Servizio slipping eseguito con successo.");
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Errore nell'esecuzione del servizio slipping.");
//             return false;
//         }
//         return true;
//     }

//     bool call_open_service()
//     {
//         if (!open_client_->wait_for_service(std::chrono::seconds(5))) {
//             RCLCPP_ERROR(this->get_logger(), "Servizio open non disponibile.");
//             return false;
//         }

//         auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
//         request->data = true;

//         auto future = open_client_->async_send_request(request);
//         if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
//             RCLCPP_ERROR(this->get_logger(), "Timeout per il servizio open.");
//             return false;
//         }

//         if (future.get()->success) {
//             RCLCPP_INFO(this->get_logger(), "Servizio open eseguito con successo.");
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Errore nell'esecuzione del servizio open.");
//             return false;
//         }
//         return true;
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TaskNode>();
//     node->run();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

