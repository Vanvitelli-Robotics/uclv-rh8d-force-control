#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>
#include <memory>

using std::placeholders::_1;

class Close : public rclcpp::Node
{
public:
    std::string measured_norm_topic_; // quello che esce da force norm node

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_velocity_pub_;

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_; // Message for measured normalized forces
    bool measured_norm_forces_received_ = false;                                         // Flag indicating if measured forces data has been received

    ///////////////////////////
    std::string start_stop_service_name_;
    std::string measured_velocity_topic_;
    std::vector<int64_t> motor_ids_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;

    Close()
        : Node("close_node"),
          measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "norm_forces")),
          measured_velocity_topic_(this->declare_parameter<std::string>("measured_velocity_topic", "measured_velocity")), // Get proportional result topic from parameters
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", {35, 36, 37, 38})),

          start_stop_service_name_(this->declare_parameter<std::string>("start_stop_service_name", "close"))

    {
        // Create subscription to measured normalized forces
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10, std::bind(&Close::measured_norm_forces_callback, this, std::placeholders::_1));

        ////////////////////////////////////
        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            start_stop_service_name_, std::bind(&Close::service_close_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create a publisher for desired motor positions
        measured_velocity_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_velocity_topic_, 10);
    }

private:
    // Callback function for receiving measured normalized forces
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_norm_forces_ = *msg;          // Store the received message
        measured_norm_forces_received_ = true;
        // compute();                             // Compute and publish the motor errors
    }

    // check servizio close attivato
    // se attivato:
    //              check della norma
    //              < soglia : invia messaggio su topic measured_velocity
    //              > soglia : attiva servizio di start-stop del proporzionale
    void compute()
    {
    }

    // Callback function for the start/stop service
    void service_close_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            RCLCPP_INFO(this->get_logger(), "servizio true");
            uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped nonloso;
            if (measured_norm_forces_received_)
            {   
                            RCLCPP_INFO(this->get_logger(), "norm forces received true");
                // check soglia

                for (int64_t motor_id : motor_ids_)
                {
                    nonloso.ids.push_back(motor_id);
                    nonloso.data.push_back(500); // velocità
                }
                measured_velocity_pub_->publish(nonloso);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "servizio false");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto close_node = std::make_shared<Close>();
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