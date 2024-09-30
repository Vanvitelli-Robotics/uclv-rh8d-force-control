#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/slipping_avoidance.hpp"

using std::placeholders::_1;

class SlippingAvoidance : public rclcpp::Node
{
public:
    double coefficient_;
    std::string sensor_state_topic_;
    std::string desired_forces_topic_;
    std::string activation_service_;
    std::string desired_norm_topic_;

    std::vector<double> data_vec;
    std::vector<int64_t> ids_vec;

    uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors initial_sensor_state_;
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_forces_;

    bool desired_norm_forces_received_ = false;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr sensor_state_subscription_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_subscription;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;

    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>::SharedPtr activation_service_trigger_;

    SlippingAvoidance()
        : Node("slipping_avoidance"),
          node_activated_(false),
          coefficient_(this->declare_parameter<double>("coefficient", 0.01)),
          sensor_state_topic_(this->declare_parameter<std::string>("sensor_state_topic", "sensor_state")),
          activation_service_(this->declare_parameter<std::string>("activation_service", "slipping")),
          desired_norm_topic_(this->declare_parameter<std::string>("desired_norm_topic", "/cmd/desired_norm_forces"))
    {

        sensor_state_subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            sensor_state_topic_, 10, std::bind(&SlippingAvoidance::sensor_state_callback, this, _1));

        activation_service_trigger_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>(
            activation_service_, std::bind(&SlippingAvoidance::activate_callback, this, std::placeholders::_1, std::placeholders::_2));

        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            desired_norm_topic_, 10);
    }

private:
    void sensor_state_callback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        if (node_activated_)
        {
            RCLCPP_INFO(this->get_logger(), "Node is active, processing sensor data...");
            uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped newcmd;
            for (size_t i = 0; i < msg->forces.size(); ++i)
            {
                double x = msg->forces[i].x - initial_sensor_state_.forces[i].x;
                double y = msg->forces[i].y - initial_sensor_state_.forces[i].y;

                double abs = (std::sqrt(std::pow(x, 2) + std::pow(y, 2))) / 1000.0;
                double cmd = coefficient_ * abs;
                RCLCPP_INFO(this->get_logger(), "cmd: %f", cmd);

                newcmd.ids.push_back(ids_vec[i]);
                newcmd.data.push_back(data_vec[i] + cmd);
            }

            desired_norm_publisher_->publish(newcmd);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Node is inactive. Waiting for activation...");
        }
    }

    void initialize_vectors(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance::Request> request)
    {
        if (request->data.size() == request->ids.size()) {
            data_vec = std::move(request->data);
            ids_vec = std::move(request->ids);
            RCLCPP_INFO(this->get_logger(), "Data and IDs vectors initialized successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Data and IDs vectors have different sizes.");
        }
    }

    void activate_callback(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance::Request> request,
                           std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance::Response> response)
    {
        node_activated_ = !node_activated_;
        response->success = true;
        response->message = node_activated_ ? "Node activated!" : "Node deactivated!";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());

        if (node_activated_)
        {
            // Wait for initial forces
            if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
                    initial_sensor_state_, this->shared_from_this(), "sensor_state", std::chrono::seconds(1)))
            {
                uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors zero_sensor_state;
                for (size_t i = 0; i < initial_sensor_state_.forces.size(); i++)
                {
                    zero_sensor_state.ids.push_back(initial_sensor_state_.ids[i]);
                    RCLCPP_INFO(this->get_logger(), "id: %d", initial_sensor_state_.ids[i]);

                    auto vec = initial_sensor_state_.forces[i];
                    zero_sensor_state.forces.push_back(vec);
                }
                initial_sensor_state_ = zero_sensor_state;
            }

            // Initialize data and ids vectors from the service request
            initialize_vectors(request);
        }
    }

    bool node_activated_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlippingAvoidance>());
    rclcpp::shutdown();
    return 0;
}