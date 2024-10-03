#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/slipping_avoidance.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

using std::placeholders::_1;

class SlippingAvoidance : public rclcpp::Node
{
public:
    std::vector<double> coefficients_;
    double coefficient_;

    double x;
    double y;

    std::string sensor_state_topic_;
    std::string desired_forces_topic_;
    std::string activation_service_;
    std::string desired_norm_topic_;
    std::string difference_topic_;

    std::vector<double> data_vec;
    std::vector<uint16_t> ids_vec;

    uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors initial_sensor_state_;
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_forces_;

    bool desired_norm_forces_received_ = false;
    bool node_activated_ = false;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr sensor_state_subscription_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_subscription;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr difference_publisher_;

    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>::SharedPtr activation_service_trigger_;

    SlippingAvoidance()
        : Node("slipping_avoidance"),
          coefficients_(this->declare_parameter<std::vector<double>>("coefficients", std::vector<double>())),
          sensor_state_topic_(this->declare_parameter<std::string>("sensor_state_topic", "")),
          activation_service_(this->declare_parameter<std::string>("activation_service", "")),
          desired_norm_topic_(this->declare_parameter<std::string>("desired_norm_topic", "")),
          difference_topic_(this->declare_parameter<std::string>("difference_topic", "/force_difference"))  // Initialize difference topic
    {
        check_parameters();

        sensor_state_subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            sensor_state_topic_, 10, std::bind(&SlippingAvoidance::sensor_state_callback, this, _1));

        activation_service_trigger_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>(
            activation_service_, std::bind(&SlippingAvoidance::activate_callback, this, std::placeholders::_1, std::placeholders::_2));

        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            desired_norm_topic_, 10);

        difference_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(difference_topic_, 10);
    }

private:

    void check_parameters()
    {
        auto check_string_parameter = [this](const std::string &param_name, const std::string &value) {
            if (value.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid value.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };

        auto check_vector_parameter = [this](const std::string &param_name, const std::vector<double> &value) {
            if (value.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is missing or empty. Please provide a valid vector.", param_name.c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Invalid or missing parameter: '" + param_name + "'");
            }
        };

        check_vector_parameter("coefficients", coefficients_);
        check_string_parameter("sensor_state_topic", sensor_state_topic_);
        check_string_parameter("activation_service", activation_service_);
        check_string_parameter("desired_norm_topic", desired_norm_topic_);
        check_string_parameter("difference_topic", difference_topic_);

        RCLCPP_INFO(this->get_logger(), "All required parameters are set correctly.");
    }

    void sensor_state_callback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        if (node_activated_)
        {
            uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped newcmd;

            for (size_t i = 0; i < msg->forces.size(); ++i)
            {
                newcmd.ids.push_back(ids_vec[i]);

                if (coefficients_[i] != 0)
                {
                    coeffiecient_ = coefficients_[i];
                    x = msg->forces[i].x - initial_sensor_state_.forces[i].x;
                    y = msg->forces[i].y - initial_sensor_state_.forces[i].y;

                    geometry_msgs::msg::Vector3Stamped difference_msg;
                    difference_msg.header.stamp = this->get_clock()->now();
                    difference_msg.vector.x = x;
                    difference_msg.vector.y = y;
                    difference_msg.vector.z = 0.0;

                    difference_publisher_->publish(difference_msg);
                }

                double abs = (std::sqrt(std::pow(x, 2) + std::pow(y, 2))) / 1000.0;
                double cmd = coeffiecient_ * abs;

                newcmd.data.push_back(data_vec[i] + cmd);
            }

            desired_norm_publisher_->publish(newcmd);
        }
    }

    void initialize_vectors(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance::Request> request)
    {
        if (request->data.size() == request->ids.size())
        {
            data_vec = std::move(request->data);
            ids_vec = std::move(request->ids);
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
            if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
                    initial_sensor_state_, this->shared_from_this(), sensor_state_topic_, std::chrono::seconds(1)))
            {
                uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors zero_sensor_state;
                for (size_t i = 0; i < initial_sensor_state_.forces.size(); i++)
                {
                    zero_sensor_state.ids.push_back(initial_sensor_state_.ids[i]);

                    auto vec = initial_sensor_state_.forces[i];
                    zero_sensor_state.forces.push_back(vec);
                }
                initial_sensor_state_ = zero_sensor_state;
            }

            initialize_vectors(request);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto slipping_avoidance_node = std::make_shared<SlippingAvoidance>();
        rclcpp::spin(slipping_avoidance_node);
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
