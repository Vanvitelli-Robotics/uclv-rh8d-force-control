#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "std_srvs/srv/set_bool.hpp"

// using namespace uclv::dynamixel_utils;

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;
    int millisecondsTimer_;
    bool initial_condition_received_;

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;
    std::vector<float> positions;
    std::vector<uint8_t> ids;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr motor_position_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    EulerIntegrator()
        : Node("euler_integrator"),
          initial_condition_received_(false)
    {
        dt_ = this->declare_parameter<double>("dt", 0.1);
        millisecondsTimer_ = this->declare_parameter<int>("millisecondsTimer", 2);

        // IL SUBSCRIBER PER LE CONDIZIONI INIZIALI LO FACCIAMO CON WAIT FOR MSG. crea il subscriber e lo usa una sola volta.
        // lo start e stop lo facciamo con std srvs set bool, il tipo di messaggio quindi non si deve creare
        // la posizione desiderata, per ora, la pubblico su /motor_position_fake, un nuovo topic per non confondermi con /motor_position

        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "startstop", std::bind(&EulerIntegrator::service_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));

        // Subscription to the motor position topic (for initial condition)
        motor_position_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "motor_state", 1, std::bind(&EulerIntegrator::initialPositionCallback, this, std::placeholders::_1));

        // Publisher for the desired motor position topic (?)
        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "desired_position", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(millisecondsTimer_),
            std::bind(&EulerIntegrator::integrate, this));
    }

private:
    void initialPositionCallback(const uclv_seed_robotics_ros_interfaces::msg::MotorPositions::SharedPtr pos)
    {
        if (!initial_condition_received_)
        {
            positions = pos->positions;
            ids = pos->ids;

            initial_condition_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received initial motor positions.");
        }
        else
            return;
    }

    void integrate()
    {
        if (!initial_condition_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial motor positions...");
            return;
        }
        // else
        // {
        //     std::cout << "initial condition ok\n";
        // }

        /*
            Eulero:
                posizione desiderata istante k+1 = posizione desiderata istante k + velocità istante k * dt

                consideriamo la condizione iniziale

                pubblico su topic le posizioni desiderate s
        */

        // for (size_t i = 0; i < positions.size(); i++)
        // {
        //     std::cout << "ID: " << (unsigned int)ids[i] << " position: " << positions[i] << "\n";
        // }
    }

    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::cout << "service callback\n";
        // qui devono esser prese le condizioni iniziali con wait for msg
        // e poi deve essere avviato  o stoppato il timer_, quindi l'esecuzione della timer_callback
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto euler_integrator_node = std::make_shared<EulerIntegrator>();
        rclcpp::spin(euler_integrator_node);
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
