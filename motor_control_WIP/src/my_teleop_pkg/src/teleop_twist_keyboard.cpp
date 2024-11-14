#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <iostream>
#include <map>
#include <string>

// Message for usage
const char *msg = R"(
Hello, this is my custom C++ teleop controller. This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
)";

std::map<char, std::tuple<float, float, float, float>> moveBindings{
    {'i', {1, 0, 0, 0}}, {'o', {1, 0, 0, -1}}, {'j', {0, 0, 0, 1}}, {'l', {0, 0, 0, -1}}, {'u', {1, 0, 0, 1}}, {',', {-1, 0, 0, 0}}, {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}}};

std::map<char, std::tuple<float, float>> speedBindings{
    {'q', {1.1, 1.1}}, {'z', {0.9, 0.9}}, {'w', {1.1, 1}}, {'x', {0.9, 1}}, {'e', {1, 1.1}}, {'c', {1, 0.9}}};

// Terminal settings management for non-blocking input
struct TerminalSettings
{
    termios original;
    void save() { tcgetattr(STDIN_FILENO, &original); }
    void restore() { tcsetattr(STDIN_FILENO, TCSANOW, &original); }
};

char getKey()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    char key = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    return key;
}

void printVels(float speed, float turn)
{
    std::cout << "currently:\tspeed " << speed << "\tturn " << turn << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop_twist_keyboard");

    bool stamped = node->declare_parameter("stamped", false);
    std::string frame_id = node->declare_parameter("frame_id", "");

    if (!stamped && !frame_id.empty())
    {
        throw std::runtime_error("'frame_id' can only be set when 'stamped' is True");
    }

    using TwistMsg = geometry_msgs::msg::Twist;
    using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

    // Define separate publishers
    std::shared_ptr<rclcpp::Publisher<TwistMsg>> pub_twist;
    std::shared_ptr<rclcpp::Publisher<TwistStampedMsg>> pub_twist_stamped;

    if (stamped)
    {
        pub_twist_stamped = node->create_publisher<TwistStampedMsg>("cmd_vel", 10);
    }
    else
    {
        pub_twist = node->create_publisher<TwistMsg>("cmd_vel", 10);
    }

    float speed = 1.0;
    float turn = 1.0;

    TerminalSettings terminal;
    terminal.save();

    try
    {
        std::cout << msg;
        printVels(speed, turn);

        while (rclcpp::ok())
        {
            char key = getKey();
            float x = 0, y = 0, z = 0, th = 0;

            if (moveBindings.count(key))
            {
                std::tie(x, y, z, th) = moveBindings[key];
            }
            else if (speedBindings.count(key))
            {
                float speed_mult, turn_mult;
                std::tie(speed_mult, turn_mult) = speedBindings[key];
                speed *= speed_mult;
                turn *= turn_mult;
                printVels(speed, turn);
            }
            else if (key == '\x03')
            {
                break;
            }

            // Ensure z is always 0
            z = 0;

            if (stamped)
            {
                auto msg = TwistStampedMsg();
                msg.header.stamp = node->get_clock()->now();
                msg.header.frame_id = frame_id;
                msg.twist.linear.x = x * speed;
                msg.twist.linear.y = y * speed;
                msg.twist.linear.z = z; // z is always 0
                msg.twist.angular.z = th * turn;
                pub_twist_stamped->publish(msg);

                RCLCPP_INFO(node->get_logger(), "Publishing TwistStamped: linear=(%.2f, %.2f, %.2f), angular=(%.2f, %.2f, %.2f)",
                            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
            }
            else
            {
                auto msg = TwistMsg();
                msg.linear.x = x * speed;
                msg.linear.y = y * speed;
                msg.linear.z = z; // z is always 0
                msg.angular.z = th * turn;
                pub_twist->publish(msg);

                RCLCPP_INFO(node->get_logger(), "Publishing Twist: linear=(%.2f, %.2f, %.2f), angular=(%.2f, %.2f, %.2f)",
                            msg.linear.x, msg.linear.y, msg.linear.z,
                            msg.angular.x, msg.angular.y, msg.angular.z);
            }
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
    }

    terminal.restore();
    rclcpp::shutdown();
    return 0;
}
