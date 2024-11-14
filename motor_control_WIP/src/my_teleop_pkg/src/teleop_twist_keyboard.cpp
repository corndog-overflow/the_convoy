#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <tuple>
#include <iostream>

// Message for usage
const char *msg = R"(
Hello, this is my custom C++ teleop controller.
Use the following keys to control the robot:
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

std::map<char, std::tuple<float, float>> moveBindings{
    {'i', {1, 0}}, {'o', {1, -1}}, {'j', {0, 1}}, {'l', {0, -1}}, {'u', {1, 1}}, {',', {-1, 0}}, {'.', {-1, 1}}, {'m', {-1, -1}}};

std::map<char, std::tuple<float, float>> speedBindings{
    {'q', {1.1, 1.1}}, {'z', {0.9, 0.9}}, {'w', {1.1, 1}}, {'x', {0.9, 1}}, {'e', {1, 1.1}}, {'c', {1, 0.9}}};

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
    std::cout << "Speed: " << speed << " | Turn: " << turn << std::endl;
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

    auto pub_twist = stamped
                         ? nullptr
                         : node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto pub_twist_stamped = stamped
                                 ? node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10)
                                 : nullptr;

    float speed = 1.0, turn = 1.0;
    TerminalSettings terminal;
    terminal.save();

    try
    {
        std::cout << msg;
        printVels(speed, turn);

        while (rclcpp::ok())
        {
            char key = getKey();
            float x = 0, th = 0;

            if (moveBindings.count(key))
            {
                std::tie(x, th) = moveBindings[key];
            }
            else if (speedBindings.count(key))
            {
                float speed_mult, turn_mult;
                std::tie(speed_mult, turn_mult) = speedBindings[key];
                speed *= speed_mult;
                turn *= turn_mult;
                printVels(speed, turn);
                continue;
            }
            else if (key == '\x03') // CTRL-C to exit
            {
                break;
            }

            if (stamped)
            {
                geometry_msgs::msg::TwistStamped msg;
                msg.header.stamp = node->get_clock()->now();
                msg.header.frame_id = frame_id;
                msg.twist.linear.x = x * speed;
                msg.twist.angular.z = th * turn;
                pub_twist_stamped->publish(msg);

                RCLCPP_INFO(node->get_logger(), "Key '%c' | Published TwistStamped: [Header: {frame_id: %s}, Twist: {linear: [%.2f, %.2f, %.2f], angular: [%.2f, %.2f, %.2f]}]",
                            key, frame_id.c_str(), msg.twist.linear.x, 0.0, 0.0,
                            0.0, 0.0, msg.twist.angular.z);
            }
            else
            {
                geometry_msgs::msg::Twist msg;
                msg.linear.x = x * speed;
                msg.angular.z = th * turn;
                pub_twist->publish(msg);

                RCLCPP_INFO(node->get_logger(), "Key '%c' | Published Twist: [linear: [%.2f, %.2f, %.2f], angular: [%.2f, %.2f, %.2f]]",
                            key, msg.linear.x, 0.0, 0.0,
                            0.0, 0.0, msg.angular.z);
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
