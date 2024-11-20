#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <tuple>
#include <iostream>
#include <thread>
#include <atomic> // For atomic flag

const char *usage_msg = R"(
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

P : Toggle align robot to face the person

CTRL-C to quit
)";

// Movement key bindings
std::map<char, std::tuple<float, float>> moveBindings{
    {'i', {1, 0}}, {'o', {1, -1}}, {'j', {0, 1}}, {'l', {0, -1}}, {'u', {1, 1}}, {',', {-1, 0}}, {'.', {-1, 1}}, {'m', {-1, -1}}};

// Speed adjustment key bindings
std::map<char, std::tuple<float, float>> speedBindings{
    {'q', {1.1, 1.1}}, {'z', {0.9, 0.9}}, {'w', {1.1, 1}}, {'x', {0.9, 1}}, {'e', {1, 1.1}}, {'c', {1, 0.9}}};

struct TerminalSettings
{
    termios original;

    void save() { tcgetattr(STDIN_FILENO, &original); }
    void restore() { tcsetattr(STDIN_FILENO, TCSANOW, &original); }
};

// Capture keyboard input without waiting for enter
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

// Display current status
void printStatus(float speed, float turn, float angle, float distance)
{
    std::cout << "\rSpeed: " << speed
              << " | Turn: " << turn
              << " | Angle: " << angle << " degrees"
              << " | Distance: " << distance << "       "
              << std::flush;
}

// Function to track and align robot to a person
void trackPerson(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
                 float &person_angle, float &person_distance,
                 float target_distance, rclcpp::Node::SharedPtr node,
                 std::atomic<bool> &tracking)
{
    geometry_msgs::msg::Twist cmd_msg;

    while (rclcpp::ok() && tracking)
    {
        // Adjust angular velocity to align with the person
        cmd_msg.angular.z = (std::abs(person_angle) > 1.0) ? (person_angle > 0 ? -0.3 : 0.3) : 0.0;

        // Adjust linear velocity to move towards the person
        cmd_msg.linear.x = (person_distance > target_distance) ? 0.2 : 0.0;

        pub->publish(cmd_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(node->get_logger(),
                    "Tracking... Angle: %.2f degrees, Distance: %.2f units",
                    person_angle, person_distance);
    }

    // Stop motion
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 0.0;
    pub->publish(cmd_msg);
    RCLCPP_INFO(node->get_logger(), "Stopped tracking.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop_twist_keyboard");
    auto pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    float speed = 1.0, turn = 1.0;
    float person_angle = 0.0, person_distance = -1.0;
    std::atomic<bool> tracking{false}; // Atomic flag for tracking

    // Subscribe to person tracking topics
    auto sub_angle = node->create_subscription<std_msgs::msg::Float64>(
        "person_angle", 10,
        [&person_angle](std_msgs::msg::Float64::SharedPtr msg)
        {
            person_angle = msg->data;
        });

    auto sub_distance = node->create_subscription<std_msgs::msg::Float64>(
        "person_distance", 10,
        [&person_distance](std_msgs::msg::Float64::SharedPtr msg)
        {
            person_distance = msg->data;
        });

    // Periodically print status
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&]()
        { printStatus(speed, turn, person_angle, person_distance); });

    // Spin in a separate thread
    std::thread spin_thread([&]()
                            { rclcpp::spin(node); });

    TerminalSettings terminal;
    terminal.save();

    try
    {
        std::cout << usage_msg;

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
                auto [speed_mult, turn_mult] = speedBindings[key];
                speed *= speed_mult;
                turn *= turn_mult;
                continue;
            }
            else if (key == 'P' || key == 'p')
            {
                // Toggle tracking
                tracking = !tracking;
                if (tracking)
                {
                    std::thread(trackPerson, pub_twist, std::ref(person_angle), std::ref(person_distance), 2.0, node, std::ref(tracking)).detach();
                }
                continue;
            }
            else if (key == '\x03')
            { // CTRL-C
                break;
            }

            // Publish the twist message
            geometry_msgs::msg::Twist msg;
            msg.linear.x = x * speed;
            msg.angular.z = th * turn;
            pub_twist->publish(msg);
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
    }

    terminal.restore();
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
