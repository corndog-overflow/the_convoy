#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <tuple>
#include <iostream>
#include <thread>
#include <atomic>

// Help message displayed to users showing keyboard controls
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

// Map keyboard keys to linear (x) and angular (z) velocity values
std::map<char, std::tuple<float, float>> moveBindings{
    {'i', {1, 0}}, {'o', {1, -1}}, {'j', {0, 1}}, {'l', {0, -1}}, {'u', {1, 1}}, {',', {-1, 0}}, {'.', {-1, 1}}, {'m', {-1, -1}}};

// Map keyboard keys to speed multiplier values
std::map<char, std::tuple<float, float>> speedBindings{
    {'q', {1.1, 1.1}}, {'z', {0.9, 0.9}}, {'w', {1.1, 1}}, {'x', {0.9, 1}}, {'e', {1, 1.1}}, {'c', {1, 0.9}}};

// Helper struct to manage terminal settings
struct TerminalSettings
{
    termios original;
    void save() { tcgetattr(STDIN_FILENO, &original); }
    void restore() { tcsetattr(STDIN_FILENO, TCSANOW, &original); }
};

// Read a single keypress from terminal without waiting for Enter
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

// Display current robot status including speed, turn rate, and person tracking info
void printStatus(float speed, float turn, float angle, float distance)
{
    std::cout << "\rSpeed: " << speed
              << " | Turn: " << turn
              << " | Angle: " << angle << " degrees"
              << " | Distance: " << distance << "       "
              << std::flush;
}

// PID controller implementation for smooth motion control
class PIDController
{
private:
    double kp_, ki_, kd_;            // PID gains
    double integral_ = 0.0;          // Integral term accumulator
    double prev_error_ = 0.0;        // Previous error for derivative calculation
    double output_min_, output_max_; // Output limits
    std::chrono::steady_clock::time_point prev_time_;
    bool first_run_ = true;

public:
    // Constructor to initialize PID parameters and limits
    PIDController(double kp, double ki, double kd, double output_min, double output_max)
        : kp_(kp), ki_(ki), kd_(kd), output_min_(output_min), output_max_(output_max) {}

    // Calculate PID control output based on error
    double compute(double error)
    {
        auto current_time = std::chrono::steady_clock::now();
        if (first_run_)
        {
            prev_time_ = current_time;
            first_run_ = false;
            return 0.0;
        }

        double dt = std::chrono::duration<double>(current_time - prev_time_).count();
        prev_time_ = current_time;

        // Calculate PID terms
        double p_term = kp_ * error;
        integral_ += error * dt;
        double i_term = ki_ * integral_;
        double derivative = (error - prev_error_) / dt;
        double d_term = kd_ * derivative;

        // Compute and limit output
        double output = p_term + i_term + d_term;
        output = std::min(std::max(output, output_min_), output_max_);
        prev_error_ = error;

        return output;
    }

    // Reset controller state
    void reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        first_run_ = true;
    }
};

// Function to handle person tracking behavior
void trackPerson(rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub,
                 float &person_angle, float &person_distance,
                 float target_distance, rclcpp::Node::SharedPtr node,
                 std::atomic<bool> &tracking)
{
    PIDController angular_pid(0.05, 0.001, 0.01, -0.5, 0.5);
    PIDController linear_pid(0.5, 0.0, 0.2, -0.5, 0.5);
    geometry_msgs::msg::TwistStamped cmd_msg;

    double prev_distance = person_distance;
    const double APPROACH_SPEED_LIMIT = 0.3;

    while (rclcpp::ok() && tracking)
    {
        // Calculate errors for PID controllers
        double angular_error = person_angle;
        double linear_error = person_distance - target_distance;
        double approach_velocity = (prev_distance - person_distance) / 0.05;
        prev_distance = person_distance;

        // Compute control commands
        cmd_msg.twist.angular.z = -angular_pid.compute(angular_error);
        double pid_output = linear_pid.compute(linear_error);

        // Limit approach speed based on distance
        double distance_to_target = std::abs(linear_error);
        double max_speed = std::min(APPROACH_SPEED_LIMIT, std::max(0.1, distance_to_target * 0.5));
        cmd_msg.twist.linear.x = std::clamp(pid_output, -max_speed, max_speed);

        // Safety check: stop if approaching too fast when close
        if (approach_velocity > 0.5 && distance_to_target < 1.0)
        {
            cmd_msg.twist.linear.x = 0.0;
            linear_pid.reset();
        }

        // Set the timestamp
        cmd_msg.header.stamp = node->get_clock()->now();
        cmd_msg.header.frame_id = "base_link";

        pub->publish(cmd_msg);
        RCLCPP_INFO(node->get_logger(),
                    "Tracking... Angle: %.2f deg (cmd: %.2f), Distance: %.2f m (cmd: %.2f), Approach Speed: %.2f",
                    person_angle, cmd_msg.twist.angular.z, person_distance, cmd_msg.twist.linear.x, approach_velocity);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Stop robot when tracking ends
    cmd_msg.twist.linear.x = 0.0;
    cmd_msg.twist.angular.z = 0.0;
    cmd_msg.header.stamp = node->get_clock()->now();
    pub->publish(cmd_msg);
    RCLCPP_INFO(node->get_logger(), "Stopped tracking.");
}

int main(int argc, char **argv)
{
    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop_twist_keyboard");
    auto pub_twist = node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    // Initialize control variables
    float speed = 1.0, turn = 1.0;
    float person_angle = 0.0, person_distance = -1.0;
    std::atomic<bool> tracking{false};

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

    // Create timer for status updates
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&]()
        { printStatus(speed, turn, person_angle, person_distance); });

    // Start ROS2 spin thread
    std::thread spin_thread([&]()
                            { rclcpp::spin(node); });

    // Setup terminal for keyboard input
    TerminalSettings terminal;
    terminal.save();

    try
    {
        std::cout << usage_msg;

        // Main control loop
        while (rclcpp::ok())
        {
            char key = getKey();
            float x = 0, th = 0;

            // Handle movement commands
            if (moveBindings.count(key))
            {
                std::tie(x, th) = moveBindings[key];
            }
            // Handle speed adjustment commands
            else if (speedBindings.count(key))
            {
                auto [speed_mult, turn_mult] = speedBindings[key];
                speed *= speed_mult;
                turn *= turn_mult;
                continue;
            }
            // Toggle person tracking
            else if (key == 'P' || key == 'p')
            {
                tracking = !tracking;
                if (tracking)
                {
                    std::thread(trackPerson, pub_twist, std::ref(person_angle),
                                std::ref(person_distance), 2.0, node,
                                std::ref(tracking))
                        .detach();
                }
                continue;
            }
            // Exit on Ctrl-C
            else if (key == '\x03')
            {
                break;
            }

            // Publish velocity command with timestamp
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = node->get_clock()->now();
            msg.header.frame_id = "base_link";
            msg.twist.linear.x = x * speed;
            msg.twist.angular.z = th * turn;
            pub_twist->publish(msg);
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
    }

    // Cleanup and exit
    terminal.restore();
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}