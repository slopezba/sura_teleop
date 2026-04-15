#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class CirtesubTeleop : public rclcpp::Node
{
public:
  CirtesubTeleop()
  : Node("cirtesub_teleop")
  {
    declare_parameter<double>("rate", 20.0);
    declare_parameter<std::string>("joy_topic", "/joy");
    declare_parameter<std::string>("controller_switch_service", "/controller_manager/switch_controller");
    declare_parameter<std::string>("stabilize_controller.name", "stabilize_controller");
    declare_parameter<std::string>(
      "stabilize_controller.feedforward_topic",
      "/stabilize_controller/feedforward");
    declare_parameter<int>("buttons.toggle_stabilize", 3);
    declare_parameter<int>("axes.surge", 1);
    declare_parameter<int>("axes.sway", 0);
    declare_parameter<int>("axes.heave", 4);
    declare_parameter<int>("axes.yaw", 3);
    declare_parameter<double>("scales.surge", 1.0);
    declare_parameter<double>("scales.sway", 1.0);
    declare_parameter<double>("scales.heave", 1.0);
    declare_parameter<double>("scales.yaw", 1.0);
    declare_parameter<double>("deadzone", 0.05);

    rate_ = get_parameter("rate").as_double();
    joy_topic_ = get_parameter("joy_topic").as_string();
    controller_switch_service_ = get_parameter("controller_switch_service").as_string();
    stabilize_controller_name_ = get_parameter("stabilize_controller.name").as_string();
    feedforward_topic_ = get_parameter("stabilize_controller.feedforward_topic").as_string();
    toggle_stabilize_button_ = get_parameter("buttons.toggle_stabilize").as_int();
    surge_axis_ = get_parameter("axes.surge").as_int();
    sway_axis_ = get_parameter("axes.sway").as_int();
    heave_axis_ = get_parameter("axes.heave").as_int();
    yaw_axis_ = get_parameter("axes.yaw").as_int();
    surge_scale_ = get_parameter("scales.surge").as_double();
    sway_scale_ = get_parameter("scales.sway").as_double();
    heave_scale_ = get_parameter("scales.heave").as_double();
    yaw_scale_ = get_parameter("scales.yaw").as_double();
    deadzone_ = std::max(0.0, get_parameter("deadzone").as_double());

    if (rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid rate %.3f Hz, using 20.0 Hz.", rate_);
      rate_ = 20.0;
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CirtesubTeleop::joyCallback, this, std::placeholders::_1));

    feedforward_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      feedforward_topic_,
      rclcpp::SystemDefaultsQoS());

    switch_controller_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(controller_switch_service_);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&CirtesubTeleop::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Teleop ready. Toggle button=%d, stabilize controller='%s', feedforward topic='%s'.",
      toggle_stabilize_button_,
      stabilize_controller_name_.c_str(),
      feedforward_topic_.c_str());
  }

private:
  using JoyMsg = sensor_msgs::msg::Joy;
  using TwistMsg = geometry_msgs::msg::Twist;
  using SwitchControllerSrv = controller_manager_msgs::srv::SwitchController;

  void joyCallback(const JoyMsg::SharedPtr msg)
  {
    last_joy_msg_ = msg;

    if (!isValidButtonIndex(msg->buttons, toggle_stabilize_button_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Toggle button index %d is out of range for the current joystick message.",
        toggle_stabilize_button_);
      return;
    }

    const bool button_pressed = msg->buttons[static_cast<size_t>(toggle_stabilize_button_)] != 0;

    if (button_pressed && !last_toggle_button_state_) {
      requestStabilizeState(!stabilize_enabled_);
    }

    last_toggle_button_state_ = button_pressed;
  }

  void timerCallback()
  {
    if (!stabilize_enabled_) {
      return;
    }

    TwistMsg cmd;
    if (last_joy_msg_ != nullptr) {
      cmd.linear.x = readAxis(last_joy_msg_->axes, surge_axis_) * surge_scale_;
      cmd.linear.y = readAxis(last_joy_msg_->axes, sway_axis_) * sway_scale_;
      cmd.linear.z = readAxis(last_joy_msg_->axes, heave_axis_) * heave_scale_;
      cmd.angular.z = readAxis(last_joy_msg_->axes, yaw_axis_) * yaw_scale_;
    }

    feedforward_pub_->publish(cmd);
  }

  void requestStabilizeState(bool enable)
  {
    if (switch_in_progress_) {
      RCLCPP_WARN(get_logger(), "Ignoring toggle request because a switch is already in progress.");
      return;
    }

    if (!switch_controller_client_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(
        get_logger(),
        "Controller switch service '%s' is not available.",
        controller_switch_service_.c_str());
      return;
    }

    auto request = std::make_shared<SwitchControllerSrv::Request>();
    if (enable) {
      request->activate_controllers.push_back(stabilize_controller_name_);
    } else {
      request->deactivate_controllers.push_back(stabilize_controller_name_);
    }
    request->strictness = SwitchControllerSrv::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 2;
    request->timeout.nanosec = 0;

    switch_in_progress_ = true;
    const auto future = switch_controller_client_->async_send_request(
      request,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        switch_in_progress_ = false;
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            stabilize_controller_name_.c_str());
          return;
        }

        stabilize_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          stabilize_controller_name_.c_str(),
          stabilize_enabled_ ? "activated" : "deactivated");

        if (!stabilize_enabled_) {
          feedforward_pub_->publish(TwistMsg{});
        }
      });

    (void)future;
  }

  double readAxis(const std::vector<float> & axes, int index) const
  {
    if (!isValidAxisIndex(axes, index)) {
      return 0.0;
    }

    const double value = static_cast<double>(axes[static_cast<size_t>(index)]);
    return std::fabs(value) < deadzone_ ? 0.0 : value;
  }

  bool isValidAxisIndex(const std::vector<float> & axes, int index) const
  {
    return index >= 0 && static_cast<size_t>(index) < axes.size();
  }

  bool isValidButtonIndex(const std::vector<int32_t> & buttons, int index) const
  {
    return index >= 0 && static_cast<size_t>(index) < buttons.size();
  }

  double rate_{20.0};
  double surge_scale_{1.0};
  double sway_scale_{1.0};
  double heave_scale_{1.0};
  double yaw_scale_{1.0};
  double deadzone_{0.05};

  int toggle_stabilize_button_{3};
  int surge_axis_{1};
  int sway_axis_{0};
  int heave_axis_{4};
  int yaw_axis_{3};

  bool stabilize_enabled_{false};
  bool last_toggle_button_state_{false};
  bool switch_in_progress_{false};

  std::string joy_topic_;
  std::string controller_switch_service_;
  std::string stabilize_controller_name_;
  std::string feedforward_topic_;

  JoyMsg::SharedPtr last_joy_msg_;

  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr feedforward_pub_;
  rclcpp::Client<SwitchControllerSrv>::SharedPtr switch_controller_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CirtesubTeleop>());
  rclcpp::shutdown();
  return 0;
}
