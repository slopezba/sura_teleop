#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_msgs/msg/sura_velocity_command.hpp"
#include "sura_msgs/msg/sura_wrench_command.hpp"
#include "sura_msgs/srv/clear_controller_intents.hpp"
#include "sura_msgs/srv/controller_interlock.hpp"

class CirtesubTeleop : public rclcpp::Node
{
public:
  CirtesubTeleop()
  : Node("cirtesub_teleop")
  {
    declare_parameter<double>("rate", 20.0);
    declare_parameter<std::string>("joy_topic", "/joy");
    declare_parameter<std::string>("requester", "teleop");
    declare_parameter<int>("priority", 80);
    declare_parameter<int>("idle_priority", 50);
    declare_parameter<double>("idle_priority_delay", 3.0);
    declare_parameter<std::string>(
      "arbitrator_velocity_topic", "controller/arbitrator/velocity");
    declare_parameter<std::string>(
      "arbitrator_wrench_topic", "controller/arbitrator/wrench");
    declare_parameter<std::string>(
      "clear_controller_intents_service", "controller/arbitrator/clear_controller_intents");
    declare_parameter<std::string>(
      "controller_interlock_service", "controller/arbitrator/controller_interlock");
    declare_parameter<bool>("autonomous_mode.enabled", true);
    declare_parameter<std::string>("autonomous_mode.topic", "teleop/autonomous_enabled");
    declare_parameter<std::string>("autonomous_mode.behavior", "deadman");
    declare_parameter<int>("autonomous_mode.axis", 2);
    declare_parameter<double>("autonomous_mode.value", -1.0);
    declare_parameter<double>("autonomous_mode.tolerance", 0.05);
    declare_parameter<std::string>(
      "controller_switch_service", "/cirtesub/controller/arbitrator/switch_controller");
    declare_parameter<std::string>(
      "controller_list_service", "/cirtesub/controller/controller_manager/list_controllers");
    declare_parameter<std::string>("body_force_controller.name", "body_force");
    declare_parameter<std::string>(
      "body_force_controller.command_topic",
      "/cirtesub/controller/body_force/command");
    declare_parameter<double>("body_force_controller.feedforward_gain_x", 20.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_y", 20.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_z", 90.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_roll", 20.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_pitch", 20.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_yaw", 1.0);
    declare_parameter<std::string>("body_velocity_controller.name", "body_velocity");
    declare_parameter<std::string>(
      "body_velocity_controller.setpoint_topic",
      "/cirtesub/controller/body_velocity/setpoint");
    declare_parameter<std::string>("position_hold_controller.name", "position_hold");
    declare_parameter<std::string>(
      "position_hold_controller.reposition_controller_name",
      "position_hold_reposition");
    declare_parameter<std::string>(
      "position_hold_controller.feedforward_topic",
      "/cirtesub/controller/position_hold/feedforward");
    declare_parameter<std::string>(
      "position_hold_controller.reposition_feedforward_topic",
      "/cirtesub/controller/position_hold/reposition_feedforward");
    declare_parameter<std::string>("stabilize_controller.name", "stabilize");
    declare_parameter<std::string>(
      "stabilize_controller.feedforward_topic",
      "/cirtesub/controller/stabilize/feedforward");
    declare_parameter<double>("stabilize_controller.feedforward_gain_x", 20.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_y", 20.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_z", 90.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_roll", 20.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_pitch", 20.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_yaw", 1.0);
    declare_parameter<std::string>(
      "stabilize_controller.enable_roll_pitch_service",
      "/cirtesub/controller/stabilize/enable_roll_pitch");
    declare_parameter<std::string>(
      "stabilize_controller.disable_roll_pitch_service",
      "/cirtesub/controller/stabilize/disable_roll_pitch");
    declare_parameter<std::string>("depth_hold_controller.name", "depth_hold");
    declare_parameter<std::string>(
      "depth_hold_controller.feedforward_topic",
      "/cirtesub/controller/depth_hold/feedforward");
    declare_parameter<double>("depth_hold_controller.feedforward_gain_x", 20.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_y", 20.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_z", 90.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_roll", 20.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_pitch", 20.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_yaw", 1.0);
    declare_parameter<std::string>(
      "depth_hold_controller.enable_roll_pitch_service",
      "/cirtesub/controller/depth_hold/enable_roll_pitch");
    declare_parameter<std::string>(
      "depth_hold_controller.disable_roll_pitch_service",
      "/cirtesub/controller/depth_hold/disable_roll_pitch");
    declare_parameter<int>("buttons.a", 0);
    declare_parameter<int>("buttons.b", 1);
    declare_parameter<int>("buttons.x", 2);
    declare_parameter<int>("buttons.lb", 4);
    declare_parameter<int>("buttons.rb", 5);
    declare_parameter<int>("buttons.y", 3);
    declare_parameter<int>("buttons.left_stick", 9);
    declare_parameter<int>("buttons.right_stick", 10);
    declare_parameter<int>("axes.hat_horizontal", 6);
    declare_parameter<int>("axes.hat_vertical", 7);
    declare_parameter<int>("axes.surge", 1);
    declare_parameter<int>("axes.sway", 0);
    declare_parameter<int>("axes.yaw", 3);
    declare_parameter<int>("axes.heave", 4);
    declare_parameter<int>("axes.roll", 3);
    declare_parameter<int>("axes.pitch", 4);
    declare_parameter<int>("axes.lt", 2);
    declare_parameter<int>("axes.rt", 5);
    declare_parameter<std::string>(
      "alpha_left_forward_velocity_controller.name",
      "alpha_left_forward_velocity_controller");
    declare_parameter<std::string>(
      "alpha_right_forward_velocity_controller.name",
      "alpha_right_forward_velocity_controller");
    declare_parameter<std::string>(
      "alpha_left_joint_trajectory_controller.name",
      "alpha_left_joint_trajectory_controller");
    declare_parameter<std::string>(
      "alpha_right_joint_trajectory_controller.name",
      "alpha_right_joint_trajectory_controller");
    declare_parameter<std::string>(
      "alpha_left_forward_velocity_controller.command_topic",
      "/cirtesub/controller/alpha_left_forward_velocity_controller/commands");
    declare_parameter<std::string>(
      "alpha_right_forward_velocity_controller.command_topic",
      "/cirtesub/controller/alpha_right_forward_velocity_controller/commands");
    declare_parameter<double>("alpha_forward_command_rate", 10.0);
    declare_parameter<double>("scales.surge", 1.0);
    declare_parameter<double>("scales.sway", 1.0);
    declare_parameter<double>("scales.yaw", 1.0);
    declare_parameter<double>("scales.heave", 1.0);
    declare_parameter<double>("scales.roll", 1.0);
    declare_parameter<double>("scales.pitch", 1.0);
    declare_parameter<double>("scales.alpha_axis_a_velocity", 0.01);
    declare_parameter<double>("deadzone", 0.05);

    rate_ = get_parameter("rate").as_double();
    joy_topic_ = get_parameter("joy_topic").as_string();
    requester_ = get_parameter("requester").as_string();
    priority_ = static_cast<int>(std::clamp<int64_t>(get_parameter("priority").as_int(), 1, 100));
    idle_priority_ = static_cast<int>(
      std::clamp<int64_t>(get_parameter("idle_priority").as_int(), 1, 100));
    idle_priority_delay_ = std::max(0.0, get_parameter("idle_priority_delay").as_double());
    arbitrator_velocity_topic_ = get_parameter("arbitrator_velocity_topic").as_string();
    arbitrator_wrench_topic_ = get_parameter("arbitrator_wrench_topic").as_string();
    clear_controller_intents_service_ =
      get_parameter("clear_controller_intents_service").as_string();
    controller_interlock_service_ =
      get_parameter("controller_interlock_service").as_string();
    autonomous_mode_enabled_ = get_parameter("autonomous_mode.enabled").as_bool();
    autonomous_mode_topic_ = get_parameter("autonomous_mode.topic").as_string();
    autonomous_mode_behavior_ = get_parameter("autonomous_mode.behavior").as_string();
    autonomous_mode_axis_ = get_parameter("autonomous_mode.axis").as_int();
    autonomous_mode_value_ = get_parameter("autonomous_mode.value").as_double();
    autonomous_mode_tolerance_ =
      std::max(0.0, get_parameter("autonomous_mode.tolerance").as_double());
    controller_switch_service_ = get_parameter("controller_switch_service").as_string();
    controller_list_service_ = get_parameter("controller_list_service").as_string();
    body_force_controller_name_ = get_parameter("body_force_controller.name").as_string();
    body_force_command_topic_ = get_parameter("body_force_controller.command_topic").as_string();
    body_force_feedforward_gain_x_ =
      get_parameter("body_force_controller.feedforward_gain_x").as_double();
    body_force_feedforward_gain_y_ =
      get_parameter("body_force_controller.feedforward_gain_y").as_double();
    body_force_feedforward_gain_z_ =
      get_parameter("body_force_controller.feedforward_gain_z").as_double();
    body_force_feedforward_gain_roll_ =
      get_parameter("body_force_controller.feedforward_gain_roll").as_double();
    body_force_feedforward_gain_pitch_ =
      get_parameter("body_force_controller.feedforward_gain_pitch").as_double();
    body_force_feedforward_gain_yaw_ =
      get_parameter("body_force_controller.feedforward_gain_yaw").as_double();
    body_velocity_controller_name_ = get_parameter("body_velocity_controller.name").as_string();
    body_velocity_setpoint_topic_ =
      get_parameter("body_velocity_controller.setpoint_topic").as_string();
    position_hold_controller_name_ = get_parameter("position_hold_controller.name").as_string();
    position_hold_reposition_controller_name_ =
      get_parameter("position_hold_controller.reposition_controller_name").as_string();
    position_hold_feedforward_topic_ =
      get_parameter("position_hold_controller.feedforward_topic").as_string();
    position_hold_reposition_feedforward_topic_ =
      get_parameter("position_hold_controller.reposition_feedforward_topic").as_string();
    stabilize_controller_name_ = get_parameter("stabilize_controller.name").as_string();
    stabilize_feedforward_topic_ =
      get_parameter("stabilize_controller.feedforward_topic").as_string();
    stabilize_feedforward_gain_x_ =
      get_parameter("stabilize_controller.feedforward_gain_x").as_double();
    stabilize_feedforward_gain_y_ =
      get_parameter("stabilize_controller.feedforward_gain_y").as_double();
    stabilize_feedforward_gain_z_ =
      get_parameter("stabilize_controller.feedforward_gain_z").as_double();
    stabilize_feedforward_gain_roll_ =
      get_parameter("stabilize_controller.feedforward_gain_roll").as_double();
    stabilize_feedforward_gain_pitch_ =
      get_parameter("stabilize_controller.feedforward_gain_pitch").as_double();
    stabilize_feedforward_gain_yaw_ =
      get_parameter("stabilize_controller.feedforward_gain_yaw").as_double();
    stabilize_enable_roll_pitch_service_name_ =
      get_parameter("stabilize_controller.enable_roll_pitch_service").as_string();
    stabilize_disable_roll_pitch_service_name_ =
      get_parameter("stabilize_controller.disable_roll_pitch_service").as_string();
    depth_hold_controller_name_ = get_parameter("depth_hold_controller.name").as_string();
    depth_hold_feedforward_topic_ =
      get_parameter("depth_hold_controller.feedforward_topic").as_string();
    depth_hold_feedforward_gain_x_ =
      get_parameter("depth_hold_controller.feedforward_gain_x").as_double();
    depth_hold_feedforward_gain_y_ =
      get_parameter("depth_hold_controller.feedforward_gain_y").as_double();
    depth_hold_feedforward_gain_z_ =
      get_parameter("depth_hold_controller.feedforward_gain_z").as_double();
    depth_hold_feedforward_gain_roll_ =
      get_parameter("depth_hold_controller.feedforward_gain_roll").as_double();
    depth_hold_feedforward_gain_pitch_ =
      get_parameter("depth_hold_controller.feedforward_gain_pitch").as_double();
    depth_hold_feedforward_gain_yaw_ =
      get_parameter("depth_hold_controller.feedforward_gain_yaw").as_double();
    depth_hold_enable_roll_pitch_service_name_ =
      get_parameter("depth_hold_controller.enable_roll_pitch_service").as_string();
    depth_hold_disable_roll_pitch_service_name_ =
      get_parameter("depth_hold_controller.disable_roll_pitch_service").as_string();
    a_button_ = get_parameter("buttons.a").as_int();
    b_button_ = get_parameter("buttons.b").as_int();
    x_button_ = get_parameter("buttons.x").as_int();
    lb_button_ = get_parameter("buttons.lb").as_int();
    rb_button_ = get_parameter("buttons.rb").as_int();
    y_button_ = get_parameter("buttons.y").as_int();
    left_stick_button_ = get_parameter("buttons.left_stick").as_int();
    right_stick_button_ = get_parameter("buttons.right_stick").as_int();
    hat_horizontal_axis_ = get_parameter("axes.hat_horizontal").as_int();
    hat_vertical_axis_ = get_parameter("axes.hat_vertical").as_int();
    surge_axis_ = get_parameter("axes.surge").as_int();
    sway_axis_ = get_parameter("axes.sway").as_int();
    yaw_axis_ = get_parameter("axes.yaw").as_int();
    heave_axis_ = get_parameter("axes.heave").as_int();
    roll_axis_ = get_parameter("axes.roll").as_int();
    pitch_axis_ = get_parameter("axes.pitch").as_int();
    lt_axis_ = get_parameter("axes.lt").as_int();
    rt_axis_ = get_parameter("axes.rt").as_int();
    alpha_left_forward_velocity_controller_name_ =
      get_parameter("alpha_left_forward_velocity_controller.name").as_string();
    alpha_right_forward_velocity_controller_name_ =
      get_parameter("alpha_right_forward_velocity_controller.name").as_string();
    alpha_left_joint_trajectory_controller_name_ =
      get_parameter("alpha_left_joint_trajectory_controller.name").as_string();
    alpha_right_joint_trajectory_controller_name_ =
      get_parameter("alpha_right_joint_trajectory_controller.name").as_string();
    alpha_left_forward_velocity_command_topic_ =
      get_parameter("alpha_left_forward_velocity_controller.command_topic").as_string();
    alpha_right_forward_velocity_command_topic_ =
      get_parameter("alpha_right_forward_velocity_controller.command_topic").as_string();
    alpha_forward_command_rate_ = get_parameter("alpha_forward_command_rate").as_double();
    surge_scale_ = get_parameter("scales.surge").as_double();
    sway_scale_ = get_parameter("scales.sway").as_double();
    yaw_scale_ = get_parameter("scales.yaw").as_double();
    heave_scale_ = get_parameter("scales.heave").as_double();
    roll_scale_ = get_parameter("scales.roll").as_double();
    pitch_scale_ = get_parameter("scales.pitch").as_double();
    alpha_axis_a_velocity_scale_ = get_parameter("scales.alpha_axis_a_velocity").as_double();
    deadzone_ = std::max(0.0, get_parameter("deadzone").as_double());

    if (rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid rate %.3f Hz, using 20.0 Hz.", rate_);
      rate_ = 20.0;
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CirtesubTeleop::joyCallback, this, std::placeholders::_1));

    updateWrenchPublisher(stabilize_controller_name_);

    switch_controller_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(controller_switch_service_);
    list_controllers_client_ =
      create_client<controller_manager_msgs::srv::ListControllers>(controller_list_service_);
    clear_controller_intents_client_ =
      create_client<sura_msgs::srv::ClearControllerIntents>(clear_controller_intents_service_);
    controller_interlock_client_ =
      create_client<sura_msgs::srv::ControllerInterlock>(controller_interlock_service_);
    stabilize_enable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(stabilize_enable_roll_pitch_service_name_);
    stabilize_disable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(stabilize_disable_roll_pitch_service_name_);
    depth_hold_enable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(depth_hold_enable_roll_pitch_service_name_);
    depth_hold_disable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(depth_hold_disable_roll_pitch_service_name_);
    alpha_left_forward_velocity_command_pub_ = create_publisher<Float64MultiArrayMsg>(
      alpha_left_forward_velocity_command_topic_,
      rclcpp::SystemDefaultsQoS());
    alpha_right_forward_velocity_command_pub_ = create_publisher<Float64MultiArrayMsg>(
      alpha_right_forward_velocity_command_topic_,
      rclcpp::SystemDefaultsQoS());
    velocity_intent_pub_ = create_publisher<SuraVelocityCommandMsg>(
      arbitrator_velocity_topic_,
      rclcpp::SystemDefaultsQoS());
    wrench_intent_pub_ = create_publisher<SuraWrenchCommandMsg>(
      arbitrator_wrench_topic_,
      rclcpp::SystemDefaultsQoS());
    autonomous_enabled_pub_ = create_publisher<BoolMsg>(
      autonomous_mode_topic_,
      rclcpp::SystemDefaultsQoS());
    publishAutonomousMode(false);
    position_hold_feedforward_pub_ = create_publisher<TwistMsg>(
      position_hold_feedforward_topic_,
      rclcpp::SystemDefaultsQoS());
    position_hold_reposition_feedforward_pub_ = create_publisher<TwistMsg>(
      position_hold_reposition_feedforward_topic_,
      rclcpp::SystemDefaultsQoS());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&CirtesubTeleop::timerCallback, this));
    controller_interlock_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CirtesubTeleop::updateControllerInterlockState, this));

    if (alpha_forward_command_rate_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid Alpha forward command rate %.3f Hz, using 10.0 Hz.",
        alpha_forward_command_rate_);
      alpha_forward_command_rate_ = 10.0;
    }

    alpha_forward_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / alpha_forward_command_rate_),
      std::bind(&CirtesubTeleop::alphaForwardTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Teleop ready. R3 selects AUV mode, L3 selects arm mode, RB+X toggles '%s', RB+B toggles '%s', RB+Y toggles '%s', RB+A toggles '%s', RB+LB toggles '%s', RB+hat left selects '%s', RB+hat right selects '%s', active command topic='%s'.",
      body_velocity_controller_name_.c_str(),
      position_hold_controller_name_.c_str(),
      stabilize_controller_name_.c_str(),
      depth_hold_controller_name_.c_str(),
      body_force_controller_name_.c_str(),
      alpha_left_forward_velocity_controller_name_.c_str(),
      alpha_right_forward_velocity_controller_name_.c_str(),
      active_command_controller_.c_str());
  }

private:
  using JoyMsg = sensor_msgs::msg::Joy;
  using TwistMsg = geometry_msgs::msg::Twist;
  using WrenchMsg = geometry_msgs::msg::Wrench;
  using Float64MultiArrayMsg = std_msgs::msg::Float64MultiArray;
  using BoolMsg = std_msgs::msg::Bool;
  using SuraVelocityCommandMsg = sura_msgs::msg::SuraVelocityCommand;
  using SuraWrenchCommandMsg = sura_msgs::msg::SuraWrenchCommand;
  using ListControllersSrv = controller_manager_msgs::srv::ListControllers;
  using SwitchControllerSrv = controller_manager_msgs::srv::SwitchController;
  using ControllerInterlockSrv = sura_msgs::srv::ControllerInterlock;

  enum class AlphaForwardControllerSelection
  {
    None,
    Left,
    Right
  };

  enum class TeleopMode
  {
    Auv,
    Arm
  };

  enum class CommandOutputMode
  {
    None,
    Twist,
    Wrench
  };

  void updateTwistPublisher(const std::string & controller_name)
  {
    active_command_controller_ = controller_name;
    command_output_mode_ = CommandOutputMode::Twist;
  }

  void updateWrenchPublisher(const std::string & controller_name)
  {
    active_command_controller_ = controller_name;
    command_output_mode_ = CommandOutputMode::Wrench;
  }

  void publishZeroFeedforward()
  {
    if (active_command_controller_.empty()) {
      return;
    }
    if (command_output_mode_ == CommandOutputMode::Twist) {
      publishVelocityIntent(active_command_controller_, TwistMsg{});
    }
    if (command_output_mode_ == CommandOutputMode::Wrench) {
      publishWrenchIntent(active_command_controller_, WrenchMsg{});
    }
  }

  void publishVelocityIntent(
    const std::string & controller_name,
    const TwistMsg & velocity,
    int priority = -1)
  {
    const bool position_hold_reposition_mode =
      position_hold_enabled_ || controller_name == position_hold_controller_name_;

    // Manual teleop in position_hold must be a direct "jog/reposition" signal so the hold
    // setpoint follows the vehicle and releasing the joystick keeps the new pose.  We still
    // also publish an arbitrator intent below, so higher-priority velocity requests can
    // preempt it and be routed as temporary overrides.
    if (position_hold_reposition_mode && position_hold_reposition_feedforward_pub_) {
      position_hold_reposition_feedforward_pub_->publish(velocity);
    }

    if (!velocity_intent_pub_) {
      return;
    }

    const int message_priority = normalizePriority(priority);

    SuraVelocityCommandMsg msg;
    msg.header.stamp = now();
    msg.requester = requester_;
    msg.controller = position_hold_reposition_mode ?
      position_hold_reposition_controller_name_ : controller_name;
    msg.priority = static_cast<uint8_t>(message_priority);
    msg.velocity = velocity;
    velocity_intent_pub_->publish(msg);
  }

  void publishWrenchIntent(
    const std::string & controller_name,
    const WrenchMsg & wrench,
    int priority = -1)
  {
    if (!wrench_intent_pub_) {
      return;
    }

    const int message_priority = normalizePriority(priority);

    SuraWrenchCommandMsg msg;
    msg.header.stamp = now();
    msg.requester = requester_;
    msg.controller = controller_name;
    msg.priority = static_cast<uint8_t>(message_priority);
    msg.wrench = wrench;
    wrench_intent_pub_->publish(msg);
  }

  void clearControllerIntents(const std::string & controller_name)
  {
    if (controller_name.empty() || !clear_controller_intents_client_) {
      return;
    }
    if (!clear_controller_intents_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Clear controller intents service '%s' is not available.",
        clear_controller_intents_service_.c_str());
      return;
    }
    auto request = std::make_shared<sura_msgs::srv::ClearControllerIntents::Request>();
    request->controller = controller_name;
    (void)clear_controller_intents_client_->async_send_request(request);
  }

  void clearManualControllerIntents()
  {
    clearControllerIntents(body_force_controller_name_);
    clearControllerIntents(body_velocity_controller_name_);
    clearControllerIntents(position_hold_controller_name_);
    clearControllerIntents(position_hold_reposition_controller_name_);
    clearControllerIntents(stabilize_controller_name_);
    clearControllerIntents(depth_hold_controller_name_);
  }

  void joyCallback(const JoyMsg::SharedPtr msg)
  {
    last_joy_msg_ = msg;
    updateAutonomousMode(*msg);

    if (autonomous_enabled_) {
      last_body_velocity_combo_state_ = false;
      last_position_hold_combo_state_ = false;
      last_stabilize_combo_state_ = false;
      last_depth_hold_combo_state_ = false;
      last_body_force_combo_state_ = false;
      last_left_stick_button_state_ = false;
      last_right_stick_button_state_ = false;
      return;
    }

    if (
      !isValidButtonIndex(msg->buttons, a_button_) ||
      !isValidButtonIndex(msg->buttons, b_button_) ||
      !isValidButtonIndex(msg->buttons, x_button_) ||
      !isValidButtonIndex(msg->buttons, lb_button_) ||
      !isValidButtonIndex(msg->buttons, rb_button_) ||
      !isValidButtonIndex(msg->buttons, y_button_) ||
      !isValidButtonIndex(msg->buttons, left_stick_button_) ||
      !isValidButtonIndex(msg->buttons, right_stick_button_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Configured button indices are out of range for the current joystick message.");
      return;
    }

    const bool a_pressed = msg->buttons[static_cast<size_t>(a_button_)] != 0;
    const bool b_pressed = msg->buttons[static_cast<size_t>(b_button_)] != 0;
    const bool x_pressed = msg->buttons[static_cast<size_t>(x_button_)] != 0;
    const bool lb_pressed = msg->buttons[static_cast<size_t>(lb_button_)] != 0;
    const bool rb_pressed = msg->buttons[static_cast<size_t>(rb_button_)] != 0;
    const bool y_pressed = msg->buttons[static_cast<size_t>(y_button_)] != 0;
    const bool left_stick_pressed =
      msg->buttons[static_cast<size_t>(left_stick_button_)] != 0;
    const bool right_stick_pressed =
      msg->buttons[static_cast<size_t>(right_stick_button_)] != 0;

    if (left_stick_pressed && !last_left_stick_button_state_) {
      setTeleopMode(TeleopMode::Arm);
    }

    if (right_stick_pressed && !last_right_stick_button_state_) {
      setTeleopMode(TeleopMode::Auv);
    }

    const bool body_velocity_combo_pressed = rb_pressed && x_pressed;
    const bool position_hold_combo_pressed = rb_pressed && b_pressed;
    const bool stabilize_combo_pressed = rb_pressed && y_pressed;
    const bool depth_hold_combo_pressed = rb_pressed && a_pressed;
    const bool body_force_combo_pressed = rb_pressed && lb_pressed;

    if (teleop_mode_ == TeleopMode::Auv &&
      body_velocity_combo_pressed && !last_body_velocity_combo_state_)
    {
      requestBodyVelocityToggle();
    }

    if (teleop_mode_ == TeleopMode::Auv &&
      position_hold_combo_pressed && !last_position_hold_combo_state_)
    {
      requestPositionHoldToggle();
    }

    if (teleop_mode_ == TeleopMode::Auv &&
      stabilize_combo_pressed && !last_stabilize_combo_state_)
    {
      requestStabilizeToggle();
    }

    if (teleop_mode_ == TeleopMode::Auv &&
      depth_hold_combo_pressed && !last_depth_hold_combo_state_)
    {
      requestDepthHoldToggle();
    }

    if (teleop_mode_ == TeleopMode::Auv &&
      body_force_combo_pressed && !last_body_force_combo_state_)
    {
      requestBodyForceToggle();
    }

    if (teleop_mode_ == TeleopMode::Arm) {
      processAlphaForwardControllerSelection(*msg, rb_pressed);
    } else {
      last_hat_horizontal_state_ = 0;
    }

    if (teleop_mode_ == TeleopMode::Auv) {
      processHatCommands(*msg);
    } else {
      last_hat_vertical_state_ = 0;
    }

    last_body_velocity_combo_state_ = body_velocity_combo_pressed;
    last_position_hold_combo_state_ = position_hold_combo_pressed;
    last_stabilize_combo_state_ = stabilize_combo_pressed;
    last_depth_hold_combo_state_ = depth_hold_combo_pressed;
    last_body_force_combo_state_ = body_force_combo_pressed;
    last_left_stick_button_state_ = left_stick_pressed;
    last_right_stick_button_state_ = right_stick_pressed;
  }

  void timerCallback()
  {
    if (teleop_mode_ != TeleopMode::Auv) {
      return;
    }

    if (autonomous_enabled_) {
      return;
    }

    const bool direct_body_force_enabled = body_force_enabled_ && !body_velocity_enabled_ &&
      !position_hold_enabled_ && !stabilize_enabled_ && !depth_hold_enabled_;

    if (!direct_body_force_enabled && !body_velocity_enabled_ && !position_hold_enabled_ &&
      !stabilize_enabled_ && !depth_hold_enabled_)
    {
      return;
    }

    TwistMsg twist_cmd;
    WrenchMsg wrench_cmd;
    if (last_joy_msg_ != nullptr) {
      const bool lb_pressed = isValidButtonIndex(last_joy_msg_->buttons, lb_button_) &&
        last_joy_msg_->buttons[static_cast<size_t>(lb_button_)] != 0;

      if (lb_pressed) {
        twist_cmd.angular.x = readAxis(last_joy_msg_->axes, roll_axis_) * roll_scale_;
        twist_cmd.angular.y = readAxis(last_joy_msg_->axes, pitch_axis_) * pitch_scale_;
        wrench_cmd.torque.x = twist_cmd.angular.x;
        wrench_cmd.torque.y = twist_cmd.angular.y;
      } else {
        twist_cmd.linear.x = readAxis(last_joy_msg_->axes, surge_axis_) * surge_scale_;
        twist_cmd.linear.y = readAxis(last_joy_msg_->axes, sway_axis_) * sway_scale_;
        twist_cmd.linear.z = -readAxis(last_joy_msg_->axes, heave_axis_) * heave_scale_;
        twist_cmd.angular.z = readAxis(last_joy_msg_->axes, yaw_axis_) * yaw_scale_;
        wrench_cmd.force.x = twist_cmd.linear.x;
        wrench_cmd.force.y = twist_cmd.linear.y;
        wrench_cmd.force.z = twist_cmd.linear.z;
        wrench_cmd.torque.z = twist_cmd.angular.z;
      }
    }

    if (position_hold_enabled_ && active_command_controller_ != position_hold_controller_name_) {
      // body_velocity is active underneath position_hold, but manual joystick commands
      // must be treated as hold reposition commands, not as temporary overrides.
      updateTwistPublisher(position_hold_controller_name_);
    }

    if (direct_body_force_enabled && active_command_controller_ != body_force_controller_name_) {
      updateWrenchPublisher(body_force_controller_name_);
    }

    const bool has_manual_input =
      command_output_mode_ == CommandOutputMode::Twist ?
      hasTwistInput(twist_cmd) :
      command_output_mode_ == CommandOutputMode::Wrench && hasWrenchInput(wrench_cmd);
    const int teleop_priority = computeTeleopPriority(has_manual_input);

    if (command_output_mode_ == CommandOutputMode::Twist) {
      publishVelocityIntent(active_command_controller_, twist_cmd, teleop_priority);
    } else if (command_output_mode_ == CommandOutputMode::Wrench) {
      if (direct_body_force_enabled) {
        wrench_cmd.force.x *= body_force_feedforward_gain_x_;
        wrench_cmd.force.y *= body_force_feedforward_gain_y_;
        wrench_cmd.force.z *= body_force_feedforward_gain_z_;
        wrench_cmd.torque.x *= body_force_feedforward_gain_roll_;
        wrench_cmd.torque.y *= body_force_feedforward_gain_pitch_;
        wrench_cmd.torque.z *= body_force_feedforward_gain_yaw_;
        publishWrenchIntent(active_command_controller_, wrench_cmd, teleop_priority);
        return;
      }

      double feedforward_gain_x = stabilize_feedforward_gain_x_;
      double feedforward_gain_y = stabilize_feedforward_gain_y_;
      double feedforward_gain_z = stabilize_feedforward_gain_z_;
      double feedforward_gain_roll = stabilize_feedforward_gain_roll_;
      double feedforward_gain_pitch = stabilize_feedforward_gain_pitch_;
      double feedforward_gain_yaw = stabilize_feedforward_gain_yaw_;

      if (depth_hold_enabled_ && !position_hold_enabled_) {
        feedforward_gain_x = depth_hold_feedforward_gain_x_;
        feedforward_gain_y = depth_hold_feedforward_gain_y_;
        feedforward_gain_z = depth_hold_feedforward_gain_z_;
        feedforward_gain_roll = depth_hold_feedforward_gain_roll_;
        feedforward_gain_pitch = depth_hold_feedforward_gain_pitch_;
        feedforward_gain_yaw = depth_hold_feedforward_gain_yaw_;
      }

      wrench_cmd.force.x *= feedforward_gain_x;
      wrench_cmd.force.y *= feedforward_gain_y;
      wrench_cmd.force.z *= feedforward_gain_z;
      wrench_cmd.torque.x *= feedforward_gain_roll;
      wrench_cmd.torque.y *= feedforward_gain_pitch;
      wrench_cmd.torque.z *= feedforward_gain_yaw;
      publishWrenchIntent(active_command_controller_, wrench_cmd, teleop_priority);
    }
  }

  void alphaForwardTimerCallback()
  {
    if (teleop_mode_ != TeleopMode::Arm) {
      return;
    }

    if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::None ||
      last_joy_msg_ == nullptr)
    {
      return;
    }

    double axis_b_command = readAxis(last_joy_msg_->axes, yaw_axis_);
    double axis_c_command = readAxis(last_joy_msg_->axes, heave_axis_);
    double axis_d_command = readAxis(last_joy_msg_->axes, sway_axis_);
    double axis_e_command = readAxis(last_joy_msg_->axes, surge_axis_);
    const double lt_command = readTriggerAxis(last_joy_msg_->axes, lt_axis_);
    const double rt_command = readTriggerAxis(last_joy_msg_->axes, rt_axis_);
    const double axis_a_command = (rt_command - lt_command) * alpha_axis_a_velocity_scale_;

    if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::Left) {
      axis_d_command = -axis_d_command;
    } else if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::Right) {
      axis_e_command = -axis_e_command;
    }

    Float64MultiArrayMsg command_msg;
    command_msg.data = {
      axis_a_command,
      axis_b_command,
      axis_c_command,
      axis_d_command,
      axis_e_command};

    if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::Left) {
      alpha_left_forward_velocity_command_pub_->publish(command_msg);
    } else if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::Right) {
      alpha_right_forward_velocity_command_pub_->publish(command_msg);
    }
  }

  using ControllerStateMap = std::unordered_map<std::string, std::string>;

  bool isControllerActive(
    const ControllerStateMap & controller_states,
    const std::string & controller_name) const
  {
    const auto iterator = controller_states.find(controller_name);
    return iterator != controller_states.end() && iterator->second == "active";
  }

  void syncAuvControllerStates(const ControllerStateMap & controller_states)
  {
    body_force_enabled_ = isControllerActive(controller_states, body_force_controller_name_);
    body_velocity_enabled_ = isControllerActive(controller_states, body_velocity_controller_name_);
    position_hold_enabled_ = isControllerActive(controller_states, position_hold_controller_name_);
    stabilize_enabled_ = isControllerActive(controller_states, stabilize_controller_name_);
    depth_hold_enabled_ = isControllerActive(controller_states, depth_hold_controller_name_);
  }

  void requestControllerStates(
    std::function<void(const ControllerStateMap &)> response_callback,
    const char * busy_message)
  {
    if (switch_in_progress_) {
      RCLCPP_WARN(get_logger(), "%s", busy_message);
      return;
    }

    if (!list_controllers_client_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(
        get_logger(),
        "Controller list service '%s' is not available.",
        controller_list_service_.c_str());
      return;
    }

    switch_in_progress_ = true;
    auto request = std::make_shared<ListControllersSrv::Request>();
    const auto future = list_controllers_client_->async_send_request(
      request,
      [this, response_callback](rclcpp::Client<ListControllersSrv>::SharedFuture future_response)
      {
        switch_in_progress_ = false;

        ControllerStateMap controller_states;
        for (const auto & controller : future_response.get()->controller) {
          controller_states[controller.name] = controller.state;
        }

        syncAuvControllerStates(controller_states);
        response_callback(controller_states);
      });

    (void)future;
  }

  void deduplicateControllers(std::vector<std::string> & controller_names) const
  {
    std::sort(controller_names.begin(), controller_names.end());
    controller_names.erase(
      std::unique(controller_names.begin(), controller_names.end()),
      controller_names.end());
  }

  void requestStabilizeToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestStabilizeState(
          !isControllerActive(controller_states, stabilize_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestStabilizeState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, depth_hold_controller_name_)) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
      if (!isControllerActive(controller_states, stabilize_controller_name_)) {
        activate_controllers.push_back(stabilize_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        stabilize_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            stabilize_controller_name_.c_str());
          return;
        }

        if (enable) {
          body_force_enabled_ = true;
          body_velocity_enabled_ = false;
          position_hold_enabled_ = false;
          clearControllerIntents(position_hold_controller_name_);
          depth_hold_enabled_ = false;
          updateWrenchPublisher(stabilize_controller_name_);
        } else {
          body_velocity_enabled_ = false;
        }
        stabilize_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          stabilize_controller_name_.c_str(),
          stabilize_enabled_ ? "activated" : "deactivated");

        if (!stabilize_enabled_) {
          publishZeroFeedforward();
          clearControllerIntents(stabilize_controller_name_);
        }
      });
  }

  void requestDepthHoldToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestDepthHoldState(
          !isControllerActive(controller_states, depth_hold_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestDepthHoldState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (!isControllerActive(controller_states, depth_hold_controller_name_)) {
        activate_controllers.push_back(depth_hold_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, depth_hold_controller_name_)) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        depth_hold_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            depth_hold_controller_name_.c_str());
          return;
        }

        if (enable) {
          body_force_enabled_ = true;
          body_velocity_enabled_ = false;
          position_hold_enabled_ = false;
          clearControllerIntents(position_hold_controller_name_);
          stabilize_enabled_ = false;
          updateWrenchPublisher(depth_hold_controller_name_);
        }
        depth_hold_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          depth_hold_controller_name_.c_str(),
          depth_hold_enabled_ ? "activated" : "deactivated");

        if (!depth_hold_enabled_) {
          publishZeroFeedforward();
          clearControllerIntents(depth_hold_controller_name_);
          updateWrenchPublisher(stabilize_controller_name_);
        }
      });
  }

  void requestBodyVelocityToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestBodyVelocityState(
          !isControllerActive(controller_states, body_velocity_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestBodyVelocityState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
      }
      if (!isControllerActive(controller_states, position_hold_controller_name_) &&
        !isControllerActive(controller_states, body_velocity_controller_name_))
      {
        activate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (!isControllerActive(controller_states, depth_hold_controller_name_)) {
        activate_controllers.push_back(depth_hold_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        body_velocity_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            body_velocity_controller_name_.c_str());
          return;
        }

        if (enable) {
          body_force_enabled_ = true;
          stabilize_enabled_ = false;
          position_hold_enabled_ = false;
          clearControllerIntents(position_hold_controller_name_);
          depth_hold_enabled_ = true;
          updateTwistPublisher(body_velocity_controller_name_);
        }
        body_velocity_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          body_velocity_controller_name_.c_str(),
          body_velocity_enabled_ ? "activated" : "deactivated");

        if (!body_velocity_enabled_) {
          publishZeroFeedforward();
          clearControllerIntents(body_velocity_controller_name_);
          depth_hold_enabled_ = true;
          updateWrenchPublisher(depth_hold_controller_name_);
        }
      });
  }

  void requestPositionHoldToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestPositionHoldState(
          !isControllerActive(controller_states, position_hold_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestPositionHoldState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (!isControllerActive(controller_states, depth_hold_controller_name_)) {
        activate_controllers.push_back(depth_hold_controller_name_);
      }
      if (!isControllerActive(controller_states, body_velocity_controller_name_)) {
        activate_controllers.push_back(body_velocity_controller_name_);
      }
      if (!isControllerActive(controller_states, position_hold_controller_name_)) {
        activate_controllers.push_back(position_hold_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
      }
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        position_hold_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            position_hold_controller_name_.c_str());
          return;
        }

        if (enable) {
          body_force_enabled_ = true;
          body_velocity_enabled_ = false;
          stabilize_enabled_ = false;
          depth_hold_enabled_ = true;
          position_hold_enabled_ = true;
          updateTwistPublisher(position_hold_controller_name_);
        } else {
          position_hold_enabled_ = false;
          publishZeroFeedforward();
          clearControllerIntents(position_hold_controller_name_);
          depth_hold_enabled_ = true;
          updateWrenchPublisher(depth_hold_controller_name_);
        }

        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          position_hold_controller_name_.c_str(),
          position_hold_enabled_ ? "activated" : "deactivated");
      });
  }

  void requestBodyForceToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestBodyForceState(
          !isControllerActive(controller_states, body_force_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestBodyForceState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (isControllerActive(controller_states, depth_hold_controller_name_)) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
      if (isControllerActive(controller_states, body_force_controller_name_)) {
        deactivate_controllers.push_back(body_force_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        body_force_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            body_force_controller_name_.c_str());
          return;
        }

        body_force_enabled_ = enable;
        if (!enable) {
          body_velocity_enabled_ = false;
          position_hold_enabled_ = false;
          stabilize_enabled_ = false;
          depth_hold_enabled_ = false;
          publishZeroFeedforward();
          updateWrenchPublisher(stabilize_controller_name_);
        }

        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          body_force_controller_name_.c_str(),
          body_force_enabled_ ? "activated" : "deactivated");
      });
  }

  void sendSwitchRequest(
    const std::vector<std::string> & activate_controllers,
    const std::vector<std::string> & deactivate_controllers,
    std::function<void(rclcpp::Client<SwitchControllerSrv>::SharedFuture)> response_callback)
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
    request->activate_controllers = activate_controllers;
    request->deactivate_controllers = deactivate_controllers;
    request->strictness = SwitchControllerSrv::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 2;
    request->timeout.nanosec = 0;

    switch_in_progress_ = true;
    const auto future = switch_controller_client_->async_send_request(
      request,
      [this, response_callback](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        switch_in_progress_ = false;
        response_callback(future_response);
      });

    (void)future;
  }

  bool activationBlockedByInterlock(
    const std::vector<std::string> & activate_controllers,
    std::string & message)
  {
    if (!controller_interlock_enabled_) {
      return false;
    }

    for (const auto & controller : activate_controllers) {
      if (std::find(
          controller_interlock_blocked_controllers_.begin(),
          controller_interlock_blocked_controllers_.end(),
          controller) != controller_interlock_blocked_controllers_.end())
      {
        message =
          "controller '" + controller + "' is blocked by safety interlock: " +
          controller_interlock_reason_;
        return true;
      }
    }

    return false;
  }

  void updateAutonomousMode(const JoyMsg & msg)
  {
    const bool pattern_active = autonomousModePatternActive(msg);
    const bool was_autonomous_enabled = autonomous_enabled_;

    if (!autonomous_mode_enabled_) {
      autonomous_enabled_ = false;
    } else if (autonomous_mode_behavior_ == "toggle") {
      if (pattern_active && !last_autonomous_mode_pattern_state_) {
        autonomous_enabled_ = !autonomous_enabled_;
      }
    } else {
      autonomous_enabled_ = pattern_active;
    }

    last_autonomous_mode_pattern_state_ = pattern_active;
    publishAutonomousMode(autonomous_enabled_);

    if (was_autonomous_enabled != autonomous_enabled_) {
      if (autonomous_enabled_) {
        clearManualControllerIntents();
      }
      RCLCPP_INFO(
        get_logger(),
        "Autonomous mode %s.",
        autonomous_enabled_ ? "enabled" : "disabled");
    }

  }

  bool autonomousModePatternActive(const JoyMsg & msg) const
  {
    if (!autonomous_mode_enabled_) {
      return false;
    }
    if (!isValidAxisIndex(msg.axes, autonomous_mode_axis_)) {
      return false;
    }

    return std::abs(
      static_cast<double>(msg.axes[static_cast<size_t>(autonomous_mode_axis_)]) -
      autonomous_mode_value_) <= autonomous_mode_tolerance_;
  }

  void publishAutonomousMode(bool enabled)
  {
    if (!autonomous_enabled_pub_) {
      return;
    }

    BoolMsg msg;
    msg.data = enabled;
    autonomous_enabled_pub_->publish(msg);
  }

  void updateControllerInterlockState()
  {
    if (controller_interlock_query_in_progress_) {
      return;
    }
    if (!controller_interlock_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<ControllerInterlockSrv::Request>();
    request->command = ControllerInterlockSrv::Request::QUERY;

    controller_interlock_query_in_progress_ = true;
    const auto future = controller_interlock_client_->async_send_request(
      request,
      [this](rclcpp::Client<ControllerInterlockSrv>::SharedFuture future_response)
      {
        controller_interlock_query_in_progress_ = false;
        const auto response = future_response.get();
        if (!response->success) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Controller interlock query failed: %s",
            response->message.c_str());
          return;
        }

        controller_interlock_enabled_ = response->enabled;
        controller_interlock_reason_ = response->reason;
        controller_interlock_blocked_controllers_ = response->blocked_controllers;
      });
    (void)future;
  }

  void processAlphaForwardControllerSelection(const JoyMsg & msg, bool rb_pressed)
  {
    if (!rb_pressed) {
      last_hat_horizontal_state_ = 0;
      return;
    }

    const double hat_value = readAxis(msg.axes, hat_horizontal_axis_);
    int current_hat_horizontal_state = 0;
    if (hat_value > 0.5) {
      current_hat_horizontal_state = 1;
    } else if (hat_value < -0.5) {
      current_hat_horizontal_state = -1;
    }

    if (current_hat_horizontal_state != last_hat_horizontal_state_) {
      if (current_hat_horizontal_state > 0) {
        requestAlphaForwardControllerSelection(AlphaForwardControllerSelection::Left);
      } else if (current_hat_horizontal_state < 0) {
        requestAlphaForwardControllerSelection(AlphaForwardControllerSelection::Right);
      }
    }

    last_hat_horizontal_state_ = current_hat_horizontal_state;
  }

  void requestAlphaForwardControllerSelection(AlphaForwardControllerSelection selection)
  {
    if (switch_in_progress_) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring Alpha controller request because a switch is already in progress.");
      return;
    }

    if (selection == alpha_forward_controller_selection_) {
      return;
    }

    if (!list_controllers_client_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(
        get_logger(),
        "Controller list service '%s' is not available.",
        controller_list_service_.c_str());
      return;
    }

    switch_in_progress_ = true;
    auto request = std::make_shared<ListControllersSrv::Request>();
    const auto future = list_controllers_client_->async_send_request(
      request,
      [this, selection](rclcpp::Client<ListControllersSrv>::SharedFuture future_response)
      {
        switch_in_progress_ = false;

        std::unordered_map<std::string, std::string> controller_states;
        for (const auto & controller : future_response.get()->controller) {
          controller_states[controller.name] = controller.state;
        }

        const auto is_active = [&controller_states](const std::string & controller_name) {
            const auto iterator = controller_states.find(controller_name);
            return iterator != controller_states.end() && iterator->second == "active";
          };

        std::vector<std::string> activate_controllers;
        std::vector<std::string> deactivate_controllers;

        if (selection == AlphaForwardControllerSelection::Left) {
          if (!is_active(alpha_left_forward_velocity_controller_name_)) {
            activate_controllers.push_back(alpha_left_forward_velocity_controller_name_);
          }
          if (is_active(alpha_left_joint_trajectory_controller_name_)) {
            deactivate_controllers.push_back(alpha_left_joint_trajectory_controller_name_);
          }
          if (is_active(alpha_right_forward_velocity_controller_name_)) {
            deactivate_controllers.push_back(alpha_right_forward_velocity_controller_name_);
          }
        } else if (selection == AlphaForwardControllerSelection::Right) {
          if (!is_active(alpha_right_forward_velocity_controller_name_)) {
            activate_controllers.push_back(alpha_right_forward_velocity_controller_name_);
          }
          if (is_active(alpha_right_joint_trajectory_controller_name_)) {
            deactivate_controllers.push_back(alpha_right_joint_trajectory_controller_name_);
          }
          if (is_active(alpha_left_forward_velocity_controller_name_)) {
            deactivate_controllers.push_back(alpha_left_forward_velocity_controller_name_);
          }
        } else {
          if (is_active(alpha_left_forward_velocity_controller_name_)) {
            deactivate_controllers.push_back(alpha_left_forward_velocity_controller_name_);
          }
          if (is_active(alpha_right_forward_velocity_controller_name_)) {
            deactivate_controllers.push_back(alpha_right_forward_velocity_controller_name_);
          }
        }

        if (activate_controllers.empty() && deactivate_controllers.empty()) {
          alpha_forward_controller_selection_ = selection;
          RCLCPP_INFO(
            get_logger(),
            "Active Alpha forward velocity controller: %s.",
            getAlphaForwardControllerLabel(selection).c_str());
          return;
        }

        sendSwitchRequest(
          activate_controllers,
          deactivate_controllers,
          [this, selection](rclcpp::Client<SwitchControllerSrv>::SharedFuture switch_future)
          {
            const auto response = switch_future.get();
            if (!response->ok) {
              RCLCPP_ERROR(get_logger(), "Failed to switch Alpha forward velocity controllers.");
              return;
            }

            alpha_forward_controller_selection_ = selection;
            RCLCPP_INFO(
              get_logger(),
              "Active Alpha forward velocity controller: %s.",
              getAlphaForwardControllerLabel(selection).c_str());
          });
      });

    (void)future;
  }

  void setTeleopMode(TeleopMode mode)
  {
    if (teleop_mode_ == mode) {
      return;
    }

    teleop_mode_ = mode;
    RCLCPP_INFO(
      get_logger(),
      "Teleop mode changed to %s.",
      teleop_mode_ == TeleopMode::Arm ? "arm" : "auv");
  }

  std::string getAlphaForwardControllerLabel(AlphaForwardControllerSelection selection) const
  {
    if (selection == AlphaForwardControllerSelection::Left) {
      return alpha_left_forward_velocity_controller_name_;
    }
    if (selection == AlphaForwardControllerSelection::Right) {
      return alpha_right_forward_velocity_controller_name_;
    }
    return "none";
  }

  void processHatCommands(const JoyMsg & msg)
  {
    if (!stabilize_enabled_ && !depth_hold_enabled_) {
      last_hat_vertical_state_ = 0;
      return;
    }

    const double hat_value = readAxis(msg.axes, hat_vertical_axis_);
    int current_hat_vertical_state = 0;
    if (hat_value > 0.5) {
      current_hat_vertical_state = 1;
    } else if (hat_value < -0.5) {
      current_hat_vertical_state = -1;
    }

    if (current_hat_vertical_state != last_hat_vertical_state_) {
      if (current_hat_vertical_state > 0) {
        requestRollPitchService(getActiveEnableRollPitchClient(), "enable");
      } else if (current_hat_vertical_state < 0) {
        requestRollPitchService(getActiveDisableRollPitchClient(), "disable");
      }
    }

    last_hat_vertical_state_ = current_hat_vertical_state;
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr getActiveEnableRollPitchClient() const
  {
    if (depth_hold_enabled_) {
      return depth_hold_enable_roll_pitch_client_;
    }
    return stabilize_enable_roll_pitch_client_;
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr getActiveDisableRollPitchClient() const
  {
    if (depth_hold_enabled_) {
      return depth_hold_disable_roll_pitch_client_;
    }
    return stabilize_disable_roll_pitch_client_;
  }

  void requestRollPitchService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & action_name)
  {
    if (!client->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(
        get_logger(),
        "Roll/pitch %s service is not available.",
        action_name.c_str());
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = client->async_send_request(
      request,
      [this, action_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->success) {
          RCLCPP_WARN(
            get_logger(),
            "Failed to %s roll/pitch: %s",
            action_name.c_str(),
            response->message.c_str());
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "Roll/pitch %s request accepted.",
          action_name.c_str());
      });

    (void)future;
  }

  int normalizePriority(int priority) const
  {
    const int selected_priority = priority < 0 ? priority_ : priority;
    return static_cast<int>(std::clamp<int64_t>(selected_priority, 1, 100));
  }

  bool hasTwistInput(const TwistMsg & twist) const
  {
    constexpr double epsilon = 1e-9;
    return std::abs(twist.linear.x) > epsilon ||
      std::abs(twist.linear.y) > epsilon ||
      std::abs(twist.linear.z) > epsilon ||
      std::abs(twist.angular.x) > epsilon ||
      std::abs(twist.angular.y) > epsilon ||
      std::abs(twist.angular.z) > epsilon;
  }

  bool hasWrenchInput(const WrenchMsg & wrench) const
  {
    constexpr double epsilon = 1e-9;
    return std::abs(wrench.force.x) > epsilon ||
      std::abs(wrench.force.y) > epsilon ||
      std::abs(wrench.force.z) > epsilon ||
      std::abs(wrench.torque.x) > epsilon ||
      std::abs(wrench.torque.y) > epsilon ||
      std::abs(wrench.torque.z) > epsilon;
  }

  int computeTeleopPriority(bool has_manual_input)
  {
    const int64_t now_ns = now().nanoseconds();

    if (has_manual_input) {
      last_manual_input_time_ns_ = now_ns;
      return priority_;
    }

    if (last_manual_input_time_ns_ == 0) {
      return idle_priority_;
    }

    const double idle_seconds =
      static_cast<double>(now_ns - last_manual_input_time_ns_) * 1e-9;

    return idle_seconds >= idle_priority_delay_ ? idle_priority_ : priority_;
  }

  double readAxis(const std::vector<float> & axes, int index) const
  {
    if (!isValidAxisIndex(axes, index)) {
      return 0.0;
    }

    const double value = static_cast<double>(axes[static_cast<size_t>(index)]);
    return std::fabs(value) < deadzone_ ? 0.0 : value;
  }

  double readTriggerAxis(const std::vector<float> & axes, int index) const
  {
    if (!isValidAxisIndex(axes, index)) {
      return 0.0;
    }

    const double value = static_cast<double>(axes[static_cast<size_t>(index)]);
    return std::clamp((1.0 - value) * 0.5, 0.0, 1.0);
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
  double roll_scale_{1.0};
  double pitch_scale_{1.0};
  double deadzone_{0.05};

  int lb_button_{4};
  int rb_button_{5};
  int a_button_{0};
  int b_button_{1};
  int x_button_{2};
  int y_button_{3};
  int left_stick_button_{9};
  int right_stick_button_{10};
  int hat_horizontal_axis_{6};
  int hat_vertical_axis_{7};
  int surge_axis_{4};
  int sway_axis_{3};
  int yaw_axis_{0};
  int heave_axis_{1};
  int roll_axis_{3};
  int pitch_axis_{4};
  int lt_axis_{2};
  int rt_axis_{5};

  bool body_force_enabled_{false};
  bool body_velocity_enabled_{false};
  bool position_hold_enabled_{false};
  bool stabilize_enabled_{false};
  bool depth_hold_enabled_{false};
  bool last_body_velocity_combo_state_{false};
  bool last_position_hold_combo_state_{false};
  bool last_stabilize_combo_state_{false};
  bool last_depth_hold_combo_state_{false};
  bool last_body_force_combo_state_{false};
  bool last_left_stick_button_state_{false};
  bool last_right_stick_button_state_{false};
  bool switch_in_progress_{false};
  bool controller_interlock_enabled_{false};
  bool controller_interlock_query_in_progress_{false};
  bool autonomous_mode_enabled_{true};
  bool autonomous_enabled_{false};
  bool last_autonomous_mode_pattern_state_{false};
  int last_hat_horizontal_state_{0};
  int last_hat_vertical_state_{0};

  std::string joy_topic_;
  std::string requester_;
  int priority_{80};
  int idle_priority_{50};
  double idle_priority_delay_{3.0};
  int64_t last_manual_input_time_ns_{0};
  int autonomous_mode_axis_{2};
  std::string arbitrator_velocity_topic_;
  std::string arbitrator_wrench_topic_;
  std::string clear_controller_intents_service_;
  std::string controller_interlock_service_;
  std::string controller_interlock_reason_;
  std::string autonomous_mode_topic_;
  std::string autonomous_mode_behavior_;
  std::string controller_switch_service_;
  std::string controller_list_service_;
  std::string body_force_controller_name_;
  std::string body_velocity_controller_name_;
  std::string position_hold_controller_name_;
  std::string position_hold_reposition_controller_name_;
  std::string stabilize_controller_name_;
  std::string depth_hold_controller_name_;
  std::string alpha_left_forward_velocity_controller_name_;
  std::string alpha_right_forward_velocity_controller_name_;
  std::string alpha_left_joint_trajectory_controller_name_;
  std::string alpha_right_joint_trajectory_controller_name_;
  std::string active_command_controller_;
  std::string body_force_command_topic_;
  std::string body_velocity_setpoint_topic_;
  std::string position_hold_feedforward_topic_;
  std::string position_hold_reposition_feedforward_topic_;
  std::string stabilize_feedforward_topic_;
  std::string depth_hold_feedforward_topic_;
  std::string alpha_left_forward_velocity_command_topic_;
  std::string alpha_right_forward_velocity_command_topic_;
  std::string stabilize_enable_roll_pitch_service_name_;
  std::string stabilize_disable_roll_pitch_service_name_;
  std::string depth_hold_enable_roll_pitch_service_name_;
  std::string depth_hold_disable_roll_pitch_service_name_;
  double alpha_forward_command_rate_{10.0};
  double body_force_feedforward_gain_x_{20.0};
  double body_force_feedforward_gain_y_{20.0};
  double body_force_feedforward_gain_z_{90.0};
  double body_force_feedforward_gain_roll_{20.0};
  double body_force_feedforward_gain_pitch_{20.0};
  double body_force_feedforward_gain_yaw_{1.0};
  double stabilize_feedforward_gain_x_{20.0};
  double stabilize_feedforward_gain_y_{20.0};
  double stabilize_feedforward_gain_z_{90.0};
  double stabilize_feedforward_gain_roll_{20.0};
  double stabilize_feedforward_gain_pitch_{20.0};
  double stabilize_feedforward_gain_yaw_{1.0};
  double depth_hold_feedforward_gain_x_{20.0};
  double depth_hold_feedforward_gain_y_{20.0};
  double depth_hold_feedforward_gain_z_{90.0};
  double depth_hold_feedforward_gain_roll_{20.0};
  double depth_hold_feedforward_gain_pitch_{20.0};
  double depth_hold_feedforward_gain_yaw_{1.0};
  double alpha_axis_a_velocity_scale_{0.01};
  double autonomous_mode_value_{-1.0};
  double autonomous_mode_tolerance_{0.05};

  JoyMsg::SharedPtr last_joy_msg_;
  TeleopMode teleop_mode_{TeleopMode::Auv};
  CommandOutputMode command_output_mode_{CommandOutputMode::None};
  AlphaForwardControllerSelection alpha_forward_controller_selection_{
    AlphaForwardControllerSelection::None};

  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::Publisher<SuraVelocityCommandMsg>::SharedPtr velocity_intent_pub_;
  rclcpp::Publisher<SuraWrenchCommandMsg>::SharedPtr wrench_intent_pub_;
  rclcpp::Publisher<BoolMsg>::SharedPtr autonomous_enabled_pub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr position_hold_feedforward_pub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr position_hold_reposition_feedforward_pub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr alpha_left_forward_velocity_command_pub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr alpha_right_forward_velocity_command_pub_;
  rclcpp::Client<ListControllersSrv>::SharedPtr list_controllers_client_;
  rclcpp::Client<SwitchControllerSrv>::SharedPtr switch_controller_client_;
  rclcpp::Client<sura_msgs::srv::ClearControllerIntents>::SharedPtr clear_controller_intents_client_;
  rclcpp::Client<ControllerInterlockSrv>::SharedPtr controller_interlock_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stabilize_enable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stabilize_disable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr depth_hold_enable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr depth_hold_disable_roll_pitch_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr alpha_forward_timer_;
  rclcpp::TimerBase::SharedPtr controller_interlock_timer_;
  std::vector<std::string> controller_interlock_blocked_controllers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CirtesubTeleop>());
  rclcpp::shutdown();
  return 0;
}
