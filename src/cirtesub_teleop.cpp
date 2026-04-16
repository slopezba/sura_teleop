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
#include "std_srvs/srv/trigger.hpp"

class CirtesubTeleop : public rclcpp::Node
{
public:
  CirtesubTeleop()
  : Node("cirtesub_teleop")
  {
    declare_parameter<double>("rate", 20.0);
    declare_parameter<std::string>("joy_topic", "/joy");
    declare_parameter<std::string>("controller_switch_service", "/controller_manager/switch_controller");
    declare_parameter<std::string>("body_force_controller.name", "body_force_controller");
    declare_parameter<std::string>("body_velocity_controller.name", "body_velocity_controller");
    declare_parameter<std::string>(
      "body_velocity_controller.setpoint_topic",
      "/body_velocity_controller/setpoint");
    declare_parameter<std::string>("stabilize_controller.name", "stabilize_controller");
    declare_parameter<std::string>(
      "stabilize_controller.feedforward_topic",
      "/stabilize_controller/feedforward");
    declare_parameter<std::string>(
      "stabilize_controller.enable_roll_pitch_service",
      "/stabilize_controller/enable_roll_pitch");
    declare_parameter<std::string>(
      "stabilize_controller.disable_roll_pitch_service",
      "/stabilize_controller/disable_roll_pitch");
    declare_parameter<std::string>("depth_hold_controller.name", "depth_hold_controller");
    declare_parameter<std::string>(
      "depth_hold_controller.feedforward_topic",
      "/depth_hold_controller/feedforward");
    declare_parameter<std::string>(
      "depth_hold_controller.enable_roll_pitch_service",
      "/depth_hold_controller/enable_roll_pitch");
    declare_parameter<std::string>(
      "depth_hold_controller.disable_roll_pitch_service",
      "/depth_hold_controller/disable_roll_pitch");
    declare_parameter<int>("buttons.a", 0);
    declare_parameter<int>("buttons.x", 2);
    declare_parameter<int>("buttons.lb", 4);
    declare_parameter<int>("buttons.rb", 5);
    declare_parameter<int>("buttons.y", 3);
    declare_parameter<int>("axes.hat_vertical", 7);
    declare_parameter<int>("axes.surge", 1);
    declare_parameter<int>("axes.sway", 0);
    declare_parameter<int>("axes.yaw", 3);
    declare_parameter<int>("axes.heave", 4);
    declare_parameter<int>("axes.roll", 3);
    declare_parameter<int>("axes.pitch", 4);
    declare_parameter<double>("scales.surge", 1.0);
    declare_parameter<double>("scales.sway", 1.0);
    declare_parameter<double>("scales.yaw", 1.0);
    declare_parameter<double>("scales.heave", 1.0);
    declare_parameter<double>("scales.roll", 1.0);
    declare_parameter<double>("scales.pitch", 1.0);
    declare_parameter<double>("deadzone", 0.05);

    rate_ = get_parameter("rate").as_double();
    joy_topic_ = get_parameter("joy_topic").as_string();
    controller_switch_service_ = get_parameter("controller_switch_service").as_string();
    body_force_controller_name_ = get_parameter("body_force_controller.name").as_string();
    body_velocity_controller_name_ = get_parameter("body_velocity_controller.name").as_string();
    body_velocity_setpoint_topic_ =
      get_parameter("body_velocity_controller.setpoint_topic").as_string();
    stabilize_controller_name_ = get_parameter("stabilize_controller.name").as_string();
    stabilize_feedforward_topic_ =
      get_parameter("stabilize_controller.feedforward_topic").as_string();
    stabilize_enable_roll_pitch_service_name_ =
      get_parameter("stabilize_controller.enable_roll_pitch_service").as_string();
    stabilize_disable_roll_pitch_service_name_ =
      get_parameter("stabilize_controller.disable_roll_pitch_service").as_string();
    depth_hold_controller_name_ = get_parameter("depth_hold_controller.name").as_string();
    depth_hold_feedforward_topic_ =
      get_parameter("depth_hold_controller.feedforward_topic").as_string();
    depth_hold_enable_roll_pitch_service_name_ =
      get_parameter("depth_hold_controller.enable_roll_pitch_service").as_string();
    depth_hold_disable_roll_pitch_service_name_ =
      get_parameter("depth_hold_controller.disable_roll_pitch_service").as_string();
    a_button_ = get_parameter("buttons.a").as_int();
    x_button_ = get_parameter("buttons.x").as_int();
    lb_button_ = get_parameter("buttons.lb").as_int();
    rb_button_ = get_parameter("buttons.rb").as_int();
    y_button_ = get_parameter("buttons.y").as_int();
    hat_vertical_axis_ = get_parameter("axes.hat_vertical").as_int();
    surge_axis_ = get_parameter("axes.surge").as_int();
    sway_axis_ = get_parameter("axes.sway").as_int();
    yaw_axis_ = get_parameter("axes.yaw").as_int();
    heave_axis_ = get_parameter("axes.heave").as_int();
    roll_axis_ = get_parameter("axes.roll").as_int();
    pitch_axis_ = get_parameter("axes.pitch").as_int();
    surge_scale_ = get_parameter("scales.surge").as_double();
    sway_scale_ = get_parameter("scales.sway").as_double();
    yaw_scale_ = get_parameter("scales.yaw").as_double();
    heave_scale_ = get_parameter("scales.heave").as_double();
    roll_scale_ = get_parameter("scales.roll").as_double();
    pitch_scale_ = get_parameter("scales.pitch").as_double();
    deadzone_ = std::max(0.0, get_parameter("deadzone").as_double());

    if (rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid rate %.3f Hz, using 20.0 Hz.", rate_);
      rate_ = 20.0;
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CirtesubTeleop::joyCallback, this, std::placeholders::_1));

    updateFeedforwardPublisher(stabilize_feedforward_topic_);

    switch_controller_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(controller_switch_service_);
    stabilize_enable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(stabilize_enable_roll_pitch_service_name_);
    stabilize_disable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(stabilize_disable_roll_pitch_service_name_);
    depth_hold_enable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(depth_hold_enable_roll_pitch_service_name_);
    depth_hold_disable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(depth_hold_disable_roll_pitch_service_name_);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&CirtesubTeleop::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Teleop ready. RB+X toggles '%s', RB+Y toggles '%s', RB+A toggles '%s', RB+LB toggles '%s', active command topic='%s'.",
      body_velocity_controller_name_.c_str(),
      stabilize_controller_name_.c_str(),
      depth_hold_controller_name_.c_str(),
      body_force_controller_name_.c_str(),
      feedforward_topic_.c_str());
  }

private:
  using JoyMsg = sensor_msgs::msg::Joy;
  using TwistMsg = geometry_msgs::msg::Twist;
  using SwitchControllerSrv = controller_manager_msgs::srv::SwitchController;

  void updateFeedforwardPublisher(const std::string & topic_name)
  {
    feedforward_topic_ = topic_name;
    feedforward_pub_ = create_publisher<TwistMsg>(
      feedforward_topic_,
      rclcpp::SystemDefaultsQoS());
  }

  void publishZeroFeedforward()
  {
    if (feedforward_pub_) {
      feedforward_pub_->publish(TwistMsg{});
    }
  }

  void joyCallback(const JoyMsg::SharedPtr msg)
  {
    last_joy_msg_ = msg;

    if (
      !isValidButtonIndex(msg->buttons, a_button_) ||
      !isValidButtonIndex(msg->buttons, x_button_) ||
      !isValidButtonIndex(msg->buttons, lb_button_) ||
      !isValidButtonIndex(msg->buttons, rb_button_) ||
      !isValidButtonIndex(msg->buttons, y_button_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Configured button indices are out of range for the current joystick message.");
      return;
    }

    const bool a_pressed = msg->buttons[static_cast<size_t>(a_button_)] != 0;
    const bool x_pressed = msg->buttons[static_cast<size_t>(x_button_)] != 0;
    const bool lb_pressed = msg->buttons[static_cast<size_t>(lb_button_)] != 0;
    const bool rb_pressed = msg->buttons[static_cast<size_t>(rb_button_)] != 0;
    const bool y_pressed = msg->buttons[static_cast<size_t>(y_button_)] != 0;

    const bool body_velocity_combo_pressed = rb_pressed && x_pressed;
    const bool stabilize_combo_pressed = rb_pressed && y_pressed;
    const bool depth_hold_combo_pressed = rb_pressed && a_pressed;
    const bool body_force_combo_pressed = rb_pressed && lb_pressed;

    if (body_velocity_combo_pressed && !last_body_velocity_combo_state_) {
      requestBodyVelocityState(!body_velocity_enabled_);
    }

    if (stabilize_combo_pressed && !last_stabilize_combo_state_) {
      requestStabilizeState(!stabilize_enabled_);
    }

    if (depth_hold_combo_pressed && !last_depth_hold_combo_state_) {
      requestDepthHoldState(!depth_hold_enabled_);
    }

    if (body_force_combo_pressed && !last_body_force_combo_state_) {
      requestBodyForceState(!body_force_enabled_);
    }

    processHatCommands(*msg);

    last_body_velocity_combo_state_ = body_velocity_combo_pressed;
    last_stabilize_combo_state_ = stabilize_combo_pressed;
    last_depth_hold_combo_state_ = depth_hold_combo_pressed;
    last_body_force_combo_state_ = body_force_combo_pressed;
  }

  void timerCallback()
  {
    if (!body_velocity_enabled_ && !stabilize_enabled_ && !depth_hold_enabled_) {
      return;
    }

    TwistMsg cmd;
    if (last_joy_msg_ != nullptr) {
      const bool lb_pressed = isValidButtonIndex(last_joy_msg_->buttons, lb_button_) &&
        last_joy_msg_->buttons[static_cast<size_t>(lb_button_)] != 0;

      if (lb_pressed) {
        cmd.angular.x = readAxis(last_joy_msg_->axes, roll_axis_) * roll_scale_;
        cmd.angular.y = readAxis(last_joy_msg_->axes, pitch_axis_) * pitch_scale_;
      } else {
        cmd.linear.x = readAxis(last_joy_msg_->axes, surge_axis_) * surge_scale_;
        cmd.linear.y = readAxis(last_joy_msg_->axes, sway_axis_) * sway_scale_;
        cmd.linear.z = -readAxis(last_joy_msg_->axes, heave_axis_) * heave_scale_;
        cmd.angular.z = readAxis(last_joy_msg_->axes, yaw_axis_) * yaw_scale_;
      }
    }

    feedforward_pub_->publish(cmd);
  }

  void requestStabilizeState(bool enable)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!body_force_enabled_) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (body_velocity_enabled_) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (depth_hold_enabled_) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
      activate_controllers.push_back(stabilize_controller_name_);
    } else {
      deactivate_controllers.push_back(stabilize_controller_name_);
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
          depth_hold_enabled_ = false;
          updateFeedforwardPublisher(stabilize_feedforward_topic_);
        }
        stabilize_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          stabilize_controller_name_.c_str(),
          stabilize_enabled_ ? "activated" : "deactivated");

        if (!stabilize_enabled_) {
          publishZeroFeedforward();
        }
      });
  }

  void requestDepthHoldState(bool enable)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!body_force_enabled_) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (body_velocity_enabled_) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (stabilize_enabled_) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      activate_controllers.push_back(depth_hold_controller_name_);
    } else {
      deactivate_controllers.push_back(depth_hold_controller_name_);
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
          stabilize_enabled_ = false;
          updateFeedforwardPublisher(depth_hold_feedforward_topic_);
        }
        depth_hold_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          depth_hold_controller_name_.c_str(),
          depth_hold_enabled_ ? "activated" : "deactivated");

        if (!depth_hold_enabled_) {
          publishZeroFeedforward();
          updateFeedforwardPublisher(stabilize_feedforward_topic_);
        }
      });
  }

  void requestBodyVelocityState(bool enable)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!body_force_enabled_) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (stabilize_enabled_) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (depth_hold_enabled_) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
      activate_controllers.push_back(body_velocity_controller_name_);
    } else {
      deactivate_controllers.push_back(body_velocity_controller_name_);
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
          depth_hold_enabled_ = false;
          updateFeedforwardPublisher(body_velocity_setpoint_topic_);
        }
        body_velocity_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          body_velocity_controller_name_.c_str(),
          body_velocity_enabled_ ? "activated" : "deactivated");

        if (!body_velocity_enabled_) {
          publishZeroFeedforward();
          updateFeedforwardPublisher(stabilize_feedforward_topic_);
        }
      });
  }

  void requestBodyForceState(bool enable)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      activate_controllers.push_back(body_force_controller_name_);
    } else {
      if (body_velocity_enabled_) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (stabilize_enabled_) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (depth_hold_enabled_) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
      deactivate_controllers.push_back(body_force_controller_name_);
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
          stabilize_enabled_ = false;
          depth_hold_enabled_ = false;
          publishZeroFeedforward();
          updateFeedforwardPublisher(stabilize_feedforward_topic_);
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
  double roll_scale_{1.0};
  double pitch_scale_{1.0};
  double deadzone_{0.05};

  int lb_button_{4};
  int rb_button_{5};
  int a_button_{0};
  int x_button_{2};
  int y_button_{3};
  int hat_vertical_axis_{7};
  int surge_axis_{4};
  int sway_axis_{3};
  int yaw_axis_{0};
  int heave_axis_{1};
  int roll_axis_{3};
  int pitch_axis_{4};

  bool body_force_enabled_{false};
  bool body_velocity_enabled_{false};
  bool stabilize_enabled_{false};
  bool depth_hold_enabled_{false};
  bool last_body_velocity_combo_state_{false};
  bool last_stabilize_combo_state_{false};
  bool last_depth_hold_combo_state_{false};
  bool last_body_force_combo_state_{false};
  bool switch_in_progress_{false};
  int last_hat_vertical_state_{0};

  std::string joy_topic_;
  std::string controller_switch_service_;
  std::string body_force_controller_name_;
  std::string body_velocity_controller_name_;
  std::string stabilize_controller_name_;
  std::string depth_hold_controller_name_;
  std::string feedforward_topic_;
  std::string body_velocity_setpoint_topic_;
  std::string stabilize_feedforward_topic_;
  std::string depth_hold_feedforward_topic_;
  std::string stabilize_enable_roll_pitch_service_name_;
  std::string stabilize_disable_roll_pitch_service_name_;
  std::string depth_hold_enable_roll_pitch_service_name_;
  std::string depth_hold_disable_roll_pitch_service_name_;

  JoyMsg::SharedPtr last_joy_msg_;

  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr feedforward_pub_;
  rclcpp::Client<SwitchControllerSrv>::SharedPtr switch_controller_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stabilize_enable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stabilize_disable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr depth_hold_enable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr depth_hold_disable_roll_pitch_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CirtesubTeleop>());
  rclcpp::shutdown();
  return 0;
}
