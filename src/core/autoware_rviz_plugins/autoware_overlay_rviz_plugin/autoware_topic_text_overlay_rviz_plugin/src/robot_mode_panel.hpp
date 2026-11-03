#ifndef ROBOT_MODE_PANEL_HPP_
#define ROBOT_MODE_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <std_msgs/msg/string.hpp>

#include <string>

class QLabel;

namespace autoware::topic_text_overlay_rviz_plugin
{
class RobotModePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RobotModePanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void update_mode(const std::string & mode);

  QLabel * mode_value_;
  QLabel * detail_value_;
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
};
}  // namespace autoware::topic_text_overlay_rviz_plugin

#endif  // ROBOT_MODE_PANEL_HPP_
