#ifndef ROBOT_STATUS_PANEL_HPP_
#define ROBOT_STATUS_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <std_msgs/msg/string.hpp>

#include <string>

class QLabel;

namespace autoware::topic_text_overlay_rviz_plugin
{
class RobotStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RobotStatusPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void update_status(const std::string & status);

  QLabel * status_value_;
  QLabel * detail_value_;
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};
}  // namespace autoware::topic_text_overlay_rviz_plugin

#endif  // ROBOT_STATUS_PANEL_HPP_
