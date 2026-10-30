#include "robot_status_panel.hpp"

#include <QFont>
#include <QGridLayout>
#include <QLabel>
#include <QMetaObject>
#include <QSizePolicy>
#include <QString>
#include <Qt>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

#include <string>

namespace autoware::topic_text_overlay_rviz_plugin
{
namespace
{
struct StatusStyle
{
  const char * color;
  const char * detail;
};

StatusStyle style_for(const std::string & status)
{
  if (status == "notready") {
    return {"#bfc5c9", "Waiting for localization or sensors"};
  }
  if (status == "ready") {
    return {"#23e344", "Ready for a goal"};
  }
  if (status == "failure") {
    return {"#ff5555", "Route planning or route state failed"};
  }
  if (status == "navigate") {
    return {"#00e5ea", "Autonomous navigation active"};
  }
  if (status == "reach") {
    return {"#65e84a", "Goal reached; vehicle stopped"};
  }
  if (status == "manual") {
    return {"#ffd24d", "Joystick control active"};
  }
  return {"#bfc5c9", "Waiting for /robot_status"};
}
}  // namespace

RobotStatusPanel::RobotStatusPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setObjectName("RobotStatusPanel");
  setStyleSheet("QWidget#RobotStatusPanel { background-color: #f0f0f0; }");
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  auto * layout = new QGridLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setHorizontalSpacing(10);
  layout->setVerticalSpacing(5);
  layout->setColumnStretch(0, 1);
  layout->setColumnStretch(1, 2);

  auto * status_name = new QLabel("STATUS:", this);
  status_name->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_value_ = new QLabel("WAITING", this);
  status_value_->setAlignment(Qt::AlignCenter);
  status_value_->setMinimumSize(210, 22);
  status_value_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  QFont status_font = status_value_->font();
  status_font.setBold(true);
  status_value_->setFont(status_font);

  auto * detail_name = new QLabel("DETAIL:", this);
  detail_name->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  detail_value_ = new QLabel(this);
  detail_value_->setAlignment(Qt::AlignCenter);
  detail_value_->setWordWrap(true);

  layout->addWidget(status_name, 0, 0);
  layout->addWidget(status_value_, 0, 1);
  layout->addWidget(detail_name, 1, 0);
  layout->addWidget(detail_value_, 1, 1);
  update_status("");
}

void RobotStatusPanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  status_sub_ = raw_node_->create_subscription<std_msgs::msg::String>(
    "/robot_status", qos, [this](const std_msgs::msg::String::ConstSharedPtr msg) {
      const std::string status = msg->data;
      QMetaObject::invokeMethod(
        this, [this, status]() { update_status(status); }, Qt::QueuedConnection);
    });
}

void RobotStatusPanel::update_status(const std::string & status)
{
  const auto style = style_for(status);
  const QString text = status.empty() ? "WAITING" : QString::fromStdString(status).toUpper();
  status_value_->setText(text);
  status_value_->setStyleSheet(
    QString("QLabel { background-color: %1; color: #101010; border: none; }").arg(style.color));
  detail_value_->setText(style.detail);
}
}  // namespace autoware::topic_text_overlay_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::topic_text_overlay_rviz_plugin::RobotStatusPanel, rviz_common::Panel)
