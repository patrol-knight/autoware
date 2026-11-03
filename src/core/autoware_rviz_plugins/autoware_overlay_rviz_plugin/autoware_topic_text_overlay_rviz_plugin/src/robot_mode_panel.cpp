#include "robot_mode_panel.hpp"

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
struct ModeStyle
{
  const char * color;
  const char * detail;
};

ModeStyle style_for(const std::string & mode)
{
  if (mode == "none") {
    return {"#bfc5c9", "No control mode selected"};
  }
  if (mode == "manual") {
    return {"#ffd24d", "Manual control active"};
  }
  if (mode == "auto") {
    return {"#00e5ea", "Autonomous control active"};
  }
  if (mode.empty()) {
    return {"#bfc5c9", "Waiting for /robot_mode"};
  }
  return {"#bfc5c9", "Unknown robot mode"};
}
}  // namespace

RobotModePanel::RobotModePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setObjectName("RobotModePanel");
  setStyleSheet("QWidget#RobotModePanel { background-color: #f0f0f0; }");
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  auto * layout = new QGridLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setHorizontalSpacing(10);
  layout->setVerticalSpacing(5);
  layout->setColumnStretch(0, 1);
  layout->setColumnStretch(1, 2);

  auto * mode_name = new QLabel("MODE:", this);
  mode_name->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  mode_value_ = new QLabel("WAITING", this);
  mode_value_->setAlignment(Qt::AlignCenter);
  mode_value_->setMinimumSize(210, 22);
  mode_value_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  QFont mode_font = mode_value_->font();
  mode_font.setBold(true);
  mode_value_->setFont(mode_font);

  auto * detail_name = new QLabel("DETAIL:", this);
  detail_name->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  detail_value_ = new QLabel(this);
  detail_value_->setAlignment(Qt::AlignCenter);
  detail_value_->setWordWrap(true);

  layout->addWidget(mode_name, 0, 0);
  layout->addWidget(mode_value_, 0, 1);
  layout->addWidget(detail_name, 1, 0);
  layout->addWidget(detail_value_, 1, 1);
  update_mode("");
}

void RobotModePanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  mode_sub_ = raw_node_->create_subscription<std_msgs::msg::String>(
    "/robot_mode", qos, [this](const std_msgs::msg::String::ConstSharedPtr msg) {
      const std::string mode = msg->data;
      QMetaObject::invokeMethod(
        this, [this, mode]() { update_mode(mode); }, Qt::QueuedConnection);
    });
}

void RobotModePanel::update_mode(const std::string & mode)
{
  const auto style = style_for(mode);
  const QString text = mode.empty() ? "WAITING" : QString::fromStdString(mode).toUpper();
  mode_value_->setText(text);
  mode_value_->setStyleSheet(
    QString("QLabel { background-color: %1; color: #101010; border: none; }").arg(style.color));
  detail_value_->setText(style.detail);
}
}  // namespace autoware::topic_text_overlay_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::topic_text_overlay_rviz_plugin::RobotModePanel, rviz_common::Panel)
