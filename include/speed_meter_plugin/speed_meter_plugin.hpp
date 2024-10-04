#ifndef SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_
#define SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_

#include "speed_meter_plugin/visibility_control.h"

#ifndef Q_MOC_RUN
#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/uniform_string_stream.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#endif

namespace Ui
{

    // class SpeedMeter;

} // namespace Ui

// namespace rviz_common
// {
//   namespace properties
//   {
//     class ColorProperty;
//   }
// }

namespace speed_meter_plugin
{

  class SpeedMeterPlugin : public rviz_common::Display
  {
    Q_OBJECT
  public:
    SpeedMeterPlugin();
    ~SpeedMeterPlugin();

    void onInitialize() override;
    void update(float dt, float ros_dt) override;

  protected:
    void onEnable() override;
    void onDisable() override;

    void reset();
    void subscribe();
    void unsubscribe();
    void setupPanel();

  protected Q_SLOTS:
    // void updateQosProfile();
    // void updateQueueSize();
    void updateMessageType();
    void updateColor();
    void updateTopic();

  protected:
    // int queue_size_{5};
    size_t messages_received_{0};

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr l_speed_float32_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr l_speed_float64_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr t_speed_float32_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr t_speed_float64_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr c_speed_float32_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr c_speed_float64_sub_;

    std::mutex l_speed_mutex_;
    std::mutex t_speed_mutex_;
    std::mutex c_speed_mutex_;

  private:
    // rmw_qos_profile_t qos_profile_;
    // std::unique_ptr<rviz_common::properties::EditableEnumProperty> reliability_policy_property_;
    // std::unique_ptr<rviz_common::properties::IntProperty> queue_size_property_;
    std::unique_ptr<rviz_common::properties::EnumProperty> msg_type_property_;
    std::unique_ptr<rviz_common::properties::FloatProperty> l_speed_float_property_;
    std::unique_ptr<rviz_common::properties::RosTopicProperty> l_speed_topic_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> l_speed_color_property_;
    std::unique_ptr<rviz_common::properties::FloatProperty> t_speed_float_property_;
    std::unique_ptr<rviz_common::properties::RosTopicProperty> t_speed_topic_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> t_speed_color_property_;
    std::unique_ptr<rviz_common::properties::FloatProperty> c_speed_float_property_;
    std::unique_ptr<rviz_common::properties::RosTopicProperty> c_speed_topic_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> c_speed_color_property_;
    std::unique_ptr<rviz_common::properties::StringProperty> unit_property_;
    std::unique_ptr<rviz_common::properties::FloatProperty> coefficient_property_;
    std::unique_ptr<rviz_common::properties::Property> bg_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> bg_color_property_;
    std::unique_ptr<rviz_common::properties::Property> scale_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> scale_color_property_;

    double_t l_speed_value_{0.0};
    double_t t_speed_value_{0.0};
    double_t c_speed_value_{0.0};

    std::unique_ptr<QWidget> panel_;
  };

} // namespace speed_meter_plugin

#endif // SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_
