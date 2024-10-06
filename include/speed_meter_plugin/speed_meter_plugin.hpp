#ifndef SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_
#define SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_

#include "speed_meter_plugin/visibility_control.h"

#ifndef Q_MOC_RUN
#include <any>
#include <QtGui/QPainter>
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

// namespace Ui
// {
//     // class SpeedMeter;
// } // namespace Ui

// namespace rviz_common
// {
//   namespace properties
//   {
//     class ColorProperty;
//   }
// }

namespace speed_meter_plugin
{
  struct speed
  {
    QString name;
    double value{0.0};
    double min;
    double max;
    std::mutex mutex;
    size_t received_cnt{0UL};
  };

  class SpeedMeterWidget : public QWidget
  {
    Q_OBJECT
  public:
    SpeedMeterWidget(
        speed *l_speed_ptr,
        speed *t_speed_ptr,
        speed *c_speed_ptr,
        QWidget *parent = nullptr);

  protected:
    void paintEvent(QPaintEvent *event) override;

  private:
    speed *l_speed_;
    speed *t_speed_;
    speed *c_speed_;
  };

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

    template <typename msgT>
    typename rclcpp::Subscription<msgT>::SharedPtr create_speed_sub(speed &speed_struct, std::string &topic_name);

  protected Q_SLOTS:
    // void updateQosProfile();
    // void updateQueueSize();
    void updateMessageType();
    void updateColor();
    void updateTopic();

  protected:
    // int queue_size_{5};

    // rclcpp::Subscription<std::any>::SharedPtr
    std::any l_speed_sub_;
    std::any t_speed_sub_;
    std::any c_speed_sub_;

    speed l_speed_;
    speed t_speed_;
    speed c_speed_;

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

    std::unique_ptr<SpeedMeterWidget> panel_;
  };

} // namespace speed_meter_plugin

#endif // SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_
