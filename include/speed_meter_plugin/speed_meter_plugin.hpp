#ifndef SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_
#define SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_

#include "speed_meter_plugin/visibility_control.h"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/color_property.hpp>
#endif

namespace rviz_common
{
  namespace properties
  {
    class ColorProperty;
  }
}

namespace speed_meter_plugin
{

  class SpeedMeterPlugin : public rviz_common::Display
  {
    Q_OBJECT
  public:
    SpeedMeterPlugin();
    ~SpeedMeterPlugin();

    void onInitialize() override;

  private Q_SLOTS:
    void updateColor();

  private:
    std::unique_ptr<rviz_common::properties::ColorProperty> bg_color_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> scale_color_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> t_speed_color_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> c_speed_color_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> v_speed_color_property_;
  };

} // namespace speed_meter_plugin

#endif // SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_
