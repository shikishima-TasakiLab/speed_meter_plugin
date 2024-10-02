#include "speed_meter_plugin/speed_meter_plugin.hpp"

namespace speed_meter_plugin
{

  SpeedMeterPlugin::SpeedMeterPlugin()
  {
    bg_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("background/color", QColor(0, 0, 0), "", this, SLOT(updateColor()));
    scale_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("scale/color", QColor(0, 0, 0), "", this, SLOT(updateColor()));
    t_speed_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("target_speed/color", QColor(0, 0, 0), "", this, SLOT(updateColor()));
    c_speed_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("control_speed/color", QColor(0, 0, 0), "", this, SLOT(updateColor()));
    v_speed_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("current_speed/color", QColor(0, 0, 0), "", this, SLOT(updateColor()));
  }

  SpeedMeterPlugin::~SpeedMeterPlugin()
  {
  }

  void SpeedMeterPlugin::onInitialize()
  {
  }

  void SpeedMeterPlugin::updateColor()
  {
  }

} // namespace speed_meter_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(speed_meter_plugin::SpeedMeterPlugin, rviz_common::Display)
