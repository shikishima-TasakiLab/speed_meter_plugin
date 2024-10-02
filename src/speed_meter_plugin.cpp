#include "speed_meter_plugin/speed_meter_plugin.hpp"

namespace speed_meter_plugin
{

  SpeedMeterPlugin::SpeedMeterPlugin(QWidget *parent)
      : rviz_common::Panel(parent)
  {
  }

  SpeedMeterPlugin::~SpeedMeterPlugin()
  {
  }

  void SpeedMeterPlugin::onInitialize()
  {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

  void SpeedMeterPlugin::load(const rviz_common::Config &config)
  {
  }

  void SpeedMeterPlugin::save(rviz_common::Config config) const
  {
  }

} // namespace speed_meter_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(speed_meter_plugin::SpeedMeterPlugin, rviz_common::Panel)
