#ifndef SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_
#define SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_

#include "speed_meter_plugin/visibility_control.h"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#endif

namespace speed_meter_plugin
{

  class SpeedMeterPlugin : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    SpeedMeterPlugin(QWidget *parent = nullptr);
    ~SpeedMeterPlugin();

    void onInitialize() override;
    void load(const rviz_common::Config &config) override;
    void save(rviz_common::Config config) const override;

  protected:
    rclcpp::Node::SharedPtr nh_;
  };

} // namespace speed_meter_plugin

#endif // SPEED_METER_PLUGIN__SPEED_METER_PLUGIN_HPP_
