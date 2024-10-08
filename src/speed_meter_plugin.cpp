#include "speed_meter_plugin/speed_meter_plugin.hpp"

namespace speed_meter_plugin
{
  SpeedMeterWidget::SpeedMeterWidget(
      speed *l_speed_ptr,
      speed *t_speed_ptr,
      speed *c_speed_ptr,
      meter_scale_marks *meter_scale_ptr,
      QColor *bg_color_ptr,
      QWidget *parent)
      : QWidget(parent), l_speed_(l_speed_ptr), t_speed_(t_speed_ptr), c_speed_(c_speed_ptr), meter_scale_(meter_scale_ptr)
  {
    changeBgColor(bg_color_ptr);
  }

  void SpeedMeterWidget::changeBgColor(QColor *bg_color)
  {
    QColor color = (bg_color == nullptr) ? Qt::black : *bg_color;

    QPalette bg_pal(palette());
    bg_pal.setColor(QPalette::Window, color);
    setAutoFillBackground(true);
    setPalette(bg_pal);
  }

  void SpeedMeterWidget::paintEvent(QPaintEvent *event)
  {
    // QWidget::paintEvent(event);
    (void)event;

    int w_side = qMin(width(), height());

    QPainter painter(this);

    painter.translate(width() / 2, height() / 2);
    painter.scale(w_side / 256.0, w_side / 256.0);

    double value_min = -meter_scale_->zero_offset;
    double value_max = value_min + meter_scale_->range;
    double scale_origin = 300.0 * meter_scale_->zero_offset / meter_scale_->range - 240.0;
    double step_deg = 300.0 * meter_scale_->sub.step / meter_scale_->range;
    double theta = scale_origin;

    // Sub Scale
    {
      painter.setPen(Qt::NoPen);
      painter.setBrush(meter_scale_->sub.color);
      painter.save();
      painter.rotate(theta);
      for (; theta <= 60.0; theta += step_deg)
      {
        painter.drawRect(97, -1, 6, 2);
        painter.rotate(step_deg);
      }
      painter.restore();
      painter.save();
      theta = scale_origin - step_deg;
      painter.rotate(theta);
      for (; theta >= -240.0; theta -= step_deg)
      {
        painter.drawRect(97, -1, 6, 2);
        painter.rotate(-step_deg);
      }
      painter.restore();
    }

    // Main Scale
    {
      step_deg = 300.0 * meter_scale_->main.step / meter_scale_->range;
      theta = scale_origin;
      painter.setPen(Qt::NoPen);
      painter.setBrush(meter_scale_->main.color);
      painter.save();
      painter.rotate(theta);
      for (; theta <= 60.0; theta += step_deg)
      {
        painter.drawRect(97, -2, 14, 4);
        painter.rotate(step_deg);
      }
      painter.restore();
      painter.save();
      theta = scale_origin - step_deg;
      painter.rotate(theta);
      for (; theta >= -240.0; theta -= step_deg)
      {
        painter.drawRect(97, -2, 14, 4);
        painter.rotate(-step_deg);
      }
      painter.restore();
    }

    // Unit
    {
      QFont font_unit = font();
      font_unit.setPointSize(meter_scale_->unit.fontsize);
      painter.setFont(font_unit);
      painter.setPen(meter_scale_->unit.color);
      painter.setBrush(Qt::NoBrush);
      painter.drawText(-128, 64, 256, 32, Qt::AlignCenter, meter_scale_->unit.name);
    }

    // Current Speed (chart)
    {
      QPainterPath bg_path;
      bg_path.arcTo(-64, -64, 128, 128, 240.0, -300.0);
      bg_path.arcTo(-96, -96, 192, 192, -60.0, 300.0);
      painter.setPen(Qt::NoPen);
      painter.setBrush(c_speed_->bg);
      painter.drawPath(bg_path);
    }
    if (c_speed_->enable)
    {
      double c_value = qMin(qMax(value_min, c_speed_->value), value_max);
      QPainterPath fg_path;
      theta = -300.0 * c_value / meter_scale_->range;
      fg_path.arcTo(-64, -64, 128, 128, -scale_origin, theta);
      fg_path.arcTo(-96, -96, 192, 192, theta - scale_origin, -theta);
      painter.setPen(Qt::NoPen);
      painter.setBrush(c_speed_->fg);
      painter.drawPath(fg_path);
    }

    // Target Speed
    if (t_speed_->enable)
    {
      painter.setPen(Qt::NoPen);
      painter.setBrush(t_speed_->fg);
      double t_value = qMin(qMax(value_min, t_speed_->value), value_max);
      theta = 300.0 * t_value / meter_scale_->range;
      painter.save();
      painter.rotate(scale_origin + theta);
      painter.drawRect(64, -2, 32, 4);
      painter.restore();
    }

    // Speed Limit
    if (l_speed_->enable)
    {
      painter.setPen(Qt::NoPen);
      painter.setBrush(l_speed_->fg);
      double l_value = qMin(qMax(value_min, l_speed_->value), value_max);
      theta = 300.0 * l_value / meter_scale_->range;
      QPainterPath fg_path;
      fg_path.moveTo(105, 0);
      fg_path.lineTo(123, -9);
      fg_path.lineTo(123, 9);
      painter.save();
      painter.rotate(scale_origin + theta);
      painter.drawPath(fg_path);
      painter.restore();
    }

    // Current Speed (text)
    {
      QFont font_cs = font();
      font_cs.setPointSize(c_speed_->fontsize);
      painter.setFont(font_cs);
      painter.setPen(meter_scale_->unit.color);
      painter.setBrush(Qt::NoBrush);
      painter.drawText(-128, -64, 256, 128, Qt::AlignCenter, QString::number(c_speed_->value, 'f', meter_scale_->precision));
    }
  }

  SpeedMeterPlugin::SpeedMeterPlugin()
  {
    l_speed_.name = "Speed Limit";
    t_speed_.name = "Target Speed";
    c_speed_.name = "Current Speed";

    // qos_profile_ = rmw_qos_profile_sensor_data;
    // reliability_policy_property_ = std::make_unique<rviz_common::properties::EditableEnumProperty>("Reliability Policy", "Best effort", "", this, SLOT(updateQosProfile()));
    // reliability_policy_property_->addOption("System Default");
    // reliability_policy_property_->addOption("Reliable");
    // reliability_policy_property_->addOption("Best effort");
    // queue_size_property_ = std::make_unique<rviz_common::properties::IntProperty>("Queue Size", queue_size_, "", this, SLOT(updateQueueSize()));
    // queue_size_property_->setMin(1);
    msg_type_property_ = std::make_unique<rviz_common::properties::EnumProperty>("Message Type", "std_msgs/msg/Float64", "", this, SLOT(updateMessageType()));
    msg_type_property_->addOption("std_msgs/msg/Float32", 0);
    msg_type_property_->addOption("std_msgs/msg/Float64", 1);
    l_speed_enable_property_ = std::make_unique<rviz_common::properties::BoolProperty>(l_speed_.name, true, "", this, SLOT(updateValues()));
    l_speed_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>("topic", "", "std_msgs/msg/Float64", "", l_speed_enable_property_.get(), SLOT(updateTopic()), this);
    l_speed_fg_property_ = std::make_unique<rviz_common::properties::ColorProperty>("color", QColor(255, 0, 0), "", l_speed_enable_property_.get(), SLOT(updateColor()), this);
    t_speed_enable_property_ = std::make_unique<rviz_common::properties::BoolProperty>(t_speed_.name, true, "", this, SLOT(updateValues()));
    t_speed_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>("topic", "", "std_msgs/msg/Float64", "", t_speed_enable_property_.get(), SLOT(updateTopic()), this);
    t_speed_fg_property_ = std::make_unique<rviz_common::properties::ColorProperty>("color", QColor(255, 255, 0), "", t_speed_enable_property_.get(), SLOT(updateColor()), this);
    c_speed_enable_property_ = std::make_unique<rviz_common::properties::BoolProperty>(c_speed_.name, true, "", this, SLOT(updateValues()));
    c_speed_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>("topic", "", "std_msgs/msg/Float64", "", c_speed_enable_property_.get(), SLOT(updateTopic()), this);
    c_speed_fg_property_ = std::make_unique<rviz_common::properties::ColorProperty>("fg color", QColor(0, 255, 0), "", c_speed_enable_property_.get(), SLOT(updateColor()), this);
    c_speed_bg_property_ = std::make_unique<rviz_common::properties::ColorProperty>("bg color", QColor(32, 32, 32), "", c_speed_enable_property_.get(), SLOT(updateColor()), this);
    c_speed_fontsize_property_ = std::make_unique<rviz_common::properties::IntProperty>("font size", 28, "", c_speed_enable_property_.get(), SLOT(updateValues()), this);
    unit_property_ = std::make_unique<rviz_common::properties::StringProperty>("Unit", "", "", this, SLOT(updateValues()));
    coefficient_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Coefficient", 3.6f, "", unit_property_.get(), SLOT(updateValues()), this);
    unit_precision_property_ = std::make_unique<rviz_common::properties::IntProperty>("precision", 0, "", unit_property_.get(), SLOT(updateValues()), this);
    unit_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("color", QColor(255, 255, 255), "", unit_property_.get(), SLOT(updateColor()), this);
    unit_fontsize_property_ = std::make_unique<rviz_common::properties::IntProperty>("font size", 16, "", unit_property_.get(), SLOT(updateValues()), this);
    bg_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("Background", QColor(0, 0, 0), "", this, SLOT(updateBgColor()));
    scale_main_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("Scale", QColor(255, 255, 255), "", this, SLOT(updateColor()));
    scale_range_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Range", 100.0f, "", scale_main_color_property_.get(), SLOT(updateValues()), this);
    scale_zero_offset_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Zero Offset", 0.0f, "", scale_main_color_property_.get(), SLOT(updateValues()), this);
    scale_main_step_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Main step", 10.0f, "", scale_main_color_property_.get(), SLOT(updateValues()), this);
    scale_sub_step_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Sub step", 5.0f, "", scale_main_color_property_.get(), SLOT(updateValues()), this);
    scale_sub_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("Sub color", QColor(0, 0, 0), "", scale_main_color_property_.get(), SLOT(updateColor()), this);
  }

  SpeedMeterPlugin::~SpeedMeterPlugin()
  {
    if (initialized())
      unsubscribe();
  }

  void SpeedMeterPlugin::onInitialize()
  {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    l_speed_topic_property_->initialize(rviz_ros_node);
    t_speed_topic_property_->initialize(rviz_ros_node);
    c_speed_topic_property_->initialize(rviz_ros_node);

    setupPanel();
  }

  // void SpeedMeterPlugin::updateQosProfile()
  // {
  //   qos_profile_ = rmw_qos_profile_default;
  //   updateQueueSize();

  //   QString policy = reliability_policy_property_->getString();

  //   if (policy == "Best effort")
  //   {
  //     qos_profile_.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  //   }
  //   else if (policy == "Reliable")
  //   {
  //     qos_profile_.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  //   }
  //   else
  //   {
  //     qos_profile_.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  //   }

  //   updateTopic();
  // }

  // void SpeedMeterPlugin::updateQueueSize()
  // {
  //   queue_size_ = queue_size_property_->getInt();
  //   qos_profile_.depth = queue_size_;
  // }

  void SpeedMeterPlugin::updateMessageType(QString l_speed_topic, QString t_speed_topic, QString c_speed_topic)
  {
    QString msg_type;
    switch (msg_type_property_->getOptionInt())
    {
    case 1:
      msg_type = "std_msgs/msg/Float64";
      break;
    default:
      msg_type = "std_msgs/msg/Float32";
      break;
    }

    l_speed_topic_property_->setMessageType(msg_type);
    t_speed_topic_property_->setMessageType(msg_type);
    c_speed_topic_property_->setMessageType(msg_type);
    l_speed_topic_property_->setString(l_speed_topic);
    t_speed_topic_property_->setString(t_speed_topic);
    c_speed_topic_property_->setString(c_speed_topic);
    Q_EMIT l_speed_topic_property_->requestOptions(nullptr);
    Q_EMIT t_speed_topic_property_->requestOptions(nullptr);
    Q_EMIT c_speed_topic_property_->requestOptions(nullptr);

    unsubscribe();
  }

  void SpeedMeterPlugin::load(const rviz_common::Config &config)
  {
    rviz_common::Display::load(config);

    updateMessageType(
        l_speed_topic_property_->getString(),
        t_speed_topic_property_->getString(),
        c_speed_topic_property_->getString());
    updateBgColor();
    updateColor();
    updateTopic();
    updateValues();
  }

  // void SpeedMeterPlugin::save(rviz_common::Config config) const
  // {
  //   rviz_common::Display::save(config);
  // }

  void SpeedMeterPlugin::updateColor()
  {
    l_speed_.fg = l_speed_fg_property_->getColor();
    t_speed_.fg = t_speed_fg_property_->getColor();
    c_speed_.fg = c_speed_fg_property_->getColor();
    c_speed_.bg = c_speed_bg_property_->getColor();
    meter_scale_.main.color = scale_main_color_property_->getColor();
    meter_scale_.sub.color = scale_sub_color_property_->getColor();
    meter_scale_.unit.color = unit_color_property_->getColor();
  }

  void SpeedMeterPlugin::updateBgColor()
  {
    bg_color_ = bg_color_property_->getColor();
    panel_->changeBgColor(&bg_color_);
  }

  void SpeedMeterPlugin::updateTopic()
  {
    unsubscribe();
    reset();
    subscribe();
  }

  void SpeedMeterPlugin::updateValues()
  {
    l_speed_.enable = l_speed_enable_property_->getBool();
    t_speed_.enable = t_speed_enable_property_->getBool();
    c_speed_.enable = c_speed_enable_property_->getBool();
    c_speed_.fontsize = c_speed_fontsize_property_->getInt();
    meter_scale_.range = scale_range_property_->getFloat();
    meter_scale_.zero_offset = scale_zero_offset_property_->getFloat();
    meter_scale_.coefficient = coefficient_property_->getFloat();
    meter_scale_.precision = unit_precision_property_->getInt();
    meter_scale_.main.step = scale_main_step_property_->getFloat();
    meter_scale_.sub.step = scale_sub_step_property_->getFloat();
    meter_scale_.unit.name = unit_property_->getString();
    meter_scale_.unit.fontsize = unit_fontsize_property_->getInt();
  }

  void SpeedMeterPlugin::onEnable()
  {
    subscribe();
  }

  void SpeedMeterPlugin::onDisable()
  {
    unsubscribe();
  }

  void SpeedMeterPlugin::reset()
  {
    l_speed_.received_cnt = 0UL;
    t_speed_.received_cnt = 0UL;
    c_speed_.received_cnt = 0UL;

    setStatus(
        rviz_common::properties::StatusProperty::Ok,
        l_speed_.name,
        QString::number(l_speed_.received_cnt) + " messages received");
    setStatus(
        rviz_common::properties::StatusProperty::Ok,
        t_speed_.name,
        QString::number(t_speed_.received_cnt) + " messages received");
    setStatus(
        rviz_common::properties::StatusProperty::Ok,
        c_speed_.name,
        QString::number(c_speed_.received_cnt) + " messages received");
    setStatus(
        rviz_common::properties::StatusProperty::Ok,
        "Message",
        "Ok");
  }

  void SpeedMeterPlugin::subscribe()
  {
    if (!isEnabled())
      return;

    int msg_type = msg_type_property_->getOptionInt();

    std::string l_speed_topic = l_speed_topic_property_->getStdString();
    if (!l_speed_topic.empty())
    {
      switch (msg_type)
      {
      case 1:
        // msg_type = "std_msgs/msg/Float64";
        l_speed_sub_ = create_speed_sub<std_msgs::msg::Float64>(l_speed_, l_speed_topic);
        break;
      default:
        // msg_type = "std_msgs/msg/Float32";
        l_speed_sub_ = create_speed_sub<std_msgs::msg::Float32>(l_speed_, l_speed_topic);
        break;
      }
    }

    std::string t_speed_topic = t_speed_topic_property_->getStdString();
    if (!t_speed_topic.empty())
    {
      switch (msg_type)
      {
      case 1:
        // msg_type = "std_msgs/msg/Float64";
        t_speed_sub_ = create_speed_sub<std_msgs::msg::Float64>(t_speed_, t_speed_topic);
        break;
      default:
        // msg_type = "std_msgs/msg/Float32";
        t_speed_sub_ = create_speed_sub<std_msgs::msg::Float32>(t_speed_, t_speed_topic);
        break;
      }
    }

    std::string c_speed_topic = c_speed_topic_property_->getStdString();
    if (!c_speed_topic.empty())
    {
      switch (msg_type)
      {
      case 1:
        // msg_type = "std_msgs/msg/Float64";
        c_speed_sub_ = create_speed_sub<std_msgs::msg::Float64>(c_speed_, c_speed_topic);
        break;
      default:
        // msg_type = "std_msgs/msg/Float32";
        c_speed_sub_ = create_speed_sub<std_msgs::msg::Float32>(c_speed_, c_speed_topic);
        break;
      }
    }
  }

  template <typename msgT>
  typename rclcpp::Subscription<msgT>::SharedPtr SpeedMeterPlugin::create_speed_sub(speed &speed_struct, std::string &topic_name)
  {
    auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();
    return rviz_ros_node_
        ->get_raw_node()
        ->create_subscription<msgT>(
            topic_name,
            rclcpp::SensorDataQoS(),
            [this, &speed_struct](const std::shared_ptr<msgT> msg)
            {
              std::lock_guard<std::mutex> lock(speed_struct.mutex);
              speed_struct.value = msg->data;
              speed_struct.received_cnt++;
              setStatus(
                  rviz_common::properties::StatusProperty::Ok,
                  speed_struct.name,
                  QString::number(speed_struct.received_cnt) + " messages received");
            });
  }

  void SpeedMeterPlugin::unsubscribe()
  {
    l_speed_sub_.reset();
    t_speed_sub_.reset();
    c_speed_sub_.reset();
  }

  void SpeedMeterPlugin::update(float dt, float ros_dt)
  {
    (void)dt;
    (void)ros_dt;

    if (panel_ != nullptr)
      panel_->update();
  }

  void SpeedMeterPlugin::setupPanel()
  {
    panel_ = std::make_unique<SpeedMeterWidget>(
        &l_speed_,
        &t_speed_,
        &c_speed_,
        &meter_scale_,
        &bg_color_);
    panel_->setMinimumHeight(128);
    setAssociatedWidget(panel_.get());
  }

} // namespace speed_meter_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(speed_meter_plugin::SpeedMeterPlugin, rviz_common::Display)
