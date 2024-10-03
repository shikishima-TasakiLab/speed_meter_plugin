#include "speed_meter_plugin/speed_meter_plugin.hpp"

namespace speed_meter_plugin
{

  SpeedMeterPlugin::SpeedMeterPlugin()
  {
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
    l_speed_float_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Speed Limit", 0.0f, "", this);
    l_speed_float_property_->setReadOnly(true);
    l_speed_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>("topic", "", "std_msgs/msg/Float64", "", l_speed_float_property_.get(), SLOT(updateTopic()), this);
    l_speed_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("color", QColor(0, 0, 0), "", l_speed_float_property_.get());
    t_speed_float_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Target Speed", 0.0f, "", this);
    t_speed_float_property_->setReadOnly(true);
    t_speed_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>("topic", "", "std_msgs/msg/Float64", "", t_speed_float_property_.get(), SLOT(updateTopic()), this);
    t_speed_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("color", QColor(0, 0, 0), "", t_speed_float_property_.get());
    c_speed_float_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Current Speed", 0.0f, "", this);
    c_speed_float_property_->setReadOnly(true);
    c_speed_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>("topic", "", "std_msgs/msg/Float64", "", c_speed_float_property_.get(), SLOT(updateTopic()), this);
    c_speed_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("color", QColor(0, 0, 0), "", c_speed_float_property_.get());
    unit_property_ = std::make_unique<rviz_common::properties::StringProperty>("Unit", "km/h", "", this);
    coefficient_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Coefficient", 3.6f, "", unit_property_.get());
    bg_property_ = std::make_unique<rviz_common::properties::Property>("Background", "", "", this);
    bg_property_->setReadOnly(true);
    bg_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("color", QColor(0, 0, 0), "", bg_property_.get());
    scale_property_ = std::make_unique<rviz_common::properties::Property>("Scale", "", "", this);
    scale_property_->setReadOnly(true);
    scale_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("color", QColor(0, 0, 0), "", scale_property_.get());
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

  void SpeedMeterPlugin::updateMessageType()
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
    l_speed_topic_property_->setString("");
    t_speed_topic_property_->setString("");
    c_speed_topic_property_->setString("");
    Q_EMIT l_speed_topic_property_->requestOptions(nullptr);
    Q_EMIT t_speed_topic_property_->requestOptions(nullptr);
    Q_EMIT c_speed_topic_property_->requestOptions(nullptr);

    unsubscribe();
  }

  void SpeedMeterPlugin::updateColor()
  {
  }

  void SpeedMeterPlugin::updateTopic()
  {
    unsubscribe();
    reset();
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
    messages_received_ = 0;
    setStatus(rviz_common::properties::StatusProperty::Ok, "Speed Limit", "0 messages received");
    setStatus(rviz_common::properties::StatusProperty::Ok, "Target Speed", "0 messages received");
    setStatus(rviz_common::properties::StatusProperty::Ok, "Current Speed", "0 messages received");
    setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Ok");
  }

  void SpeedMeterPlugin::subscribe()
  {
    if (!isEnabled())
      return;

    int msg_type = msg_type_property_->getOptionInt();
    auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();

    std::string l_speed_topic = l_speed_topic_property_->getStdString();
    if (!l_speed_topic.empty())
    {
      switch (msg_type)
      {
      case 1:
        // msg_type = "std_msgs/msg/Float64";
        l_speed_float64_sub_ = rviz_ros_node_
                                   ->get_raw_node()
                                   ->create_subscription<std_msgs::msg::Float64>(
                                       l_speed_topic,
                                       rclcpp::SensorDataQoS(),
                                       [this](std_msgs::msg::Float64::ConstSharedPtr msg)
                                       {
                                         std::lock_guard<std::mutex> lock(l_speed_mutex_);
                                         l_speed_value_ = msg->data;
                                       });
        break;
      default:
        // msg_type = "std_msgs/msg/Float32";
        l_speed_float32_sub_ = rviz_ros_node_
                                   ->get_raw_node()
                                   ->create_subscription<std_msgs::msg::Float32>(
                                       l_speed_topic,
                                       rclcpp::SensorDataQoS(),
                                       [this](std_msgs::msg::Float32::ConstSharedPtr msg)
                                       {
                                         std::lock_guard<std::mutex> lock(l_speed_mutex_);
                                         l_speed_value_ = msg->data;
                                       });
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
        t_speed_float64_sub_ = rviz_ros_node_
                                   ->get_raw_node()
                                   ->create_subscription<std_msgs::msg::Float64>(
                                       t_speed_topic,
                                       rclcpp::SensorDataQoS(),
                                       [this](std_msgs::msg::Float64::ConstSharedPtr msg)
                                       {
                                         std::lock_guard<std::mutex> lock(t_speed_mutex_);
                                         t_speed_value_ = msg->data;
                                       });
        break;
      default:
        // msg_type = "std_msgs/msg/Float32";
        t_speed_float32_sub_ = rviz_ros_node_
                                   ->get_raw_node()
                                   ->create_subscription<std_msgs::msg::Float32>(
                                       t_speed_topic,
                                       rclcpp::SensorDataQoS(),
                                       [this](std_msgs::msg::Float32::ConstSharedPtr msg)
                                       {
                                         std::lock_guard<std::mutex> lock(t_speed_mutex_);
                                         t_speed_value_ = msg->data;
                                       });
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
        c_speed_float64_sub_ = rviz_ros_node_
                                   ->get_raw_node()
                                   ->create_subscription<std_msgs::msg::Float64>(
                                       c_speed_topic,
                                       rclcpp::SensorDataQoS(),
                                       [this](std_msgs::msg::Float64::ConstSharedPtr msg)
                                       {
                                         std::lock_guard<std::mutex> lock(c_speed_mutex_);
                                         c_speed_value_ = msg->data;
                                       });
        break;
      default:
        // msg_type = "std_msgs/msg/Float32";
        c_speed_float32_sub_ = rviz_ros_node_
                                   ->get_raw_node()
                                   ->create_subscription<std_msgs::msg::Float32>(
                                       c_speed_topic,
                                       rclcpp::SensorDataQoS(),
                                       [this](std_msgs::msg::Float32::ConstSharedPtr msg)
                                       {
                                         std::lock_guard<std::mutex> lock(c_speed_mutex_);
                                         c_speed_value_ = msg->data;
                                       });
        break;
      }
    }
  }

  void SpeedMeterPlugin::unsubscribe()
  {
    l_speed_float32_sub_.reset();
    l_speed_float64_sub_.reset();
    t_speed_float32_sub_.reset();
    t_speed_float64_sub_.reset();
    c_speed_float32_sub_.reset();
    c_speed_float64_sub_.reset();
  }

  void SpeedMeterPlugin::update(float dt, float ros_dt)
  {
    (void)dt;
    (void)ros_dt;
  }

} // namespace speed_meter_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(speed_meter_plugin::SpeedMeterPlugin, rviz_common::Display)
