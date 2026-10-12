#include <exception>
#include <memory>

#include "image_transport/publisher_plugin.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CompressedRepublisher : public rclcpp::Node
{
public:
  CompressedRepublisher()
  : Node("camera_image_transport"),
    loader_("image_transport", "image_transport::PublisherPlugin")
  {
    publisher_ = loader_.createSharedInstance("image_transport/compressed_pub");
    const auto topic = get_node_topics_interface()->resolve_topic_name("image", true);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    publisher_->advertise(this, topic, qos.get_rmw_qos_profile());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
        if (publisher_->getNumSubscribers() > 0) {
          publisher_->publish(*image);
        }
      });
  }

private:
  pluginlib::ClassLoader<image_transport::PublisherPlugin> loader_;
  std::shared_ptr<image_transport::PublisherPlugin> publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<CompressedRepublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_image_transport"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
