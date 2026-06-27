#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MapOdomPublisher : public rclcpp::Node {
  public:
    MapOdomPublisher() : Node("map_odom_publisher") {
        declare_parameter<std::string>("map_frame_id", "map");
        declare_parameter<std::string>("odom_frame_id", "odom");
        declare_parameter<std::string>("base_frame_id", "base_link");
        declare_parameter<double>("publish_period_s", 0.01);

        map_frame_id = get_parameter("map_frame_id").as_string();
        odom_frame_id = get_parameter("odom_frame_id").as_string();
        base_frame_id = get_parameter("base_frame_id").as_string();
        const double period = get_parameter("publish_period_s").as_double();

        tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_buffer->setUsingDedicatedThread(true);
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, this, false);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

        const auto period_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period));
        timer = create_wall_timer(period_ns, [this]() { publish(); });
    }

  private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
        if (odom->header.frame_id != map_frame_id) {
            RCLCPP_ERROR(get_logger(), "Dropping odometry: frame_id '%s' != map_frame_id '%s'",
                         odom->header.frame_id.c_str(), map_frame_id.c_str());
            return;
        }

        if (odom->child_frame_id != base_frame_id) {
            RCLCPP_ERROR(get_logger(), "Dropping odometry: child_frame_id '%s' != base_frame_id '%s'",
                         odom->child_frame_id.c_str(), base_frame_id.c_str());
            return;
        }

        this->odom = odom;
    }

    void publish() {
        if (!odom) {
            return;
        }

        geometry_msgs::msg::TransformStamped tf_odom_base;
        try {
            // We use timestamp 0 here to look up the latest transform
            tf_odom_base = tf_buffer->lookupTransform(odom_frame_id, base_frame_id, rclcpp::Time(0));
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN(get_logger(), "TF %s->%s unavailable, skipping: %s", odom_frame_id.c_str(),
                        base_frame_id.c_str(), e.what());
            return;
        }

        const auto& pos = odom->pose.pose.position;
        const auto& rot = odom->pose.pose.orientation;
        const tf2::Transform T_map_base(tf2::Quaternion(rot.x, rot.y, rot.z, rot.w), tf2::Vector3(pos.x, pos.y, pos.z));

        const auto& trans = tf_odom_base.transform.translation;
        const auto& quat = tf_odom_base.transform.rotation;
        const tf2::Transform T_odom_base(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w),
                                         tf2::Vector3(trans.x, trans.y, trans.z));

        const tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

        geometry_msgs::msg::TransformStamped out;
        out.header.stamp = odom->header.stamp;
        out.header.frame_id = map_frame_id;
        out.child_frame_id = odom_frame_id;
        out.transform = tf2::toMsg(T_map_odom);

        tf_broadcaster->sendTransform(out);
    }

    std::string map_frame_id;
    std::string odom_frame_id;
    std::string base_frame_id;

    nav_msgs::msg::Odometry::SharedPtr odom;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
