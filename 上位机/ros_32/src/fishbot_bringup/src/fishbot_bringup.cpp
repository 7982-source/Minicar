#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

/**
 * @brief 订阅里程计信息并发布TF变换的节点类
 * 
 * 该类继承自rclcpp::Node，用于订阅odom话题的里程计信息，
 * 并将接收到的位姿信息转换为TF坐标变换进行广播。
 */
class TopicSubscribe01 : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   * 
   * @param name 节点名称
   */
  TopicSubscribe01(std::string name) : Node(name)
  {
    // 创建一个订阅者，订阅"odom"话题的nav_msgs::msg::Odometry类型消息
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&TopicSubscribe01::odom_callback, this, std::placeholders::_1));

    // 创建一个tf2_ros::TransformBroadcaster用于广播坐标变换
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odom_msg_;

  /**
   * @brief 里程计消息回调函数
   * 
   * 处理接收到的odom消息，提取位姿信息并保存
   * 
   * @param msg 接收到的里程计消息指针
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "接收到里程计信息->底盘坐标系 tf :(%f,%f)", 
                msg->pose.pose.position.x, msg->pose.pose.position.y);

    // 更新odom_msg_的姿态信息
    odom_msg_.pose.pose.position.x = msg->pose.pose.position.x;
    odom_msg_.pose.pose.position.y = msg->pose.pose.position.y;
    odom_msg_.pose.pose.position.z = msg->pose.pose.position.z;

    odom_msg_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom_msg_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom_msg_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom_msg_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  };

public:
  /**
   * @brief 发布TF坐标变换
   * 
   * 将保存的里程计位姿信息作为TF变换从odom坐标系广播到base_footprint坐标系
   */
  void publish_tf()
  {
    geometry_msgs::msg::TransformStamped transform;
    double seconds = this->now().seconds();
    transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";

    transform.transform.translation.x = odom_msg_.pose.pose.position.x;
    transform.transform.translation.y = odom_msg_.pose.pose.position.y;
    transform.transform.translation.z = odom_msg_.pose.pose.position.z;
    transform.transform.rotation.x = odom_msg_.pose.pose.orientation.x;
    transform.transform.rotation.y = odom_msg_.pose.pose.orientation.y;
    transform.transform.rotation.z = odom_msg_.pose.pose.orientation.z;
    transform.transform.rotation.w = odom_msg_.pose.pose.orientation.w;

    // 广播坐标变换信息
    tf_broadcaster_->sendTransform(transform);
  }
};

/**
 * @brief 主函数
 * 
 * 初始化ROS 2节点，创建TopicSubscribe01实例，并以固定频率运行节点，
 * 处理回调和发布TF变换。
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 */
int main(int argc, char **argv)
{
  // 初始化ROS节点
  rclcpp::init(argc, argv);

  // 创建一个TopicSubscribe01节点
  auto node = std::make_shared<TopicSubscribe01>("fishbot_bringup");

  // 设置循环频率
  rclcpp::WallRate loop_rate(20.0);
  while (rclcpp::ok())
  {
    // 处理回调函数
    rclcpp::spin_some(node);

    // 发布坐标变换信息
    node->publish_tf();

    // 控制循环频率
    loop_rate.sleep();
  }

  // 关闭ROS节点
  rclcpp::shutdown();
  return 0;
}