
#include "QNode.h"

QNode::QNode(int argc, char ** argv, QObject *parent)
{
    rclcpp::init(argc, argv);
    m_node = rclcpp::Node::make_shared("ros2_qt");
    this->start();
}

void QNode::run()
{
    std_msgs::msg::Int32 pub_msg;
    pub_msg.data=0;
    rclcpp::WallRate loop_rate(100);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(m_node);

        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

rclcpp::Node::SharedPtr QNode::rosNode()
{
    return m_node;
}
