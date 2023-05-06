#ifndef SERIAL_STUDIO_QNODE_H
#define SERIAL_STUDIO_QNODE_H

#include "QObject"
#include "QThread"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
class QNode:public QThread
{
    Q_OBJECT
public:
    QNode(int argc = 0, char ** argv = nullptr, QObject *parent = nullptr);

    rclcpp::Node::SharedPtr rosNode();
protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> m_node;

Q_SIGNALS:
    void emitTopicData(QString);
};

#endif // SERIAL_STUDIO_QNODE_H
