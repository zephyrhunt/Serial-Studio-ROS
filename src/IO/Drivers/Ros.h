//
// Created by nichijou on 23-5-6.
//

#ifndef SERIAL_STUDIO_ROS_H
#define SERIAL_STUDIO_ROS_H

#include "DataTypes.h"
#include "std_msgs/msg/string.hpp"
#include "QObject"
#include <QString>
#include <QSettings>
#include <QByteArray>
#include "IO/HAL_Driver.h"
#include "QNode.h"

namespace IO
{
namespace Drivers
{
/**
 * @brief The ROS class
 * Serial Studio driver class to interact with ros node.
 */
class Ros : public HAL_Driver
{
    Q_OBJECT

//    Q_PROPERTY(QString nodeName
//                   READ nodeName
//                   NOTIFY nodeChanged)
    Q_PROPERTY(StringList topicList
                   READ topicList
                   NOTIFY availableTopicChanged)
    Q_PROPERTY(StringList serviceList
                   READ serviceList
                   NOTIFY availableServiceChanged)

    Q_PROPERTY(quint8 topicIndex
                   READ topicIndex
                   WRITE setTopicIndex
                   NOTIFY topicIndexChanged)

    Q_PROPERTY(quint8 serviceIndex
                   READ serviceIndex
                   WRITE setServiceIndex
                   NOTIFY serviceIndexChanged)

    Q_PROPERTY(bool pubEnable
                   READ pubEnable
                   WRITE setPubEnable
                   NOTIFY pubEnableChanged)

    Q_PROPERTY(bool subEnable
                   READ subEnable
                   WRITE setSubEnable
                   NOTIFY subEnableChanged)
Q_SIGNALS:
    void availableTopicChanged();
    void availableServiceChanged();
    void topicIndexChanged();
    void serviceIndexChanged();
    void pubEnableChanged();
    void subEnableChanged();

private:
    explicit Ros();
    ~Ros();

public:
    static Ros &instance();

    //
    // HAL functions
    //
    void close() override;
    bool isOpen() const override;
    bool isReadable() const override;
    bool isWritable() const override;
    bool configurationOk() const override;
    quint64 write(const QByteArray &data) override;
    bool open(const QIODevice::OpenMode mode) override;

    QString topic() const;
    quint8     topicIndex() const;
    quint8     serviceIndex() const;
    StringList topicList() const;
    StringList serviceList() const;
    bool        subEnable() const;
    bool        pubEnable() const;

    QString receiveData()const;

    void setTopicIndex(quint8 index);
    void setServiceIndex(quint8 index);
    void setSubEnable(bool is_enable);
    void setPubEnable(bool is_enable);
private Q_SLOTS:
    void onReadyRead();
//    void readSettings();
//    void writeSettings();
    void refreshRosInfo();
private:
    void  subCallBack(const std_msgs::msg::String &string);

    QString     m_reveiveData;
    QString     m_nodeName;
    QString     m_topic;
    quint8      m_topicIndex;
    quint8      m_serviceIndex;
    StringList  m_topicList;
    StringList  m_serviceList;
    bool        m_subEnable;
    bool        m_pubEnable;
    bool        m_nodeConnected;

    std_msgs::msg::Int32 m_pub_data;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;
    rclcpp::Node::SharedPtr m_node;

};
}
}

#endif // SERIAL_STUDIO_ROS_H
