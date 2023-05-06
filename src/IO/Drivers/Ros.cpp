//
// Created by nichijou on 23-5-6.
//

#include "Ros.h"
#include "Misc/TimerEvents.h"

IO::Drivers::Ros::Ros()
    :m_nodeName("NULL"),
    m_nodeConnected(false)
{
    auto qNode = new QNode();
    m_node = qNode->rosNode();
    m_publisher = m_node->create_publisher<std_msgs::msg::String>("/serial_studio/msg_pub", 10);
    m_subscription = m_node->create_subscription<std_msgs::msg::String>(
        "/serial_studio/msg_sub",
        10,
        std::bind(&IO::Drivers::Ros::subCallBack, this, std::placeholders::_1)
        );
    m_pub_data.data = 0;
    m_topicIndex = 0;

    // refresh it 10HZ
    connect(&Misc::TimerEvents::instance(), &Misc::TimerEvents::timeout10Hz,
            this, &IO::Drivers::Ros::refreshRosInfo);
}

/**
 * Destructor function, closes the ros node
 */
IO::Drivers::Ros::~Ros()
{
    rclcpp::shutdown();
}

/**
 * Returns the only instance of the class
 */
IO::Drivers::Ros &IO::Drivers::Ros::instance()
{
    static Ros singleton;
    return singleton;
}

//----------------------------------------------------------------------------------------
// HAL-driver implementation
//----------------------------------------------------------------------------------------

void IO::Drivers::Ros::close()
{
    m_nodeConnected = false;
}

bool IO::Drivers::Ros::isOpen() const
{
    return m_nodeConnected;
}

bool IO::Drivers::Ros::isReadable() const
{
    // enable
    return isOpen();
}

bool IO::Drivers::Ros::isWritable() const
{
    return isOpen(); //after ok
}

bool IO::Drivers::Ros::configurationOk() const
{
   return true;
}

quint64 IO::Drivers::Ros::write(const QByteArray &data)
{
    (void)data;
    if (isWritable()){
        std_msgs::msg::String string;
        string.data = data.toStdString();
        m_publisher->publish(string);
        return data.size();
    }
    return -1;
}

bool IO::Drivers::Ros::open(const QIODevice::OpenMode mode)
{
    m_nodeConnected = true;
    return true;
}

StringList IO::Drivers::Ros::topicList() const
{
    return m_topicList;
}
StringList IO::Drivers::Ros::serviceList() const
{
    return m_serviceList;
}

void IO::Drivers::Ros::refreshRosInfo()
{
    StringList topicListNow;
    StringList serviceListNow;
    auto topics = m_node->get_topic_names_and_types();
    topicListNow.append(tr("Select topic"));
    for(const auto &topic_:topics){
        topicListNow.append(QString::fromStdString(topic_.first));
    }

    if (topicList() != topicListNow)
    {
        m_topicList = topicListNow;
        // update current topic list Index
        // action registered topics will also be found
        for(const auto &topic_:topics){
            qDebug() << topic_.first.c_str()<<"type"<<topic_.second.at(0).c_str();
        }
        for (int i = 0; i < topicListNow.count(); ++i) //more one
        {
            auto info = topicListNow.at(i);
            if (info == topic())
            {
                m_topicIndex = i;
                break ;
            }
        }

        qDebug()<<"current:"<<m_topicIndex<<"size:"<<m_topicList.size()-1<<"is enable:"<<pubEnable();
        Q_EMIT availableTopicChanged();
    }

    auto services = m_node->get_service_names_and_types();
    serviceListNow.append(tr("Select service"));
    for(const auto &service_:services){
        serviceListNow.append(QString::fromStdString(service_.first));
    }

    if (serviceList() != serviceListNow)
    {
        m_serviceList = serviceListNow;
        // update current topic list Index
        // action registered topics will also be found
        for(const auto &topic_:topics){
            qDebug() << topic_.first.c_str()<<"type"<<topic_.second.at(0).c_str();
        }
        for (int i = 0; i < serviceListNow.count(); ++i) //more one
        {
            auto info = serviceListNow.at(i);
            if (info == topic())
            {
                m_serviceIndex = i;
                break ;
            }
        }

        qDebug()<<"current service:"<<m_serviceIndex<<"size:"<<m_serviceList.size()-1;
        Q_EMIT availableServiceChanged();
    }
}

QString IO::Drivers::Ros::topic() const
{
    return m_topic;
}


quint8 IO::Drivers::Ros::topicIndex() const
{
    return m_topicIndex;
}

void IO::Drivers::Ros::setTopicIndex(quint8 index)
{
    if (index < 0) index = 0;
    m_topicIndex = index;
    Q_EMIT topicIndexChanged();
}

void IO::Drivers::Ros::setServiceIndex(quint8 index)
{
    if (index < 0) index = 0;
    m_serviceIndex = index;
    Q_EMIT serviceIndexChanged();

}
void IO::Drivers::Ros::subCallBack(const std_msgs::msg::String &string)
{
    m_reveiveData = QString::fromStdString(string.data);
    RCLCPP_INFO(this->m_node->get_logger(), "receive:%s", m_reveiveData.toStdString().c_str());
    Q_EMIT dataReceived(m_reveiveData.toUtf8());
}

QString IO::Drivers::Ros::receiveData() const
{
    return m_reveiveData;
}
quint8 IO::Drivers::Ros::serviceIndex() const
{
    return m_serviceIndex;
}

bool IO::Drivers::Ros::subEnable() const
{
    return m_subEnable;
}
bool IO::Drivers::Ros::pubEnable() const
{
    return m_pubEnable;
}
void IO::Drivers::Ros::setSubEnable(bool is_enable)
{
    m_subEnable = is_enable;
}

void IO::Drivers::Ros::setPubEnable(bool is_enable)
{
    m_pubEnable = is_enable;
}

void IO::Drivers::Ros::onReadyRead()
{
    if (isOpen())
        Q_EMIT dataReceived(m_reveiveData.toUtf8());
}