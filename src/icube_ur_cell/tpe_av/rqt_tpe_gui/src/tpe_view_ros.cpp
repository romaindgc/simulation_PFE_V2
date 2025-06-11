#include "rqt_tpe_gui/tpe_view.h"
#include <QStringList>
#include <QDebug>
#include <QMessageBox>
#include <iostream>





/*#include <sensor_msgs/msg/region_of_interest.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>*/

using std::placeholders::_1;

namespace rqt_tpe_gui
{

const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "ur5_plate_support";
const std::string BASE_FRAME_ID = "ur5_base_link";    


void TPEView::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/msg/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/msg/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(node_);
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    RCLCPP_INFO(node_->get_logger(), "updateTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  //qSort(topics);
  std::sort(topics.begin(),topics.end());
  ui_.topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  //selectTopic(selected);
}

QList<QString> TPEView::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
  QSet<QString> message_sub_types;
  return getTopics(message_types, message_sub_types, transports).values();
}

QSet<QString> TPEView::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
{
 
  auto ros2_topics = node_->get_topic_names_and_types();
  QSet<QString> all_topics;
  for (auto topic_and_types:ros2_topics) 
  {
    auto & topic_name = topic_and_types.first;
    all_topics.insert(topic_name.c_str());
  }

  QSet<QString> topics;
  for (auto topic_and_types:ros2_topics) 
  {
    auto & topic_name = topic_and_types.first;
    auto & topic_type = topic_and_types.second[0];

    if (message_types.contains(topic_type.c_str()))
    {
      QString topic = topic_name.c_str();

      // add raw topic
      topics.insert(topic);
      //qDebug("ImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());
      RCLCPP_INFO(node_->get_logger(), "MyPlugin::getTopics() raw topic '%s'", topic.toStdString().c_str());

      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
          //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
          RCLCPP_INFO(node_->get_logger(), "MyPlugin::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());

        }
      }
    }
    if (message_sub_types.contains(topic_type.c_str()))
    {
      QString topic = topic_name.c_str();
      int index = topic.lastIndexOf("/");
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.insert(topic);
        //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
        RCLCPP_INFO(node_->get_logger(), "MyPlugin::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
      }
    }
  }
  return topics;
}

void TPEView::selectTopic(const QString& topic)
{
  int index = ui_.topics_combo_box->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(topic));
    index = ui_.topics_combo_box->findText(topic);
  }
  ui_.topics_combo_box->setCurrentIndex(index);
}

void TPEView::onTopicChanged(int index)
{
  //conversion_mat_.release();

  subscriber_.shutdown();

  // reset image on topic change
  //ui_.image_frame->setImage(QImage());

  QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    //image_transport::ImageTransport it(node_);
    //image_transport::TransportHints hints(node_, transport.toStdString());

    image_transport::TransportHints hints(node_.get(), transport.toStdString());
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  
    try {
      //subscriber_ = it.subscribe(topic.toStdString(), 1, std::bind(&MyPlugin::callbackImage, this, _1) , hints);
      subscriber_  =image_transport::create_subscription(
        node_.get(), topic.toStdString(), std::bind(
        &TPEView::callbackImage, this, std::placeholders::_1), hints.getTransport(), custom_qos_profile);
      /*subscriber_  =image_transport::create_subscription(
        node_.get(), topic.toStdString(), 
        &MyPlugin::callbackImage, hints.getTransport());*/

      //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(mw_, tr("Loading image transport plugin failed"), e.what());
    }
  }

  //onMousePublish(ui_.publish_click_location_check_box->isChecked());
}

void TPEView::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {

    //RCLCPP_INFO(node_->get_logger(), "MyPlugin::callbackImages(): new image available");
    //cv_bridge::CvImagePtr cv_ptr;

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    conversion_mat_ = cv_ptr->image;

    //cv::cvtColor(cv_ptr->image,cv_ptr->image, cv::COLOR_BGR2RGB);
    cv::cvtColor(cv_ptr->image,img_gray, cv::COLOR_BGR2GRAY);

    if (tpe.im.width ==0 && tpe.im.height==0) {

        tpe.im.buf = img_gray.data;
        tpe.im.width = img_gray.cols;
        tpe.im.height = img_gray.rows;
        if ( ! (tpe.im.coord = (unsigned char **)malloc(sizeof(unsigned char *) * tpe.im.height ) ) )	{
            return;

        }
        else	{

            for (int i=0; i< tpe.im.height; i++)
                tpe.im.coord[i] = &tpe.im.buf[i*tpe.im.width];

        }

    }

    if ( (tpe.im.width == img_gray.cols) &&  (tpe.im.height == img_gray.rows))
        emit newFrame();



}

void TPEView::init_robot__com( void ) {

  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  frame_to_publish_ = EEF_FRAME_ID;

  camera_info_subscripber_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/color/camera_info", 10, std::bind(&TPEView::callbackCameraInfo, this, _1));


}

void TPEView::callbackCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {

    tpe_control_.camera_param.alpha_u = msg->k[0];
    tpe_control_.camera_param.alpha_v = msg->k[4];
    tpe_control_.camera_param.u0 = msg->k[2];
    tpe_control_.camera_param.v0 = msg->k[5];

}

int TPEView::sendVelocity(double *control, bool norm) {
  
    (void)norm;

    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();


    twist_msg->header.stamp = node_->now();
    twist_msg->header.frame_id = frame_to_publish_;

    twist_msg->twist.linear.x = control[0];
    twist_msg->twist.linear.y = control[1];
    twist_msg->twist.linear.z = control[2];

    twist_msg->twist.angular.z = control[3];

    twist_pub_->publish(std::move(twist_msg));


    /*auto commands_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();   
    for (unsigned int i = 0; i < 4 ; i++ ) {
        commands_msg->data.push_back(control[i]);

    }
    commands_publisher_->publish(std::move(commands_msg));*/

  return 0;
}

int TPEView::VitSensor2VitRob(double *vitsensor, double *vitrob) {

    double vitbase[4];

  for (int i = 0; i < 4; i++)
    vitrob[i] = -vitsensor[i];

  vitrob[1] = - vitrob[1];
  

   
  return 0;
}



}