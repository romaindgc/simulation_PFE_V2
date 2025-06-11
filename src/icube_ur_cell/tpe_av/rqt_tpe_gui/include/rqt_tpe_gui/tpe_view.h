/*
  Copyright 2016 Lucas Walter
*/
#ifndef RQT_TPE_VIEW_H
#define RQT_TPE_VIEW_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_mainwindow.h>
#include <QWidget>
#include <QMainWindow>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QTimer>
#include <QRect>
#include <QPoint>

//#include "tpe.h"



#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include "GraphicsScene.h"


#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tpe_base/tpe_base.h>
#include <pluginlib/class_loader.hpp>

#include "std_srvs/srv/set_bool.hpp"
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

/*
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tpe_robot_interfaces/msg/multi_region_of_interest.hpp"

#include "tpe_robot_interfaces/srv/tpe_trigger.hpp"*/

#include <opencv2/core/core.hpp>

//#include "rcl_interfaces/srv/set_parameters.hpp"
//#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "rqt_tpe_gui/structav.h"


namespace rqt_tpe_gui
{

class TPEView
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  TPEView();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

  struct tpe_t tpe;
  struct control_t m_control;


public slots :

    void NewPointClicked(QPoint p);
    void setMouseCursorPos(QPoint p);
    void setPointSelected();

protected slots :
  virtual void updateTopicList();
  void doUpdate();


protected slots:

  virtual void onTopicChanged(int index);


private slots : 
    void VelocitySliderPressed();
    void VelocitySliderRealesed();
    void VelocitySliderTimerIdle();
    void VelocitySliderValueChanged(int val);

    void comboxOnValueChange (int value);

    void referenceTargetSelection(int state );
    void referenceTargetTracking(int state );
    void currentTargetSelection(int state );
    void currentTargetTracking(int state );
    void referenceTargetResetClicked( );
    void currentTargetResetClicked( );

    void StopButtonClicked( void  ) ;
/*
    void onCloseLoopChanged( bool checked );
*/
    void dSpinBoxGTValueChanged (double );
    void dSpinBoxGRValueChanged (double );
/*    void dSpinBoxDepthValueChanged (double );

    void GotoButtonClicked(  );
*/






signals:
  void newFrame( );

protected :

  virtual void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) ;
  virtual void selectTopic(const QString& topic);


private:
  //Ui::MyPluginWidget ui_;
  Ui::MainWindow ui_;
  
  //QWidget* widget_;
  QMainWindow *mw_;

  GraphicsScene *scene;
  QGraphicsPixmapItem *pixmapItem;
  int m_width,m_height;

  //bool drawBox;
  //QRect *box;

  QList <STRUCT_TARGET_AV> listTarget;
  QList <STRUCT_POINT_AV> listPointCur;

  QList <QRect*> listBox;
  QList <QRect*> listBoxRef;
  QList <QRect*> listBoxCur;

  QList <QGraphicsRectItem *> listrecItem;
  QList <QGraphicsRectItem *> listrecItemRef;
  QList <QGraphicsRectItem *> listrecItemCur;

  QList <bool> listDrawBox;

  QPoint startPoint;
  QPoint mouseCursorPos;



  //void updateTopicList();
  QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);
  QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);

//  virtual void callbackRoiUpdate(const tpe_robot_interfaces::msg::MultiRegionOfInterest::SharedPtr msg) ;

//  void trackingTarget(QList <QGraphicsRectItem *>listrecItem_, QList <QRect*>listBox_);
  
  image_transport::Subscriber subscriber_;
  cv_bridge::CvImagePtr cv_ptr;

  cv::Mat img_gray;
  cv::Mat conversion_mat_;


  pluginlib::ClassLoader<tpe_base::TpeBase> tpe_base_loader_{ "tpe_base", "tpe_base::TpeBase"};
  std::string tpe_av_plugin;
  std::shared_ptr<tpe_base::TpeBase> p_tpe_av;


  void trackingTarget(QList <QGraphicsRectItem *>listrecItem_, int idx);
  void initBox(QList <QGraphicsRectItem *>listrecItem_, int idx);
  void resetBox(QList <QGraphicsRectItem *>listrecItem_, int idx);


  void init_robot__com( void );

  QTimer *VelocitySliderTimer;
  bool StateMouseButtonSlider;
  int in_event;

  int VitSensor2VitRob(double *vitcam, double *vitrob);
  int sendVelocity(double *control, bool norm =true);

struct tpe_control_vision_t tpe_control_;

 
  //rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr camera_velocities_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::string frame_to_publish_;
  double control[4];

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscripber_;
  virtual void callbackCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) ;
  //rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr robot_joint_trajectory_publisher_;
  //rclcpp::Publisher<tpe_robot_interfaces::msg::MultiRegionOfInterest>::SharedPtr roi_des_publisher_;
  //rclcpp::Subscription<tpe_robot_interfaces::msg::MultiRegionOfInterest>::SharedPtr roi_list_subscripber_; 

  //rclcpp::Client<tpe_robot_interfaces::srv::TpeTrigger>::SharedPtr client_tracking;
  //void service_response_callback(rclcpp::Client<tpe_robot_interfaces::srv::TpeTrigger>::SharedFuture future);
  //tpe_robot_interfaces::srv::TpeTrigger::Response::SharedPtr service_response_; 

  //rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_control;
  //void service_control_response_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);
  //std_srvs::srv::SetBool::Response::SharedPtr service_control_response_; 
  //void setParamControl (std::string param_name, double value);

  //rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_reset_control;
  //void service_reset_response_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future);
  //std_srvs::srv::Empty::Response::SharedPtr service_reset_response_; 


  // set control parameter 
  //rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr setparam_control_client;
  //void setparam_control_service_response_callback(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future);
  //rcl_interfaces::srv::SetParameters::Response::SharedPtr setparam_control_service_response_; 

//  void initBox(QList <QGraphicsRectItem *>listrecItem_, int idx);
//  void resetBox(QList <QGraphicsRectItem *>listrecItem_, int idx);

//  bool b_isEnabled[2]; 
//  bool b_isTracking[2];
//  int idx_cur_;

//  int sendPosition( int idx );
  //std::vector<std::vector<double>> joint_position;
  //std::vector<std::string> joint_name;

};
}  // namespace rqt_tpe_gui
#endif  // RQT_EXAMPLE_CPP_MY_PLUGIN_H
