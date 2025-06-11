#include "rqt_tpe_gui/tpe_view.h"
#include <QStringList>
#include <QDebug>
#include <QMessageBox>
#include <iostream>

//#include "rqt_tpe_gui/strucav.h"

/*include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/region_of_interest.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>*/




using std::placeholders::_1;

namespace rqt_tpe_gui
{

//const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
//const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
//const size_t ROS_QUEUE_SIZE = 10;
//const std::string EEF_FRAME_ID = "tool0";
//const std::string BASE_FRAME_ID = "base_link";


TPEView::TPEView()
  : rqt_gui_cpp::Plugin()
//  , widget_(0) 
    ,mw_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("TPEView");
}

void TPEView::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  //widget_ = new QWidget();
  mw_ = new QMainWindow();

  // extend the widget with all attributes and children from UI file
  //ui_.setupUi(widget_);
  ui_.setupUi(mw_);

  
  // add widget to the user interface
  //context.addWidget(widget_);
  context.addWidget(mw_);

  scene = new GraphicsScene(this);
  pixmapItem = new QGraphicsPixmapItem();
  scene->addItem(pixmapItem);
  ui_.gViewCam->setScene(scene);



  connect(scene,SIGNAL(clickedPoint(QPoint)),this,SLOT(NewPointClicked(QPoint)));
  connect(scene,SIGNAL(setMouseCursorPos(QPoint)),this,SLOT(setMouseCursorPos(QPoint)));
  connect(scene,SIGNAL(PointSelectionValidation()),this,SLOT(setPointSelected()));

  //topic list
  updateTopicList();
  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));
  connect(this,SIGNAL(newFrame()), this, SLOT(doUpdate()));


  tpe_av_plugin = "tpe_av_plugins::TpeAv3D";
  try {
      RCLCPP_INFO(node_->get_logger(), "plugin name to load : %s " , tpe_av_plugin.c_str());
      p_tpe_av = tpe_base_loader_.createSharedInstance(tpe_av_plugin);
  } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "The plugin failed to load for some reason. Error: %s\n",
        ex.what());
  }


    tpe.im.width = 0;
    tpe.im.height = 0;

  //camera control :
  ui_.cBoxAxisCamera->addItem("vx");
  ui_.cBoxAxisCamera->addItem("vy");
  ui_.cBoxAxisCamera->addItem("vz");
  ui_.cBoxAxisCamera->addItem("rz");
  VelocitySliderTimer = new QTimer();
  VelocitySliderTimer->setInterval(0);



  connect(VelocitySliderTimer, SIGNAL(timeout()), this, SLOT(VelocitySliderTimerIdle()));

  connect(ui_.hSliderVelocityCamera,SIGNAL(sliderPressed()),this,SLOT(VelocitySliderPressed()));
  connect(ui_.hSliderVelocityCamera,SIGNAL(sliderReleased()),this,SLOT(VelocitySliderRealesed()));
  connect(ui_.hSliderVelocityCamera,SIGNAL(valueChanged(int)),this,SLOT(VelocitySliderValueChanged(int)));
  in_event = 0;

  connect(ui_.cBoxSelectionReference,SIGNAL(stateChanged(int)),this,SLOT(referenceTargetSelection(int)));
  connect(ui_.cBoxTrackingReference,SIGNAL(stateChanged(int)),this,SLOT(referenceTargetTracking(int)));
  connect(ui_.cBoxSelectionCurrent,SIGNAL(stateChanged(int)),this,SLOT(currentTargetSelection(int)));
  connect(ui_.cBoxTrackingCurrent,SIGNAL(stateChanged(int)),this,SLOT(currentTargetTracking(int)));
  connect(ui_.pButtonResetReference,SIGNAL(clicked(bool)),this,SLOT(referenceTargetResetClicked()));
  connect(ui_.pButtonResetCurrent,SIGNAL(clicked(bool)),this,SLOT(currentTargetResetClicked()));

  connect(ui_.pButtonStop,SIGNAL(clicked(bool)),this,SLOT(StopButtonClicked()));
  //connect(ui_.rButtonCL,SIGNAL(toggled(bool)),this,SLOT(onCloseLoopChanged(bool)));

  connect(ui_.dSpinBoxGR,SIGNAL(valueChanged(double)),this,SLOT(dSpinBoxGRValueChanged (double  )));
  connect(ui_.dSpinBoxGT,SIGNAL(valueChanged(double)),this,SLOT(dSpinBoxGTValueChanged (double  )));
  tpe_control_.control_param.gain_t = ui_.dSpinBoxGT->value();
  tpe_control_.control_param.gain_r = ui_.dSpinBoxGR->value();


  //connect(ui_.dSpinBoxDepth,SIGNAL(valueChanged(double)),this,SLOT(dSpinBoxDepthValueChanged (double )));


  //connect(ui_.pButtonGoto,SIGNAL(clicked(bool)),this,SLOT(GotoButtonClicked()));

  init_robot__com();
  
//  //twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
//  //frame_to_publish_ = EEF_FRAME_ID;


	//camera_velocities_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/tpe_robot_control/commands_sensors", 10);
	//roi_des_publisher_ = node_->create_publisher<tpe_robot_interfaces::msg::MultiRegionOfInterest>("/tpe_gui/roi_position", 10);
  //roi_list_subscripber_ = node_->create_subscription<tpe_robot_interfaces::msg::MultiRegionOfInterest>(
  //  "/roi_list", 10, std::bind(&TPEView::callbackRoiUpdate, this, _1));


  //client_tracking = node_->create_client<tpe_robot_interfaces::srv::TpeTrigger>("trigTracking");
  //client_control = node_->create_client<std_srvs::srv::SetBool>("/tpe_robot_control/trigControl");
  //client_reset_control = node_->create_client<std_srvs::srv::Empty>("/tpe_robot_control/reset");

  //robot_joint_trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/tpe_robot_trajectory_controller/joint_trajectory", 10);
  //joint_name.push_back("jointX");joint_name.push_back("jointY");joint_name.push_back("jointZ");joint_name.push_back("jointR");

  //std::vector<double> position;
  //position.push_back(NAN);position.push_back(NAN);position.push_back(NAN);position.push_back(NAN);
  //joint_position.push_back(position);
  //position.clear();
  //position.push_back(0.035);position.push_back(0.345);position.push_back(-0.015);position.push_back(0.0);
  //joint_position.push_back(position);
  //position.clear();
  //position.push_back(0.202);position.push_back(0.345);position.push_back(-0.015);position.push_back(0.0);
  //joint_position.push_back(position);

  //setparam_control_client = node_->create_client<rcl_interfaces::srv::SetParameters>("/tpe_robot_control/set_parameters");

  startPoint.setX(0);
  startPoint.setY(0);
  mouseCursorPos.setX(0);
  mouseCursorPos.setY(0);


  for (int i=0; i < TPE_NB_MARQUEUR; i++) {
      ui_.comboBoxObjectNumCur->addItem(QString::number(i+1));
  }

  connect(ui_.comboBoxObjectNumCur,SIGNAL(currentIndexChanged(int )),this,SLOT(comboxOnValueChange(int)));
  ui_.lblObjectNumberRef->setText(QString::number(1));

  for (int i=0; i < TPE_NB_MARQUEUR; i++) {
    listBox.append(new QRect(startPoint.x(),startPoint.y(),0,0));
    listBoxRef.append(new QRect(startPoint.x(),startPoint.y(),0,0));
    listBoxCur.append(new QRect(startPoint.x(),startPoint.y(),0,0));

    listrecItem.append(new QGraphicsRectItem());
    listrecItem[i]->setPen(QPen(Qt::red,2));
    scene->addItem(listrecItem[i]);

    listrecItemCur.append(new QGraphicsRectItem());
    listrecItemCur[i]->setPen(QPen(Qt::green,2));
    scene->addItem(listrecItemCur[i]);

    listrecItemRef.append(new QGraphicsRectItem());
    listrecItemRef[i]->setPen(QPen(Qt::blue,2));
    scene->addItem(listrecItemRef[i]);

    listDrawBox.append(false);
    }


  for (int i=0; i < 2; i++) {
    tpe.cibleN[i].b_isEnabled = 0;
    tpe.cibleN[i].b_isTracking = 0;
  }

  //tpe_control_.target_param.geometry = std::vector<double>{{-TPE_PAS/2.0,0.0,TPE_PAS/2.0,TPE_PAS/7.0}};
  //tpe_control_.target_param.geometry = std::vector<double>{{-TPE_PAS/2.0,0.0,TPE_PAS/2.0,0.0}};
  tpe_control_.target_param.geometry = std::vector<double>{{0.0,TPE_PAS/2,0.0,-TPE_PAS/2}};
 /*auto ros2_topics = node_->get_topic_names_and_types();
 for (auto topic_and_types : ros2_topics) {

      auto & topic_name = topic_and_types.first;
      auto & topic_type = topic_and_types.second[0];  // explicitly take the first
      RCLCPP_INFO(node_->get_logger(), "topic :%s (%s)...", topic_name.c_str(), topic_type.c_str());

 }*/

}

void TPEView::shutdownPlugin()
{
  // unregister all publishers here
  if (tpe.im.coord != NULL)
    free(tpe.im.coord);

}

void TPEView::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;

  // instance_settings.setValue(k, v)
}

void TPEView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void TPEView::VelocitySliderPressed(){


    //RCLCPP_INFO(node_->get_logger(), "Slider pressed");
    StateMouseButtonSlider = true;
    VelocitySliderTimer->start();

}
void TPEView::VelocitySliderRealesed(){

    //RCLCPP_INFO(node_->get_logger(), "Dial Realesed");
    StateMouseButtonSlider = false;


}
void TPEView::VelocitySliderTimerIdle() {

    double controlRob[4];
    if (StateMouseButtonSlider && (ui_.hSliderVelocityCamera->value()!= 0))   {

        //RCLCPP_INFO(node_->get_logger(), "slider value : %d" ,ui_.hSliderVelocityCamera->value());
        if (ui_.rButtonOL->isChecked()) {
          this->VitSensor2VitRob(control, controlRob);
          this->sendVelocity(controlRob);
        }

        //VelocitySliderValueChanged(ui_.hSliderVelocityCamera->value());
        //ui_.hSliderVelocityCamera->setValue(0);
        //VelocitySliderTimer->stop();
    }


    if (!StateMouseButtonSlider && (ui_.hSliderVelocityCamera->value()!= 0))   {

        ui_.hSliderVelocityCamera->setValue(0);
        VelocitySliderTimer->stop();
    }
}

void TPEView::VelocitySliderValueChanged(int val) {

    double controlRob[4];
    //RCLCPP_INFO(node_->get_logger(), "slider value : %d" ,val);

    if (in_event)   {
        RCLCPP_INFO(node_->get_logger(), "On fait rien");
        return;
    }

    in_event = 1;

    //double control[4];
    for (int i = 0; i < 4 ; i++)
        control[i]=0.0;

    int idx = ui_.cBoxAxisCamera->currentIndex();
    control[idx] = ((double)val)/100.0;

    //RCLCPP_INFO(node_->get_logger(), "slider value : %f %f %f %f" ,control[0],control[1],control[2],control[3]);
    if (ui_.rButtonOL->isChecked()) {
          this->VitSensor2VitRob(control, controlRob);
          this->sendVelocity(controlRob);
      }

    in_event = 0;


}

/*int TPEView::sendVelocity(double *control ) {

  //  double ComRob[4];
  //  char c_cmd[512];
  //  std::string cmd;
    
    auto camera_velocities_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    //auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    //VitCam2VitRob(control,ComRob);

    for (int i = 0; i < 4 ; i++ ) {
	    //camera_velocities_msg->data.push_back((float)ComRob[i]);
	    camera_velocities_msg->data.push_back((float)control[i]);
    }

  //  twist_msg->header.stamp = node_->now();
  //  twist_msg->header.frame_id = frame_to_publish_;

  //  twist_msg->twist.linear.x = control[0];
  //  twist_msg->twist.linear.y = control[1];
  //  twist_msg->twist.linear.z = control[2];
  //  twist_pub_->publish(std::move(twist_msg));

    //RCLCPP_INFO(node_->get_logger(), "publish vel : %f %f %f %f" ,ComRob[0],ComRob[1],ComRob[2],ComRob[3]);
  camera_velocities_publisher_->publish(std::move(camera_velocities_msg));


    return 0;
}*/
/*int TPEView::VitCam2VitRob(double *vitcam, double *vitrob) {

    double vitbase[4];

    vitbase[0]=vitcam[2];
    vitbase[1]=-vitcam[0];
    vitbase[2]=-vitcam[1];
    vitbase[3]=vitcam[3];

//    vitrob[0]=vitbase[3]; // R robot - R camera !! ATTENTION A VERIFIER SUR LA MANIP !!
    vitrob[0]=-vitbase[3]; // R robot - R camera
    vitrob[1]=-vitbase[1];// Y robot - X camera
    vitrob[2]=vitbase[0]; 
//    vitrob[3]=-vitbase[2]; !! ATTENTION A VERIFIER SUR LA MANIP !!
    vitrob[3]=vitbase[2]; 

    return 0;


}*/

void TPEView::comboxOnValueChange (int value) {

    ui_.lblObjectNumberRef->setText(QString::number(value+1));

}

void TPEView::NewPointClicked(QPoint p) {

  //std::cout << "NewPointClicked" << std::endl;
  RCLCPP_INFO(node_->get_logger(), "NewPointClicked : (%d,%d), (%d,%d)...", m_width, m_height, p.x(), p.y());


    if ( p.x() < 0 || p.y() < 0 || p.x() >=m_width ||  p.y() >=m_height)
        return;


    startPoint = p;

    QRect *box;
    box = listBox.at(ui_.comboBoxObjectNumCur->currentIndex());

    box->setX(startPoint.x());
    box->setY(startPoint.y());
    box->setWidth(0);
    box->setHeight(0);

    //drawBox=true;
    listDrawBox[ui_.comboBoxObjectNumCur->currentIndex()]=true;


}




void TPEView::setMouseCursorPos(QPoint p)    {

  //std::cout << "setMouseCursorPos" << std::endl;


    mouseCursorPos=p;
    if( mouseCursorPos.x() >= m_width)
        mouseCursorPos.setX(m_width-1);
    if( mouseCursorPos.y() >= m_height)
        mouseCursorPos.setY(m_height-1);

    bool drawBox;
    drawBox = listDrawBox[ui_.comboBoxObjectNumCur->currentIndex()];
    QRect *box;
    if(drawBox)
        {
            box = listBox.at(ui_.comboBoxObjectNumCur->currentIndex());
            box->setWidth(mouseCursorPos.x()-startPoint.x());
            box->setHeight(mouseCursorPos.y()-startPoint.y());
        }


}

void TPEView::setPointSelected() {

//   std::cout << "setPointSelected" << std::endl;
    int idx = ui_.comboBoxObjectNumCur->currentIndex();
    if (++idx < TPE_NB_MARQUEUR)
        ui_.comboBoxObjectNumCur->setCurrentIndex(idx);
    else
       ui_.comboBoxObjectNumCur->setCurrentIndex(0);

}

void TPEView::referenceTargetSelection(int state ) {

  RCLCPP_INFO(node_->get_logger(), "reference selection state : %d " , state);
    if (state) {

        initBox(listrecItemRef, 0);
        if (!tpe.cibleN[0].b_isEnabled) {
            ui_.cBoxSelectionReference->setCheckState(Qt::CheckState(false));
        }
    }

    else {
       resetBox(listrecItemRef, 0);
       ui_.cBoxTrackingReference->setChecked(false);

    }


}

void TPEView::referenceTargetTracking(int state ) {

    if (state) {
        tpe.cibleN[0].b_isTracking = 1;
    }
    else {
        tpe.cibleN[0].b_isTracking = 0;
    }

}

void TPEView::currentTargetSelection(int state ) {

    RCLCPP_INFO(node_->get_logger(), "current selection state : %d " , state);


     if (state) {
        initBox(listrecItemCur, 1);
        ui_.cBoxTrackingReference->setChecked(false);

        if (!tpe.cibleN[1].b_isEnabled) {
            ui_.cBoxSelectionCurrent->setCheckState(Qt::CheckState(false));
        }

    }
    else {
       resetBox(listrecItemCur, 1);
       ui_.cBoxTrackingCurrent->setChecked(false);

    }
}

void TPEView::currentTargetTracking(int state ) {

  qDebug() << "current tracking state :" << state;
    if (state) {
        tpe.cibleN[1].b_isTracking = 1;
    }
    else {
        tpe.cibleN[1].b_isTracking = 0;
    }

}

void TPEView::trackingTarget(QList <QGraphicsRectItem *>listrecItem_, int idx) {


    p_tpe_av->ImageProcessing(&tpe,idx);
    p_tpe_av->Update_aoi(&tpe,idx);
    p_tpe_av->Update_mesure(&tpe, tpe_control_, idx );
    for (int i=0; i < listrecItem_.size();i++) {
        listrecItem_[i]->setRect(tpe.cibleN[idx].marque[i].aoi_x0,tpe.cibleN[idx].marque[i].aoi_y0,
                    tpe.cibleN[idx].marque[i].aoi_dx, tpe.cibleN[idx].marque[i].aoi_dy);
    }
    RCLCPP_INFO(node_->get_logger(), "position(%d) : %f %f %f %f %f " , idx, tpe.info_image[idx][0], tpe.info_image[idx][1],
    tpe.info_image[idx][2],tpe.info_image[idx][3], tpe.info_image[idx][4]);

}



/*void TPEView::onCloseLoopChanged( bool checked ) {
  
  RCLCPP_INFO(node_->get_logger(), "Close Loop state changed : %d " , (int)checked);
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    if (checked) {
        request->data = true;
        RCLCPP_INFO(node_->get_logger(), "Close Loop state changed : %d " , (int)request->data);
    }
    else {
        request->data = false;
        RCLCPP_INFO(node_->get_logger(), "Close Loop state changed : %d " , (int)request->data);
     }

  auto result = client_control->async_send_request(request,std::bind(&TPEView::service_control_response_callback,
                                                                                this, _1));
}
*/

/*void TPEView::service_response_callback(rclcpp::Client<tpe_robot_interfaces::srv::TpeTrigger>::SharedFuture future) {
        
    service_response_ = future.get();  // Save service response to the pointer declared outside the scope for further process
    //std::cout << (int) service_response_->success << std::endl;
    RCLCPP_INFO(node_->get_logger(), "response server : %d " , service_response_->success);

}*/

/*void TPEView::service_control_response_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        
    service_control_response_ = future.get();  // Save service response to the pointer declared outside the scope for further process
    //std::cout << (int) service_response_->success << std::endl;
    RCLCPP_INFO(node_->get_logger(), "response  control server : %d " , service_control_response_->success);

}*/

/*void TPEView::setparam_control_service_response_callback(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {

    setparam_control_service_response_ = future.get();  // Save service response to the pointer declared outside the scope for further process
    //std::cout << (int) service_response_->success << std::endl;
    RCLCPP_INFO(node_->get_logger(), "response  control server : %d " , setparam_control_service_response_->results[0].successful);


}*/

/*void TPEView::service_reset_response_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {

  service_reset_response_ = future.get(); 
  RCLCPP_INFO(node_->get_logger(), "response  control reset server " );

}
*/

void TPEView::initBox(QList <QGraphicsRectItem *>listrecItem_, int idx) {

    int count=0;
    RCLCPP_INFO(node_->get_logger(), "!!!!!InitBox!!!!! : %d " , count);
    for (int i=0; i < listBox.size();i++) {
        if (listDrawBox[i]) {
            listrecItem_[i]->setRect(*listBox[i]);
            tpe.cibleN[idx].marque[i].aoi_x0 = listBox[i]->x();
            tpe.cibleN[idx].marque[i].aoi_y0 = listBox[i]->y();
            tpe.cibleN[idx].marque[i].aoi_dx = listBox[i]->width();
            tpe.cibleN[idx].marque[i].aoi_dy = listBox[i]->height();

            listBox[i]->setWidth(0);
            listBox[i]->setHeight(0);
            listrecItem[i]->setRect(0,0,0,0);
            listDrawBox[i]=false;
            count ++;

        }
    }

    if (count == TPE_NB_MARQUEUR ) {
        tpe.cibleN[idx].b_isEnabled = 1;
        tpe.cibleN[idx].nb_marque = count;
        //p_tpe->Moment(&tpe,idx);
        p_tpe_av->ImageProcessing(&tpe,idx);
        p_tpe_av->Update_aoi(&tpe,idx);
        p_tpe_av->Update_mesure(&tpe, tpe_control_, idx );
        for (int i=0; i < listBox.size();i++) {
            listrecItem_[i]->setRect(tpe.cibleN[idx].marque[i].aoi_x0,tpe.cibleN[idx].marque[i].aoi_y0,
                        tpe.cibleN[idx].marque[i].aoi_dx, tpe.cibleN[idx].marque[i].aoi_dy);
        }
    }



}

void TPEView::resetBox(QList <QGraphicsRectItem *>listrecItem_, int idx) {

    tpe.cibleN[idx].b_isEnabled = 0;
    for (int i=0; i < listBox.size();i++) {
            listrecItem_[i]->setRect(0,0,0,0);
    }

}

void TPEView::referenceTargetResetClicked( ) {

    ui_.cBoxTrackingReference->setChecked(false);
    ui_.cBoxSelectionReference->setChecked(false);
    //auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    //auto result = client_reset_control->async_send_request(request,std::bind(&TPEView::service_reset_response_callback,
    //                                                                            this, _1));


}

void TPEView::currentTargetResetClicked( ){

    ui_.cBoxTrackingCurrent->setChecked(false);
    ui_.cBoxSelectionCurrent->setChecked(false);
    //auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    //auto result = client_reset_control->async_send_request(request,std::bind(&TPEView::service_reset_response_callback,
    //                                                                            this, _1));

}


/*void TPEView::updateTopicList()
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
}*/

/*QList<QString> TPEView::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
  QSet<QString> message_sub_types;
  return getTopics(message_types, message_sub_types, transports).values();
}*/

/*QSet<QString> TPEView::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
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
}*/

/*void TPEView::selectTopic(const QString& topic)
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
}*/

/*void TPEView::onTopicChanged(int index)
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
        &MyPlugin::callbackImage, hints.getTransport());*

      //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(mw_, tr("Loading image transport plugin failed"), e.what());
    }
  }

  //onMousePublish(ui_.publish_click_location_check_box->isChecked());
}*/




/*void TPEView::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {

    //RCLCPP_INFO(node_->get_logger(), "MyPlugin::callbackImages(): new image available");
    //cv_bridge::CvImagePtr cv_ptr;

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    conversion_mat_ = cv_ptr->image;
    emit newFrame();



}*/


/*void TPEView::callbackRoiUpdate(const tpe_robot_interfaces::msg::MultiRegionOfInterest::SharedPtr msg) {

    QRect *boxTmp;
    QList <QRect*> *listBox_;

    if (idx_cur_ == 0)
      listBox_ = &listBoxRef;
    else if (idx_cur_ == 1)
     listBox_ = &listBoxCur;
    else
      return;

    if (msg->roilist.size() != listBox_->size()) 
      return;

    for (unsigned int i=0; i < msg->roilist.size(); i++) {

      boxTmp = listBox_->at(i);
      boxTmp->setX(msg->roilist.at(i).x_offset);        
      boxTmp->setY(msg->roilist.at(i).y_offset);
      boxTmp->setWidth(msg->roilist.at(i).width) ;
      boxTmp->setHeight(msg->roilist.at(i).height) ;
 
    }



}*/

/*void TPEView::trackingTarget(QList <QGraphicsRectItem *>listrecItem_, QList <QRect*>listBox_) {


    for (int i=0; i < listrecItem_.size();i++) {
        //RCLCPP_INFO(node_->get_logger(), "TrackingTarget: %d %d %d %d", listBox_[i]->width(),listBox_[i]->height(),listBox_[i]->x(), listBox_[i]->y() );
        listrecItem_[i]->setRect(listBox_[i]->x(),listBox_[i]->y(),
                        listBox_[i]->width(), listBox_[i]->height());
    }

}*/


void TPEView::doUpdate()
{

    double control[4];
    double controlRob[4];

    if ((m_width !=conversion_mat_.cols) || (m_height!=conversion_mat_.rows)) {
      m_width = conversion_mat_.cols;
      m_height = conversion_mat_.rows;
      //RCLCPP_INFO(node_->get_logger(), "MyPlugin::doUpdate() - 1 -  '%d, %d'", m_width, m_height);

      ui_.gViewCam->setFixedSize(m_width+50,m_height+50);
    }

    QImage p_QImg(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    QPixmap Pixcur(QPixmap::fromImage(p_QImg));
    pixmapItem->setPixmap(Pixcur);

  
    for (int i=0; i < TPE_NB_MARQUEUR; i++) {
      //RCLCPP_INFO(node_->get_logger(), "MyPlugin::doUpdate() '%d, %d'", i, (int)listDrawBox[i]);
 
      if (listDrawBox[i])
        listrecItem[i]->setRect(*listBox[i]);
    }

    if( tpe.cibleN[0].b_isTracking /*|| tpe.cibleN[0].b_isEnabled */) {

        trackingTarget(listrecItemRef, 0);

    }

    if( tpe.cibleN[1].b_isTracking /*|| tpe.cibleN[1].b_isEnabled */) {

        trackingTarget(listrecItemCur, 1);


    }

    if ( (tpe.cibleN[0].b_isEnabled && !tpe.cibleN[0].b_isTracking) && (tpe.cibleN[1].b_isEnabled && tpe.cibleN[1].b_isTracking)) {

        p_tpe_av->Commande ( tpe, tpe_control_, control);
        this->VitSensor2VitRob(control, controlRob);
        //RCLCPP_INFO(node_->get_logger(), "control : %f %f %f %f  " , control[0], control[1],
        //control[2],control[3]);


        if (ui_.rButtonCL->isChecked()) {
           this->VitSensor2VitRob(control, controlRob);
           this->sendVelocity(controlRob, false);
        }
        //    sendVelocity(control,false);
        RCLCPP_INFO(node_->get_logger(), "control : %f %f %f %f  " , controlRob[0], controlRob[1],
        controlRob[2],controlRob[3]);

    }




}

void TPEView::StopButtonClicked(  ) {




    double controlRob[4];
    for (int i = 0; i < 4 ; i++)
        controlRob[i]=0.0;

    ui_.rButtonOL->setChecked(true);
    this->sendVelocity(controlRob, false);



}

/*void TPEView::setParamControl (std::string param_name, double value) {

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto param_msg = rcl_interfaces::msg::Parameter();    
    auto paramvalue_msg = rcl_interfaces::msg::ParameterValue(); 

    param_msg.name= param_name; 
    paramvalue_msg.type=3;
    paramvalue_msg.double_value = value;

    param_msg.value = paramvalue_msg ;

    request->parameters.push_back(param_msg);


    auto result = setparam_control_client->async_send_request(request,std::bind(&TPEView::setparam_control_service_response_callback,
                                                                                this, _1));

}*/

void TPEView::dSpinBoxGTValueChanged (double value ) {

    tpe_control_.control_param.gain_t = value;
}

void TPEView::dSpinBoxGRValueChanged (double value ){

    tpe_control_.control_param.gain_r = value;
}

/*void TPEView::dSpinBoxDepthValueChanged (double value ){

    setParamControl(std::string("z_des"), value);
    //m_control.z_des = value;
}*/

/*void TPEView::GotoButtonClicked(  ) {

  int idx = ui_.cBoxPosition->currentIndex();
  sendPosition(idx);

}*/

/*int TPEView::sendPosition( int idx ) {

  auto trajectory = trajectory_msgs::msg::JointTrajectory();
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();

  trajectory.joint_names = joint_name;
  //trajectory.header.stamp = node_->get_clock()->now(); //rclcpp::Clock().now(); //node_.get_clock();

  if ( idx < (int) joint_position.size() ) {
    point.positions = joint_position.at(idx+1);
    point.time_from_start.sec = 5;
    trajectory.points.push_back(point);

    robot_joint_trajectory_publisher_->publish(std::move(trajectory));
    RCLCPP_INFO(node_->get_logger(), "sendTrajectory '%d'", idx+1);
    RCLCPP_INFO(node_->get_logger(), "Position : %f,%f,%f,%f", point.positions[0],point.positions[1],point.positions[2],point.positions[3] );

  }

  return 0;
}*/

}  // namespace rqt_tpe_gui




#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rqt_tpe_gui::TPEView, rqt_gui_cpp::Plugin)
