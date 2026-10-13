#include "robotiq_2f_gripper_rviz/gripper_panel.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.h>

namespace robotiq_2f_gripper_rviz
{

namespace
{
// Same limits as the Robotiq driver
const int SPEED_MIN_MM_S = 13;
const int SPEED_MAX_MM_S = 100;
const int FORCE_MIN = 0;
const int FORCE_MAX = 100;
}  // namespace

GripperPanel::GripperPanel(QWidget* parent)
  : rviz::Panel(parent)
  , action_name_("command_robotiq_action")
  , joint_name_("finger_joint")
  , stroke_(0.085)
  , goal_width_(-1.0)
  , joint_width_(-1.0)
  , was_connected_(false)
{
  QHBoxLayout* name_layout = new QHBoxLayout;
  name_layout->addWidget(new QLabel("Action"));
  action_name_edit_ = new QLineEdit(QString::fromStdString(action_name_));
  action_name_edit_->setToolTip("Name of the CommandRobotiqGripper action server");
  name_layout->addWidget(action_name_edit_);

  speed_label_ = new QLabel;
  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(SPEED_MIN_MM_S, SPEED_MAX_MM_S);
  speed_slider_->setValue(SPEED_MAX_MM_S);

  force_label_ = new QLabel;
  force_slider_ = new QSlider(Qt::Horizontal);
  force_slider_->setRange(FORCE_MIN, FORCE_MAX);
  force_slider_->setValue(50);

  open_button_ = new QPushButton("OPEN");
  close_button_ = new QPushButton("CLOSE");
  stop_button_ = new QPushButton("STOP");
  QHBoxLayout* button_layout = new QHBoxLayout;
  for (QPushButton* button : { open_button_, close_button_, stop_button_ })
  {
    button->setMinimumHeight(40);
    button->setEnabled(false);
    button_layout->addWidget(button);
  }

  status_label_ = new QLabel;
  status_label_->setWordWrap(true);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(name_layout);
  layout->addWidget(speed_label_);
  layout->addWidget(speed_slider_);
  layout->addWidget(force_label_);
  layout->addWidget(force_slider_);
  layout->addLayout(button_layout);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(speed_slider_, &QSlider::valueChanged, this, &GripperPanel::refresh);
  connect(force_slider_, &QSlider::valueChanged, this, &GripperPanel::refresh);
  connect(open_button_, &QPushButton::clicked, this, &GripperPanel::onOpen);
  connect(close_button_, &QPushButton::clicked, this, &GripperPanel::onClose);
  connect(stop_button_, &QPushButton::clicked, this, &GripperPanel::onStop);
  connect(action_name_edit_, &QLineEdit::editingFinished, this, &GripperPanel::onActionNameChanged);

  // Own callback queue and spinner, independent of how RViz spins
  nh_.setCallbackQueue(&queue_);
  joint_state_sub_ = nh_.subscribe("joint_states", 1, &GripperPanel::jointStateCb, this);
  spinner_.reset(new ros::AsyncSpinner(1, &queue_));
  spinner_->start();
  connectToServer(action_name_);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &GripperPanel::refresh);
  timer_->start(100);
  refresh();
}

GripperPanel::~GripperPanel()
{
  timer_->stop();
  client_.reset();
  spinner_->stop();
}

void GripperPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString name;
  if (config.mapGetString("action_name", &name) && !name.isEmpty())
  {
    action_name_edit_->setText(name);
    connectToServer(name.toStdString());
  }
  float value;
  if (config.mapGetFloat("speed", &value))
    speed_slider_->setValue(static_cast<int>(value * 1000.0f + 0.5f));
  if (config.mapGetFloat("force", &value))
    force_slider_->setValue(static_cast<int>(value + 0.5f));
  if (config.mapGetFloat("stroke", &value))
    stroke_ = value;
  QString joint;
  if (config.mapGetString("joint_name", &joint) && !joint.isEmpty())
    joint_name_ = joint.toStdString();
}

void GripperPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("action_name", action_name_edit_->text());
  config.mapSetValue("speed", speed_slider_->value() / 1000.0);
  config.mapSetValue("force", force_slider_->value());
  config.mapSetValue("stroke", stroke_);
  config.mapSetValue("joint_name", QString::fromStdString(joint_name_));
}

void GripperPanel::connectToServer(const std::string& action_name)
{
  client_.reset();
  action_name_ = action_name;
  // The client spins its own thread, so the panel never blocks on the server
  client_.reset(new Client(nh_, action_name_, true));
  std::lock_guard<std::mutex> lock(mutex_);
  was_connected_ = false;
  goal_width_ = -1.0;
  status_text_ = "Waiting for action server \"" + action_name_ + "\"...";
}

void GripperPanel::onActionNameChanged()
{
  const std::string name = action_name_edit_->text().trimmed().toStdString();
  if (!name.empty() && name != action_name_)
    connectToServer(name);
}

void GripperPanel::onOpen()
{
  sendGoal(stroke_, false);
}

void GripperPanel::onClose()
{
  sendGoal(0.0, false);
}

void GripperPanel::onStop()
{
  sendGoal(0.0, true);
}

void GripperPanel::sendGoal(double position, bool stop)
{
  if (!client_ || !client_->isServerConnected())
    return;
  robotiq_2f_gripper_msgs::CommandRobotiqGripperGoal goal;
  goal.emergency_release = false;
  goal.stop = stop;
  goal.position = position;
  goal.speed = speed_slider_->value() / 1000.0;
  goal.force = force_slider_->value();
  ROS_INFO("Gripper goal: pos=%.3f speed=%.3f force=%.0f stop=%d", goal.position, goal.speed, goal.force, stop);
  client_->sendGoal(goal, boost::bind(&GripperPanel::doneCb, this, _1, _2), Client::SimpleActiveCallback(),
                    boost::bind(&GripperPanel::feedbackCb, this, _1));
  std::lock_guard<std::mutex> lock(mutex_);
  status_text_ = stop ? "Stopping" : "Moving";
}

void GripperPanel::feedbackCb(const FeedbackPtr& feedback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  goal_width_ = feedback->position;
  status_text_ = feedback->is_moving ? "Moving" : feedback->obj_detected ? "Holding object" : "Idle";
  if (feedback->fault_status)
    status_text_ = QString("Fault: 0x%1").arg(feedback->fault_status, 2, 16, QChar('0')).toUpper().toStdString();
}

void GripperPanel::doneCb(const actionlib::SimpleClientGoalState& state, const ResultPtr& result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (result)
    goal_width_ = result->position;
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    status_text_ = (result && result->obj_detected) ? "Done (object detected)" : "Done";
  else
    status_text_ = "Failed: " + state.toString();
}

void GripperPanel::jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
{
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
  {
    if (msg->name[i] != joint_name_)
      continue;
    // Linear model of the driver: 0 rad open, 0.8 rad (0.7 rad for the 2F-140) closed
    const double max_angle = stroke_ > 0.1 ? 0.7 : 0.8;
    std::lock_guard<std::mutex> lock(mutex_);
    joint_width_ = std::min(stroke_, std::max(0.0, stroke_ * (1.0 - msg->position[i] / max_angle)));
    return;
  }
}

void GripperPanel::refresh()
{
  speed_label_->setText(QString("Speed: %1 m/s").arg(speed_slider_->value() / 1000.0, 0, 'f', 3));
  force_label_->setText(QString("Force: %1 %").arg(force_slider_->value()));

  const bool connected = client_ && client_->isServerConnected();
  QString text;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (connected && !was_connected_)
    {
      was_connected_ = true;
      status_text_ = "Ready";
    }
    text = QString::fromStdString(connected ? status_text_ :
                                              "Waiting for action server \"" + action_name_ + "\"...");
    const double width = goal_width_ >= 0.0 ? goal_width_ : joint_width_;
    if (width >= 0.0)
      text += QString("   |   width: %1 mm").arg(width * 1000.0, 0, 'f', 1);
  }
  status_label_->setText(text);
  open_button_->setEnabled(connected);
  close_button_->setEnabled(connected);
  stop_button_->setEnabled(connected);
}

}  // namespace robotiq_2f_gripper_rviz

PLUGINLIB_EXPORT_CLASS(robotiq_2f_gripper_rviz::GripperPanel, rviz::Panel)
