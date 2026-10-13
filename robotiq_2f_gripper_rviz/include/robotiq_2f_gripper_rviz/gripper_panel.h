#ifndef ROBOTIQ_2F_GRIPPER_RVIZ_GRIPPER_PANEL_H
#define ROBOTIQ_2F_GRIPPER_RVIZ_GRIPPER_PANEL_H

#ifndef Q_MOC_RUN
#include <memory>
#include <mutex>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/JointState.h>
#endif

class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QTimer;

namespace robotiq_2f_gripper_rviz
{

/** RViz panel with the controls of example/03_gui: speed and force sliders, OPEN and CLOSE buttons
 *  and a status line. It sends CommandRobotiqGripper goals to the action server of
 *  robotiq_2f_action_server.py and shows the gripper width from the action feedback or, when idle,
 *  from /joint_states. */
class GripperPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit GripperPanel(QWidget* parent = nullptr);
  ~GripperPanel() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void onOpen();
  void onClose();
  void onStop();
  void onActionNameChanged();
  void refresh();

private:
  typedef actionlib::SimpleActionClient<robotiq_2f_gripper_msgs::CommandRobotiqGripperAction> Client;
  typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedbackConstPtr FeedbackPtr;
  typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperResultConstPtr ResultPtr;

  void connectToServer(const std::string& action_name);
  void sendGoal(double position, bool stop);
  // Called from the action client's thread
  void feedbackCb(const FeedbackPtr& feedback);
  void doneCb(const actionlib::SimpleClientGoalState& state, const ResultPtr& result);
  // Called from the panel's spinner thread
  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg);

  QLineEdit* action_name_edit_;
  QLabel* speed_label_;
  QSlider* speed_slider_;
  QLabel* force_label_;
  QSlider* force_slider_;
  QPushButton* open_button_;
  QPushButton* close_button_;
  QPushButton* stop_button_;
  QLabel* status_label_;
  QTimer* timer_;

  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber joint_state_sub_;
  std::unique_ptr<Client> client_;
  std::string action_name_;
  std::string joint_name_;
  double stroke_;  // [m] 0.085 for the 2F-85, 0.140 for the 2F-140

  std::mutex mutex_;
  std::string status_text_;
  double goal_width_;   // [m] width from the last feedback or result, negative when unknown
  double joint_width_;  // [m] width from /joint_states, negative when unknown
  bool was_connected_;
};

}  // namespace robotiq_2f_gripper_rviz

#endif  // ROBOTIQ_2F_GRIPPER_RVIZ_GRIPPER_PANEL_H
