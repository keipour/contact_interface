#ifndef _CONTACT_INTERFACE_NODE_H_
#define _CONTACT_INTERFACE_NODE_H_

#include <base/BaseNode.h>
#include <string>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <contact_interface/ContactCommand.h>
#include <contact_interface/ContactStatus.h>

class ContactInterfaceNode : public BaseNode 
{
private:
  // enums
  enum ContactStatus
  {
    None = contact_interface::ContactStatus::CONTACT_NONE,
    Approaching = contact_interface::ContactStatus::CONTACT_APPROACH,
    InContact = contact_interface::ContactStatus::CONTACT_CONTACT,
    Departing = contact_interface::ContactStatus::CONTACT_DEPART
  };

  enum TaskStatus
  {
    NotReceived = contact_interface::ContactStatus::TASK_NONE,
    InProgress = contact_interface::ContactStatus::TASK_IN_PROGRESS,
    Completed = contact_interface::ContactStatus::TASK_COMPLETED,
    Aborted = contact_interface::ContactStatus::TASK_ABORTED
  };

  // parameters
  float status_pub_rate;

  // variables
  bool is_offboard = false;
  bool is_armed = false;
  ContactStatus contact_status = ContactStatus::None;
  int contact_status_seq = 0;
  TaskStatus task_status = TaskStatus::NotReceived;
  float current_force = 0.F;
  float current_moment = 0.F;
  geometry_msgs::PoseStamped current_pose;
  contact_interface::ContactCommand current_command;

  // subscribers
  ros::Subscriber mode_sub, arm_sub, state_sub, pose_sub, contact_command_sub;
  ros::Timer status_publisher_timer;

  // publishers
  ros::Publisher pose_command_pub, contact_status_pub;

  // callbacks
  void mode_callback(const std_msgs::Bool::ConstPtr &msg);
  void arm_callback(const std_msgs::Bool::ConstPtr &msg);
  void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void state_callback(const mavros_msgs::State::ConstPtr &msg);
  void contact_command_callback(const contact_interface::ContactCommand::ConstPtr &msg);
  void status_publisher_timer_callback(const ros::TimerEvent &te);

  // functions
  void publish_status();
  void contact_approach();
  void contact_depart();
  void contact_contact();

public:
  ContactInterfaceNode(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~ContactInterfaceNode();
};

#endif
