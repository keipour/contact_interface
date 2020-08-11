#ifndef _CONTACT_INTERFACE_NODE_H_
#define _CONTACT_INTERFACE_NODE_H_

#include <base/BaseNode.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <contact_interface/ContactCommand.h>
#include <contact_interface/ContactStatus.h>
#include <tf/transform_datatypes.h>
#include <core_px4_interface/AttMode.h>

class ContactInterfaceNode : public BaseNode 
{
private:
  // enums
  enum ContactStatus
  {
    None = contact_interface::ContactStatus::CONTACT_NONE,
    Approaching = contact_interface::ContactStatus::CONTACT_APPROACH,
    InContact = contact_interface::ContactStatus::CONTACT_CONTACT,
    Departing = contact_interface::ContactStatus::CONTACT_DEPART,
    Waiting = contact_interface::ContactStatus::CONTACT_WAITING
  };

  enum TaskStatus
  {
    NotReceived = contact_interface::ContactStatus::TASK_NONE,
    InProgress = contact_interface::ContactStatus::TASK_IN_PROGRESS,
    Completed = contact_interface::ContactStatus::TASK_COMPLETED,
    Aborted = contact_interface::ContactStatus::TASK_ABORTED
  };

  enum ApproachStatus
  {
      GettingClose = 0,
      LockingTilt = 1,
      FinalStage = 2
  };

  // parameters
  float status_pub_rate, distance_step, stop_distance, tilt_lock_distance, normal_xy_speed, approach_xy_speed;

  // variables
  bool is_offboard = false;
  bool is_armed = false;
  int contact_status_seq = 0;
  ros::Time delay_started;

  float current_force = 0.F;
  float current_moment = 0.F;

  ContactStatus contact_status = ContactStatus::None;
  TaskStatus task_status = TaskStatus::NotReceived;
  ApproachStatus approach_status = ApproachStatus::GettingClose;

  contact_interface::ContactCommand current_command;

  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped starting_pose;
  geometry_msgs::PoseStamped contact_pose;

  // subscribers
  ros::Subscriber mode_sub, arm_sub, state_sub, pose_sub, contact_command_sub, depth_sub, heading_sub;
  ros::Timer status_publisher_timer;

  // publishers
  ros::Publisher pose_command_pub, contact_status_pub, att_mode_pub, max_xy_vel_pub;

  // callbacks
  void mode_callback(const std_msgs::Bool::ConstPtr &msg);
  void arm_callback(const std_msgs::Bool::ConstPtr &msg);
  void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void contact_command_callback(const contact_interface::ContactCommand::ConstPtr &msg);
  void depth_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
  void heading_callback(const std_msgs::Float32::ConstPtr &msg);
  void status_publisher_timer_callback(const ros::TimerEvent &te);

  // functions
  void publish_status();
  void publish_pose_command(const geometry_msgs::Pose &pose);
  void publish_att_mode(const int mode);
  void publish_max_xy_vel(const float speed);
  void approach();
  void approach_getting_close();
  void approach_locking_tilt();
  void approach_final_stage();
  void depart();
  void contact();
  void fly_forward(const double dist_forward);
  void change_status(ApproachStatus as, ContactStatus cs, TaskStatus ts);

public:
  ContactInterfaceNode(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~ContactInterfaceNode();
};

#endif
