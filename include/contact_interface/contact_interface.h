#ifndef _CONTACT_INTERFACE_NODE_H_
#define _CONTACT_INTERFACE_NODE_H_

#include <base/BaseNode.h>
#include <string>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

class ContactInterfaceNode : public BaseNode 
{
private:
  // variables
  bool is_offboard = false;
  bool is_armed = false;

  // subscribers
  ros::Subscriber mode_sub, arm_sub, state_sub, pose_sub;

  // publishers
  ros::Publisher pose_command_pub;

  // callbacks
  void mode_callback(const std_msgs::Bool::ConstPtr &msg);
  void arm_callback(const std_msgs::Bool::ConstPtr &msg);
  void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void state_callback(const mavros_msgs::State::ConstPtr &msg);

public:
  ContactInterfaceNode(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~ContactInterfaceNode();

};

#endif
