#include <contact_interface/contact_interface.h>

ContactInterfaceNode::ContactInterfaceNode(std::string node_name)
  : BaseNode(node_name){
}

bool ContactInterfaceNode::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init subscribers
  state_sub = nh->subscribe<mavros_msgs::State>("mavros/state", 10, &ContactInterfaceNode::state_callback, this);
  pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &ContactInterfaceNode::pose_callback, this);
  mode_sub = nh->subscribe<std_msgs::Bool>("has_control", 10, &ContactInterfaceNode::mode_callback, this);
  arm_sub = nh->subscribe<std_msgs::Bool>("is_armed", 10, &ContactInterfaceNode::arm_callback, this);
  contact_command_sub = nh->subscribe<contact_interface::ContactCommand>("controller/contact/command", 10, &ContactInterfaceNode::contact_command_callback, this);

  // init publishers
  pose_command_pub = nh->advertise<geometry_msgs::PoseStamped>("pose_command", 10);
  contact_status_pub = nh->advertise<geometry_msgs::PoseStamped>("controller/contact/status", 10);

  return true;
}

bool ContactInterfaceNode::execute()
{
  // ros::Time::now() - last_request > ros::Duration(5.0))
  
  return true;
}

void ContactInterfaceNode::mode_callback(const std_msgs::Bool::ConstPtr &msg)
{
  is_offboard = msg->data;
}

void ContactInterfaceNode::arm_callback(const std_msgs::Bool::ConstPtr &msg)
{
  is_armed = msg->data;
}

void ContactInterfaceNode::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
}

void ContactInterfaceNode::state_callback(const mavros_msgs::State::ConstPtr &msg)
{
}

void ContactInterfaceNode::contact_command_callback(const contact_interface::ContactCommand::ConstPtr &msg)
{
}

ContactInterfaceNode::~ContactInterfaceNode() = default;

BaseNode* BaseNode::get()
{
  ContactInterfaceNode* control_interface_node = new ContactInterfaceNode("ControlInterfaceNode");
  return control_interface_node;
}
