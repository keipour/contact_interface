#include <contact_interface/contact_interface.h>

ContactInterfaceNode::ContactInterfaceNode(std::string node_name)
  : BaseNode(node_name){
}

bool ContactInterfaceNode::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init parameters
  status_pub_rate = pnh->param("contact_status_pub_rate", 20.);
  distance_step = pnh->param("distance_step", 0.1F);
  stop_distance = pnh->param("stop_distance", 0.2F);

  // init subscribers
  pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &ContactInterfaceNode::pose_callback, this);
  mode_sub = nh->subscribe<std_msgs::Bool>("has_control", 10, &ContactInterfaceNode::mode_callback, this);
  arm_sub = nh->subscribe<std_msgs::Bool>("is_armed", 10, &ContactInterfaceNode::arm_callback, this);
  heading_sub = nh->subscribe<std_msgs::Float32>("point_cloud/heading_angle", 10, &ContactInterfaceNode::heading_callback, this);
  depth_sub = nh->subscribe<geometry_msgs::Vector3Stamped>("point_cloud/pose", 10, &ContactInterfaceNode::depth_callback, this);
  contact_command_sub = nh->subscribe<contact_interface::ContactCommand>("controller/contact/command", 10, &ContactInterfaceNode::contact_command_callback, this);
  status_publisher_timer = nh->createTimer(ros::Duration(1. / status_pub_rate), &ContactInterfaceNode::status_publisher_timer_callback, this);

  // init publishers
  pose_command_pub = nh->advertise<geometry_msgs::PoseStamped>("pose_command", 10);
  contact_status_pub = nh->advertise<contact_interface::ContactStatus>("controller/contact/status", 10);

  return true;
}

bool ContactInterfaceNode::execute()
{
  // return if it's not in offboard mode or if it's not armed
  if (!is_offboard || !is_armed)
  {
    // just return if all the parameters are already set correctly
    if (contact_status == ContactStatus::None && task_status != TaskStatus::InProgress)
      return true;
    
    contact_status = ContactStatus::None;
    if (task_status == TaskStatus::InProgress)
      task_status = TaskStatus::Aborted;

    current_force = 0;
    current_moment = 0;

    publish_status();
    return true;
  }

  switch (contact_status)
  {
  case ContactStatus::Approaching :
    approach();
    break;
  case ContactStatus::InContact :
    contact();
    break;
  case ContactStatus::Departing :
    depart();
    break;
  default:
    break;
  }
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
  current_pose = *msg;
}

void ContactInterfaceNode::contact_command_callback(const contact_interface::ContactCommand::ConstPtr &msg)
{
  // return if it's not in offboard mode or if it's not armed
  if (!is_offboard || !is_armed)
  {
    contact_status = ContactStatus::None;
    task_status = TaskStatus::Aborted;
    publish_status();
    return;
  }
  
  // Set the starting point to return to it later
  starting_pose = current_pose;
  current_command = *msg;

  // Set the task and contact statuses
  task_status = TaskStatus::InProgress;
  contact_status = ContactStatus::Waiting;
}

void ContactInterfaceNode::depth_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  // return if it's not in offboard mode or if it's not armed
%  ROS_WARN("Got it!");
  if ((contact_status != ContactStatus::Approaching && contact_status != ContactStatus::Waiting)
   || task_status != TaskStatus::InProgress)
    return;

  // Read the current distance from the wall
  float dist = msg->vector.z / 100.F;
%  ROS_WARN("Looks good! %0.3lf", dist);

  if (isnan(dist)) 
    return;

  if (dist <= stop_distance)
  {
%    ROS_WARN("Ooooops! %0.3f", dist);
    contact_started = ros::Time::now();
    contact_status = ContactStatus::InContact;
    return;
  }

  contact_status = ContactStatus::Approaching;
  
  double dist_forward = std::min(dist - stop_distance, distance_step);
%  ROS_WARN("Yaaaay! %0.3lf", dist_forward);
  fly_forward(dist_forward);
}

void ContactInterfaceNode::heading_callback(const std_msgs::Float32::ConstPtr &msg)
{
  //current_pose = *msg;
}

void ContactInterfaceNode::fly_forward(const double dist_forward)
{
  current_command.ContactPoint = current_pose.pose;

  // Get the current yaw
  tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
  double roll, pitch, yaw_enu;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw_enu);
  double yaw = M_PI / 2 - yaw_enu;

  // Set the destination as a bit forward in the direction of the current yaw
  current_command.ContactPoint.position.x += dist_forward * std::sin(yaw);
  current_command.ContactPoint.position.y += dist_forward * std::cos(yaw);
}

void ContactInterfaceNode::publish_status()
{
  // initialize the message
  contact_interface::ContactStatus msg;
  msg.Header.stamp = ros::Time::now();
  msg.Header.frame_id = current_command.Header.frame_id;
  msg.Header.seq = contact_status_seq++;
  msg.AppliedForce = current_force;
  msg.AppliedMoment = current_moment;
  msg.EndEffectorPose = current_pose.pose;
  msg.ContactStatus = contact_status;
  msg.TaskStatus = task_status;

  // publish the message
  contact_status_pub.publish(msg);
}

void ContactInterfaceNode::status_publisher_timer_callback(const ros::TimerEvent &te)
{
  publish_status();
}

void ::ContactInterfaceNode::publish_pose_command(const geometry_msgs::Pose &pose)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = current_command.Header.frame_id;
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.pose = pose;

  pose_command_pub.publish(msg);
}

void ContactInterfaceNode::approach()
{
  current_force = 0;
  current_moment = 0;
  
  double dx = current_pose.pose.position.x - current_command.ContactPoint.position.x;
  double dy = current_pose.pose.position.y - current_command.ContactPoint.position.y;
  double dz = current_pose.pose.position.z - current_command.ContactPoint.position.z;
  double dist = dx*dx + dy*dy + dz*dz;

  const double dist_threshold = 0.25F; // 25 cm
  if (dist > dist_threshold * dist_threshold)
  {
    publish_pose_command(current_command.ContactPoint);
  }
  else
  {
    contact_started = ros::Time::now();
    contact_status = ContactStatus::InContact;
  }
}

void ContactInterfaceNode::contact()
{
  const float contact_duration = 5; // in seconds
  current_force = current_command.Force; // in Newtons
  current_moment = current_command.Moment; // in N.m

  if (ros::Time::now() - contact_started > ros::Duration(contact_duration))
  {
    contact_status = ContactStatus::Departing;
  }

}

void ContactInterfaceNode::depart()
{
  current_force = 0;
  current_moment = 0;

  double dx = current_pose.pose.position.x - starting_pose.pose.position.x;
  double dy = current_pose.pose.position.y - starting_pose.pose.position.y;
  double dz = current_pose.pose.position.z - starting_pose.pose.position.z;
  double dist = dx * dx + dy * dy + dz * dz;

  const double dist_threshold = 0.25F; // 25 cm
  if (dist > dist_threshold * dist_threshold)
  {
    publish_pose_command(starting_pose.pose);
  }
  else
  {
    contact_status = ContactStatus::None;
    task_status = TaskStatus::Completed;
  }
}

ContactInterfaceNode::~ContactInterfaceNode() = default;

BaseNode* BaseNode::get()
{
  ContactInterfaceNode* control_interface_node = new ContactInterfaceNode("ControlInterfaceNode");
  return control_interface_node;
}
